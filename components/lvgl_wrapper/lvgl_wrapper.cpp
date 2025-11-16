/**
 * @file lvgl_wrapper.cpp
 * @brief LVGL initialization and driver implementation
 */

#include "lvgl_wrapper.hpp"

#include "board_config.hpp"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "gt911_simple.hpp"

constexpr static const char *TAG = "LVGL_WRAPPER";

namespace lvgl_wrapper {

// Forward declarations for driver callbacks
static void displayFlushCb(lv_display_t *disp, const lv_area_t *area, uint8_t *pxMap);
static void touchpadReadCb(lv_indev_t *indev, lv_indev_data_t *data);

// Static hardware handles reference (for callbacks)
static BoardDrivers::HardwareHandles sHw = {};

esp_err_t init(const BoardDrivers::HardwareHandles &hw, LvglHandles &lvglHandles) {
    ESP_LOGI(
        TAG,
        "Initializing LVGL v%d.%d.%d",
        lv_version_major(),
        lv_version_minor(),
        lv_version_patch()
    );

    // Store hardware handles for callbacks
    sHw = hw;

    // 1. Initialize LVGL library
    lv_init();

    // 2. Create LVGL port configuration
    const lvgl_port_cfg_t LVGL_CFG = {
        .task_priority = 4,        // LVGL task priority
        .task_stack = 6144,        // Stack size in bytes
        .task_affinity = -1,       // Core affinity (-1 = no affinity)
        .task_max_sleep_ms = 500,  // Maximum sleep time
        .task_stack_caps = 0,      // Default stack capabilities (MALLOC_CAP_DEFAULT)
        .timer_period_ms = 5       // Timer period (5ms = 200Hz refresh)
    };

    ESP_ERROR_CHECK(lvgl_port_init(&LVGL_CFG));

    // 3. Create LVGL display manually (RGB panels don't work well with lvgl_port_add_disp)
    const size_t BUFFER_SIZE = BoardConfig::LCD_WIDTH * 40;  // 40 lines buffer

    // Allocate draw buffers (DMA capable)
    void *buf1 = heap_caps_malloc(
        BUFFER_SIZE * sizeof(uint16_t),
        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
    );
    void *buf2 = heap_caps_malloc(
        BUFFER_SIZE * sizeof(uint16_t),
        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
    );

    if (buf1 == nullptr || buf2 == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffers");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Allocated LVGL buffers: %zu bytes each", BUFFER_SIZE * sizeof(uint16_t));

    // Create LVGL display
    lvglHandles.display = lv_display_create(BoardConfig::LCD_WIDTH, BoardConfig::LCD_HEIGHT);
    if (lvglHandles.display == nullptr) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return ESP_FAIL;
    }

    // Set draw buffers
    lv_display_set_buffers(
        lvglHandles.display,
        buf1,
        buf2,
        BUFFER_SIZE * sizeof(uint16_t),
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );

    // Set color format (RGB565)
    lv_display_set_color_format(lvglHandles.display, LV_COLOR_FORMAT_RGB565);

    // Set flush callback
    lv_display_set_flush_cb(lvglHandles.display, displayFlushCb);

    ESP_LOGI(TAG, "Display created: %dx%d", BoardConfig::LCD_WIDTH, BoardConfig::LCD_HEIGHT);

    // 5. Create touch input device manually (no lvgl_port_add_touch in this version)
    lvglHandles.touchpad = lv_indev_create();
    if (lvglHandles.touchpad == nullptr) {
        ESP_LOGE(TAG, "Failed to create LVGL touch device");
        return ESP_FAIL;
    }

    lv_indev_set_type(lvglHandles.touchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lvglHandles.touchpad, touchpadReadCb);
    lv_indev_set_display(lvglHandles.touchpad, lvglHandles.display);

    ESP_LOGI(TAG, "Touch input device created");

    // 7. Set default theme
    lv_theme_t *theme = lv_theme_default_init(
        lvglHandles.display,
        lv_palette_main(LV_PALETTE_BLUE),  // Primary color
        lv_palette_main(LV_PALETTE_RED),   // Secondary color
        true,                              // Dark mode
        LV_FONT_DEFAULT                    // Default font
    );
    lv_display_set_theme(lvglHandles.display, theme);

    ESP_LOGI(TAG, "LVGL initialization complete!");

    return ESP_OK;
}

uint32_t getTimerPeriodMs() {
    return 5;  // Match the timer_period_ms from config
}

bool lock(uint32_t timeoutMs) {
    return lvgl_port_lock(timeoutMs);
}

void unlock() {
    lvgl_port_unlock();
}

// ============================================================================
// Private callback functions
// ============================================================================

/**
 * @brief LVGL display flush callback
 * @details Called by LVGL to transfer rendered buffer to display
 */
static void displayFlushCb(lv_display_t *disp, const lv_area_t *area, uint8_t *pxMap) {
    if (sHw.lcdHandle == nullptr || sHw.lcdHandle->rgbPanel == nullptr) {
        ESP_LOGE(TAG, "LCD handle is null in flush callback!");
        lv_display_flush_ready(disp);
        return;
    }

    // Calculate dimensions
    int x1 = area->x1;
    int y1 = area->y1;
    int x2 = area->x2 + 1;  // LVGL uses inclusive coordinates
    int y2 = area->y2 + 1;

    // Send bitmap to RGB panel
    esp_lcd_panel_draw_bitmap(sHw.lcdHandle->rgbPanel, x1, y1, x2, y2, pxMap);

    // Notify LVGL that flush is complete
    lv_display_flush_ready(disp);
}

/**
 * @brief LVGL touch input read callback
 * @details Called by LVGL to read touch input state
 */
static void touchpadReadCb(lv_indev_t *indev, lv_indev_data_t *data) {
    if (sHw.touch == nullptr) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    gt911::TouchPoint touchPoints[gt911::MAX_TOUCH_POINTS];
    uint8_t numTouches = 0;

    // Read touch data from GT911
    if (gt911::readTouchData(sHw.touch, touchPoints, gt911::MAX_TOUCH_POINTS, &numTouches) ==
        ESP_OK) {
        if (numTouches > 0) {
            // Accept ALL touch sizes (even very light touches)
            // Android-style: no minimum size threshold
            data->point.x = touchPoints[0].x;
            data->point.y = touchPoints[0].y;
            data->state = LV_INDEV_STATE_PRESSED;

            // Debug: Log touch coordinates every 10th touch to avoid spam
            static int touchCount = 0;
            if (++touchCount % 10 == 0) {
                ESP_LOGI(
                    TAG,
                    "Touch: x=%d, y=%d, size=%d",
                    touchPoints[0].x,
                    touchPoints[0].y,
                    touchPoints[0].size
                );
            }
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

}  // namespace lvgl_wrapper
