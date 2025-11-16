/**
 * @file lcd_st7701_rgb.cpp
 * @brief ST7701 LCD Controller + RGB Interface Driver Implementation
 * @details Waveshare yaklaşımını takip eder - Manuel SPI + Ayrı RGB panel
 */

#include "lcd_st7701_rgb.hpp"
#include <cstring>
#include "board_config.hpp"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace BoardDrivers::lcd {

using namespace BoardConfig;

constexpr static const char *TAG = "LCD_ST7701_RGB";

// =============================================================================
// STEP 1: Manuel SPI Komut Gönderme Fonksiyonları (Waveshare Tarzı)
// =============================================================================

/**
 * @brief ST7701'e SPI üzerinden komut gönder
 */
static esp_err_t st7701_send_command(spi_device_handle_t spi, uint8_t cmd) {
    spi_transaction_t trans = {};
    trans.length = 0;  // Sadece address (komut) gönder
    trans.rxlength = 0;
    trans.cmd = 0;     // D/C = 0 (command)
    trans.addr = cmd;  // Komut address line'a yazılır

    return spi_device_transmit(spi, &trans);
}

/**
 * @brief ST7701'e SPI üzerinden veri gönder
 */
static esp_err_t st7701_send_data(spi_device_handle_t spi, uint8_t data) {
    spi_transaction_t trans = {};
    trans.length = 0;
    trans.rxlength = 0;
    trans.cmd = 1;      // D/C = 1 (data)
    trans.addr = data;  // Veri address line'a yazılır

    return spi_device_transmit(spi, &trans);
}

// =============================================================================
// STEP 2: ST7701 Init Sequence (Waveshare'den Birebir)
// =============================================================================

/**
 * @brief ST7701 initialization komutlarını gönder (2.8" panel için)
 */
static esp_err_t st7701SendInitCommands(spi_device_handle_t spi) {
    ESP_LOGI(TAG, "Sending ST7701 init commands (Waveshare 2.8inch sequence)...");

// Macro for easier command sending
#define CMD(c) st7701_send_command(spi, c)  // SPI_WriteComm
#define DATA(d) st7701_send_data(spi, d)    // DATA
#define DELAY(ms) vTaskDelay(pdMS_TO_TICKS(ms))

    CMD(0xFF);
    DATA(0x77);
    DATA(0x01);
    DATA(0x00);
    DATA(0x00);
    DATA(0x13);
    CMD(0xEF);
    DATA(0x08);
    CMD(0xFF);
    DATA(0x77);
    DATA(0x01);
    DATA(0x00);
    DATA(0x00);
    DATA(0x10);
    CMD(0xC0);
    DATA(0x4F);
    DATA(0x00);
    CMD(0xC1);
    DATA(0x10);
    DATA(0x02);
    CMD(0xC2);
    DATA(0x07);
    DATA(0x02);
    CMD(0xCC);
    DATA(0x10);
    CMD(0xB0);
    DATA(0x00);
    DATA(0x10);
    DATA(0x17);
    DATA(0x0D);
    DATA(0x11);
    DATA(0x06);
    DATA(0x05);
    DATA(0x08);
    DATA(0x07);
    DATA(0x1F);
    DATA(0x04);
    DATA(0x11);
    DATA(0x0E);
    DATA(0x29);
    DATA(0x30);
    DATA(0x1F);
    CMD(0xB1);
    DATA(0x00);
    DATA(0x0D);
    DATA(0x14);
    DATA(0x0E);
    DATA(0x11);
    DATA(0x06);
    DATA(0x04);
    DATA(0x08);
    DATA(0x08);
    DATA(0x20);
    DATA(0x05);
    DATA(0x13);
    DATA(0x13);
    DATA(0x26);
    DATA(0x30);
    DATA(0x1F);
    CMD(0xFF);
    DATA(0x77);
    DATA(0x01);
    DATA(0x00);
    DATA(0x00);
    DATA(0x11);
    CMD(0xB0);
    DATA(0x65);
    CMD(0xB1);
    DATA(0x71);
    CMD(0xB2);
    DATA(0x82);  // 87
    CMD(0xB3);
    DATA(0x80);
    CMD(0xB5);
    DATA(0x42);  // 4D
    CMD(0xB7);
    DATA(0x85);
    CMD(0xB8);
    DATA(0x20);
    CMD(0xC0);
    DATA(0x09);
    CMD(0xC1);
    DATA(0x78);
    CMD(0xC2);
    DATA(0x78);
    CMD(0xD0);
    DATA(0x88);
    CMD(0xEE);
    DATA(0x42);

    CMD(0xE0);
    DATA(0x00);
    DATA(0x00);
    DATA(0x02);
    CMD(0xE1);
    DATA(0x04);
    DATA(0xA0);
    DATA(0x06);
    DATA(0xA0);
    DATA(0x05);
    DATA(0xA0);
    DATA(0x07);
    DATA(0xA0);
    DATA(0x00);
    DATA(0x44);
    DATA(0x44);
    CMD(0xE2);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    CMD(0xE3);
    DATA(0x00);
    DATA(0x00);
    DATA(0x22);
    DATA(0x22);
    CMD(0xE4);
    DATA(0x44);
    DATA(0x44);
    CMD(0xE5);
    DATA(0x0c);
    DATA(0x90);
    DATA(0xA0);
    DATA(0xA0);
    DATA(0x0E);
    DATA(0x92);
    DATA(0xA0);
    DATA(0xA0);
    DATA(0x08);
    DATA(0x8C);
    DATA(0xA0);
    DATA(0xA0);
    DATA(0x0A);
    DATA(0x8E);
    DATA(0xA0);
    DATA(0xA0);
    CMD(0xE6);
    DATA(0x00);
    DATA(0x00);
    DATA(0x22);
    DATA(0x22);
    CMD(0xE7);
    DATA(0x44);
    DATA(0x44);
    CMD(0xE8);
    DATA(0x0D);
    DATA(0x91);
    DATA(0xA0);
    DATA(0xA0);
    DATA(0x0F);
    DATA(0x93);
    DATA(0xA0);
    DATA(0xA0);
    DATA(0x09);
    DATA(0x8D);
    DATA(0xA0);
    DATA(0xA0);
    DATA(0x0B);
    DATA(0x8F);
    DATA(0xA0);
    DATA(0xA0);
    // CMD(0xE9);DATA( 2);DATA(CMD(0x36);DATA(0x00);
    CMD(0xEB);
    DATA(0x00);
    DATA(0x00);
    DATA(0xE4);
    DATA(0xE4);
    DATA(0x44);
    DATA(0x00);
    DATA(0x40);

    // CRITICAL: MADCTL (Memory Data Access Control) - Controls coordinate mapping
    // MY=0, MX=0, MV=0, ML=0, RGB=0 -> Portrait mode (480x640), no transform
    CMD(0x36);
    DATA(0x00);

    CMD(0xED);
    DATA(0xFF);
    DATA(0xF5);
    DATA(0x47);
    DATA(0x6F);
    DATA(0x0B);
    DATA(0xA1);
    DATA(0xAB);
    DATA(0xFF);
    DATA(0xFF);
    DATA(0xBA);
    DATA(0x1A);
    DATA(0xB0);
    DATA(0xF6);
    DATA(0x74);
    DATA(0x5F);
    DATA(0xFF);
    CMD(0xEF);
    DATA(0x08);
    DATA(0x08);
    DATA(0x08);
    DATA(0x40);
    DATA(0x3F);
    DATA(0x64);
    CMD(0xFF);
    DATA(0x77);
    DATA(0x01);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    CMD(0xFF);
    DATA(0x77);
    DATA(0x01);
    DATA(0x00);
    DATA(0x00);
    DATA(0x13);
    CMD(0xE6);
    DATA(0x16);
    DATA(0x7C);
    CMD(0xE8);
    DATA(0x00);
    DATA(0x0E);
    CMD(0xFF);
    DATA(0x77);
    DATA(0x01);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    CMD(0x11);
    DATA(0x00);
    DELAY(200);
    CMD(0xFF);
    DATA(0x77);
    DATA(0x01);
    DATA(0x00);
    DATA(0x00);
    DATA(0x13);
    CMD(0xE8);
    DATA(0x00);
    DATA(0x0C);
    DELAY(150);
    CMD(0xE8);
    DATA(0x00);
    DATA(0x00);
    CMD(0xFF);
    DATA(0x77);
    DATA(0x01);
    DATA(0x00);
    DATA(0x00);
    DATA(0x00);
    CMD(0x29);
    DATA(0x00);
    CMD(0x35);
    DATA(0x00);

    CMD(0x11);
    DATA(0x00);  // sleep out
    DELAY(200);

    CMD(0x29);
    DATA(0x00);  // display on
    CMD(0x29);
    DATA(0x00);  // display on
    DELAY(100);

#undef CMD
#undef DATA
#undef DELAY

    ESP_LOGI(TAG, "✓ ST7701 init commands sent successfully");
    return ESP_OK;
}

// =============================================================================
// STEP 3: Public API Implementation
// =============================================================================

esp_err_t st7701RgbInit(const ST7701Config &config, ST7701Handle **outHandle) {
    ESP_LOGI(TAG, "=== Initializing ST7701 + RGB (Waveshare Style) ===");

    esp_err_t ret = 0;

    // Allocate handle
    auto *handle = (ST7701Handle *)calloc(1, sizeof(ST7701Handle));

    if (handle == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate handle");
        return ESP_ERR_NO_MEM;
    }

    handle->ioExpander = config.ioExpander;
    handle->width = config.width;
    handle->height = config.height;

    // --- STEP 1: LCD Reset (IO Expander) ---
    // NOT: Waveshare LCD power'ı TCA9554 ile kontrol etmiyor, doğrudan powered
    ESP_LOGI(TAG, "Step 1: LCD Reset via IO Expander (EXIO1 = P0_0)");

    // Reset: LOW -> HIGH (Waveshare sequence)
    ESP_ERROR_CHECK(esp_io_expander_set_level(handle->ioExpander, TCA9554_LCD_RESET_MASK, 0));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(esp_io_expander_set_level(handle->ioExpander, TCA9554_LCD_RESET_MASK, 1));
    vTaskDelay(pdMS_TO_TICKS(10));

    // CS Enable (LOW = active) - CRITICAL: EXIO3 = P0_2 NOT P0_1!
    ESP_LOGI(TAG, "Step 2: CS Enable (EXIO3 = P0_2, mask 0x04)");
    ESP_ERROR_CHECK(esp_io_expander_set_level(handle->ioExpander, TCA9554_LCD_CS_MASK, 0));
    vTaskDelay(pdMS_TO_TICKS(10));

    // --- STEP 2: SPI Bus for ST7701 Commands ---
    ESP_LOGI(TAG, "Step 3: Initialize SPI for ST7701 commands");

    spi_bus_config_t busConfig = {
        .mosi_io_num = config.spiMosi,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = config.spiSclk,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 0,
    };

    vTaskDelay(pdMS_TO_TICKS(100));
    ret = spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // INVALID_STATE = already initialized
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        free(handle);
        return ret;
    }

    // SPI Device config (3-wire SPI for ST7701)
    spi_device_interface_config_t devConfig = {};
    devConfig.command_bits = 1;                  // D/C bit
    devConfig.address_bits = 8;                  // Command/data byte
    devConfig.mode = 0;                          // SPI mode 0
    devConfig.clock_speed_hz = 4 * 1000 * 1000;  // 4MHz (Waveshare)
    devConfig.spics_io_num = -1;                 // CS controlled by IO expander
    devConfig.queue_size = 1;

    ret = spi_bus_add_device(SPI2_HOST, &devConfig, &handle->spiDevice);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        free(handle);
        return ret;
    }

    ESP_LOGI(TAG, "✓ SPI configured");

    // --- STEP 3: Send ST7701 Init Commands ---
    ESP_LOGI(TAG, "Step 4: Sending ST7701 init commands");
    ret = st7701SendInitCommands(handle->spiDevice);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ST7701 init failed");
        spi_bus_remove_device(handle->spiDevice);
        free(handle);
        return ret;
    }

    // KRİTİK: CS'i henüz kapatma! Waveshare sırası:
    // 1. ST7701 init (CS enable)
    // 2. RGB panel oluştur
    // 3. RGB panel reset/init
    // 4. CS disable

    // --- STEP 4: Create RGB Panel (Ayrı!) ---
    ESP_LOGI(TAG, "Step 5: Creating RGB panel (separate from ST7701)");

    esp_lcd_rgb_panel_config_t rgbConfig = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings =
            {
                .pclk_hz = config.pixelClockHz,
                // Match Waveshare EXACTLY: h_res=480, v_res=640 (portrait panel)
                .h_res = 480,
                .v_res = 640,
                .hsync_pulse_width = 8,
                .hsync_back_porch = 10,
                .hsync_front_porch = 50,
                .vsync_pulse_width = 2,
                .vsync_back_porch = 18,
                .vsync_front_porch = 8,
                .flags =
                    {
                        .hsync_idle_low = 0,
                        .vsync_idle_low = 0,
                        .de_idle_high = 0,
                        .pclk_active_neg = 0,
                    },
            },
        .data_width = 16,
        .bits_per_pixel = 16,
        .num_fbs = 1,
        .bounce_buffer_size_px = 10 * 480,  // CRITICAL: Match Waveshare! 10 * H_RES
        .dma_burst_size = 64,               // CRITICAL: DMA burst size (replaces psram_trans_align)
        .hsync_gpio_num = LCD_PIN_HSYNC,
        .vsync_gpio_num = LCD_PIN_VSYNC,
        .de_gpio_num = LCD_PIN_DE,
        .pclk_gpio_num = LCD_PIN_PCLK,
        .disp_gpio_num = -1,
        .data_gpio_nums =
            {
                LCD_PIN_DATA0,
                LCD_PIN_DATA1,
                LCD_PIN_DATA2,
                LCD_PIN_DATA3,
                LCD_PIN_DATA4,
                LCD_PIN_DATA5,
                LCD_PIN_DATA6,
                LCD_PIN_DATA7,
                LCD_PIN_DATA8,
                LCD_PIN_DATA9,
                LCD_PIN_DATA10,
                LCD_PIN_DATA11,
                LCD_PIN_DATA12,
                LCD_PIN_DATA13,
                LCD_PIN_DATA14,
                LCD_PIN_DATA15,
            },
        .flags =
            {
                .fb_in_psram = 1,
            },
    };

    ret = esp_lcd_new_rgb_panel(&rgbConfig, &handle->rgbPanel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RGB panel create failed: %s", esp_err_to_name(ret));
        spi_bus_remove_device(handle->spiDevice);
        free(handle);
        return ret;
    }

    ESP_LOGI(TAG, "✓ RGB panel created");

    // --- Step 6: RGB panel reset & init (Waveshare yapar!) ---
    ESP_LOGI(TAG, "Step 6: RGB panel reset & init");
    ret = esp_lcd_panel_reset(handle->rgbPanel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RGB panel reset failed: %s", esp_err_to_name(ret));
        esp_lcd_panel_del(handle->rgbPanel);
        spi_bus_remove_device(handle->spiDevice);
        free(handle);
        return ret;
    }

    ret = esp_lcd_panel_init(handle->rgbPanel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RGB panel init failed: %s", esp_err_to_name(ret));
        esp_lcd_panel_del(handle->rgbPanel);
        spi_bus_remove_device(handle->spiDevice);
        free(handle);
        return ret;
    }

    ESP_LOGI(TAG, "✓ RGB panel initialized");

    // --- Step 7: CS Disable (RGB init'ten SONRA!) ---
    ESP_LOGI(TAG, "Step 7: CS Disable (EXIO3 = P0_2, mask 0x04 -> HIGH)");
    ret = esp_io_expander_set_level(handle->ioExpander, TCA9554_LCD_CS_MASK, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable CS: %s", esp_err_to_name(ret));
        esp_lcd_panel_del(handle->rgbPanel);
        spi_bus_remove_device(handle->spiDevice);
        free(handle);
        return ret;
    }

    // KRİTİK GECİKME: Waveshare'de backlight açmadan önce 10ms bekleniyor
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "✓ ST7701 + RGB initialization complete (Waveshare style)");

    *outHandle = handle;
    return ESP_OK;
}

esp_err_t st7701RgbGetFrameBuffer(ST7701Handle *handle, uint32_t fbNum, void **frameBuffer) {
    if ((handle == nullptr) || (handle->rgbPanel == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_lcd_rgb_panel_get_frame_buffer(handle->rgbPanel, fbNum, frameBuffer);
}

esp_err_t st7701RgbDrawBitmap(
    ST7701Handle *handle,
    int xStart,
    int yStart,
    int xEnd,
    int yEnd,
    const void *colorData
) {
    if ((handle == nullptr) || (handle->rgbPanel == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_lcd_panel_draw_bitmap(handle->rgbPanel, xStart, yStart, xEnd, yEnd, colorData);
}

esp_err_t st7701RgbDisplayOnOff(ST7701Handle *handle, bool on) {
    if ((handle == nullptr) || (handle->rgbPanel == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_lcd_panel_disp_on_off(handle->rgbPanel, on);
}

esp_err_t st7701RgbDel(ST7701Handle *handle) {
    if (handle == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->spiDevice != nullptr) {
        spi_bus_remove_device(handle->spiDevice);
    }

    if (handle->rgbPanel != nullptr) {
        esp_lcd_panel_del(handle->rgbPanel);
    }

    free(handle);
    return ESP_OK;
}

}  // namespace BoardDrivers::lcd
