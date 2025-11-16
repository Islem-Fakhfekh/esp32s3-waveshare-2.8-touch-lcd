idf_component_register(
    SRCS "power_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_lcd
        esp_driver_gpio
        esp_timer
        esp_adc
        espressif__esp_lcd_touch_gt911
        waveshare__qmi8658
)
