idf_component_register(
    SRCS "test.cpp" "ui.cpp" "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        board_drivers
        power_manager
        lvgl_wrapper
        imu_services
        rtc_services
        sd_services
)
