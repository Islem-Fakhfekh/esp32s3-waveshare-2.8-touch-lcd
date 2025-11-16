idf_component_register(
    SRCS "src/rtc_services.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        waveshare__pcf85063a
        esp_timer
)
