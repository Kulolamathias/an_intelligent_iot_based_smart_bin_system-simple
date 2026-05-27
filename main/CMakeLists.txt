idf_component_register(
    SRCS 
        "main.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        # core
        # services
        nvs_flash
        driver
        config
        common_utils
        # tests
        esp_event
        esp_timer
        efuse
        drivers
)