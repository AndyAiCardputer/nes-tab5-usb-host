idf_component_register(
    SRCS "src/battery_monitor.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2c
)

