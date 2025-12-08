idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES nes_emulator driver freertos nvs_flash fatfs m5stack_tab5 usb_host_hid
)

# Disable graphic library in BSP (we use direct LCD API)
target_compile_definitions(${COMPONENT_LIB} PRIVATE BSP_CONFIG_NO_GRAPHIC_LIB=1)

