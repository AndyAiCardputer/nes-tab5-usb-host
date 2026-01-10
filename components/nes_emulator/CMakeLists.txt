# NES Emulator Component for Tab5
# Includes nofrendo library and OSD functions

# Collect all nofrendo source files
file(GLOB_RECURSE NOFRENDO_SOURCES
    "nofrendo/*.c"
)

# OSD source files (to be created)
set(OSD_SOURCES
    "src/nes_osd.c"
)

idf_component_register(
    SRCS 
        ${NOFRENDO_SOURCES}
        ${OSD_SOURCES}
    INCLUDE_DIRS 
        "include"
        "nofrendo"
        "nofrendo/nes"
        "nofrendo/mappers"
        "nofrendo/sndhrdw"
        "nofrendo/cpu"
        "nofrendo/libsnss"
    REQUIRES 
        driver 
        esp_lcd 
        freertos 
        nvs_flash 
        fatfs
        m5stack_tab5
        usb_host_hid
        battery_monitor
    PRIV_REQUIRES 
        esp_codec_dev
        esp_mm
)

# Disable graphic library in BSP (we use direct LCD API)
target_compile_definitions(${COMPONENT_LIB} PRIVATE BSP_CONFIG_NO_GRAPHIC_LIB=1)

