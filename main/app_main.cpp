/*
 * NES Emulator for M5Stack Tab5 with File Browser
 * ESP-IDF Framework
 */

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nes_osd.h"
#include "bsp/m5stack_tab5.h"
#include "battery_monitor.h"
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid.h"

// Nofrendo
extern "C" {
    #include "nofrendo.h"
    int nofrendo_main(int argc, char *argv[]);
}

static const char *TAG = "NES_FILE_BROWSER";

// Display dimensions (landscape: 1280x720)
#define DISPLAY_WIDTH  1280
#define DISPLAY_HEIGHT 720

// File browser state
#define MAX_ROMS 100
static char romFiles[MAX_ROMS][256];
static int romCount = 0;
static int selectedFile = 0;
static const int MAX_VISIBLE_FILES = 16;  // Number of files visible on screen
static bool showBrowser = true;

// Colors (RGB565)
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_YELLOW  0xFFE0
#define COLOR_BLUE    0x001F
#define COLOR_GREEN   0x07E0
#define COLOR_RED     0xF800

// USB Host handles
static usb_host_client_handle_t s_usb_client_handle = NULL;

// USB Host client event callback
static void usb_host_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(TAG, "USB Host: New device connected on address %d", event_msg->new_dev.address);
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(TAG, "USB Host: Device disconnected");
            break;
        default:
            break;
    }
}

// USB Host client event handler task
static void usb_host_client_task(void* arg)
{
    while (1) {
        if (s_usb_client_handle) {
            usb_host_client_handle_events(s_usb_client_handle, portMAX_DELAY);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// HID Device event callback
static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void* arg)
{
    hid_host_dev_params_t dev_params;
    esp_err_t ret = hid_host_device_get_params(hid_device_handle, &dev_params);
    if (ret != ESP_OK) {
        return;
    }
    
    switch (event) {
        case HID_HOST_DRIVER_EVENT_CONNECTED: {
            hid_host_dev_info_t dev_info;
            ret = hid_host_get_device_info(hid_device_handle, &dev_info);
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "USB Gamepad CONNECTED");
                ESP_LOGI(TAG, "  VID: 0x%04X", dev_info.VID);
                ESP_LOGI(TAG, "  PID: 0x%04X", dev_info.PID);
                
                // Set VID/PID in nes_osd for gamepad parsing
                nes_usb_gamepad_set_vid_pid(dev_info.VID, dev_info.PID);
            }
            
            const hid_host_device_config_t dev_config = {
                .callback = nes_usb_hid_interface_callback,
                .callback_arg = NULL
            };
            
            ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
            ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
            
            // Wait for device to initialize (as in working nes_tab5_usb_host project)
            vTaskDelay(pdMS_TO_TICKS(100));
            
            ESP_LOGI(TAG, "USB Gamepad ready!");
            break;
        }
        
        default:
            break;
    }
}

// Scan ROM files from /sd/roms/ directory
static void scanROMFiles(void)
{
    ESP_LOGI(TAG, "Scanning /sd/roms/ for .nes files...");
    
    romCount = 0;
    
    DIR* dir = opendir("/sd/roms");
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open /sd/roms directory");
        return;
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL && romCount < MAX_ROMS) {
        // Skip hidden files and directories
        if (entry->d_name[0] == '.') {
            continue;
        }
        
        // Check if file has .nes extension (case insensitive)
        const char* name = entry->d_name;
        size_t len = strlen(name);
        if (len >= 4) {
            const char* ext = name + len - 4;
            if (strcasecmp(ext, ".nes") == 0) {
                // Copy filename (without path)
                strncpy(romFiles[romCount], name, sizeof(romFiles[0]) - 1);
                romFiles[romCount][sizeof(romFiles[0]) - 1] = '\0';
                romCount++;
                ESP_LOGI(TAG, "  Found: %s", name);
            }
        }
    }
    
    closedir(dir);
    
    ESP_LOGI(TAG, "Found %d ROM files", romCount);
    
    if (romCount == 0) {
        ESP_LOGW(TAG, "No ROM files found in /sd/roms/");
    }
}

// Draw file browser
static void drawFileBrowser(void)
{
    // Clear screen with black
    nes_display_clear(COLOR_BLACK);
    
    // Title
    nes_display_draw_string(20, 20, "Select NES Game", COLOR_WHITE, 2);
    
    // File counter (e.g., "1/10")
    if (romCount > 0) {
        char counter[32];
        snprintf(counter, sizeof(counter), "%d/%d", selectedFile + 1, romCount);
        // Move counter left to avoid overlap with battery indicator (battery starts at x=1230)
        // Counter width: ~64 pixels, battery width: ~50 pixels + text
        // Position counter at x=1100 to leave ~80 pixels gap
        int counter_x = DISPLAY_WIDTH - strlen(counter) * 8 * 2 - 180;  // Changed from -20 to -180
        nes_display_draw_string(counter_x, 20, counter, COLOR_WHITE, 2);
    }
    
    // Draw battery indicator in top-right corner
    nes_display_draw_battery_indicator();
    
    // File list area (no border)
    int list_x = 20;
    int list_y = 60;
    
    if (romCount == 0) {
        // No ROMs found message
        nes_display_draw_string(40, DISPLAY_HEIGHT / 2 - 20, "No ROM files found", COLOR_RED, 2);
        nes_display_draw_string(40, DISPLAY_HEIGHT / 2 + 20, "Place .nes files in /sd/roms/", COLOR_WHITE, 1);
    } else {
        // Calculate which files to show (keep selected file centered)
        int visibleStart = selectedFile - MAX_VISIBLE_FILES / 2;
        if (visibleStart < 0) {
            visibleStart = 0;
        }
        if (visibleStart + MAX_VISIBLE_FILES > romCount) {
            visibleStart = romCount - MAX_VISIBLE_FILES;
            if (visibleStart < 0) {
                visibleStart = 0;
            }
        }
        
        // Draw file list
        int file_y = list_y;
        for (int i = 0; i < MAX_VISIBLE_FILES && (visibleStart + i) < romCount; i++) {
            int fileIndex = visibleStart + i;
            bool isSelected = (fileIndex == selectedFile);
            
            // File name
            const char* fileName = romFiles[fileIndex];
            
            // Selected file: larger text (1.5x = 3x scale from 2x base), yellow color
            // Unselected files: normal text (2x scale), white color
            if (isSelected) {
                nes_display_draw_string(list_x, file_y, fileName, COLOR_YELLOW, 3);
            } else {
                nes_display_draw_string(list_x, file_y, fileName, COLOR_WHITE, 2);
            }
            
            file_y += 40;  // Spacing between files
        }
    }
    
    // Flush to display
    nes_display_flush();
}

// Handle input for file browser
static void handleFileBrowserInput(void)
{
    // Update input state safely (without nofrendo event system)
    nes_input_update_state_safe();
    
    // Handle navigation (D-Pad up/down)
    static bool up_was_pressed = false;
    static bool down_was_pressed = false;
    static bool start_was_pressed = false;
    static bool a_was_pressed = false;
    
    bool up_pressed = nes_input_is_up_pressed();
    bool down_pressed = nes_input_is_down_pressed();
    bool start_pressed = nes_input_is_start_pressed();
    bool a_pressed = nes_input_is_a_pressed();
    
    // Up button (with edge detection)
    if (up_pressed && !up_was_pressed) {
        if (selectedFile > 0) {
            selectedFile--;
        }
    }
    up_was_pressed = up_pressed;
    
    // Down button (with edge detection)
    if (down_pressed && !down_was_pressed) {
        if (selectedFile < romCount - 1) {
            selectedFile++;
        }
    }
    down_was_pressed = down_pressed;
    
    // Start or A button - launch game
    if ((start_pressed && !start_was_pressed) || (a_pressed && !a_was_pressed)) {
        if (romCount > 0 && selectedFile >= 0 && selectedFile < romCount) {
            showBrowser = false;  // Exit browser loop
        }
    }
    start_was_pressed = start_pressed;
    a_was_pressed = a_pressed;
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "NES Emulator for M5Stack Tab5");
    ESP_LOGI(TAG, "With File Browser");
    ESP_LOGI(TAG, "ESP-IDF Framework");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize OSD (display, sound, input)
    ESP_LOGI(TAG, "Initializing OSD...");
    ret = nes_osd_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OSD initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "NES Emulator initialized successfully!");
    
    // Initialize SD card
    ESP_LOGI(TAG, "Initializing SD card...");
    char mount_point[] = "/sd";
    esp_err_t sd_ret = bsp_sdcard_init(mount_point, 5);
    if (sd_ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card initialization failed: %s", esp_err_to_name(sd_ret));
        ESP_LOGE(TAG, "Please insert SD card and restart");
        
        // Show error on display
        nes_display_clear(COLOR_BLACK);
        nes_display_draw_string(20, DISPLAY_HEIGHT / 2 - 20, "SD Card Error", COLOR_RED, 2);
        nes_display_draw_string(20, DISPLAY_HEIGHT / 2 + 20, "Insert SD card and restart", COLOR_WHITE, 1);
        nes_display_flush();
        
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "SD card initialized and mounted at /sd");
    
    // Check if USB-C is connected (may conflict with USB Host)
    bool usb_c_connected = bsp_usb_c_detect();
    if (usb_c_connected) {
        ESP_LOGW(TAG, "USB-C cable detected - USB Host may not work");
        ESP_LOGW(TAG, "Disconnect USB-C cable from computer for USB Host to work");
    }
    
    // Start USB Host
    ESP_LOGI(TAG, "Initializing USB Host...");
    esp_err_t usb_host_ret = bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, true);
    if (usb_host_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start USB Host: %s", esp_err_to_name(usb_host_ret));
        ESP_LOGW(TAG, "USB Host disabled - USB gamepad will not work");
        // Continue without USB Host
    } else {
        // Warning: USB Host may not work when device is connected to computer via USB-C
        if (usb_c_connected) {
            ESP_LOGW(TAG, "NOTE: USB Host may not work when connected to computer via USB-C");
            ESP_LOGW(TAG, "Disconnect USB-C cable from computer for USB Host to work");
        }
        
        // Enable USB-A port power
        bsp_set_usb_5v_en(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Enable USB Host root port power
        ret = usb_host_lib_set_root_port_power(true);
        if (ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGI(TAG, "USB Host root port power already enabled");
        }
        
        // Register USB Host client
        usb_host_client_config_t client_config = {
            .is_synchronous = false,
            .max_num_event_msg = 5,
            .async = {
                .client_event_callback = usb_host_event_callback,
                .callback_arg = NULL
            }
        };
        
        ret = usb_host_client_register(&client_config, &s_usb_client_handle);
        if (ret == ESP_OK) {
            xTaskCreate(usb_host_client_task, "usb_host_client", 4096, NULL, 5, NULL);
            ESP_LOGI(TAG, "USB Host client registered");
        } else {
            ESP_LOGW(TAG, "Failed to register USB Host client: %s", esp_err_to_name(ret));
        }
        
        // Install HID Host driver
        const hid_host_driver_config_t hid_host_driver_config = {
            .create_background_task = true,
            .task_priority = 5,
            .stack_size = 4096,
            .core_id = 0,
            .callback = hid_host_device_event,
            .callback_arg = NULL
        };
        
        ret = hid_host_install(&hid_host_driver_config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "HID Host driver installed");
            ESP_LOGI(TAG, "Waiting for USB gamepad to be connected...");
            ESP_LOGI(TAG, "Connect gamepad to USB-A port on Tab5");
        } else {
            ESP_LOGW(TAG, "Failed to install HID Host driver: %s", esp_err_to_name(ret));
        }
    }
    
    // Initialize battery monitor AFTER USB Host (to avoid conflicts)
    ESP_LOGI(TAG, "Enabling battery charging...");
    bsp_set_charge_en(true);
    ESP_LOGI(TAG, "Battery charging enabled");
    
    ESP_LOGI(TAG, "Initializing battery monitor...");
    i2c_master_bus_handle_t i2c_bus = bsp_i2c_get_handle();
    if (i2c_bus) {
        ret = battery_monitor_init(i2c_bus);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Battery monitor initialized successfully");
            battery_status_t status;
            if (battery_monitor_read(&status) == ESP_OK) {
                ESP_LOGI(TAG, "Battery: %d%%, %dmV, Charging: %s", 
                         status.level, status.voltage_mv, 
                         status.is_charging ? "YES" : "NO");
            }
        } else {
            ESP_LOGW(TAG, "Battery monitor initialization failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "I2C bus not available for battery monitor");
    }
    
    // Scan ROM files
    scanROMFiles();
    
    // File browser loop
    showBrowser = true;
    selectedFile = 0;
    
    while (showBrowser) {
        // Draw file browser
        drawFileBrowser();
        
        // Handle input
        handleFileBrowserInput();
        
        // Small delay
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Launch selected ROM
    if (romCount > 0 && selectedFile >= 0 && selectedFile < romCount) {
        // Use static buffer instead of stack variable to prevent stack overflow
        static char rom_path[512];
        snprintf(rom_path, sizeof(rom_path), "/sd/roms/%s", romFiles[selectedFile]);
        
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Starting NES emulator...");
        ESP_LOGI(TAG, "ROM: %s", rom_path);
        ESP_LOGI(TAG, "========================================");
        
        // Run nofrendo (this is blocking)
        char* argv_[1] = { rom_path };
        nofrendo_main(1, argv_);
        
        ESP_LOGE(TAG, "NES emulator exited unexpectedly!");
    }
    
    // Should not reach here
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

