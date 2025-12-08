/*
 * NES Emulator for M5Stack Tab5 with USB Host Support
 * ESP-IDF Framework
 */

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nes_osd.h"
#include "bsp/m5stack_tab5.h"
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid.h"

// Nofrendo
extern "C" {
    #include "nofrendo.h"
    int nofrendo_main(int argc, char *argv[]);
}

static const char *TAG = "NES_USB_HOST";

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
            
            ESP_LOGI(TAG, "USB Gamepad ready!");
            break;
        }
        
        default:
            break;
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "NES Emulator for M5Stack Tab5");
    ESP_LOGI(TAG, "With USB Host Gamepad Support");
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
        ESP_LOGE(TAG, "Halting...");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "SD card initialized and mounted at /sd");
    
    // Start USB Host
    ESP_LOGI(TAG, "Initializing USB Host...");
    ESP_ERROR_CHECK(bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, true));
    
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
    
    // Check for ROM file in /sd/roms/game.nes
    ESP_LOGI(TAG, "Checking for ROM file...");
    const char* rom_path = "/sd/roms/game.nes";
    struct stat st;
    
    if (stat(rom_path, &st) == 0) {
        ESP_LOGI(TAG, "  ✓ ROM found: %s (%ld bytes)", rom_path, st.st_size);
        
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Starting NES emulator...");
        ESP_LOGI(TAG, "========================================");
        
        // Run nofrendo (this is blocking)
        char* argv_[1] = { (char*)rom_path };
        nofrendo_main(1, argv_);
        
        ESP_LOGE(TAG, "NES emulator exited unexpectedly!");
    } else {
        ESP_LOGE(TAG, "  ✗ ROM not found: %s", rom_path);
        ESP_LOGE(TAG, "  Please copy game.nes to /sd/roms/ folder");
        ESP_LOGE(TAG, "  Halting...");
    }
    
    // Should not reach here
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

