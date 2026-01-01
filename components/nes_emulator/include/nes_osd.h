/*
 * NES OSD Header for ESP-IDF
 * OSD functions for nofrendo NES emulator
 */

#ifndef NES_OSD_H
#define NES_OSD_H

#include "esp_err.h"

// Include USB Host HID types if available
#ifdef __cplusplus
extern "C" {
#endif

#include "usb/hid_host.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

// OSD initialization
esp_err_t nes_osd_init(void);
void nes_osd_shutdown(void);

// Display functions
esp_err_t nes_display_init(void);
void nes_display_render_frame(const uint8_t** data, int width, int height);

// Input functions
esp_err_t nes_input_init(void);
void nes_input_process(void);

// Audio functions
esp_err_t nes_audio_init(void);
void nes_audio_shutdown(void);

// USB Host HID callback (for gamepad input)
void nes_usb_hid_interface_callback(hid_host_device_handle_t hid_device_handle,
                                     const hid_host_interface_event_t event,
                                     void* arg);

// Set USB gamepad VID/PID (called from HID device event callback)
void nes_usb_gamepad_set_vid_pid(uint16_t vid, uint16_t pid);

// File browser drawing functions
void nes_display_clear(uint16_t color);
void nes_display_draw_char(int x, int y, char c, uint16_t color, int scale);
void nes_display_draw_string(int x, int y, const char* str, uint16_t color, int scale);
void nes_display_draw_rect(int x, int y, int w, int h, uint16_t color);
void nes_display_flush(void);
uint16_t* nes_display_get_framebuffer(void);
void nes_display_test_gradient(void);  // Test function for stride/pitch issues
void nes_display_draw_battery_indicator(void);  // Draw battery indicator in top-right corner

// Input state reading functions (for file browser)
// Update input state without nofrendo event system (safe for file browser)
void nes_input_update_state_safe(void);
// Returns true if button is currently pressed
bool nes_input_is_up_pressed(void);
bool nes_input_is_down_pressed(void);
bool nes_input_is_left_pressed(void);
bool nes_input_is_right_pressed(void);
bool nes_input_is_start_pressed(void);
bool nes_input_is_select_pressed(void);
bool nes_input_is_a_pressed(void);
bool nes_input_is_b_pressed(void);

#ifdef __cplusplus
}
#endif

#endif // NES_OSD_H

