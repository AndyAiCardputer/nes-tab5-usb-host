/*
 * NES OSD for M5Stack Tab5 (ESP-IDF)
 * OSD functions for nofrendo NES emulator
 */

#include "nes_osd.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "bsp/m5stack_tab5.h"
#include "bsp/display.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_types.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

// Nofrendo includes
#include "osd.h"
#include "noftypes.h"
#include "vid_drv.h"
#include "bitmap.h"
#include "event.h"
#include "nofrendo.h"
#include "log.h"
#include "nofconfig.h"
#include "nes/nesinput.h"

// USB Host includes
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid.h"
#include <string.h>

static const char *TAG = "NES_OSD";

// Display handles
static bsp_lcd_handles_t lcd_handles;
static esp_lcd_panel_handle_t panel_handle = NULL;
static bool display_initialized = false;

// NES screen dimensions
#define NES_SCREEN_WIDTH  256
#define NES_SCREEN_HEIGHT 240

// Tab5 display dimensions (portrait: 720×1280)
#define DISPLAY_PHYSICAL_WIDTH   720
#define DISPLAY_PHYSICAL_HEIGHT  1280

// Scale for portrait orientation (same as ZX Spectrum)
// 256 × 2.8125 = 720 (fits exactly in width)
// 240 × 2.8125 = 675 (centered in height)
#define SCALE_NUMERATOR 45  // 2.8125 = 45/16
#define SCALE_DENOMINATOR 16

// Framebuffer for scaling
static uint16_t* framebuffer = NULL;

// ============================================================================
// PAHUB AND I2C KEYBOARD SUPPORT
// ============================================================================

#define PAHUB_ADDR 0x70
#define PAHUB_CH_JOYSTICK2 0  // Joystick2 on Channel 0
#define PAHUB_CH_SCROLL_A   1  // Scroll #1 (Button A) on Channel 1
#define PAHUB_CH_SCROLL_B   2  // Scroll #2 (Button B) on Channel 2
#define PAHUB_CH_KEYBOARD 3  // CardKeyBoard on Channel 3
#define CARDKEYBOARD_ADDR 0x5F
#define JOYSTICK2_ADDR 0x63
#define SCROLL_ADDR 0x40

// PaHub state
static i2c_master_dev_handle_t pahub_dev_handle = NULL;
static uint8_t pahub_current_channel = 0xFF;
static bool pahub_available = false;
static bool keyboard_available = false;
static bool joystick2_available = false;
static bool scroll_a_available = false;
static bool scroll_b_available = false;

// CardKeyBoard state
static i2c_master_dev_handle_t keyboard_dev_handle = NULL;
static unsigned long last_keyboard_read = 0;
#define KEYBOARD_POLL_INTERVAL 10  // ms (reduced for lower latency)

// Joystick2 state
static i2c_master_dev_handle_t joystick2_dev_handle = NULL;
#define JOYSTICK2_REG_ADC_X_8   0x10
#define JOYSTICK2_REG_ADC_Y_8   0x11
#define JOYSTICK2_REG_BUTTON    0x20

// Scroll buttons state (only buttons, no encoder)
static i2c_master_dev_handle_t scroll_a_dev_handle = NULL;
static i2c_master_dev_handle_t scroll_b_dev_handle = NULL;
#define SCROLL_BUTTON_REG 0x20
#define DEBOUNCE_DELAY_MS 50

// Button state with debounce
typedef struct {
    bool current_state;
    bool last_state;
    unsigned long last_change_time;
    bool debounced_state;
} ButtonState;

static ButtonState button_a_state = {false, false, 0, false};
static ButtonState button_b_state = {false, false, 0, false};

// Select PaHub channel
static esp_err_t select_pahub_channel(uint8_t channel) {
    if (channel > 5) return ESP_ERR_INVALID_ARG;
    if (pahub_current_channel == channel) return ESP_OK;
    
    uint8_t data = 1 << channel;
    esp_err_t ret = i2c_master_transmit(pahub_dev_handle, &data, 1, 100);  // 100 ms timeout, not pdMS_TO_TICKS!
    
    if (ret == ESP_OK) {
        pahub_current_channel = channel;
        vTaskDelay(pdMS_TO_TICKS(1));  // vTaskDelay uses ticks - this is correct
    } else {
        ESP_LOGW(TAG, "PaHub channel %d select error: %s", channel, esp_err_to_name(ret));
    }
    
    return ret;
}

// Read CardKeyBoard key via PaHub Channel 3
// CardKeyBoard works as Slave device - read directly without register
static uint8_t read_keyboard_key(void) {
    if (!keyboard_available || !pahub_available) return 0;
    
    unsigned long now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_keyboard_read < KEYBOARD_POLL_INTERVAL) {
        return 0;
    }
    last_keyboard_read = now;
    
    // Select keyboard channel
    if (select_pahub_channel(PAHUB_CH_KEYBOARD) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to select PaHub channel %d for keyboard", PAHUB_CH_KEYBOARD);
        return 0;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));  // Delay for I2C stabilization after channel switch
    
    uint8_t key = 0xAA;  // Initialize with non-zero to detect if not overwritten
    esp_err_t ret = i2c_master_receive(keyboard_dev_handle, &key, 1, 50);  // 50 ms timeout, not pdMS_TO_TICKS!
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Keyboard read error: %s (key=0x%02X)", esp_err_to_name(ret), key);
        return 0;
    }
    
    ESP_LOGD(TAG, "Keyboard raw key=0x%02X", key);
    
    // Return raw value without filtering - filtering is done above
    return key;
}

// Read Joystick2 data via PaHub Channel 0
static bool read_joystick2(uint8_t* x, uint8_t* y, uint8_t* button) {
    if (!joystick2_available || !pahub_available || !joystick2_dev_handle) return false;
    
    // Select Joystick2 channel
    if (select_pahub_channel(PAHUB_CH_JOYSTICK2) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to select PaHub channel %d for joystick", PAHUB_CH_JOYSTICK2);
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Read X
    uint8_t reg = JOYSTICK2_REG_ADC_X_8;
    esp_err_t ret = i2c_master_transmit_receive(joystick2_dev_handle, &reg, 1, x, 1, 50);  // 50 ms
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Joystick2 X read error: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Read Y
    reg = JOYSTICK2_REG_ADC_Y_8;
    ret = i2c_master_transmit_receive(joystick2_dev_handle, &reg, 1, y, 1, 50);  // 50 ms
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Joystick2 Y read error: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Read button
    reg = JOYSTICK2_REG_BUTTON;
    uint8_t btn = 0;
    ret = i2c_master_transmit_receive(joystick2_dev_handle, &reg, 1, &btn, 1, 50);  // 50 ms
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Joystick2 button read error: %s", esp_err_to_name(ret));
        return false;
    }
    *button = (btn == 0) ? 1 : 0;  // Invert
    
    ESP_LOGD(TAG, "Joystick2: x=%d y=%d btn=%d", *x, *y, *button);
    
    return true;
}

// Read button from Scroll unit (only button, no encoder)
static bool read_scroll_button(uint8_t channel, i2c_master_dev_handle_t dev_handle, bool* button_pressed) {
    if (!pahub_available || !dev_handle) return false;
    
    // Select channel
    if (select_pahub_channel(channel) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to select PaHub channel %d for scroll button", channel);
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Read button register (0x20: 0 = pressed, 1 = not pressed)
    uint8_t reg = SCROLL_BUTTON_REG;
    uint8_t btn = 0;
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, &btn, 1, 50);  // 50 ms
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Scroll button read error on channel %d: %s", channel, esp_err_to_name(ret));
        return false;
    }
    
    *button_pressed = (btn == 0);
    return true;
}

// Update button state with debounce
static void update_button_state(ButtonState* state, uint8_t channel, i2c_master_dev_handle_t dev_handle) {
    bool current_raw = false;
    if (!read_scroll_button(channel, dev_handle, &current_raw)) {
        return;  // Read failed, keep last state
    }
    
    state->current_state = current_raw;
    unsigned long now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // If state changed, update timestamp
    if (state->current_state != state->last_state) {
        state->last_change_time = now;
        state->last_state = state->current_state;
    }
    
    // Debounce: only update debounced state if stable for DEBOUNCE_DELAY_MS
    if ((now - state->last_change_time) >= DEBOUNCE_DELAY_MS) {
        state->debounced_state = state->current_state;
    }
}

esp_err_t nes_display_init(void)
{
    if (display_initialized) {
        ESP_LOGW(TAG, "Display already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing Tab5 display...");

    // Initialize display using BSP
    bsp_display_config_t display_config = {};
    esp_err_t ret = bsp_display_new_with_handles_to_st7123(&display_config, &lcd_handles);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display: %s", esp_err_to_name(ret));
        return ret;
    }

    panel_handle = lcd_handles.panel;
    if (panel_handle == NULL) {
        ESP_LOGE(TAG, "Panel handle is NULL");
        return ESP_FAIL;
    }

    // Turn on display
    ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn on display: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize brightness
    ret = bsp_display_brightness_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize brightness: %s", esp_err_to_name(ret));
    }
    bsp_display_brightness_set(80);

    // Allocate framebuffer in PSRAM (physical size!)
    size_t fb_size = DISPLAY_PHYSICAL_WIDTH * DISPLAY_PHYSICAL_HEIGHT * sizeof(uint16_t);
    framebuffer = (uint16_t*)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    if (!framebuffer) {
        framebuffer = (uint16_t*)malloc(fb_size);
        if (!framebuffer) {
            ESP_LOGE(TAG, "Failed to allocate framebuffer");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGW(TAG, "Framebuffer allocated in internal RAM");
    } else {
        ESP_LOGI(TAG, "Framebuffer allocated in PSRAM (%zu bytes)", fb_size);
    }

    display_initialized = true;
    ESP_LOGI(TAG, "Display initialized: %dx%d", DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT);
    return ESP_OK;
}

void nes_display_render_frame(const uint8_t** data, int width, int height)
{
    (void)width; (void)height;  // Use NES_SCREEN_WIDTH/HEIGHT
    
    if (!display_initialized || !data || !framebuffer) {
        return;
    }

    // TODO: Implement scaling 256×240 → 720×1280 (portrait)
    // For now, just clear screen
    memset(framebuffer, 0, DISPLAY_PHYSICAL_WIDTH * DISPLAY_PHYSICAL_HEIGHT * sizeof(uint16_t));
    
    // Draw framebuffer to display
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, framebuffer);
}

esp_err_t nes_input_init(void)
{
    ESP_LOGI(TAG, "Initializing input...");
    
    // Initialize external I2C for PaHub
    esp_err_t ret = bsp_ext_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init external I2C: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "Input initialization: External I2C not available");
        return ret;
    }
    
    i2c_master_bus_handle_t i2c_bus = bsp_ext_i2c_get_handle();
    
    // Initialize PaHub device
    i2c_device_config_t pahub_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PAHUB_ADDR,
        .scl_speed_hz = 100000,  // 100 kHz
    };
    
    ret = i2c_master_bus_add_device(i2c_bus, &pahub_config, &pahub_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PaHub device: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "Input initialization: PaHub not available");
        return ret;
    }
    
    // Deselect all channels initially
    uint8_t data = 0x00;
    ret = i2c_master_transmit(pahub_dev_handle, &data, 1, 100);  // 100 ms timeout, not pdMS_TO_TICKS!
    if (ret == ESP_OK) {
        pahub_available = true;
        pahub_current_channel = 0xFF;
        ESP_LOGI(TAG, "PaHub detected at 0x%02X", PAHUB_ADDR);
    } else {
        ESP_LOGW(TAG, "PaHub not found at 0x%02X: %s", PAHUB_ADDR, esp_err_to_name(ret));
        return ret;
    }
    
    // Detect Joystick2 on Channel 0
    ESP_LOGI(TAG, "Checking Joystick2 on Channel 0...");
    if (select_pahub_channel(PAHUB_CH_JOYSTICK2) == ESP_OK) {
        i2c_device_config_t joystick2_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = JOYSTICK2_ADDR,
            .scl_speed_hz = 100000,
        };
        
        ret = i2c_master_bus_add_device(i2c_bus, &joystick2_config, &joystick2_dev_handle);
        if (ret == ESP_OK) {
            // Test read
            uint8_t reg = JOYSTICK2_REG_ADC_X_8;
            uint8_t test_val = 0;
            ret = i2c_master_transmit_receive(joystick2_dev_handle, &reg, 1, &test_val, 1, 50);  // 50 ms
            if (ret == ESP_OK) {
                joystick2_available = true;
                ESP_LOGI(TAG, "Joystick2 found at 0x%02X (Channel 0)", JOYSTICK2_ADDR);
            } else {
                ESP_LOGW(TAG, "Joystick2 not responding: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "Failed to add Joystick2 device: %s", esp_err_to_name(ret));
        }
    }
    
    // Detect Button A (Scroll unit on Channel 1, only button)
    ESP_LOGI(TAG, "Checking Button A on Channel 1...");
    if (select_pahub_channel(PAHUB_CH_SCROLL_A) == ESP_OK) {
        i2c_device_config_t scroll_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = SCROLL_ADDR,
            .scl_speed_hz = 100000,
        };
        
        ret = i2c_master_bus_add_device(i2c_bus, &scroll_config, &scroll_a_dev_handle);
        if (ret == ESP_OK) {
            scroll_a_available = true;
            ESP_LOGI(TAG, "Button A found at 0x%02X (Channel 1)", SCROLL_ADDR);
        } else {
            ESP_LOGW(TAG, "Failed to add Button A device: %s", esp_err_to_name(ret));
        }
    }
    
    // Detect Button B (Scroll unit on Channel 2, only button)
    ESP_LOGI(TAG, "Checking Button B on Channel 2...");
    if (select_pahub_channel(PAHUB_CH_SCROLL_B) == ESP_OK) {
        i2c_device_config_t scroll_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = SCROLL_ADDR,
            .scl_speed_hz = 100000,
        };
        
        ret = i2c_master_bus_add_device(i2c_bus, &scroll_config, &scroll_b_dev_handle);
        if (ret == ESP_OK) {
            scroll_b_available = true;
            ESP_LOGI(TAG, "Button B found at 0x%02X (Channel 2)", SCROLL_ADDR);
        } else {
            ESP_LOGW(TAG, "Failed to add Button B device: %s", esp_err_to_name(ret));
        }
    }
    
    // Detect CardKeyBoard on Channel 3
    ESP_LOGI(TAG, "Checking CardKeyBoard on Channel 3...");
    if (select_pahub_channel(PAHUB_CH_KEYBOARD) == ESP_OK) {
        // Delay after channel switch before device initialization
        vTaskDelay(pdMS_TO_TICKS(10));
        
        i2c_device_config_t keyboard_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = CARDKEYBOARD_ADDR,
            .scl_speed_hz = 100000,
        };
        
        ret = i2c_master_bus_add_device(i2c_bus, &keyboard_config, &keyboard_dev_handle);
        if (ret == ESP_OK) {
            // CardKeyBoard - Slave device, read directly without register
            // Try several times with logging
            uint8_t test_key = 0;
            bool found = false;
            
            for (int i = 0; i < 3; i++) {
                vTaskDelay(pdMS_TO_TICKS(10));
                test_key = 0xAA;  // Initialize with non-zero to detect if not overwritten
                ret = i2c_master_receive(keyboard_dev_handle, &test_key, 1, 50);  // 50 ms timeout, not pdMS_TO_TICKS!
                
                ESP_LOGI(TAG, "CardKeyBoard probe #%d: ret=%s, key=0x%02X", 
                         i, esp_err_to_name(ret), test_key);
                
                if (ret == ESP_OK) {
                    keyboard_available = true;
                    found = true;
                    ESP_LOGI(TAG, "CardKeyBoard found at 0x%02X (Channel 3)", CARDKEYBOARD_ADDR);
                    break;
                }
            }
            
            if (!found) {
                ESP_LOGW(TAG, "CardKeyBoard not responding after 3 attempts: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "Failed to add CardKeyBoard device: %s", esp_err_to_name(ret));
        }
    }
    
    if (!keyboard_available && !joystick2_available && !scroll_a_available && !scroll_b_available) {
        ESP_LOGW(TAG, "Input initialization: No input devices available");
    } else {
        ESP_LOGI(TAG, "Input initialization: Devices ready (KB:%d Joy:%d ScrollA:%d ScrollB:%d)", 
                 keyboard_available, joystick2_available, scroll_a_available, scroll_b_available);
    }
    
    return ESP_OK;
}

void nes_input_process(void)
{
    // This function is called from osd_getinput() now
    // Keep it empty or remove it
}

// ============================================================================
// AUDIO - ES8388 Codec Implementation (based on audio_test)
// ============================================================================

// Audio settings
#define AUDIO_SAMPLE_RATE     48000  // 48 kHz for ES8388 (same as audio_test)
#define NES_FPS               60     // NES runs at 60 FPS (NTSC)
#define AUDIO_CHANNELS        2      // STEREO
#define AUDIO_BITS            16
#define AUDIO_CHUNK_SIZE      (AUDIO_SAMPLE_RATE / NES_FPS)  // 800 samples for synchronization with 60 FPS (48000/60=800)
#define AUDIO_AMPLITUDE_SCALE 0.25f  // Amplitude scaling (25% of maximum to avoid clipping)

// Test sine wave for codec/I2S testing (comment out to use APU)
// #define USE_TEST_SINE  // Commented out to use real APU

// Audio state
static esp_codec_dev_handle_t s_codec_handle = NULL;
static bool s_audio_initialized = false;
static void (*s_audio_cb)(void *buffer, int length) = NULL;  // Audio callback from NES APU

// Audio task state
static TaskHandle_t s_audio_task_handle = NULL;
static bool s_audio_task_running = false;

// Forward declaration
static void audio_playback_task(void* pvParameters);

esp_err_t nes_audio_init(void)
{
    ESP_LOGI(TAG, "Initializing ES8388 audio codec...");
    
    // 1. Initialize BSP I2C (if not already initialized)
    esp_err_t ret = bsp_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 2. Initialize I2S (standard I2S mode, not TDM!)
    ret = bsp_audio_init(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 3. Initialize ES8388 codec
    s_codec_handle = bsp_audio_codec_speaker_init();
    if (s_codec_handle == NULL) {
        ESP_LOGE(TAG, "Codec init failed!");
        return ESP_FAIL;
    }
    
    // 4. Configure codec format: STEREO, 16-bit, 48000 Hz
    esp_codec_dev_sample_info_t sample_info = {
        .bits_per_sample = AUDIO_BITS,
        .channel = AUDIO_CHANNELS,        // STEREO
        .channel_mask = 0,
        .sample_rate = AUDIO_SAMPLE_RATE, // 48000 Hz for ES8388
        .mclk_multiple = 0,
    };
    
    // 5. Open codec
    ret = esp_codec_dev_open(s_codec_handle, &sample_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Codec open failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 6. Set volume (0-100) - increase to 80% as in audio_test for normal volume
    ret = esp_codec_dev_set_out_vol(s_codec_handle, 100);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Set volume failed: %s", esp_err_to_name(ret));
    }
    
    // 7. Unmute
    ret = esp_codec_dev_set_out_mute(s_codec_handle, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Unmute failed: %s", esp_err_to_name(ret));
    }
    
    // 8. Start audio playback task (without queue)
    s_audio_task_running = true;
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        audio_playback_task,      // Task function
        "audio_playback",         // Task name
        8192,                     // Stack size
        NULL,                     // Parameters
        5,                        // Priority (high priority)
        &s_audio_task_handle,     // Task handle
        1                         // Core 1 (avoid conflict with video on core 0)
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task!");
        s_audio_task_running = false;
        return ESP_FAIL;
    }
    
    s_audio_initialized = true;
    ESP_LOGI(TAG, "✓ Audio initialized successfully");
    ESP_LOGI(TAG, "  Sample rate: %d Hz", AUDIO_SAMPLE_RATE);
    ESP_LOGI(TAG, "  Format: STEREO, %d-bit", AUDIO_BITS);
    ESP_LOGI(TAG, "  Chunk size: %d samples (independent from FPS)", AUDIO_CHUNK_SIZE);
    ESP_LOGI(TAG, "  APU refresh_rate: 60 Hz (expects %d samples per tick)", AUDIO_CHUNK_SIZE);
    ESP_LOGI(TAG, "  Audio task started on core 1");
    
    return ESP_OK;
}

void nes_audio_shutdown(void)
{
    // Stop audio task
    s_audio_task_running = false;
    if (s_audio_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(100));  // Wait for task to finish
        s_audio_task_handle = NULL;
        ESP_LOGI(TAG, "Audio task should be stopped now");
    }
    
    // Close codec
    if (s_codec_handle) {
        esp_codec_dev_close(s_codec_handle);
        s_codec_handle = NULL;
    }
    
    s_audio_initialized = false;
    ESP_LOGI(TAG, "Audio shutdown complete");
}

esp_err_t nes_osd_init(void)
{
    ESP_LOGI(TAG, "Initializing NES OSD...");
    
    esp_err_t ret = nes_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Display init failed");
        return ret;
    }
    
    ret = nes_input_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Input init failed (non-critical)");
    }
    
    ret = nes_audio_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Audio init failed (non-critical)");
    }
    
    ESP_LOGI(TAG, "NES OSD initialized");
    return ESP_OK;
}

void nes_osd_shutdown(void)
{
    nes_audio_shutdown();
    display_initialized = false;
    if (framebuffer) {
        free(framebuffer);
        framebuffer = NULL;
    }
}

// ============================================================================
// NOFRENDO OSD FUNCTIONS
// ============================================================================

// Memory allocation
void *mem_alloc(int size, bool prefer_fast_memory)
{
    (void)prefer_fast_memory;
    // Try PSRAM first for large allocations
    if (size > 1024) {
        void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
        if (ptr) return ptr;
    }
    return malloc(size);
}

// Video driver functions (for viddriver_t structure)
static int vid_driver_init(int width, int height)
{
    (void)width; (void)height;
    return 0;  // Already initialized in nes_display_init()
}

static void vid_driver_shutdown(void)
{
    // Cleanup handled by osd_shutdown
}

static int vid_set_mode(int width, int height)
{
    (void)width; (void)height;
    return 0;
}

// Palette conversion (RGB to RGB565)
static uint16_t nes_palette[256] = {0};

static void vid_set_palette(rgb_t *pal)
{
    // Convert RGB palette to RGB565
    for (int i = 0; i < 256; i++) {
        uint16_t r = (pal[i].r >> 3) & 0x1F;
        uint16_t g = (pal[i].g >> 2) & 0x3F;
        uint16_t b = (pal[i].b >> 3) & 0x1F;
        nes_palette[i] = (r << 11) | (g << 5) | b;
    }
}

static void vid_clear(uint8_t color)
{
    (void)color;
    if (framebuffer) {
        memset(framebuffer, 0, DISPLAY_PHYSICAL_WIDTH * DISPLAY_PHYSICAL_HEIGHT * sizeof(uint16_t));
    }
}

// NES framebuffer (256x240)
static uint8_t nes_framebuffer[NES_SCREEN_WIDTH * NES_SCREEN_HEIGHT];
static bitmap_t *nes_bitmap = NULL;

static bitmap_t *vid_lock_write(void)
{
    if (!nes_bitmap) {
        nes_bitmap = bmp_createhw(nes_framebuffer, NES_SCREEN_WIDTH, NES_SCREEN_HEIGHT, NES_SCREEN_WIDTH);
    }
    return nes_bitmap;
}

static void vid_free_write(int num_dirties, rect_t *dirty_rects)
{
    (void)num_dirties; (void)dirty_rects;
    // Persistent framebuffer, no free needed
}

/**
 * Audio playback task
 * Генерирует аудио независимо от FPS (как в audio_test)
 */
static void audio_playback_task(void* pvParameters)
{
    // Buffers for audio generation (static, not on stack)
#ifndef USE_TEST_SINE
    static int16_t mono_buf[AUDIO_CHUNK_SIZE];
#endif
    static int16_t stereo_buf[AUDIO_CHUNK_SIZE * 2];
    
    ESP_LOGI(TAG, "Audio playback task started");
    ESP_LOGI(TAG, "  Chunk size: %d samples (independent from FPS)", AUDIO_CHUNK_SIZE);
    ESP_LOGI(TAG, "  APU expects: %d samples per tick (refresh_rate=60)", AUDIO_CHUNK_SIZE);
    
    while (s_audio_task_running) {
        // Initialization check (safety)
        if (!s_codec_handle || !s_audio_initialized) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
#ifdef USE_TEST_SINE
        // TEST SINE WAVE for codec/I2S testing (instead of APU)
        static float phase = 0.0f;
        static bool sine_logged = false;
        
        float freq = 440.0f; // A note
        float step = 2.0f * M_PI * freq / (float)AUDIO_SAMPLE_RATE;
        
        for (int i = 0; i < AUDIO_CHUNK_SIZE; i++) {
            float v = sinf(phase);
            phase += step;
            if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
            
            int16_t mono = (int16_t)(v * 6553.0f);  // 20% of amplitude
            int16_t sample = (int16_t)((float)mono * AUDIO_AMPLITUDE_SCALE);
            
            stereo_buf[i * 2] = sample;      // Left channel
            stereo_buf[i * 2 + 1] = sample;  // Right channel
        }
        
        // Log only once
        if (!sine_logged) {
            ESP_LOGI(TAG, "Using TEST SINE (440 Hz) instead of APU");
            sine_logged = true;
        }
#else
        // 1. Request APU to generate AUDIO_CHUNK_SIZE mono samples
        // APU expects 800 samples per tick (refresh_rate=60, sample_rate=48000)
        if (!s_audio_cb) {
            // APU not ready yet, fill with silence
            memset(stereo_buf, 0, AUDIO_CHUNK_SIZE * 2 * sizeof(int16_t));
        } else {
            s_audio_cb((void*)mono_buf, AUDIO_CHUNK_SIZE);
            
            // 2. Convert MONO → STEREO with scaling
            for (int i = 0; i < AUDIO_CHUNK_SIZE; i++) {
                int16_t sample = (int16_t)((float)mono_buf[i] * AUDIO_AMPLITUDE_SCALE);
                stereo_buf[i * 2] = sample;      // Left channel
                stereo_buf[i * 2 + 1] = sample;  // Right channel
            }
        }
#endif
        
        // 3. Write to codec
        int bytes_to_write = AUDIO_CHUNK_SIZE * 2 * sizeof(int16_t);
        int ret = esp_codec_dev_write(s_codec_handle, stereo_buf, bytes_to_write);
        
        if (ret < 0) {
            static int error_count = 0;
            if (error_count++ < 10) {
                ESP_LOGW(TAG, "Audio write failed: %s", esp_err_to_name(ret));
            }
        }
        
        // 4. REMOVED: vTaskDelay causes crackling (interrupts continuous I2S DMA stream)
        // If CPU overload issues occur - can try to restore with 1ms delay
        // vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_LOGI(TAG, "Audio playback task stopped");
    vTaskDelete(NULL);
}


static void vid_custom_blit(bitmap_t *bmp, int num_dirties, rect_t *dirty_rects)
{
    (void)num_dirties; (void)dirty_rects;
    
    static int blit_count = 0;
    blit_count++;
    if (blit_count <= 10 || (blit_count % 60) == 0) {
        ESP_LOGI(TAG, "vid_custom_blit #%d called", blit_count);
    }
    
    if (!bmp || !bmp->line || !framebuffer || !display_initialized) {
        return;
    }
    
    const uint8_t **src_lines = (const uint8_t **)bmp->line;
    
    // Black background (faster than memset for large buffer)
    const uint16_t bg = 0x0000;  // black
    for (int i = 0; i < DISPLAY_PHYSICAL_WIDTH * DISPLAY_PHYSICAL_HEIGHT; i++) {
        framebuffer[i] = bg;
    }
    
    // Logical NES dimensions after scaling
    const int LOGICAL_HEIGHT = (NES_SCREEN_HEIGHT * SCALE_NUMERATOR) / SCALE_DENOMINATOR;  // 675
    
    // After rotation: 675×720 → 720×675
    const int offset_x = 0;
    const int offset_y = (DISPLAY_PHYSICAL_HEIGHT - LOGICAL_HEIGHT) / 2; // (1280 - 675)/2 = 302
    
    // Render NES → logic (720×675) → rotation → physical (720×1280)
    for (int sy = 0; sy < NES_SCREEN_HEIGHT; sy++) {
        for (int sx = 0; sx < NES_SCREEN_WIDTH; sx++) {
            uint8_t palette_idx = src_lines[sy][sx];
            uint16_t rgb565 = nes_palette[palette_idx];
            
            int base_log_x = (sx * SCALE_NUMERATOR) / SCALE_DENOMINATOR;
            int base_log_y = (sy * SCALE_NUMERATOR) / SCALE_DENOMINATOR;
            
            int next_log_x = ((sx + 1) * SCALE_NUMERATOR) / SCALE_DENOMINATOR;
            int next_log_y = ((sy + 1) * SCALE_NUMERATOR) / SCALE_DENOMINATOR;
            
            int pixel_width  = next_log_x - base_log_x;
            int pixel_height = next_log_y - base_log_y;
            
            for (int dy = 0; dy < pixel_height; dy++) {
                int log_y = base_log_y + dy;
                for (int dx = 0; dx < pixel_width; dx++) {
                    int log_x = base_log_x + dx;
                    
                    // Rotate 90° right: (x,y) -> (y, H - 1 - x)
                    int phys_x = log_y + offset_x;
                    int phys_y = LOGICAL_HEIGHT - 1 - log_x + offset_y;
                    
                    if (phys_x >= 0 && phys_x < DISPLAY_PHYSICAL_WIDTH &&
                        phys_y >= 0 && phys_y < DISPLAY_PHYSICAL_HEIGHT) {
                        framebuffer[phys_y * DISPLAY_PHYSICAL_WIDTH + phys_x] = rgb565;
                    }
                }
            }
        }
    }
    
    // Draw to display
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 
                              DISPLAY_PHYSICAL_WIDTH, DISPLAY_PHYSICAL_HEIGHT, framebuffer);
    
    // Audio is now independent from FPS, called in audio_playback_task
}

static viddriver_t tab5_driver = {
    "Tab5 LCD",
    vid_driver_init, vid_driver_shutdown, vid_set_mode, vid_set_palette, vid_clear,
    vid_lock_write, vid_free_write, vid_custom_blit,
    false
};

// OSD functions for nofrendo
void osd_getvideoinfo(vidinfo_t *info)
{
    info->default_width = NES_SCREEN_WIDTH;
    info->default_height = NES_SCREEN_HEIGHT;
    info->driver = &tab5_driver;
}

// Timer
static TimerHandle_t nes_timer = NULL;

int osd_installtimer(int frequency, void *func, int funcsize, void *counter, int countersize)
{
    (void)funcsize; (void)counter; (void)countersize;
    
    if (nes_timer) {
        xTimerDelete(nes_timer, 0);
        nes_timer = NULL;
    }
    
    nes_timer = xTimerCreate("nes", pdMS_TO_TICKS(1000 / frequency), pdTRUE, NULL, (TimerCallbackFunction_t)func);
    if (nes_timer) {
        xTimerStart(nes_timer, 0);
        return 0;
    }
    return -1;
}

// Sound functions for nofrendo OSD
int osd_init_sound(void)
{
    // Audio already initialized in nes_audio_init()
    // Just return success
    return 0;
}

void osd_stopsound(void)
{
    nes_audio_shutdown();
    s_audio_cb = NULL;
}

void osd_setsound(void (*playfunc)(void *buffer, int size))
{
    s_audio_cb = playfunc;
    ESP_LOGI(TAG, "Audio callback set: %p", playfunc);
    ESP_LOGI(TAG, "Expected apu_process address should match this callback");
}

void osd_getsoundinfo(sndinfo_t *info)
{
    info->sample_rate = AUDIO_SAMPLE_RATE;
    info->bps = AUDIO_BITS;
    ESP_LOGI(TAG, "osd_getsoundinfo: sample_rate=%d, bps=%d", info->sample_rate, info->bps);
}

// ============================================================================
// USB HOST GAMEPAD SUPPORT
// ============================================================================

// USB Gamepad state structure
typedef struct {
    uint16_t buttons;
    uint8_t  dpad;
    int8_t   left_x;
    int8_t   left_y;
    int8_t   right_x;
    int8_t   right_y;
    uint8_t  left_trigger;
    uint8_t  right_trigger;
} usb_gamepad_state_t;

static usb_gamepad_state_t s_usb_gamepad_state = {};
static bool s_usb_gamepad_connected = false;
static uint16_t s_usb_gamepad_vid = 0;
static uint16_t s_usb_gamepad_pid = 0;

// Turbo button state (for Square/Triangle)
static bool turbo_a_active = false;
static bool turbo_b_active = false;
#define TURBO_FREQUENCY 10  // Turbo presses per second (10 Hz)

// Parse USB HID gamepad report
static void parse_usb_gamepad_report(const uint8_t* data, size_t len, usb_gamepad_state_t* state)
{
    if (len < 4) {
        return;
    }
    
    // Sony DualSense (PS5) USB HID (VID: 0x054C, PID: 0x0CE6)
    if (s_usb_gamepad_vid == 0x054C && s_usb_gamepad_pid == 0x0CE6) {
        if (len < 11) {
            return;
        }
        
        int base = 0;
        if (data[0] == 0x01) {
            base = 1;
        } else {
            return;
        }
        
        state->left_x   = (int8_t)((int)data[base + 0] - 128);
        state->left_y   = (int8_t)((int)data[base + 1] - 128);
        state->right_x  = (int8_t)((int)data[base + 2] - 128);
        state->right_y  = (int8_t)((int)data[base + 3] - 128);
        
        state->left_trigger  = data[base + 4];
        state->right_trigger = data[base + 5];
        
        uint8_t b7  = data[base + 7];
        uint8_t hat = b7 & 0x0F;
        
        if (hat <= 7) {
            state->dpad = hat + 1;
        } else {
            state->dpad = 0;
        }
        
        state->buttons = 0;
        
        if (b7 & 0x10) state->buttons |= (1 << 0);  // Square
        if (b7 & 0x20) state->buttons |= (1 << 1);  // Cross
        if (b7 & 0x40) state->buttons |= (1 << 2);  // Circle
        if (b7 & 0x80) state->buttons |= (1 << 3);  // Triangle
        
        uint8_t b8 = data[base + 8];
        if (b8 & 0x01) state->buttons |= (1 << 4);   // L1
        if (b8 & 0x02) state->buttons |= (1 << 5);   // R1
        if (b8 & 0x04) state->buttons |= (1 << 11);  // L2 (trigger)
        if (b8 & 0x08) state->buttons |= (1 << 12);  // R2 (trigger)
        if (b8 & 0x10) state->buttons |= (1 << 6);  // Create (Select)
        if (b8 & 0x20) state->buttons |= (1 << 7);   // Options (Start)
        if (b8 & 0x40) state->buttons |= (1 << 8);   // L3
        if (b8 & 0x80) state->buttons |= (1 << 9);   // R3
        
        uint8_t b9 = data[base + 9];
        if (b9 & 0x01) state->buttons |= (1 << 10);  // PS
        
        if (state->left_trigger  > 30) state->buttons |= (1 << 11);
        if (state->right_trigger > 30) state->buttons |= (1 << 12);
        
        return;
    } else {
        // Generic gamepad format
        state->buttons = data[0] | (data[1] << 8);
        
        if (len >= 3) {
            uint8_t dpad_val = data[2];
            if (dpad_val <= 8) {
                state->dpad = dpad_val;
                if (len >= 9) {
                    state->left_x = (int8_t)data[3];
                    state->left_y = (int8_t)data[4];
                    state->right_x = (int8_t)data[5];
                    state->right_y = (int8_t)data[6];
                    state->left_trigger = data[7];
                    state->right_trigger = data[8];
                }
            } else {
                state->dpad = 0;
                if (len >= 8) {
                    state->left_x = (int8_t)data[2];
                    state->left_y = (int8_t)data[3];
                    state->right_x = (int8_t)data[4];
                    state->right_y = (int8_t)data[5];
                    state->left_trigger = data[6];
                    state->right_trigger = data[7];
                }
            }
        }
    }
}

// HID Interface event callback
void nes_usb_hid_interface_callback(hid_host_device_handle_t hid_device_handle,
                                    const hid_host_interface_event_t event,
                                    void* arg)
{
    hid_host_dev_params_t dev_params;
    esp_err_t ret = hid_host_device_get_params(hid_device_handle, &dev_params);
    if (ret != ESP_OK) {
        return;
    }
    
    switch (event) {
        case HID_HOST_INTERFACE_EVENT_INPUT_REPORT: {
            uint8_t data[64] = {0};
            size_t data_length = 0;
            
            ret = hid_host_device_get_raw_input_report_data(
                hid_device_handle, data, sizeof(data), &data_length);
            
            if (ret == ESP_OK && data_length > 0) {
                parse_usb_gamepad_report(data, data_length, &s_usb_gamepad_state);
            }
            break;
        }
        
        case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "USB Gamepad DISCONNECTED");
            ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
            s_usb_gamepad_connected = false;
            memset(&s_usb_gamepad_state, 0, sizeof(s_usb_gamepad_state));
            s_usb_gamepad_vid = 0;
            s_usb_gamepad_pid = 0;
            break;
            
        default:
            break;
    }
}

// Set USB gamepad VID/PID (called from HID device event callback)
void nes_usb_gamepad_set_vid_pid(uint16_t vid, uint16_t pid)
{
    s_usb_gamepad_vid = vid;
    s_usb_gamepad_pid = pid;
    s_usb_gamepad_connected = true;
    ESP_LOGI(TAG, "USB Gamepad connected: VID=0x%04X, PID=0x%04X", vid, pid);
}

// ============================================================================
// INPUT STATE
// ============================================================================

// Input state
static uint32_t nes_input_state = 0xFFFFFFFF;  // All buttons released

void osd_getinput(void)
{
    // Map CardKeyBoard keys to NES buttons
    const int ev[8] = {
        event_joypad1_up, event_joypad1_down,
        event_joypad1_left, event_joypad1_right,
        event_joypad1_select, event_joypad1_start,
        event_joypad1_a, event_joypad1_b
    };
    
    static uint32_t old_state = 0xFFFFFFFF;
    uint32_t state = 0xFFFFFFFF;  // All buttons released by default
    
    // Read keyboard key - TEMPORARILY DISABLED (CardKeyBoard not working: ESP_ERR_INVALID_RESPONSE)
    /*
    if (keyboard_available) {
        uint8_t key = read_keyboard_key();
        
        // Filtering is done here, not in read_keyboard_key()
        // 0xAA = not overwritten (read error), 0xFF = possible error, 0 = no presses
        if (key != 0 && key != 0xFF && key != 0xAA) {
            // Map keys to NES buttons (CardKeyBoard sends codes only when pressed)
            // Arrow keys (D-pad)
            if (key == 0xB5) {  // Up
                state &= ~(1UL << 0);
            }
            if (key == 0xB6) {  // Down
                state &= ~(1UL << 1);
            }
            if (key == 0xB4) {  // Left
                state &= ~(1UL << 2);
            }
            if (key == 0xB7) {  // Right
                state &= ~(1UL << 3);
            }
            
            // Control keys
            if (key == 0x20) {  // Space = Select
                state &= ~(1UL << 4);
            }
            if (key == 0x0D) {  // Enter = Start
                state &= ~(1UL << 5);
            }
            
            // Symbol keys
            if (key == 0x2F) {  // / (slash) = A button
                state &= ~(1UL << 6);
            }
            if (key == 0x2E) {  // . (dot) = B button
                state &= ~(1UL << 7);
            }
        }
    }
    */
    
    // Read Joystick2 via PaHub Channel 0
    if (joystick2_available) {
        uint8_t joy_x = 0, joy_y = 0, joy_button = 0;
        if (read_joystick2(&joy_x, &joy_y, &joy_button)) {
            const uint8_t threshold = 40;
            const uint8_t center = 127;
            
            if (joy_x < (center - threshold)) {
                state &= ~(1UL << 3);  // Right
            }
            if (joy_x > (center + threshold)) {
                state &= ~(1UL << 2);  // Left
            }
            if (joy_y < (center - threshold)) {
                state &= ~(1UL << 1);  // Down
            }
            if (joy_y > (center + threshold)) {
                state &= ~(1UL << 0);  // Up
            }
            
            // Joystick button = Start (bit 5)
            if (joy_button == 1) {
                state &= ~(1UL << 5);  // Start
            }
        }
    }
    
    // Read buttons A and B (Scroll units, only buttons, no encoder)
    // Only read every 2 frames to reduce I2C traffic
    static uint8_t input_frame_counter = 0;
    input_frame_counter++;
    
    if ((input_frame_counter % 2) == 0) {  // Read every 2 frames (~30 FPS for input)
        // Update Button A
        if (scroll_a_available) {
            update_button_state(&button_a_state, PAHUB_CH_SCROLL_A, scroll_a_dev_handle);
            if (button_a_state.debounced_state) {
                state &= ~(1UL << 6);  // A button
            }
        }
        
        // Update Button B
        if (scroll_b_available) {
            update_button_state(&button_b_state, PAHUB_CH_SCROLL_B, scroll_b_dev_handle);
            if (button_b_state.debounced_state) {
                state &= ~(1UL << 7);  // B button
            }
        }
    }
    
    // Read USB gamepad (highest priority - overrides other inputs)
    if (s_usb_gamepad_connected) {
        // Map USB gamepad buttons to NES buttons
        // NES button bits: 0=Up, 1=Down, 2=Left, 3=Right, 4=Select, 5=Start, 6=A, 7=B
        
        // D-Pad mapping
        uint8_t dpad = s_usb_gamepad_state.dpad;
        if (dpad == 1 || dpad == 8 || dpad == 2) {  // UP, UP-LEFT, UP-RIGHT
            state &= ~(1UL << 0);  // Up
        }
        if (dpad == 5 || dpad == 6 || dpad == 4) {  // DOWN, DOWN-LEFT, DOWN-RIGHT
            state &= ~(1UL << 1);  // Down
        }
        if (dpad == 7 || dpad == 8 || dpad == 6) {  // LEFT, UP-LEFT, DOWN-LEFT
            state &= ~(1UL << 2);  // Left
        }
        if (dpad == 3 || dpad == 2 || dpad == 4) {  // RIGHT, UP-RIGHT, DOWN-RIGHT
            state &= ~(1UL << 3);  // Right
        }
        
        // Left stick as D-Pad (with threshold)
        const int8_t stick_threshold = 40;
        if (s_usb_gamepad_state.left_y < -stick_threshold) {
            state &= ~(1UL << 0);  // Up
        }
        if (s_usb_gamepad_state.left_y > stick_threshold) {
            state &= ~(1UL << 1);  // Down
        }
        if (s_usb_gamepad_state.left_x < -stick_threshold) {
            state &= ~(1UL << 2);  // Left
        }
        if (s_usb_gamepad_state.left_x > stick_threshold) {
            state &= ~(1UL << 3);  // Right
        }
        
        // Button mapping (PS5 DualSense)
        // Cross (bit 1) -> NES B (bit 7)
        if (s_usb_gamepad_state.buttons & (1 << 1)) {
            state &= ~(1UL << 7);  // B button
        }
        // Circle (bit 2) -> NES A (bit 6)
        if (s_usb_gamepad_state.buttons & (1 << 2)) {
            state &= ~(1UL << 6);  // A button
        }
        // Create (bit 6) -> NES Select (bit 4)
        if (s_usb_gamepad_state.buttons & (1 << 6)) {
            state &= ~(1UL << 4);  // Select
        }
        // Options (bit 7) -> NES Start (bit 5)
        if (s_usb_gamepad_state.buttons & (1 << 7)) {
            state &= ~(1UL << 5);  // Start
        }
        
        // Turbo buttons: Square -> Turbo B, Triangle -> Turbo A
        // Turbo works by toggling button state at TURBO_FREQUENCY Hz
        static uint32_t turbo_frame_counter = 0;
        turbo_frame_counter++;
        uint32_t turbo_period = 60 / TURBO_FREQUENCY;  // Frames per turbo cycle (assuming 60 FPS)
        if (turbo_period == 0) turbo_period = 1;  // Prevent division by zero
        
        // Square (bit 0) -> Turbo B
        if (s_usb_gamepad_state.buttons & (1 << 0)) {
            turbo_b_active = true;
            // Toggle B button based on turbo counter (50% duty cycle)
            if ((turbo_frame_counter % turbo_period) < (turbo_period / 2)) {
                state &= ~(1UL << 7);  // B button pressed
            }
        } else {
            turbo_b_active = false;
        }
        
        // Triangle (bit 3) -> Turbo A
        if (s_usb_gamepad_state.buttons & (1 << 3)) {
            turbo_a_active = true;
            // Toggle A button based on turbo counter (50% duty cycle)
            if ((turbo_frame_counter % turbo_period) < (turbo_period / 2)) {
                state &= ~(1UL << 6);  // A button pressed
            }
        } else {
            turbo_a_active = false;
        }
    }
    
    // Update nes_input_state
    nes_input_state = state;
    
    // Send events for changed buttons
    uint32_t changed = state ^ old_state;
    for (int i = 0; i < 8; i++) {
        if (changed & (1UL << i)) {
            event_t evh = event_get(ev[i]);
            if (evh) {
                bool pressed = (state & (1UL << i)) == 0;
                evh(pressed ? INP_STATE_MAKE : INP_STATE_BREAK);
            }
        }
    }
    
    old_state = state;
}

void osd_getmouse(int *x, int *y, int *button)
{
    (void)x; (void)y; (void)button;
    // Not supported
}

// Filename manipulation
void osd_fullname(char *fullname, const char *shortname)
{
    strncpy(fullname, shortname, PATH_MAX);
    fullname[PATH_MAX - 1] = '\0';
}

char *osd_newextension(char *string, char *ext)
{
    size_t l = strlen(string);
    if (l >= 3 && ext && strlen(ext) >= 3) {
        string[l - 3] = ext[1];
        string[l - 2] = ext[2];
        string[l - 1] = ext[3];
    }
    return string;
}

int osd_makesnapname(char *filename, int len)
{
    (void)filename; (void)len;
    return -1;  // Not supported
}

void osd_set_sram_ptr(uint8_t *ptr, int len)
{
    (void)ptr; (void)len;
    // TODO: Implement save states
}

const uint8_t* _get_rom_ptr(void)
{
    return NULL;  // Not using XIP
}

size_t _get_rom_size(void)
{
    return 0;  // Not using XIP
}

// OSD init/shutdown wrappers
int osd_init(void)
{
    esp_err_t ret = nes_osd_init();
    return (ret == ESP_OK) ? 0 : -1;
}

void osd_shutdown(void)
{
    nes_osd_shutdown();
    if (nes_timer) {
        xTimerDelete(nes_timer, 0);
        nes_timer = NULL;
    }
    if (nes_bitmap) {
        bmp_destroy(&nes_bitmap);
    }
}

// Main entry point
char configfilename[] = "na";

int osd_main(int argc, char *argv[])
{
    (void)argc;
    config.filename = configfilename;
    return main_loop(argv[0], system_autodetect);
}

