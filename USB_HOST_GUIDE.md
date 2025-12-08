# M5Stack Tab5 USB Host + PS5 Gamepad Guide - ESP-IDF

**Version:** 1.0  
**Date:** December 6, 2025  
**Status:** ✅ Working - full DualSense (PS5) USB support  
**Tested on:** PlayStation 5 DualSense Controller (VID: 0x054C, PID: 0x0CE6)

## Table of Contents

1. [Overview](#overview)
2. [USB Host Architecture on Tab5](#usb-host-architecture-on-tab5)
3. [How USB Host Works](#how-usb-host-works)
4. [How HID Host Works](#how-hid-host-works)
5. [DualSense USB HID Report Structure](#dualsense-usb-hid-report-structure)
6. [Gamepad Data Parsing](#gamepad-data-parsing)
7. [Complete Code Example](#complete-code-example)
8. [Common Errors and Solutions](#common-errors-and-solutions)
9. [Troubleshooting](#troubleshooting)
10. [Reference Information](#reference-information)

---

## Overview

M5Stack Tab5 has a USB-A port that can operate in **USB Host** mode. This allows connecting USB devices such as:
- Gamepads (PlayStation, Xbox, Generic HID)
- Keyboards
- Mice
- Other HID devices

**Critical points:** 
- USB-A port requires **hardware power enable** via I/O expander
- USB Host library requires **separate task** for event handling
- HID Host driver works **asynchronously** via callbacks

---

## USB Host Architecture on Tab5

```
┌─────────────┐
│   ESP32-P4  │
│             │
│  ┌────────┐ │      ┌──────────┐      ┌──────────┐
│  │  USB   │─┼──────┤  USB-A   │──────┤ Gamepad  │
│  │  Host  │ │      │  Port    │      │ (PS5)    │
│  └────────┘ │      └──────────┘      └──────────┘
│             │            │
│  ┌────────┐ │      ┌──────────┐
│  │  HID   │─┼──────┤  I/O     │──────┤ USB 5V Enable
│  │  Host  │ │      │ Expander │      │ (PI4IOE5V6408)
│  └────────┘ │      └──────────┘
└─────────────┘
```

### Key Components:

1. **USB Host Library (ESP-IDF):**
   - Manages USB bus
   - Handles device connection/disconnection
   - Manages port power

2. **HID Host Driver:**
   - Works on top of USB Host
   - Parses HID descriptors
   - Reads Input Reports from devices

3. **I/O Expander (PI4IOE5V6408):**
   - Controls USB-A port power
   - Enabled via `bsp_set_usb_5v_en(true)`

4. **BSP (Board Support Package):**
   - Hardware abstraction layer
   - Initializes I2C, I/O expanders
   - Provides USB Host management functions

---

## How USB Host Works

### Step 1: Initialization

```c
// 1. Initialize NVS (required for BSP)
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
}
ESP_ERROR_CHECK(ret);

// 2. Initialize I2C (for I/O expander)
ESP_ERROR_CHECK(bsp_i2c_init());

// 3. Initialize I/O expander (for USB-A power management)
bsp_io_expander_pi4ioe_init(bsp_i2c_get_handle());

// 4. Start USB Host via BSP
ESP_ERROR_CHECK(bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, true));
```

**What happens:**
- BSP initializes ESP32-P4 USB Host controller
- Configures USB bus for Host mode operation
- Prepares system for device connection

### Step 2: Enable USB-A Port Power

```c
// Enable power at hardware level (via I/O expander)
bsp_set_usb_5v_en(true);
vTaskDelay(pdMS_TO_TICKS(100)); // Give time for power stabilization

// Enable power at USB Host library level
ret = usb_host_lib_set_root_port_power(true);
if (ret == ESP_ERR_INVALID_STATE) {
    // Already enabled - this is normal
    ESP_LOGI(TAG, "USB Host root port power already enabled");
}
```

**Why two steps:**
- `bsp_set_usb_5v_en(true)` - enables **hardware power** via I/O expander (5V to USB-A port)
- `usb_host_lib_set_root_port_power(true)` - enables **logical power** at USB Host library level

**Important:** Without the first step, gamepad won't receive power (LED won't light up), without the second - USB Host won't detect the device.

### Step 3: Register USB Host Client

```c
// Client configuration
usb_host_client_config_t client_config = {
    .is_synchronous = false,           // Asynchronous mode
    .max_num_event_msg = 5,           // Maximum events in queue
    .async = {
        .client_event_callback = usb_host_event_callback,  // Callback for events
        .callback_arg = NULL
    }
};

// Register client
usb_host_client_handle_t client_handle;
ret = usb_host_client_register(&client_config, &client_handle);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register USB Host client");
    return;
}

// Create task for USB Host event handling
xTaskCreate(usb_host_client_task, "usb_host_client", 4096, NULL, 5, NULL);
```

**What is USB Host client:**
- This is a "subscriber" to USB Host library events
- Receives notifications about device connection/disconnection
- Requires separate task for event handling

**Why separate task is needed:**
- `usb_host_client_handle_events()` - blocking function
- Must be called periodically to process events
- If called in main loop, can block other tasks

### Step 4: USB Host Event Handler Task

```c
static void usb_host_client_task(void* arg)
{
    ESP_LOGI(TAG, "USB Host client task started");
    
    while (1) {
        if (client_handle) {
            // Blocking call - processes USB Host events
            usb_host_client_handle_events(client_handle, portMAX_DELAY);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
```

**What this task does:**
- Calls `usb_host_client_handle_events()` in infinite loop
- When event occurs (device connection/disconnection), `usb_host_event_callback` is called
- Blocks until event occurs (thanks to `portMAX_DELAY`)

### Step 5: Callback for USB Host Events

```c
static void usb_host_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(TAG, "New USB device connected on address %d", 
                     event_msg->new_dev.address);
            break;
            
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(TAG, "USB device disconnected");
            break;
            
        default:
            ESP_LOGI(TAG, "USB Host event: %d", event_msg->event);
            break;
    }
}
```

**When called:**
- On USB device connection
- On USB device disconnection
- On other USB Host events

**Important:** This callback is called from `usb_host_client_task` context, so it shouldn't block for long.

---

## How HID Host Works

HID (Human Interface Device) - standard USB protocol for input devices (keyboards, mice, gamepads).

### Step 1: Install HID Host Driver

```c
const hid_host_driver_config_t hid_host_driver_config = {
    .create_background_task = true,   // Create background task for driver
    .task_priority = 5,                // Task priority
    .stack_size = 4096,                // Stack size
    .core_id = 0,                      // CPU core
    .callback = hid_host_device_event, // Callback for device events
    .callback_arg = NULL
};

ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));
```

**What it does:**
- Installs HID Host driver on top of USB Host
- Creates background task for HID event handling
- Registers callback for HID device connection notifications

### Step 2: Callback for HID Device Events

```c
static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void* arg)
{
    switch (event) {
        case HID_HOST_DRIVER_EVENT_CONNECTED: {
            // Device connected - get information
            hid_host_dev_info_t dev_info;
            esp_err_t ret = hid_host_get_device_info(hid_device_handle, &dev_info);
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "HID Device CONNECTED");
                ESP_LOGI(TAG, "  VID: 0x%04X", dev_info.VID);
                ESP_LOGI(TAG, "  PID: 0x%04X", dev_info.PID);
                ESP_LOGI(TAG, "  Product: %ls", dev_info.iProduct);
                
                // Save VID/PID for report format detection
                s_gamepad_vid = dev_info.VID;
                s_gamepad_pid = dev_info.PID;
            }
            
            // Open device
            const hid_host_device_config_t dev_config = {
                .callback = hid_host_interface_callback,  // Callback for Input Reports
                .callback_arg = NULL
            };
            ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
            
            // Start device (begin reading Input Reports)
            ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
            
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unhandled HID driver event: %d", event);
            break;
    }
}
```

**What happens:**
1. On HID device connection, `hid_host_device_event` is called with `HID_HOST_DRIVER_EVENT_CONNECTED` event
2. Get device information (VID, PID, name)
3. Open device with Input Report callback specified
4. Start device - begin receiving Input Reports

### Step 3: Callback for Input Reports

```c
static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                        const hid_host_interface_event_t event,
                                        void* arg)
{
    switch (event) {
        case HID_HOST_INTERFACE_EVENT_INPUT_REPORT: {
            // Received Input Report from device
            uint8_t data[64] = {0};
            size_t data_length = 0;
            
            // Read raw report data
            esp_err_t ret = hid_host_device_get_raw_input_report_data(
                hid_device_handle, data, sizeof(data), &data_length);
            
            if (ret == ESP_OK && data_length > 0) {
                // Parse gamepad data
                parse_gamepad_report(data, data_length, &s_gamepad_state);
                
                // Process state changes
                // ...
            }
            break;
        }
        
        case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HID Device DISCONNECTED");
            ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
            break;
            
        default:
            ESP_LOGE(TAG, "Unhandled interface event: %d", event);
            break;
    }
}
```

**What happens:**
- When gamepad sends data (button press, stick movement), `hid_host_interface_callback` is called with `HID_HOST_INTERFACE_EVENT_INPUT_REPORT` event
- Read raw data via `hid_host_device_get_raw_input_report_data()`
- Parse data into gamepad state structure
- Process changes (log, update game state, etc.)

**Call frequency:**
- DualSense sends Input Reports at ~1000 Hz (every millisecond)
- Therefore it's important to filter small changes (noise) in data

---

## DualSense USB HID Report Structure

PlayStation 5 DualSense Controller uses specific USB HID report format.

### Input Report 0x01 Format

```
Byte 0:  Report ID (always 0x01)
Byte 1:  Left Stick X  (0x00..0xFF, ~0x80 neutral)
Byte 2:  Left Stick Y  (0x00..0xFF, ~0x80 neutral)
Byte 3:  Right Stick X (0x00..0xFF, ~0x80 neutral)
Byte 4:  Right Stick Y (0x00..0xFF, ~0x80 neutral)
Byte 5:  L2 Trigger (analog, 0x00..0xFF)
Byte 6:  R2 Trigger (analog, 0x00..0xFF)
Byte 7:  SeqNo (service counter, ignore)
Byte 8:  [3:0] D-Pad (0-7 directions, 8=center)
         [4]   Square
         [5]   Cross (A)
         [6]   Circle (B)
         [7]   Triangle (Y)
Byte 9:  [0] L1
         [1] R1
         [2] L2 (digital button)
         [3] R2 (digital button)
         [4] Create (Back)
         [5] Options (Start)
         [6] L3
         [7] R3
Byte 10: [0] PS (Home)
         [1] Touchpad click
         [2] Mute
         [3..7] Reserved
Byte 11+: Additional data (sensors, gyroscope, etc.)
```

### Important Points:

1. **Report ID:**
   - Always `0x01` for Input Report
   - Some libraries may "consume" it, so check `data[0] == 0x01`

2. **Sticks:**
   - Values from `0x00` to `0xFF`
   - Neutral position: `~0x80` (128)
   - Convert to `-127..+127`: `value - 128`

3. **Triggers:**
   - Analog values from `0x00` (released) to `0xFF` (fully pressed)
   - Also digital L2/R2 buttons in byte 9
   - Use analog values for smooth control

4. **D-Pad:**
   - Values `0-7` correspond to directions
   - `8` or `0xF` = center (not pressed)
   - Mapping: `0=UP, 1=UP-RIGHT, 2=RIGHT, 3=DOWN-RIGHT, 4=DOWN, 5=DOWN-LEFT, 6=LEFT, 7=UP-LEFT`

5. **Buttons:**
   - Distributed across multiple bytes
   - Each bit = one button (1=pressed, 0=released)

---

## Gamepad Data Parsing

### Gamepad State Structure

```c
typedef struct {
    // Buttons (bitmap)
    uint16_t buttons;
    
    // D-Pad (0-8, where 0=center, 1=up, 2=up-right, etc.)
    uint8_t dpad;
    
    // Left stick (-127 to +127)
    int8_t left_x;
    int8_t left_y;
    
    // Right stick (-127 to +127)
    int8_t right_x;
    int8_t right_y;
    
    // Triggers (0-255)
    uint8_t left_trigger;
    uint8_t right_trigger;
    
    // Raw report data (for debugging)
    uint8_t raw_report[64];
    size_t raw_report_len;
} gamepad_state_t;
```

### DualSense Parser

```c
static void parse_gamepad_report(const uint8_t* data, size_t len, gamepad_state_t* state)
{
    // Check if this is DualSense
    if (s_gamepad_vid == 0x054C && s_gamepad_pid == 0x0CE6) {
        if (len < 11) {
            return; // Report too short
        }
        
        // Check Report ID
        int base = 0;
        if (data[0] == 0x01) {
            base = 1;  // data[0] = ReportID, USBGetStateData follows
        } else {
            return; // Not Input Report 0x01
        }
        
        // Sticks: 0..255 -> -127..127
        state->left_x   = (int16_t)((int)data[base + 0] - 128);
        state->left_y   = (int16_t)((int)data[base + 1] - 128);
        state->right_x  = (int16_t)((int)data[base + 2] - 128);
        state->right_y  = (int16_t)((int)data[base + 3] - 128);
        
        // Analog triggers (strictly from base+4 and base+5!)
        state->left_trigger  = data[base + 4];  // L2 axis
        state->right_trigger = data[base + 5];  // R2 axis
        
        // D-Pad + face buttons (byte base+7)
        uint8_t b7  = data[base + 7];
        uint8_t hat = b7 & 0x0F;
        
        // DualSense hat: 0=UP,1=UR,2=R,3=DR,4=D,5=DL,6=L,7=UL,8=CENTER
        if (hat <= 7) {
            state->dpad = hat + 1;  // Format: 0=CENTER, 1=UP, ...
        } else {
            state->dpad = 0;        // CENTER
        }
        
        // Reset button bits
        state->buttons = 0;
        
        // Face buttons (Square / Cross / Circle / Triangle) from byte base+7
        if (b7 & 0x10) state->buttons |= (1 << 0);  // Square
        if (b7 & 0x20) state->buttons |= (1 << 1);  // Cross
        if (b7 & 0x40) state->buttons |= (1 << 2);  // Circle
        if (b7 & 0x80) state->buttons |= (1 << 3);  // Triangle
        
        // L1/R1/L2/R2/Create/Options/L3/R3 (byte base+8)
        uint8_t b8 = data[base + 8];
        if (b8 & 0x01) state->buttons |= (1 << 4);   // L1
        if (b8 & 0x02) state->buttons |= (1 << 5);   // R1
        if (b8 & 0x04) state->buttons |= (1 << 11);  // L2 digital
        if (b8 & 0x08) state->buttons |= (1 << 12);  // R2 digital
        if (b8 & 0x10) state->buttons |= (1 << 6);   // Create (Select)
        if (b8 & 0x20) state->buttons |= (1 << 7);   // Options (Start)
        if (b8 & 0x40) state->buttons |= (1 << 8);   // L3
        if (b8 & 0x80) state->buttons |= (1 << 9);   // R3
        
        // PS/Pad/Mute (byte base+9)
        uint8_t b9 = data[base + 9];
        if (b9 & 0x01) state->buttons |= (1 << 10); // PS (Home)
        
        // Additionally: digital L2/R2 from analog values
        if (state->left_trigger  > 30) state->buttons |= (1 << 11);
        if (state->right_trigger > 30) state->buttons |= (1 << 12);
        
        // Save raw data
        memcpy(state->raw_report, data, len < 64 ? len : 64);
        state->raw_report_len = len;
        
        return;
    }
    
    // For other gamepads - generic parser
    // ...
}
```

### Noise Filtering

DualSense sends reports very frequently (~1000 Hz), so it's important to filter small changes:

```c
// Threshold for triggers (ignore changes less than 10 units)
const uint8_t TRIGGER_THRESHOLD = 10;
bool triggers_changed = 
    (abs((int)state->left_trigger - (int)s_last_left_trigger) > TRIGGER_THRESHOLD) ||
    (abs((int)state->right_trigger - (int)s_last_right_trigger) > TRIGGER_THRESHOLD);

// Threshold for sticks (ignore changes less than 5 units)
bool sticks_changed = 
    (abs(state->left_x - s_last_left_x) > 5) ||
    (abs(state->left_y - s_last_left_y) > 5) ||
    (abs(state->right_x - s_last_right_x) > 5) ||
    (abs(state->right_y - s_last_right_y) > 5);

// Process only significant changes
if (button_changes || dpad_changed || triggers_changed || sticks_changed) {
    // Update game state
    // ...
}
```

---

## Complete Code Example

See file: `main/app_main.cpp` in this project

**Key points:**
1. Initialize NVS, I2C, I/O expander
2. Start USB Host via BSP
3. Enable USB-A port power
4. Register USB Host client
5. Install HID Host driver
6. Handle connection/disconnection events
7. Parse Input Reports
8. Filter noise

---

## Common Errors and Solutions

### Error 1: Gamepad doesn't receive power (LED doesn't light up)

**Symptoms:**
- Gamepad connected, but LED doesn't light up
- USB Host doesn't detect device

**Cause:**
- Hardware power not enabled via I/O expander

**Solution:**
```c
// IMPORTANT: Enable power BEFORE connecting gamepad
bsp_set_usb_5v_en(true);
vTaskDelay(pdMS_TO_TICKS(100)); // Give time for stabilization
```

### Error 2: Gamepad connected, but not sending data

**Symptoms:**
- Gamepad detected (VID/PID visible in logs)
- But Input Reports don't arrive

**Cause:**
- Device not started via `hid_host_device_start()`

**Solution:**
```c
// After opening device MUST start it
ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));  // ← IMPORTANT!
```

### Error 3: Incorrect trigger parsing (R2 shows 172 instead of 0)

**Symptoms:**
- Triggers show strange values (172, 173, etc.)
- Values don't change when pressed

**Cause:**
- Reading wrong byte (e.g., `data[7]` instead of `data[6]`)

**Solution:**
```c
// CORRECT: triggers in base+4 and base+5
state->left_trigger  = data[base + 4];  // L2
state->right_trigger = data[base + 5];  // R2

// INCORRECT: data[7] = SeqNo (service counter)
```

### Error 4: Constant log spam (logs without stopping)

**Symptoms:**
- Logs print constantly, even when nothing is pressed

**Cause:**
- Not filtering small changes (noise)
- Logging every report instead of changes

**Solution:**
```c
// Filter small changes
const uint8_t TRIGGER_THRESHOLD = 10;
bool triggers_changed = 
    (abs((int)state->left_trigger - (int)s_last_left_trigger) > TRIGGER_THRESHOLD);

// Log only on changes
if (button_changes || dpad_changed || triggers_changed) {
    ESP_LOGI(TAG, "State changed");
}
```

### Error 5: "Control Transfer Timeout" in logs

**Symptoms:**
- Warnings about timeout appear in logs

**Cause:**
- Normal behavior for some devices
- Some devices don't support all USB commands

**Solution:**
- Ignore these warnings if device works
- They don't affect HID Input Reports functionality

### Error 6: "Report Descriptor not available"

**Symptoms:**
- Warning about Report Descriptor unavailability appears in logs

**Cause:**
- Normal behavior - device works without descriptor
- Report Descriptor only needed for generic parser

**Solution:**
- Ignore warning if using specialized parser (e.g., for DualSense)
- DualSense parser doesn't require Report Descriptor

---

## Troubleshooting

### Step 1: Power Check

```c
// In logs should be:
ESP_LOGI(TAG, "Enabling USB-A port power (hardware)...");
bsp_set_usb_5v_en(true);
ESP_LOGI(TAG, "USB Host root port power enabled successfully");
```

**If not:**
- Check I2C and I/O expander initialization
- Check that you call `bsp_set_usb_5v_en(true)` BEFORE connecting gamepad

### Step 2: Device Connection Check

```c
// In logs should be:
ESP_LOGI(TAG, ">>> USB Host: New device connected on address 1");
ESP_LOGI(TAG, "HID Device CONNECTED");
ESP_LOGI(TAG, "  VID: 0x054C");
ESP_LOGI(TAG, "  PID: 0x0CE6");
```

**If not:**
- Check that USB Host client is registered
- Check that `usb_host_client_task` is running
- Check that HID Host driver is installed

### Step 3: Input Reports Check

```c
// In logs should be (first 5 reports):
ESP_LOGI(TAG, "=== DEBUG REPORT #0 ===");
ESP_LOGI(TAG, "RAW[0..10]: 01 81 83 84 81 00 00 AC 08 00 00");
ESP_LOGI(TAG, "INTERP: base=1, L2_raw=data[5]=00 (0), R2_raw=data[6]=00 (0)");
ESP_LOGI(TAG, "PARSED: L2=0 R2=0, buttons=0x0000 dpad=0 L(0,0) R(0,0)");
```

**If not:**
- Check that you call `hid_host_device_start()` after opening device
- Check that `hid_host_interface_callback` callback is registered

### Step 4: Parsing Check

```c
// Press button and check logs:
ESP_LOGI(TAG, "Button A (bit 1): PRESSED");
ESP_LOGI(TAG, "Buttons: A");
ESP_LOGI(TAG, "D-Pad: CENTER");
```

**If buttons not recognized:**
- Check byte correctness in parser
- Use debug logging for first reports
- Compare raw data with expected format

---

## Reference Information

### sdkconfig.defaults Configuration

```ini
# USB Host configuration
CONFIG_USB_HOST_ENABLE=y
CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=256
CONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED=y
CONFIG_USB_HOST_DEBOUNCE_DELAY_MS=250
```

### Component Dependencies

```cmake
idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES m5stack_tab5 usb_host_hid nvs_flash
)
```

### idf_component.yml

```yaml
dependencies:
  idf: '>=5.4'
  espressif/usb_host_hid: ^1.0.4
```

### Initialization Order (critically important!)

1. `nvs_flash_init()` - NVS initialization
2. `bsp_i2c_init()` - I2C initialization
3. `bsp_io_expander_pi4ioe_init()` - I/O expander initialization
4. `bsp_usb_host_start()` - USB Host start
5. `bsp_set_usb_5v_en(true)` - Enable USB-A port power
6. `usb_host_lib_set_root_port_power(true)` - Enable power at library level
7. `usb_host_client_register()` - Register USB Host client
8. `xTaskCreate(usb_host_client_task)` - Create task for event handling
9. `hid_host_install()` - Install HID Host driver

**Important:** Order matters! Don't change the sequence.

---

## Useful Links

- [ESP-IDF USB Host Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-guides/usb-host.html)
- [ESP-IDF HID Host Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-guides/usb-host-hid.html)
- [DualSense USB HID Specification](https://www.psdevwiki.com/ps5/DualSense)
- [USB HID Specification](https://www.usb.org/sites/default/files/documents/hid1_11.pdf)

---

**Last Updated:** December 6, 2025  
**Author:** Andy (with AI assistance)  
**Project:** NES Emulator for M5Stack Tab5 with USB Host Support

