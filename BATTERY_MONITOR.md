# Battery Monitor Documentation

## Overview

The Tab5 NES Emulator includes a battery monitoring system that displays battery level, voltage, and charging status on the screen. The system uses the INA226 power monitor IC to measure battery voltage and current.

## Hardware

### INA226 Power Monitor

- **I2C Address**: `0x41` (7-bit)
- **Shunt Resistor**: `5 mOhm` (0.005 Ω)
- **Max Current**: `8.192 A`
- **Battery Type**: 2S Li-Po (7.4V nominal, 6.0V empty, 8.4V full)

### Battery Specifications

- **Type**: 2S Li-Po (2 cells in series)
- **Voltage Range**: 6.0V (empty) to 8.4V (full)
- **Per Cell**: 3.0V (empty) to 4.2V (full)
- **Capacity**: Varies by battery pack

## Software Components

### Battery Monitor Component

Located in: `components/battery_monitor/`

**Files:**
- `include/battery_monitor.h` - Header file with API definitions
- `src/battery_monitor.c` - Implementation

### API Functions

#### Initialization

```c
esp_err_t battery_monitor_init(i2c_master_bus_handle_t i2c_bus_handle);
```

Initializes the INA226 power monitor IC. Must be called before using other functions.

**Parameters:**
- `i2c_bus_handle` - I2C bus handle from `bsp_i2c_get_handle()`

**Returns:**
- `ESP_OK` on success
- Error code on failure

**What it does:**
1. Adds INA226 device to I2C bus (address 0x41)
2. Configures INA226 for continuous measurement mode
3. Calibrates INA226 with shunt resistor value (5 mOhm)
4. Verifies calibration register was written correctly
5. Performs test reads of voltage, shunt voltage, and current

#### Read Battery Status

```c
esp_err_t battery_monitor_read(battery_status_t *status);
```

Reads current battery status including level, voltage, and charging state.

**Parameters:**
- `status` - Pointer to `battery_status_t` structure to fill

**Returns:**
- `ESP_OK` on success
- Error code on failure

**Structure:**
```c
typedef struct {
    int level;          // Battery level 0-100%
    int voltage_mv;     // Battery voltage in millivolts
    bool is_charging;   // True if charging (determined by USB-C detection)
    bool initialized;   // True if battery monitor initialized
} battery_status_t;
```

#### Convenience Functions

```c
int battery_monitor_get_level(void);
```
Returns battery level (0-100%) or -1 on error.

```c
int battery_monitor_get_voltage_mv(void);
```
Returns battery voltage in millivolts or -1 on error.

```c
bool battery_monitor_is_charging(void);
```
Returns true if battery is charging, false otherwise.

## Charging Status Detection

The charging status is determined by checking USB-C connection using `bsp_usb_c_detect()` function from M5Stack Tab5 BSP.

**Logic:**
- When USB-C cable is connected → Battery is charging
- When USB-C cable is disconnected → Battery is discharging

**Note:** The charging status is determined in `nes_osd.c` during display rendering, not in `battery_monitor.c`. This allows the system to work even if INA226 current reading fails.

## Display Integration

The battery indicator is drawn in `components/nes_emulator/src/nes_osd.c` in the `nes_display_draw_battery_indicator()` function.

**Display Features:**
- Battery icon (80×40 pixels) in top-right corner
- Battery level percentage below icon
- "CHARGING" text (red) when USB-C is connected
- Color coding:
  - Green: >60% charge
  - Yellow: 20-60% charge
  - Red: <20% charge

**Update Frequency:**
- Battery status is read once per second
- Display is updated every frame (60 FPS)

## INA226 Calibration

The INA226 is calibrated during initialization using the following formula:

```
Current_LSB = ceil((I_max / 32767) * 100000000) / 100000000
Current_LSB = ceil(Current_LSB / 0.0001) * 0.0001

Calibration = 0.00512 / (Current_LSB × R_shunt)
```

**Parameters:**
- `I_max` = 8.192 A (maximum expected current)
- `R_shunt` = 0.005 Ω (5 mOhm shunt resistor)
- `Current_LSB` = Calculated precision (typically ~0.0001 A)

**Verification:**
After writing the calibration register, the code reads it back and verifies the value matches. This ensures the calibration was written correctly.

## Current Reading

The current is read from INA226 Current Register (0x04) and converted using:

```
Current = Current_Register_Raw × Current_LSB
```

**Current Register:**
- Signed 16-bit value
- Negative values = Current flowing INTO battery (charging)
- Positive values = Current flowing OUT of battery (discharging)

**Note:** Current reading requires proper calibration. If calibration fails, current will always read as 0.000A.

## Voltage Reading

The bus voltage is read from INA226 Bus Voltage Register (0x02) and converted using:

```
Voltage = Bus_Voltage_Register × 0.00125 V
```

**Bus Voltage Register:**
- 16-bit value (MSB always 0, voltage is always positive)
- LSB = 1.25 mV
- Full scale = 40.96 V

## Battery Level Calculation

Battery level is calculated from voltage using linear interpolation:

```
Single_Cell_Voltage = Bus_Voltage / 2  (for 2S battery)

If Single_Cell_Voltage < 3.0V:
    Level = 0%
Else if Single_Cell_Voltage > 4.2V:
    Level = 100%
Else:
    Level = ((Single_Cell_Voltage - 3.0) / (4.2 - 3.0)) × 100
```

**Voltage Thresholds:**
- Empty: 3.0V per cell (6.0V total for 2S)
- Full: 4.2V per cell (8.4V total for 2S)
- Linear interpolation between these values

## Troubleshooting

### Current Always Reads 0.000A

**Possible Causes:**
1. Calibration register not written correctly
2. I2C communication error
3. Shunt resistor not connected properly
4. No current flowing (battery idle)

**Debug Steps:**
1. Check logs for calibration register verification
2. Check logs for I2C errors
3. Verify shunt voltage reading (should be non-zero if current flows)
4. Check INA226 I2C address (should be 0x41)

### Charging Status Not Updating

**Possible Causes:**
1. USB-C detection not working
2. USB-C cable not properly connected
3. Device connected to computer (USB-C detection may always return true)

**Debug Steps:**
1. Check logs for "USB-C detect: CONNECTED/DISCONNECTED"
2. Disconnect device from computer and test
3. Verify `bsp_usb_c_detect()` function works correctly

### Battery Level Always Shows 0% or 100%

**Possible Causes:**
1. Voltage reading incorrect
2. Battery voltage outside normal range
3. INA226 not initialized

**Debug Steps:**
1. Check logs for bus voltage reading
2. Verify voltage is in range 6.0V - 8.4V
3. Check if battery monitor initialized successfully

## Logging

The battery monitor logs important information:

**During Initialization:**
```
BATTERY_MONITOR: Calibrating INA226:
BATTERY_MONITOR:   R_shunt: 0.005 ohm (5mOhm)
BATTERY_MONITOR:   I_max: 8.192A
BATTERY_MONITOR:   Current_LSB: 0.000XXX A
BATTERY_MONITOR:   Calibration value: 0xXXXX (XXXX)
BATTERY_MONITOR: Calibration register verified: 0xXXXX
BATTERY_MONITOR: Bus voltage: X.XXXV
BATTERY_MONITOR: Shunt voltage: X.XXXXXXV (raw: XXXX)
BATTERY_MONITOR: Test current read: X.XXXXXXA
```

**During Operation (every 60 reads, ~1 minute):**
```
BATTERY_MONITOR: Current: X.XXXXXXA (raw: XXXX), Shunt: X.XXXXXXV (raw: XXXX)
```

**Display Status (every 5 seconds):**
```
NES_OSD: USB-C detect: CONNECTED/DISCONNECTED, Battery: XXXXmV (XX%)
```

## Example Usage

```c
#include "battery_monitor.h"
#include "bsp/m5stack_tab5.h"

void app_main(void)
{
    // Initialize I2C bus
    i2c_master_bus_handle_t i2c_bus = bsp_i2c_get_handle();
    
    // Initialize battery monitor
    esp_err_t ret = battery_monitor_init(i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE("APP", "Failed to initialize battery monitor");
        return;
    }
    
    // Read battery status
    battery_status_t status;
    ret = battery_monitor_read(&status);
    if (ret == ESP_OK) {
        ESP_LOGI("APP", "Battery: %d%%, %dmV, Charging: %s",
                 status.level, status.voltage_mv,
                 status.is_charging ? "YES" : "NO");
    }
    
    // Or use convenience functions
    int level = battery_monitor_get_level();
    int voltage = battery_monitor_get_voltage_mv();
    bool charging = battery_monitor_is_charging();
    
    ESP_LOGI("APP", "Level: %d%%, Voltage: %dmV, Charging: %s",
             level, voltage, charging ? "YES" : "NO");
}
```

## Technical Details

### INA226 Registers Used

- **0x00** - Configuration Register (R/W)
- **0x01** - Shunt Voltage Register (R)
- **0x02** - Bus Voltage Register (R)
- **0x04** - Current Register (R)
- **0x05** - Calibration Register (R/W)

### I2C Communication

- **Speed**: 400 kHz
- **Address**: 0x41 (7-bit)
- **Timeout**: 50 ms
- **Protocol**: Standard I2C read/write

### Measurement Timing

- **Conversion Time**: 1.1 ms (for both shunt and bus voltage)
- **Averaging**: 16 samples
- **Mode**: Continuous shunt and bus voltage measurement
- **Update Rate**: ~70 Hz (1.1 ms × 16 samples = ~17.6 ms per measurement)

## References

- [INA226 Datasheet](https://www.ti.com/lit/ds/symlink/ina226.pdf)
- [INA226 Arduino Library](https://github.com/RobTillaart/INA226)
- M5Stack Tab5 BSP Documentation

## Version History

- **v1.0** (2026-01-01): Initial implementation
  - INA226 initialization and calibration
  - Battery voltage and level reading
  - USB-C charging detection
  - Display integration with battery indicator

