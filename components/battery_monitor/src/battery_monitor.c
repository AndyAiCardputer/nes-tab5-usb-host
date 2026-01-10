/*
 * Battery Monitor for M5Stack Tab5
 * Uses INA226 power monitor IC
 */

#include "battery_monitor.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "BATTERY_MONITOR";

// INA226 registers
#define INA226_REG_CONFIG      0x00
#define INA226_REG_SHUNTVOLTAGE 0x01
#define INA226_REG_BUSVOLTAGE   0x02
#define INA226_REG_POWER        0x03
#define INA226_REG_CURRENT      0x04
#define INA226_REG_CALIBRATION  0x05
#define INA226_REG_MASKENABLE   0x06
#define INA226_REG_ALERTLIMIT   0x07

// INA226 configuration values
#define INA226_AVERAGES_16      0b010
#define INA226_BUS_CONV_TIME_1100US  0b100
#define INA226_SHUNT_CONV_TIME_1100US 0b100
#define INA226_MODE_SHUNT_BUS_CONT   0b111

#define I2C_MASTER_TIMEOUT_MS 50

static i2c_master_dev_handle_t ina226_dev_handle = NULL;
static bool initialized = false;
static float current_lsb = 0.0f;  // Current LSB for reading current from INA226

// Helper functions
static int16_t read_register16(uint8_t reg)
{
    if (!ina226_dev_handle) {
        return -1;
    }
    
    uint8_t r_buffer[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(ina226_dev_handle, &reg, 1, r_buffer, 2, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
        return -1;
    }
    
    return (r_buffer[0] << 8) | r_buffer[1];
}

static esp_err_t write_register16(uint8_t reg, uint16_t val)
{
    if (!ina226_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t w_buffer[3] = {0};
    w_buffer[0] = reg;
    w_buffer[1] = (val >> 8) & 0xFF;
    w_buffer[2] = val & 0xFF;
    
    esp_err_t ret = i2c_master_transmit(ina226_dev_handle, w_buffer, 3, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

static float read_bus_voltage(void)
{
    int16_t voltage = read_register16(INA226_REG_BUSVOLTAGE);
    if (voltage < 0) {
        return -1.0f;
    }
    // INA226 bus voltage LSB = 1.25mV
    return voltage * 0.00125f;
}

static float read_current(void)
{
    if (current_lsb == 0.0f) {
        ESP_LOGW(TAG, "Current LSB not set, cannot read current");
        return 0.0f;  // Not calibrated yet
    }
    
    // First, read shunt voltage to verify I2C communication works
    int16_t shunt_raw = read_register16(INA226_REG_SHUNTVOLTAGE);
    if (shunt_raw < 0) {
        ESP_LOGW(TAG, "Failed to read shunt voltage register (I2C error)");
        return 0.0f;
    }
    
    // Now read current register
    int16_t current_raw = read_register16(INA226_REG_CURRENT);
    if (current_raw < 0) {
        ESP_LOGW(TAG, "Failed to read current register (I2C error)");
        return 0.0f;
    }
    
    // Convert signed 16-bit value to current in Amperes
    // INA226 Current Register formula: Current = (Shunt Voltage × Calibration) / 2048
    // Negative value = current flowing into battery (charging)
    // Positive value = current flowing out of battery (discharging)
    float current = (int16_t)current_raw * current_lsb;
    
    // Log occasionally for debugging
    static int log_count = 0;
    if ((log_count++ % 60) == 0) {
        float shunt_voltage = (int16_t)shunt_raw * 0.0000025f;  // 2.5µV LSB
        ESP_LOGI(TAG, "Current: %.6fA (raw: %d), Shunt: %.6fV (raw: %d)", 
                 current, current_raw, shunt_voltage, shunt_raw);
    }
    
    return current;
}

esp_err_t battery_monitor_init(i2c_master_bus_handle_t i2c_bus_handle)
{
    if (initialized) {
        ESP_LOGW(TAG, "Battery monitor already initialized");
        return ESP_OK;
    }
    
    if (!i2c_bus_handle) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Add INA226 device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA226_ADDRESS,
        .scl_speed_hz = 400000,
    };
    
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &ina226_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add INA226 device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure INA226
    uint16_t config = 0;
    config |= (INA226_AVERAGES_16 << 9);
    config |= (INA226_BUS_CONV_TIME_1100US << 6);
    config |= (INA226_SHUNT_CONV_TIME_1100US << 3);
    config |= INA226_MODE_SHUNT_BUS_CONT;
    
    ret = write_register16(INA226_REG_CONFIG, config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INA226");
        return ret;
    }
    
    // Calibrate INA226
    // Tab5 uses 2S Li-Po battery, shunt resistor = 5mOhm, max current = 8.192A
    float r_shunt = 0.005f;  // 5mOhm
    float i_max_expected = 8.192f;  // 8.192A
    
    float minimum_lsb = i_max_expected / 32767.0f;
    current_lsb = ceil(minimum_lsb * 100000000.0f) / 100000000.0f;
    current_lsb = ceil(current_lsb / 0.0001f) * 0.0001f;
    
    uint16_t calibration_value = (uint16_t)(0.00512f / (current_lsb * r_shunt));
    
    ESP_LOGI(TAG, "Calibrating INA226:");
    ESP_LOGI(TAG, "  R_shunt: %.3f ohm (5mOhm)", r_shunt);
    ESP_LOGI(TAG, "  I_max: %.3fA", i_max_expected);
    ESP_LOGI(TAG, "  Current_LSB: %.6fA", current_lsb);
    ESP_LOGI(TAG, "  Calibration value: 0x%04X (%d)", calibration_value, calibration_value);
    
    ret = write_register16(INA226_REG_CALIBRATION, calibration_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write calibration register");
        return ret;
    }
    
    // Wait for calibration to take effect (INA226 needs time to process)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Verify calibration register was written correctly
    int16_t cal_readback = read_register16(INA226_REG_CALIBRATION);
    if (cal_readback < 0) {
        ESP_LOGE(TAG, "Failed to read back calibration register");
        return ESP_FAIL;
    }
    
    if ((uint16_t)cal_readback != calibration_value) {
        ESP_LOGW(TAG, "Calibration register mismatch: wrote 0x%04X, read 0x%04X", 
                 calibration_value, (uint16_t)cal_readback);
    } else {
        ESP_LOGI(TAG, "Calibration register verified: 0x%04X", calibration_value);
    }
    
    initialized = true;
    
    // Test read voltage
    float test_voltage = read_bus_voltage();
    if (test_voltage > 0) {
        ESP_LOGI(TAG, "Bus voltage: %.3fV", test_voltage);
    } else {
        ESP_LOGW(TAG, "Bus voltage read failed");
    }
    
    // Test read shunt voltage (for debugging)
    int16_t shunt_raw = read_register16(INA226_REG_SHUNTVOLTAGE);
    if (shunt_raw >= 0) {
        float shunt_voltage = (int16_t)shunt_raw * 0.0000025f;  // 2.5µV LSB
        ESP_LOGI(TAG, "Shunt voltage: %.6fV (raw: %d)", shunt_voltage, shunt_raw);
    } else {
        ESP_LOGW(TAG, "Shunt voltage read failed");
    }
    
    // Test read current (should work now)
    float test_current = read_current();
    ESP_LOGI(TAG, "Test current read: %.6fA", test_current);
    
    return ESP_OK;
}

esp_err_t battery_monitor_read(battery_status_t *status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!initialized) {
        status->initialized = false;
        status->level = -1;
        status->voltage_mv = -1;
        status->is_charging = false;
        return ESP_ERR_INVALID_STATE;
    }
    
    status->initialized = true;
    
    // Read bus voltage (this is the battery pack voltage for 2S Li-Po)
    float bus_voltage = read_bus_voltage();
    if (bus_voltage < 0) {
        status->level = -1;
        status->voltage_mv = -1;
        status->is_charging = false;
        return ESP_FAIL;
    }
    
    // bus_voltage is already in volts (e.g., 7.602V for 2S battery)
    // Convert to mV: multiply by 1000
    status->voltage_mv = (int)(bus_voltage * 1000.0f);
    
    // For 2S Li-Po: divide by 2 to get single cell voltage
    float single_cell_mv = status->voltage_mv / 2.0f;
    
    // Calculate battery level based on voltage
    // 2S Li-Po: 6.0V (empty) to 8.4V (full) = 3.0V to 4.2V per cell
    
    if (single_cell_mv < 3000) {
        status->level = 0;
    } else if (single_cell_mv > 4200) {
        status->level = 100;
    } else {
        // Linear interpolation: (voltage - 3000) / (4200 - 3000) * 100
        status->level = (int)((single_cell_mv - 3000.0f) * 100.0f / 1200.0f);
        if (status->level < 0) status->level = 0;
        if (status->level > 100) status->level = 100;
    }
    
    // Charging status will be determined by USB-C detection in nes_osd.c
    // For now, set to false (will be overridden in display function)
    status->is_charging = false;
    
    return ESP_OK;
}

int battery_monitor_get_level(void)
{
    battery_status_t status = {0};
    if (battery_monitor_read(&status) == ESP_OK) {
        return status.level;
    }
    return -1;
}

int battery_monitor_get_voltage_mv(void)
{
    battery_status_t status = {0};
    if (battery_monitor_read(&status) == ESP_OK) {
        return status.voltage_mv;
    }
    return -1;
}

bool battery_monitor_is_charging(void)
{
    battery_status_t status = {0};
    if (battery_monitor_read(&status) == ESP_OK) {
        return status.is_charging;
    }
    return false;
}

