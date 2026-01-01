/*
 * Battery Monitor for M5Stack Tab5
 * Uses INA226 power monitor IC
 */

#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// INA226 I2C address for Tab5
#define INA226_ADDRESS 0x41

// Battery status structure
typedef struct {
    int level;          // Battery level 0-100%
    int voltage_mv;     // Battery voltage in millivolts
    bool is_charging;   // True if charging
    bool initialized;   // True if battery monitor initialized
} battery_status_t;

/**
 * @brief Initialize battery monitor (INA226)
 * 
 * @param i2c_bus_handle I2C bus handle (from bsp_i2c_get_handle())
 * @return ESP_OK on success
 */
esp_err_t battery_monitor_init(i2c_master_bus_handle_t i2c_bus_handle);

/**
 * @brief Read current battery status
 * 
 * @param status Pointer to battery_status_t structure to fill
 * @return ESP_OK on success
 */
esp_err_t battery_monitor_read(battery_status_t *status);

/**
 * @brief Get battery level (0-100%)
 * 
 * @return Battery level, or -1 on error
 */
int battery_monitor_get_level(void);

/**
 * @brief Get battery voltage in millivolts
 * 
 * @return Voltage in mV, or -1 on error
 */
int battery_monitor_get_voltage_mv(void);

/**
 * @brief Check if battery is charging
 * 
 * @return True if charging, false otherwise
 */
bool battery_monitor_is_charging(void);

#ifdef __cplusplus
}
#endif

