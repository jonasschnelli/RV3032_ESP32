/**
 * @file RV3032.h
 * @brief ESP-IDF 5.5 library for RV-3032-C7 Real-Time Clock Module
 * @author Jonas Schnelli
 * @version 1.0
 * 
 * This library provides basic functionality for the RV-3032-C7 RTC:
 * - Time/Date reading and setting
 * - Alarm functionality
 * - Optimized for ESP32 platform with ESP-IDF 5.5
 * 
 * Based on RV-3032-C7 Application Manual Rev. 1.3 (May 2023)
 */

#ifndef RV3032_H
#define RV3032_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// RV3032 I2C Address
#define RV3032_I2C_ADDR         0x51    // 7-bit address (1010001b)

// Register Addresses - Time/Date
#define RV3032_REG_HUNDREDTHS   0x00    // 100th seconds (read-only)
#define RV3032_REG_SECONDS      0x01    // Seconds (00-59)
#define RV3032_REG_MINUTES      0x02    // Minutes (00-59)
#define RV3032_REG_HOURS        0x03    // Hours (00-23)
#define RV3032_REG_WEEKDAY      0x04    // Weekday (0-6)
#define RV3032_REG_DATE         0x05    // Date (01-31)
#define RV3032_REG_MONTH        0x06    // Month (01-12)
#define RV3032_REG_YEAR         0x07    // Year (00-99)

// Register Addresses - Alarm
#define RV3032_REG_MINUTES_ALARM 0x08   // Minutes alarm + enable bit
#define RV3032_REG_HOURS_ALARM   0x09   // Hours alarm + enable bit
#define RV3032_REG_DATE_ALARM    0x0A   // Date alarm + enable bit

// Register Addresses - Control/Status
#define RV3032_REG_STATUS        0x0D   // Status register
#define RV3032_REG_CONTROL2      0x11   // Control 2 register

// Alarm Enable Bits (bit 7 of each alarm register)
#define RV3032_ALARM_ENABLE_BIT  0x00   // 0 = enabled, 1 = disabled
#define RV3032_ALARM_DISABLE_BIT 0x80

// Status Register Bits
#define RV3032_STATUS_AF         0x08   // Alarm Flag (bit 3)

// Control 2 Register Bits
#define RV3032_CTRL2_AIE         0x08   // Alarm Interrupt Enable (bit 3)

// Default I2C timeout
#define RV3032_I2C_TIMEOUT_MS    1000

/**
 * @brief Structure to hold time/date information
 */
typedef struct {
    uint8_t year;       // 0-99 (represents 2000-2099)
    uint8_t month;      // 1-12
    uint8_t date;       // 1-31
    uint8_t weekday;    // 0-6 (user defined)
    uint8_t hours;      // 0-23
    uint8_t minutes;    // 0-59
    uint8_t seconds;    // 0-59
    uint8_t hundredths; // 0-99 (read-only)
} rv3032_datetime_t;

/**
 * @brief Structure to hold alarm information
 */
typedef struct {
    uint8_t date;       // 1-31 (0 = disabled)
    uint8_t hours;      // 0-23 (255 = disabled)
    uint8_t minutes;    // 0-59 (255 = disabled)
    bool date_enable;   // Enable date matching
    bool hours_enable;  // Enable hours matching
    bool minutes_enable; // Enable minutes matching
} rv3032_alarm_t;

/**
 * @brief RV3032 configuration structure
 */
typedef struct {
    i2c_master_bus_handle_t i2c_bus_handle;    // I2C bus handle
    i2c_master_dev_handle_t i2c_dev_handle;    // I2C device handle
    gpio_num_t int_pin;                        // Interrupt pin (GPIO_NUM_NC if not used)
    void (*alarm_callback)(void);              // Alarm callback function
} rv3032_handle_t;

/**
 * @brief Initialize the RV3032 RTC
 * @param handle Pointer to RV3032 handle structure
 * @param i2c_bus_handle I2C master bus handle
 * @param int_pin GPIO pin for interrupt (GPIO_NUM_NC if not used)
 * @param alarm_callback Callback function for alarm interrupt (NULL if not used)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_init(rv3032_handle_t* handle, 
                      i2c_master_bus_handle_t i2c_bus_handle,
                      gpio_num_t int_pin,
                      void (*alarm_callback)(void));

/**
 * @brief Deinitialize the RV3032 RTC
 * @param handle Pointer to RV3032 handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_deinit(rv3032_handle_t* handle);

// === Time/Date Functions ===

/**
 * @brief Read current date and time
 * @param handle Pointer to RV3032 handle structure
 * @param dt Pointer to datetime structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_get_datetime(rv3032_handle_t* handle, rv3032_datetime_t* dt);

/**
 * @brief Set date and time
 * @param handle Pointer to RV3032 handle structure
 * @param dt Pointer to datetime structure with values to set
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_set_datetime(rv3032_handle_t* handle, const rv3032_datetime_t* dt);

/**
 * @brief Get current Unix timestamp (seconds since Jan 1, 2000)
 * @param handle Pointer to RV3032 handle structure
 * @param unix_time Pointer to store Unix timestamp
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_get_unix_time(rv3032_handle_t* handle, uint32_t* unix_time);

/**
 * @brief Set time using Unix timestamp (seconds since Jan 1, 2000)
 * @param handle Pointer to RV3032 handle structure
 * @param unix_time Unix timestamp
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_set_unix_time(rv3032_handle_t* handle, uint32_t unix_time);

// === Alarm Functions ===

/**
 * @brief Set alarm
 * @param handle Pointer to RV3032 handle structure
 * @param alarm Pointer to alarm structure with settings
 * @param enable_interrupt Enable alarm interrupt on INT pin
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_set_alarm(rv3032_handle_t* handle, 
                           const rv3032_alarm_t* alarm, 
                           bool enable_interrupt);

/**
 * @brief Get current alarm settings
 * @param handle Pointer to RV3032 handle structure
 * @param alarm Pointer to alarm structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_get_alarm(rv3032_handle_t* handle, rv3032_alarm_t* alarm);

/**
 * @brief Clear alarm flag and disable alarm
 * @param handle Pointer to RV3032 handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_clear_alarm(rv3032_handle_t* handle);

/**
 * @brief Check if alarm has triggered
 * @param handle Pointer to RV3032 handle structure
 * @param triggered Pointer to store alarm status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_is_alarm_triggered(rv3032_handle_t* handle, bool* triggered);

// === Utility Functions ===

/**
 * @brief Check if RTC is responding
 * @param handle Pointer to RV3032 handle structure
 * @param connected Pointer to store connection status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rv3032_is_connected(rv3032_handle_t* handle, bool* connected);

#ifdef __cplusplus
}
#endif

#endif // RV3032_H