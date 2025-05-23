/**
 * @file RV3032.c
 * @brief Implementation of RV3032 RTC library for ESP-IDF 5.5
 */

#include "RV3032.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char* TAG = "RV3032";

// Days in each month (non-leap year)
static const uint8_t DAYS_IN_MONTH[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// === Private Function Declarations ===

static uint8_t bcd_to_dec(uint8_t bcd);
static uint8_t dec_to_bcd(uint8_t dec);
static esp_err_t rv3032_write_register(rv3032_handle_t* handle, uint8_t reg, uint8_t value);
static esp_err_t rv3032_read_register(rv3032_handle_t* handle, uint8_t reg, uint8_t* value);
static esp_err_t rv3032_read_registers(rv3032_handle_t* handle, uint8_t reg, uint8_t* buffer, uint8_t length);
static esp_err_t rv3032_write_registers(rv3032_handle_t* handle, uint8_t reg, const uint8_t* buffer, uint8_t length);
static uint8_t days_in_month(uint8_t month, uint8_t year);
static bool is_leap_year(uint8_t year);
static uint32_t datetime_to_unix(const rv3032_datetime_t* dt);
static void unix_to_datetime(uint32_t unix_time, rv3032_datetime_t* dt);
static void IRAM_ATTR gpio_isr_handler(void* arg);

// === Public Functions ===

esp_err_t rv3032_init(rv3032_handle_t* handle, 
                      i2c_master_bus_handle_t i2c_bus_handle,
                      gpio_num_t int_pin,
                      void (*alarm_callback)(void)) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(handle, 0, sizeof(rv3032_handle_t));
    handle->i2c_bus_handle = i2c_bus_handle;
    handle->int_pin = int_pin;
    handle->alarm_callback = alarm_callback;

    // Configure I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RV3032_I2C_ADDR,
        .scl_speed_hz = 400000,  // 400kHz
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &handle->i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Test communication
    bool connected;
    ret = rv3032_is_connected(handle, &connected);
    if (ret != ESP_OK || !connected) {
        ESP_LOGE(TAG, "RV3032 not responding");
        i2c_master_bus_rm_device(handle->i2c_dev_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // Configure interrupt pin if provided
    if (int_pin != GPIO_NUM_NC && alarm_callback != NULL) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_NEGEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << int_pin),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
            i2c_master_bus_rm_device(handle->i2c_dev_handle);
            return ret;
        }

        ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "GPIO ISR service install failed: %s", esp_err_to_name(ret));
            i2c_master_bus_rm_device(handle->i2c_dev_handle);
            return ret;
        }

        ret = gpio_isr_handler_add(int_pin, gpio_isr_handler, handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "GPIO ISR handler add failed: %s", esp_err_to_name(ret));
            i2c_master_bus_rm_device(handle->i2c_dev_handle);
            return ret;
        }
    }

    ESP_LOGI(TAG, "RV3032 initialized successfully");
    return ESP_OK;
}

esp_err_t rv3032_deinit(rv3032_handle_t* handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Remove GPIO interrupt handler if configured
    if (handle->int_pin != GPIO_NUM_NC) {
        gpio_isr_handler_remove(handle->int_pin);
    }

    // Remove I2C device
    if (handle->i2c_dev_handle) {
        i2c_master_bus_rm_device(handle->i2c_dev_handle);
    }

    memset(handle, 0, sizeof(rv3032_handle_t));
    return ESP_OK;
}

esp_err_t rv3032_get_datetime(rv3032_handle_t* handle, rv3032_datetime_t* dt) {
    if (!handle || !dt) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[8];
    
    // Read all time registers in one transaction (addresses 0x00-0x07)
    esp_err_t ret = rv3032_read_registers(handle, RV3032_REG_HUNDREDTHS, buffer, 8);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert from BCD to decimal
    dt->hundredths = bcd_to_dec(buffer[0]);
    dt->seconds = bcd_to_dec(buffer[1] & 0x7F);  // Mask bit 7 (always 0)
    dt->minutes = bcd_to_dec(buffer[2] & 0x7F);  // Mask bit 7 (always 0)
    dt->hours = bcd_to_dec(buffer[3] & 0x3F);    // Mask bits 7:6 (always 0)
    dt->weekday = buffer[4] & 0x07;              // Mask bits 7:3 (always 0)
    dt->date = bcd_to_dec(buffer[5] & 0x3F);     // Mask bits 7:6 (always 0)
    dt->month = bcd_to_dec(buffer[6] & 0x1F);    // Mask bits 7:5 (always 0)
    dt->year = bcd_to_dec(buffer[7]);
    
    return ESP_OK;
}

esp_err_t rv3032_set_datetime(rv3032_handle_t* handle, const rv3032_datetime_t* dt) {
    if (!handle || !dt) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[7];
    
    // Convert to BCD and prepare buffer (skip hundredths - read-only)
    buffer[0] = dec_to_bcd(dt->seconds);
    buffer[1] = dec_to_bcd(dt->minutes);
    buffer[2] = dec_to_bcd(dt->hours);
    buffer[3] = dt->weekday & 0x07;
    buffer[4] = dec_to_bcd(dt->date);
    buffer[5] = dec_to_bcd(dt->month);
    buffer[6] = dec_to_bcd(dt->year);
    
    // Write all registers starting from seconds (0x01-0x07)
    // Writing to seconds register also clears hundredths and synchronizes
    return rv3032_write_registers(handle, RV3032_REG_SECONDS, buffer, 7);
}

esp_err_t rv3032_get_unix_time(rv3032_handle_t* handle, uint32_t* unix_time) {
    if (!handle || !unix_time) {
        return ESP_ERR_INVALID_ARG;
    }

    rv3032_datetime_t dt;
    esp_err_t ret = rv3032_get_datetime(handle, &dt);
    if (ret != ESP_OK) {
        return ret;
    }

    *unix_time = datetime_to_unix(&dt);
    return ESP_OK;
}

esp_err_t rv3032_set_unix_time(rv3032_handle_t* handle, uint32_t unix_time) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    rv3032_datetime_t dt;
    unix_to_datetime(unix_time, &dt);
    return rv3032_set_datetime(handle, &dt);
}

esp_err_t rv3032_set_alarm(rv3032_handle_t* handle, 
                           const rv3032_alarm_t* alarm, 
                           bool enable_interrupt) {
    if (!handle || !alarm) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[3];
    
    // Prepare alarm registers
    // Minutes alarm (0x08): bit 7 = enable, bits 6:0 = BCD minutes
    buffer[0] = dec_to_bcd(alarm->minutes);
    if (!alarm->minutes_enable) {
        buffer[0] |= RV3032_ALARM_DISABLE_BIT;
    }
    
    // Hours alarm (0x09): bit 7 = enable, bits 5:0 = BCD hours
    buffer[1] = dec_to_bcd(alarm->hours);
    if (!alarm->hours_enable) {
        buffer[1] |= RV3032_ALARM_DISABLE_BIT;
    }
    
    // Date alarm (0x0A): bit 7 = enable, bits 5:0 = BCD date
    buffer[2] = dec_to_bcd(alarm->date);
    if (!alarm->date_enable) {
        buffer[2] |= RV3032_ALARM_DISABLE_BIT;
    }
    
    // Write alarm registers
    esp_err_t ret = rv3032_write_registers(handle, RV3032_REG_MINUTES_ALARM, buffer, 3);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Enable/disable alarm interrupt
    uint8_t ctrl2;
    ret = rv3032_read_register(handle, RV3032_REG_CONTROL2, &ctrl2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (enable_interrupt) {
        ctrl2 |= RV3032_CTRL2_AIE;
    } else {
        ctrl2 &= ~RV3032_CTRL2_AIE;
    }
    
    return rv3032_write_register(handle, RV3032_REG_CONTROL2, ctrl2);
}

esp_err_t rv3032_get_alarm(rv3032_handle_t* handle, rv3032_alarm_t* alarm) {
    if (!handle || !alarm) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[3];
    
    // Read alarm registers
    esp_err_t ret = rv3032_read_registers(handle, RV3032_REG_MINUTES_ALARM, buffer, 3);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse alarm settings
    alarm->minutes_enable = !(buffer[0] & RV3032_ALARM_DISABLE_BIT);
    alarm->minutes = bcd_to_dec(buffer[0] & 0x7F);
    
    alarm->hours_enable = !(buffer[1] & RV3032_ALARM_DISABLE_BIT);
    alarm->hours = bcd_to_dec(buffer[1] & 0x3F);
    
    alarm->date_enable = !(buffer[2] & RV3032_ALARM_DISABLE_BIT);
    alarm->date = bcd_to_dec(buffer[2] & 0x3F);
    
    return ESP_OK;
}

esp_err_t rv3032_clear_alarm(rv3032_handle_t* handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Clear alarm flag by writing 0 to AF bit
    uint8_t status;
    esp_err_t ret = rv3032_read_register(handle, RV3032_REG_STATUS, &status);
    if (ret != ESP_OK) {
        return ret;
    }
    
    status &= ~RV3032_STATUS_AF;  // Clear AF bit
    ret = rv3032_write_register(handle, RV3032_REG_STATUS, status);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Disable alarm interrupt
    uint8_t ctrl2;
    ret = rv3032_read_register(handle, RV3032_REG_CONTROL2, &ctrl2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ctrl2 &= ~RV3032_CTRL2_AIE;  // Clear AIE bit
    return rv3032_write_register(handle, RV3032_REG_CONTROL2, ctrl2);
}

esp_err_t rv3032_is_alarm_triggered(rv3032_handle_t* handle, bool* triggered) {
    if (!handle || !triggered) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status;
    esp_err_t ret = rv3032_read_register(handle, RV3032_REG_STATUS, &status);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *triggered = (status & RV3032_STATUS_AF) != 0;
    return ESP_OK;
}

esp_err_t rv3032_is_connected(rv3032_handle_t* handle, bool* connected) {
    if (!handle || !connected) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t dummy;
    esp_err_t ret = rv3032_read_register(handle, RV3032_REG_SECONDS, &dummy);
    *connected = (ret == ESP_OK);
    return ESP_OK;
}

// === Private Helper Functions ===

static uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

static esp_err_t rv3032_write_register(rv3032_handle_t* handle, uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(handle->i2c_dev_handle, write_buf, 2, RV3032_I2C_TIMEOUT_MS);
}

static esp_err_t rv3032_read_register(rv3032_handle_t* handle, uint8_t reg, uint8_t* value) {
    return i2c_master_transmit_receive(handle->i2c_dev_handle, &reg, 1, value, 1, RV3032_I2C_TIMEOUT_MS);
}

static esp_err_t rv3032_read_registers(rv3032_handle_t* handle, uint8_t reg, uint8_t* buffer, uint8_t length) {
    return i2c_master_transmit_receive(handle->i2c_dev_handle, &reg, 1, buffer, length, RV3032_I2C_TIMEOUT_MS);
}

static esp_err_t rv3032_write_registers(rv3032_handle_t* handle, uint8_t reg, const uint8_t* buffer, uint8_t length) {
    uint8_t* write_buf = malloc(length + 1);
    if (!write_buf) {
        return ESP_ERR_NO_MEM;
    }
    
    write_buf[0] = reg;
    memcpy(&write_buf[1], buffer, length);
    
    esp_err_t ret = i2c_master_transmit(handle->i2c_dev_handle, write_buf, length + 1, RV3032_I2C_TIMEOUT_MS);
    free(write_buf);
    return ret;
}

static uint8_t days_in_month(uint8_t month, uint8_t year) {
    if (month == 2 && is_leap_year(year)) {
        return 29;
    }
    return DAYS_IN_MONTH[month - 1];
}

static bool is_leap_year(uint8_t year) {
    // Year is 0-99 representing 2000-2099
    // All years divisible by 4 in this range are leap years
    // (2100 is not a leap year, but it's outside our range)
    uint16_t full_year = 2000 + year;
    return (full_year % 4 == 0);
}

static uint32_t datetime_to_unix(const rv3032_datetime_t* dt) {
    // Calculate days since January 1, 2000
    uint32_t days = 0;
    
    // Add days for complete years
    for (uint8_t y = 0; y < dt->year; y++) {
        days += is_leap_year(y) ? 366 : 365;
    }
    
    // Add days for complete months in current year
    for (uint8_t m = 1; m < dt->month; m++) {
        days += days_in_month(m, dt->year);
    }
    
    // Add days in current month
    days += (dt->date - 1);
    
    // Convert to seconds and add time
    uint32_t seconds = days * 86400UL;  // 24 * 60 * 60
    seconds += dt->hours * 3600UL;      // 60 * 60
    seconds += dt->minutes * 60UL;
    seconds += dt->seconds;
    
    return seconds;
}

static void unix_to_datetime(uint32_t unix_time, rv3032_datetime_t* dt) {
    // Extract time components
    dt->seconds = unix_time % 60;
    unix_time /= 60;
    dt->minutes = unix_time % 60;
    unix_time /= 60;
    dt->hours = unix_time % 24;
    uint32_t days = unix_time / 24;
    
    // Find year
    dt->year = 0;
    while (true) {
        uint16_t days_in_year = is_leap_year(dt->year) ? 366 : 365;
        if (days < days_in_year) {
            break;
        }
        days -= days_in_year;
        dt->year++;
    }
    
    // Find month
    dt->month = 1;
    while (true) {
        uint8_t days_in_current_month = days_in_month(dt->month, dt->year);
        if (days < days_in_current_month) {
            break;
        }
        days -= days_in_current_month;
        dt->month++;
    }
    
    // Set date
    dt->date = days + 1;
    
    // Calculate weekday (0 = Saturday for Jan 1, 2000)
    uint32_t total_days = (unix_time / 86400UL);
    dt->weekday = (total_days + 6) % 7;  // Adjust so 0 = Sunday
    
    // Hundredths is not calculated from Unix time
    dt->hundredths = 0;
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    rv3032_handle_t* handle = (rv3032_handle_t*)arg;
    if (handle && handle->alarm_callback) {
        handle->alarm_callback();
    }
}