# RV3032 RTC Library for ESP-IDF 5.5

A lightweight, ESP-IDF 5.5 native C library for the **RV-3032-C7** Temperature Compensated Real-Time Clock Module.

## Features

- ✅ **ESP-IDF 5.5 Native**: Uses latest ESP-IDF I2C master driver and GPIO APIs
- ✅ **Time/Date Management**: Read and set date/time with high precision
- ✅ **Alarm Functionality**: Flexible alarm system with interrupt support
- ✅ **Unix Timestamp**: Convert between RTC time and Unix timestamps
- ✅ **Memory Efficient**: Minimal RAM footprint and efficient I2C communication
- ✅ **Thread Safe**: Uses ESP-IDF synchronization primitives
- ✅ **Well Documented**: Comprehensive documentation and examples
- ✅ **Extensible Design**: Easy to add more RTC features later

## Hardware Requirements

- ESP32 series microcontroller
- RV-3032-C7 RTC module
- Pull-up resistors for I2C (usually built into ESP32 boards)
- Optional: Pull-up resistor for INT pin (10kΩ recommended)

## Documentation

📖 **[RV-3032-C7 Application Manual](https://www.microcrystal.com/fileadmin/Media/Products/RTC/App.Manual/RV-3032-C7_App-Manual.pdf)** - Official datasheet and technical reference

## Quick Start

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "RV3032.h"

static i2c_master_bus_handle_t i2c_bus_handle;
static rv3032_handle_t rtc_handle;

// Initialize I2C bus
static esp_err_t init_i2c(void) {
    i2c_master_bus_config_t i2c_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&i2c_config, &i2c_bus_handle);
}

void app_main(void) {
    // Initialize I2C
    ESP_ERROR_CHECK(init_i2c());
    
    // Initialize RTC
    ESP_ERROR_CHECK(rv3032_init(&rtc_handle, i2c_bus_handle, GPIO_NUM_NC, NULL));
    
    // Set time: 2024-12-15 12:30:00
    rv3032_datetime_t dt = {
        .year = 24, .month = 12, .date = 15,
        .hours = 12, .minutes = 30, .seconds = 0
    };
    rv3032_set_datetime(&rtc_handle, &dt);
    
    while (1) {
        rv3032_datetime_t current_time;
        if (rv3032_get_datetime(&rtc_handle, &current_time) == ESP_OK) {
            printf("Time: 20%02d-%02d-%02d %02d:%02d:%02d\n",
                   current_time.year, current_time.month, current_time.date,
                   current_time.hours, current_time.minutes, current_time.seconds);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## API Reference

### Initialization Functions

#### `esp_err_t rv3032_init(rv3032_handle_t* handle, i2c_master_bus_handle_t i2c_bus_handle, gpio_num_t int_pin, void (*alarm_callback)(void))`
Initialize the RTC module.
- `handle`: Pointer to RV3032 handle structure
- `i2c_bus_handle`: ESP-IDF I2C master bus handle
- `int_pin`: GPIO pin for interrupt (GPIO_NUM_NC if not used)
- `alarm_callback`: Callback function for alarm interrupt (NULL if not used)
- **Returns**: ESP_OK on success, error code otherwise

#### `esp_err_t rv3032_deinit(rv3032_handle_t* handle)`
Deinitialize the RTC and free resources.

### Time/Date Functions

#### `esp_err_t rv3032_get_datetime(rv3032_handle_t* handle, rv3032_datetime_t* dt)`
Read current date and time.

#### `esp_err_t rv3032_set_datetime(rv3032_handle_t* handle, const rv3032_datetime_t* dt)`
Set the RTC date and time.

#### `esp_err_t rv3032_get_unix_time(rv3032_handle_t* handle, uint32_t* unix_time)`
Get Unix timestamp (seconds since Jan 1, 2000).

#### `esp_err_t rv3032_set_unix_time(rv3032_handle_t* handle, uint32_t unix_time)`
Set time using Unix timestamp.

### Alarm Functions

#### `esp_err_t rv3032_set_alarm(rv3032_handle_t* handle, const rv3032_alarm_t* alarm, bool enable_interrupt)`
Configure alarm settings.

#### `esp_err_t rv3032_get_alarm(rv3032_handle_t* handle, rv3032_alarm_t* alarm)`
Read current alarm settings.

#### `esp_err_t rv3032_clear_alarm(rv3032_handle_t* handle)`
Clear alarm flag and disable alarm.

#### `esp_err_t rv3032_is_alarm_triggered(rv3032_handle_t* handle, bool* triggered)`
Check if alarm has been triggered.

### Data Structures

#### `rv3032_datetime_t`
```c
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
```

#### `rv3032_alarm_t`
```c
typedef struct {
    uint8_t date;       // 1-31
    uint8_t hours;      // 0-23
    uint8_t minutes;    // 0-59
    bool date_enable;   // Enable date matching
    bool hours_enable;  // Enable hours matching
    bool minutes_enable; // Enable minutes matching
} rv3032_alarm_t;
```

## Alarm Examples

### Daily Alarm (Every Day at Specific Time)
```c
rv3032_alarm_t alarm = {
    .hours = 7,             // 7:00 AM
    .minutes = 30,          // 30 minutes
    .date = 1,              // Ignored
    .hours_enable = true,
    .minutes_enable = true,
    .date_enable = false    // Disable date matching
};
rv3032_set_alarm(&rtc_handle, &alarm, true);
```

### Monthly Alarm (Specific Date and Time Each Month)
```c
rv3032_alarm_t alarm = {
    .date = 15,             // 15th of each month
    .hours = 9,             // 9:00 AM
    .minutes = 0,
    .date_enable = true,
    .hours_enable = true,
    .minutes_enable = true
};
rv3032_set_alarm(&rtc_handle, &alarm, true);
```

## Interrupt Handling

```c
static volatile bool alarm_flag = false;

static void alarm_callback(void) {
    // Called from ISR context - keep it minimal
    alarm_flag = true;
}

void app_main(void) {
    // Initialize with interrupt callback
    rv3032_init(&rtc_handle, i2c_bus_handle, GPIO_NUM_2, alarm_callback);
    
    // Set up alarm...
    
    while (1) {
        if (alarm_flag) {
            alarm_flag = false;
            printf("Alarm triggered!\n");
            
            // Handle alarm event
            bool triggered;
            if (rv3032_is_alarm_triggered(&rtc_handle, &triggered) == ESP_OK && triggered) {
                // Process alarm
                rv3032_clear_alarm(&rtc_handle); // Optional: disable further alarms
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## C++ Compatibility

If you're using a C++ file (main.cpp), you need to:

1. **Include the library with extern "C"**:
   ```cpp
   extern "C" {
       #include "RV3032.h"
   }
   ```

2. **Use proper GPIO types**:
   ```cpp
   // Use GPIO_NUM_X constants instead of plain integers
   #define RTC_INT_PIN GPIO_NUM_2  // Not just "2"
   ```

3. **Initialize structs without designated initializers**:
   ```cpp
   // C++ style initialization
   rv3032_datetime_t dt = {};
   dt.year = 24;
   dt.month = 12;
   dt.date = 15;
   // ... set other fields
   
   // Instead of C99 designated initializers:
   // rv3032_datetime_t dt = {.year = 24, .month = 12, .date = 15};
   ```

Multiple devices can share the same I2C bus:

```c
// Create I2C bus once
i2c_master_bus_handle_t i2c_bus;
i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = -1,
    .scl_io_num = GPIO_NUM_22,
    .sda_io_num = GPIO_NUM_21,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_new_master_bus(&bus_config, &i2c_bus);

// Initialize RTC with shared bus
rv3032_init(&rtc_handle, i2c_bus, GPIO_NUM_2, alarm_callback);

// Initialize other I2C devices with the same bus...
```

## Technical Notes

### ESP-IDF 5.5 Compatibility
- Uses new `i2c_master` driver (replaces legacy `i2c` driver)
- Compatible with ESP-IDF 5.5+ 
- Uses ESP-IDF native error handling (`esp_err_t`)
- Thread-safe operations using ESP-IDF primitives

### I2C Communication
- Default I2C address: `0x51` (7-bit)
- Supports up to 400kHz I2C speed
- Uses efficient burst read/write operations
- Built-in timeout handling (1000ms default)

### Memory Usage
- Minimal RAM footprint (~100 bytes per handle)
- No dynamic memory allocation during normal operation
- Stack-based temporary buffers for I2C operations

## Future Extensions

This library is designed for easy extension:

- **Temperature sensor** reading (registers 0x0E, 0x0F)
- **Periodic timer** interrupts (registers 0x0B, 0x0C)
- **External event** timestamping (EVI pin)
- **Clock output** configuration (CLKOUT pin)
- **EEPROM** access for user data
- **Power management** features
- **Time stamp** functions

## Troubleshooting

### RTC Not Found
- Check I2C wiring (SDA, SCL)
- Verify 3.3V power supply
- Ensure pull-up resistors on I2C lines
- Check I2C bus configuration in menuconfig

### I2C Communication Errors
- Verify I2C pins are not used by other peripherals
- Try lower I2C speed in bus configuration
- Check for I2C conflicts with other devices

### Interrupts Not Working
- Verify INT pin connection and pull-up
- Check GPIO pin configuration
- Ensure interrupt service is installed
- Verify alarm settings are correct

### Build Errors
- Ensure ESP-IDF version is 5.5 or higher
- Check CMakeLists.txt includes required components
- Verify all source files are in SRCS list
- If building the examples: MAKE SURE THE ROOT DIRECTORY IS NAME "RV3032_ESP32" (or change the CMakeList.txt)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

Author: Jonas Schnelli

Based on the RV-3032-C7 Application Manual Rev. 1.3 (May 2023).

## Contributing

Contributions are welcome! Please:
1. Follow ESP-IDF coding standards
2. Keep the code memory efficient
3. Add appropriate error handling
4. Test thoroughly with ESP-IDF 5.5+
5. Update documentation accordingly
