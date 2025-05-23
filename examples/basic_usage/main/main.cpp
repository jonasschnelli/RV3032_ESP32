#include "driver/i2c.h"
#include "esp_log.h"

// Define pins
#define LED_RED_PIN       20
#define LED_GREEN_PIN     19
#define LED_BLUE_PIN      18
#define POWER_STATE_PIN   15
#define POWER_HOLD_PIN    6
#define RV3032_INT_PIN    2

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 40
#define I2C_MASTER_SDA_IO 39
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_ADDR_RV3032 0x51 // 7-bit address

#define RV3032_REG_SECONDS 0x01
#define RV3032_REG_MINUTES 0x02
#define RV3032_REG_HOURS   0x03
#define RV3032_REG_ALM_MIN 0x08
#define RV3032_REG_ALM_HR  0x09
#define RV3032_REG_ALM_DATE 0x0A
#define RV3032_REG_CTRL2   0x11
#define RV3032_REG_STATUS  0x0D

static const char* TAG = "power_rtc";

extern "C" {
    void app_main(void);
}

volatile bool rtc_alarm_triggered = false;

extern "C" void IRAM_ATTR rtc_int_isr_handler(void* arg) {
    rtc_alarm_triggered = true;
}

// Initialize the LED GPIOs
void led_init(void) {
    // Configure LED pins
    gpio_config_t led_conf = {};
    led_conf.mode = GPIO_MODE_OUTPUT;
    led_conf.pin_bit_mask = (1ULL << LED_RED_PIN) | (1ULL << LED_GREEN_PIN) | (1ULL << LED_BLUE_PIN);
    led_conf.intr_type = GPIO_INTR_DISABLE;
    led_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    led_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&led_conf);
    
    // Configure power state pin as input
    gpio_config_t power_conf = {};
    power_conf.mode = GPIO_MODE_INPUT;
    power_conf.pin_bit_mask = (1ULL << POWER_STATE_PIN);
    power_conf.intr_type = GPIO_INTR_DISABLE;
    power_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    power_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&power_conf);
    
    // Turn off all LEDs initially
    gpio_set_level((gpio_num_t)LED_RED_PIN, 0);
    gpio_set_level((gpio_num_t)LED_GREEN_PIN, 0);
    gpio_set_level((gpio_num_t)LED_BLUE_PIN, 0);
    
    ESP_LOGI(TAG, "GPIOs initialized");
}

// Set RGB LED color (1 = on, 0 = off for each component)
void led_set_color(int red, int green, int blue) {
    gpio_set_level((gpio_num_t)LED_RED_PIN, red ? 1 : 0);
    gpio_set_level((gpio_num_t)LED_GREEN_PIN, green ? 1 : 0);
    gpio_set_level((gpio_num_t)LED_BLUE_PIN, blue ? 1 : 0);
}

// Read power state and update LED
void update_power_indicator(void) {
    // Read power state pin (HIGH = USB, LOW = Battery)
    int power_state = gpio_get_level((gpio_num_t)POWER_STATE_PIN);
    
    if (power_state) {
        // USB power (blue)
        led_set_color(0, 0, 1);
        ESP_LOGI(TAG, "Power source: USB");
    } else {
        // Battery power (red)
        led_set_color(1, 0, 0);
        ESP_LOGI(TAG, "Power source: Battery");
    }
}

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

// Convert decimal to BCD
static uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

// Write 1 byte to RV-3032
esp_err_t rv3032_write(uint8_t reg, uint8_t val) {
    return i2c_master_write_to_device(I2C_MASTER_NUM, I2C_ADDR_RV3032, (uint8_t[]){reg, val}, 2, 100 / portTICK_PERIOD_MS);
}

// Read 1 byte from RV-3032
esp_err_t rv3032_read(uint8_t reg, uint8_t *val) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, I2C_ADDR_RV3032, &reg, 1, val, 1, 100 / portTICK_PERIOD_MS);
}

// Set alarm 30s in the future
void rv3032_set_alarm_30s_later() {
    uint8_t sec, min, hr;
    rv3032_read(RV3032_REG_SECONDS, &sec);
    rv3032_read(RV3032_REG_MINUTES, &min);
    rv3032_read(RV3032_REG_HOURS, &hr);

    sec &= 0x7F;
    min &= 0x7F;
    hr  &= 0x3F;

    uint8_t dec_sec = ((sec >> 4) * 10) + (sec & 0x0F);
    uint8_t dec_min = ((min >> 4) * 10) + (min & 0x0F);
    uint8_t dec_hr  = ((hr  >> 4) * 10) + (hr  & 0x0F);

    // Add 30 seconds
    dec_sec += 60;
    if (dec_sec >= 60) {
        dec_sec -= 60;
        dec_min += 1;
        if (dec_min >= 60) {
            dec_min = 0;
            dec_hr = (dec_hr + 1) % 24;
        }
    }

    uint8_t bcd_min = dec_to_bcd(dec_min);
    uint8_t bcd_hr  = dec_to_bcd(dec_hr);

    // Set alarm for that minute/hour, ignore date
    rv3032_write(RV3032_REG_ALM_MIN, bcd_min & 0x7F); // AE_M = 0
    rv3032_write(RV3032_REG_ALM_HR,  bcd_hr & 0x7F);   // AE_H = 0
    rv3032_write(RV3032_REG_ALM_DATE, 0x80);          // AE_D = 1 → disable date

    // Enable alarm interrupt
    uint8_t ctrl2;
    rv3032_read(RV3032_REG_CTRL2, &ctrl2);
    ctrl2 |= (1 << 3); // Set AIE
    rv3032_write(RV3032_REG_CTRL2, ctrl2);

    ESP_LOGI("RTC", "Alarm set for %02d:%02d", dec_hr, dec_min);
}



// Clear alarm flag manually
void rv3032_clear_alarm_flag() {
    uint8_t status;
    rv3032_read(RV3032_REG_STATUS, &status);
    status &= ~(1 << 3);  // Clear AF bit (bit 3)
    rv3032_write(RV3032_REG_STATUS, status);
    ESP_LOGI("RTC", "Alarm flag cleared.");
}

void rtc_int_gpio_init() {
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_NEGEDGE; // Falling edge = INT̅ active
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = (1ULL << RV3032_INT_PIN);
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // You already have R45 external
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	ESP_ERROR_CHECK(gpio_config(&io_conf));

	ESP_LOGI("RTC", "Installing ISR service...");
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_LOGI("RTC", "Attaching ISR handler...");
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)RV3032_INT_PIN, rtc_int_isr_handler, NULL));
}

void rv3032_debug_status() {
    uint8_t sec, min, hr, alm_min, alm_hr, alm_date;
    uint8_t ctrl2, status;

    rv3032_read(RV3032_REG_SECONDS, &sec);
    rv3032_read(RV3032_REG_MINUTES, &min);
    rv3032_read(RV3032_REG_HOURS, &hr);

    rv3032_read(RV3032_REG_ALM_MIN, &alm_min);
    rv3032_read(RV3032_REG_ALM_HR, &alm_hr);
    rv3032_read(RV3032_REG_ALM_DATE, &alm_date);

    rv3032_read(RV3032_REG_CTRL2, &ctrl2);
    rv3032_read(RV3032_REG_STATUS, &status);
	
    int int_level = gpio_get_level((gpio_num_t)RV3032_INT_PIN);

    ESP_LOGI("RTC-DEBUG", "Current time: %02x:%02x:%02x", hr, min, sec);
    ESP_LOGI("RTC-DEBUG", "Alarm set:    %02x:%02x date: 0x%02x", alm_hr, alm_min, alm_date);
    ESP_LOGI("RTC-DEBUG", "CTRL2: 0x%02x (AIE=%d)", ctrl2, (ctrl2 >> 3) & 1);
    ESP_LOGI("RTC-DEBUG", "STATUS: 0x%02x (AF=%d)", status, (status >> 3) & 1);
    ESP_LOGI("RTC-DEBUG", "INT̅ pin level: %d (should be LOW when asserted)", int_level);
}

void app_main(void) {
    // IMPORTANT: Set power hold HIGH immediately
    gpio_reset_pin((gpio_num_t)POWER_HOLD_PIN);
    gpio_set_direction((gpio_num_t)POWER_HOLD_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)POWER_HOLD_PIN, 1);
	
    // Initialize LED pins
    led_init();
	
	i2c_master_init();
    
	rtc_int_gpio_init();
	
	rv3032_clear_alarm_flag();
    // Initial test sequence
    ESP_LOGI(TAG, "Starting power and RTC monitor");
    
    // Blink all colors briefly to show the system is starting
    led_set_color(1, 0, 0); // Red
    vTaskDelay(pdMS_TO_TICKS(300));
	
	rv3032_set_alarm_30s_later();

	vTaskDelay(pdMS_TO_TICKS(1000));
	gpio_set_level((gpio_num_t)POWER_HOLD_PIN, 0);
}
