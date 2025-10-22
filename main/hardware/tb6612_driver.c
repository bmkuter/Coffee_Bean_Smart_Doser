/*
 * TB6612FNG Motor Driver - Hardware Abstraction Layer
 * 
 * Low-level driver for TB6612FNG dual H-bridge motor controller (Pololu #713)
 * Uses ESP32 LEDC peripheral for high-frequency PWM control
 * 
 * Only compiled when USE_TB6612_MOTOR_DRIVER is defined in coffee_doser_config.h
 */

#include "coffee_doser_config.h"

#ifdef USE_TB6612_MOTOR_DRIVER

#include "tb6612_driver.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Driver state
static bool tb6612_initialized = false;

// Internal helper functions
static esp_err_t tb6612_configure_gpio(void);
static esp_err_t tb6612_configure_ledc(void);
static esp_err_t tb6612_set_direction_pins(uint8_t motor, tb6612_direction_t direction);

esp_err_t tb6612_init(void)
{
    if (tb6612_initialized) {
        ESP_LOGW(TAG_TB6612, "TB6612FNG already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG_TB6612, "Initializing TB6612FNG dual H-bridge (Pololu #713)");
    ESP_LOGI(TAG_TB6612, "PWM frequency: %d Hz (12-bit resolution, 0-4095)", TB6612_LEDC_FREQUENCY);

    // Configure GPIO pins for direction control
    esp_err_t ret = tb6612_configure_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_TB6612, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC peripheral for PWM
    ret = tb6612_configure_ledc();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_TB6612, "Failed to configure LEDC: %s", esp_err_to_name(ret));
        return ret;
    }

#if TB6612_USE_STBY_PIN
    // Enable the driver (take out of standby)
    ret = tb6612_set_standby(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_TB6612, "Failed to enable standby pin: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG_TB6612, "Driver enabled (STBY pin HIGH)");
#endif

    tb6612_initialized = true;

    // Initialize both motors to stopped state
    tb6612_set_motor(0, TB6612_DIRECTION_STOP, 0);  // Motor A
    tb6612_set_motor(1, TB6612_DIRECTION_STOP, 0);  // Motor B

    ESP_LOGI(TAG_TB6612, "TB6612FNG initialization complete");
    return ESP_OK;
}

esp_err_t tb6612_deinit(void)
{
    if (!tb6612_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG_TB6612, "Deinitializing TB6612FNG driver");

    // Stop both motors
    tb6612_set_motor(0, TB6612_DIRECTION_STOP, 0);
    tb6612_set_motor(1, TB6612_DIRECTION_STOP, 0);

#if TB6612_USE_STBY_PIN
    // Put driver in standby
    tb6612_set_standby(false);
#endif

    // Stop LEDC channels
    ledc_stop(TB6612_LEDC_MODE, TB6612_LEDC_CHANNEL_PWMA, 0);
    ledc_stop(TB6612_LEDC_MODE, TB6612_LEDC_CHANNEL_PWMB, 0);

    // Reset GPIO pins to input (high-impedance)
    gpio_reset_pin(TB6612_AIN1_GPIO);
    gpio_reset_pin(TB6612_AIN2_GPIO);
    gpio_reset_pin(TB6612_PWMA_GPIO);
    gpio_reset_pin(TB6612_BIN1_GPIO);
    gpio_reset_pin(TB6612_BIN2_GPIO);
    gpio_reset_pin(TB6612_PWMB_GPIO);
#if TB6612_USE_STBY_PIN
    gpio_reset_pin(TB6612_STBY_GPIO);
#endif

    tb6612_initialized = false;
    ESP_LOGI(TAG_TB6612, "TB6612FNG deinitialization complete");
    return ESP_OK;
}

esp_err_t tb6612_set_pwm_value(uint8_t channel, uint16_t value)
{
    if (!tb6612_initialized) {
        ESP_LOGE(TAG_TB6612, "TB6612FNG not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // No scaling needed - both PCA9685 and TB6612 use 12-bit (0-4095)
    // Clamp to 12-bit range for safety
    if (value > TB6612_PWM_MAX_DUTY) {
        value = TB6612_PWM_MAX_DUTY;
    }

    // Map PCA9685 channel numbers to TB6612 pins
    // Motor A (Auger): Channels 8, 9, 10 → PWMA, AIN2, AIN1
    // Motor B (Air Pump): Channels 11, 12, 13 → BIN1, BIN2, PWMB
    
    esp_err_t ret = ESP_OK;
    
    switch (channel) {
        // Motor A (Auger) - Channels 8, 9, 10
        case 8:  // Motor A PWM
            ret = ledc_set_duty(TB6612_LEDC_MODE, TB6612_LEDC_CHANNEL_PWMA, value);
            if (ret == ESP_OK) {
                ret = ledc_update_duty(TB6612_LEDC_MODE, TB6612_LEDC_CHANNEL_PWMA);
            }
            break;
            
        case 9:  // Motor A IN2
            ret = gpio_set_level(TB6612_AIN2_GPIO, value > 0 ? 1 : 0);
            break;
            
        case 10: // Motor A IN1
            ret = gpio_set_level(TB6612_AIN1_GPIO, value > 0 ? 1 : 0);
            break;
            
        // Motor B (Air Pump) - Channels 11, 12, 13
        case 11: // Motor B IN1
            ret = gpio_set_level(TB6612_BIN1_GPIO, value > 0 ? 1 : 0);
            break;
            
        case 12: // Motor B IN2
            ret = gpio_set_level(TB6612_BIN2_GPIO, value > 0 ? 1 : 0);
            break;
            
        case 13: // Motor B PWM
            ret = ledc_set_duty(TB6612_LEDC_MODE, TB6612_LEDC_CHANNEL_PWMB, value);
            if (ret == ESP_OK) {
                ret = ledc_update_duty(TB6612_LEDC_MODE, TB6612_LEDC_CHANNEL_PWMB);
            }
            break;
            
        default:
            ESP_LOGW(TAG_TB6612, "Invalid channel %d (only 8-13 supported)", channel);
            return ESP_ERR_INVALID_ARG;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG_TB6612, "Failed to set channel %d to %d: %s", 
                 channel, value, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t tb6612_set_motor(uint8_t motor, tb6612_direction_t direction, uint16_t speed)
{
    if (!tb6612_initialized) {
        ESP_LOGE(TAG_TB6612, "TB6612FNG not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (motor > 1) {
        ESP_LOGE(TAG_TB6612, "Invalid motor %d (must be 0 or 1)", motor);
        return ESP_ERR_INVALID_ARG;
    }

    // Clamp speed to 12-bit range
    if (speed > TB6612_PWM_MAX_DUTY) {
        speed = TB6612_PWM_MAX_DUTY;
    }

    ESP_LOGD(TAG_TB6612, "Motor %c: direction=%d, speed=%d", 
             motor == 0 ? 'A' : 'B', direction, speed);

    // Set direction pins
    esp_err_t ret = tb6612_set_direction_pins(motor, direction);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set PWM speed
    ledc_channel_t channel = (motor == 0) ? TB6612_LEDC_CHANNEL_PWMA : TB6612_LEDC_CHANNEL_PWMB;
    ret = ledc_set_duty(TB6612_LEDC_MODE, channel, speed);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(TB6612_LEDC_MODE, channel);
    }

    return ret;
}

esp_err_t tb6612_set_standby(bool enable)
{
#if TB6612_USE_STBY_PIN
    // STBY pin: LOW = standby (disabled), HIGH = active (enabled)
    esp_err_t ret = gpio_set_level(TB6612_STBY_GPIO, enable ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_TB6612, "Failed to set standby pin: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGD(TAG_TB6612, "Standby: %s", enable ? "ACTIVE" : "STANDBY");
    return ESP_OK;
#else
    // Standby pin not used (tied to VCC)
    ESP_LOGD(TAG_TB6612, "Standby pin not configured (always active)");
    return ESP_OK;
#endif
}

// Internal helper functions

static esp_err_t tb6612_configure_gpio(void)
{
    ESP_LOGI(TAG_TB6612, "Configuring GPIO pins");

    // Configure direction control pins as outputs
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (
            (1ULL << TB6612_AIN1_GPIO) |
            (1ULL << TB6612_AIN2_GPIO) |
            (1ULL << TB6612_BIN1_GPIO) |
            (1ULL << TB6612_BIN2_GPIO)
#if TB6612_USE_STBY_PIN
            | (1ULL << TB6612_STBY_GPIO)
#endif
        )
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_TB6612, "Failed to configure direction GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize all direction pins to LOW
    gpio_set_level(TB6612_AIN1_GPIO, 0);
    gpio_set_level(TB6612_AIN2_GPIO, 0);
    gpio_set_level(TB6612_BIN1_GPIO, 0);
    gpio_set_level(TB6612_BIN2_GPIO, 0);
#if TB6612_USE_STBY_PIN
    gpio_set_level(TB6612_STBY_GPIO, 0);  // Start in standby
#endif

    ESP_LOGI(TAG_TB6612, "GPIO configuration complete");
    ESP_LOGI(TAG_TB6612, "  Motor A: AIN1=%d, AIN2=%d, PWMA=%d", 
             TB6612_AIN1_GPIO, TB6612_AIN2_GPIO, TB6612_PWMA_GPIO);
    ESP_LOGI(TAG_TB6612, "  Motor B: BIN1=%d, BIN2=%d, PWMB=%d", 
             TB6612_BIN1_GPIO, TB6612_BIN2_GPIO, TB6612_PWMB_GPIO);
#if TB6612_USE_STBY_PIN
    ESP_LOGI(TAG_TB6612, "  Standby: STBY=%d", TB6612_STBY_GPIO);
#endif

    return ESP_OK;
}

static esp_err_t tb6612_configure_ledc(void)
{
    ESP_LOGI(TAG_TB6612, "Configuring LEDC peripheral");

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = TB6612_LEDC_MODE,
        .duty_resolution = TB6612_LEDC_DUTY_RESOLUTION,
        .timer_num = TB6612_LEDC_TIMER,
        .freq_hz = TB6612_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_TB6612, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channel for Motor A (PWMA)
    ledc_channel_config_t ledc_channel_a = {
        .gpio_num = TB6612_PWMA_GPIO,
        .speed_mode = TB6612_LEDC_MODE,
        .channel = TB6612_LEDC_CHANNEL_PWMA,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = TB6612_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    ret = ledc_channel_config(&ledc_channel_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_TB6612, "Failed to configure LEDC channel A: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channel for Motor B (PWMB)
    ledc_channel_config_t ledc_channel_b = {
        .gpio_num = TB6612_PWMB_GPIO,
        .speed_mode = TB6612_LEDC_MODE,
        .channel = TB6612_LEDC_CHANNEL_PWMB,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = TB6612_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    ret = ledc_channel_config(&ledc_channel_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_TB6612, "Failed to configure LEDC channel B: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_TB6612, "LEDC configuration complete");
    ESP_LOGI(TAG_TB6612, "  Timer: %d, Mode: %d", TB6612_LEDC_TIMER, TB6612_LEDC_MODE);
    ESP_LOGI(TAG_TB6612, "  Frequency: %d Hz, Resolution: %d-bit", 
             TB6612_LEDC_FREQUENCY, TB6612_LEDC_DUTY_RESOLUTION);

    return ESP_OK;
}

static esp_err_t tb6612_set_direction_pins(uint8_t motor, tb6612_direction_t direction)
{
    int in1_pin, in2_pin;
    int in1_level = 0, in2_level = 0;

    // Select pins based on motor
    if (motor == 0) {
        in1_pin = TB6612_AIN1_GPIO;
        in2_pin = TB6612_AIN2_GPIO;
    } else {
        in1_pin = TB6612_BIN1_GPIO;
        in2_pin = TB6612_BIN2_GPIO;
    }

    // Set levels based on direction
    switch (direction) {
        case TB6612_DIRECTION_STOP:
            in1_level = 0;
            in2_level = 0;
            break;
        case TB6612_DIRECTION_FORWARD:
            in1_level = 1;
            in2_level = 0;
            break;
        case TB6612_DIRECTION_REVERSE:
            in1_level = 0;
            in2_level = 1;
            break;
        case TB6612_DIRECTION_BRAKE:
            in1_level = 1;
            in2_level = 1;
            break;
        default:
            ESP_LOGE(TAG_TB6612, "Invalid direction %d", direction);
            return ESP_ERR_INVALID_ARG;
    }

    // Set the pins
    esp_err_t ret = gpio_set_level(in1_pin, in1_level);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = gpio_set_level(in2_pin, in2_level);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGD(TAG_TB6612, "Motor %c direction: IN1=%d, IN2=%d", 
             motor == 0 ? 'A' : 'B', in1_level, in2_level);

    return ESP_OK;
}

#endif // USE_TB6612_MOTOR_DRIVER
