/*
 * Servo Driver - PWM Servo Control Implementation
 * 
 * Uses LEDC peripheral for standard 50 Hz servo control
 */

#include "coffee_doser_config.h"
#include "servo_driver.h"
#include "driver/ledc.h"
#include "esp_log.h"

// Module state
static bool servo_initialized = false;
static bool servo_enabled = false;
static uint16_t current_pulse_us = SERVO_CENTER_PULSE_US;  // Start at center

// Calculate duty cycle from pulse width
static uint32_t pulse_to_duty(uint16_t pulse_us)
{
    // Period = 1/50Hz = 20ms = 20000us
    // Max duty = 2^14 = 16384
    // duty = (pulse_us / 20000us) * 16384
    // Simplified: duty = (pulse_us * 16384) / 20000
    return ((uint32_t)pulse_us * (1 << SERVO_LEDC_RESOLUTION)) / 20000;
}

// Calculate pulse width from duty cycle
static uint16_t duty_to_pulse(uint32_t duty)
{
    // pulse_us = (duty * 20000) / 16384
    return (uint16_t)((duty * 20000) / (1 << SERVO_LEDC_RESOLUTION));
}

esp_err_t servo_init(void)
{
    if (servo_initialized) {
        ESP_LOGW(TAG_SERVO, "Servo already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG_SERVO, "Initializing servo on GPIO%d", SERVO_GPIO_PIN);

    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_LEDC_RESOLUTION,
        .timer_num = SERVO_LEDC_TIMER,
        .freq_hz = SERVO_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SERVO, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channel
    ledc_channel_config_t channel_conf = {
        .gpio_num = SERVO_GPIO_PIN,
        .speed_mode = SERVO_LEDC_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_LEDC_TIMER,
        .duty = pulse_to_duty(SERVO_CENTER_PULSE_US),
        .hpoint = 0
    };

    ret = ledc_channel_config(&channel_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SERVO, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    servo_initialized = true;
    servo_enabled = true;
    current_pulse_us = SERVO_CENTER_PULSE_US;

    ESP_LOGI(TAG_SERVO, "Servo initialized at center position (1500µs, 90°)");
    ESP_LOGI(TAG_SERVO, "PWM: %d Hz, %d-bit resolution", SERVO_FREQUENCY_HZ, SERVO_LEDC_RESOLUTION);

    return ESP_OK;
}

esp_err_t servo_deinit(void)
{
    if (!servo_initialized) {
        ESP_LOGW(TAG_SERVO, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Stop PWM output
    ledc_stop(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, 0);

    servo_initialized = false;
    servo_enabled = false;

    ESP_LOGI(TAG_SERVO, "Servo deinitialized");
    return ESP_OK;
}

esp_err_t servo_set_pulse_width(uint16_t pulse_us)
{
    if (!servo_initialized) {
        ESP_LOGE(TAG_SERVO, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate pulse width range
    if (pulse_us < SERVO_MIN_PULSE_US || pulse_us > SERVO_MAX_PULSE_US) {
        ESP_LOGE(TAG_SERVO, "Invalid pulse width: %d µs (must be %d-%d µs)", 
                 pulse_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
        return ESP_ERR_INVALID_ARG;
    }

    // Calculate duty cycle
    uint32_t duty = pulse_to_duty(pulse_us);

    // Set duty cycle
    esp_err_t ret = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SERVO, "Failed to set duty cycle: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update duty cycle
    ret = ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SERVO, "Failed to update duty cycle: %s", esp_err_to_name(ret));
        return ret;
    }

    current_pulse_us = pulse_us;
    servo_enabled = true;

    ESP_LOGD(TAG_SERVO, "Servo position set to %d µs (duty: %lu)", pulse_us, duty);
    return ESP_OK;
}

esp_err_t servo_set_angle(uint8_t angle)
{
    if (!servo_initialized) {
        ESP_LOGE(TAG_SERVO, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate angle range
    if (angle > SERVO_MAX_ANGLE) {
        ESP_LOGE(TAG_SERVO, "Invalid angle: %d° (must be 0-%d°)", angle, SERVO_MAX_ANGLE);
        return ESP_ERR_INVALID_ARG;
    }

    // Map angle to pulse width
    // pulse = MIN_PULSE + (angle / MAX_ANGLE) * (MAX_PULSE - MIN_PULSE)
    uint16_t pulse_us = SERVO_MIN_PULSE_US + 
                        ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / SERVO_MAX_ANGLE;

    ESP_LOGI(TAG_SERVO, "Setting servo to %d° (%d µs)", angle, pulse_us);
    return servo_set_pulse_width(pulse_us);
}

uint8_t servo_get_angle(void)
{
    if (!servo_initialized) {
        return 0xFF;  // Invalid value
    }

    // Map pulse width back to angle
    uint8_t angle = ((uint32_t)(current_pulse_us - SERVO_MIN_PULSE_US) * SERVO_MAX_ANGLE) / 
                    (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    return angle;
}

uint16_t servo_get_pulse_width(void)
{
    if (!servo_initialized) {
        return 0;
    }
    return current_pulse_us;
}

esp_err_t servo_disable(void)
{
    if (!servo_initialized) {
        ESP_LOGE(TAG_SERVO, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Set duty to 0 (no pulses)
    esp_err_t ret = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SERVO, "Failed to disable servo: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
    if (ret != ESP_OK) {
        return ret;
    }

    servo_enabled = false;
    ESP_LOGI(TAG_SERVO, "Servo disabled (no PWM output)");
    return ESP_OK;
}

esp_err_t servo_enable(void)
{
    if (!servo_initialized) {
        ESP_LOGE(TAG_SERVO, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (servo_enabled) {
        ESP_LOGD(TAG_SERVO, "Servo already enabled");
        return ESP_OK;
    }

    // Restore last position
    ESP_LOGI(TAG_SERVO, "Enabling servo at last position (%d µs)", current_pulse_us);
    return servo_set_pulse_width(current_pulse_us);
}
