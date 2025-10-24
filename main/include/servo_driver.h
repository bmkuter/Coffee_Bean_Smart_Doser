/*
 * Servo Driver Header - PWM Servo Control
 * 
 * Standard servo control using LEDC PWM on GPIO5
 * - 50 Hz PWM frequency (20ms period)
 * - 1000-2000 µs pulse width (typical servo range)
 * - Supports angle (0-180°) or microsecond control
 */

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "esp_err.h"
#include <stdint.h>

// Tag for ESP_LOG
#define TAG_SERVO "SERVO"

// Servo configuration
#define SERVO_GPIO_PIN          9               // GPIO 9 for servo control
#define SERVO_LEDC_TIMER        LEDC_TIMER_1    // Use timer 1 (motor uses timer 0)
#define SERVO_LEDC_CHANNEL      LEDC_CHANNEL_2  // Use channel 2 (avoid motor channels 0-5)
#define SERVO_LEDC_MODE         LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_RESOLUTION   LEDC_TIMER_14_BIT  // 14-bit for precise servo control
#define SERVO_FREQUENCY_HZ      50              // Standard servo frequency (20ms period)

// Pulse width limits (in microseconds)
// Many servos use wider range than 1000-2000µs for full 180° travel
#define SERVO_MIN_PULSE_US      500             // Minimum pulse width (0°) - wider range
#define SERVO_MAX_PULSE_US      2500            // Maximum pulse width (180°) - wider range
#define SERVO_CENTER_PULSE_US   1500            // Center position (90°)

// Angle limits
#define SERVO_MIN_ANGLE         0                       // Minimum angle (degrees)
#define SERVO_MAX_ANGLE         180                     // Maximum angle (degrees)
#define SERVO_OPEN_ANGLE         36                     // Open angle (degrees)
#define SERVO_CLOSE_ANGLE        87                     // Close angle (degrees)

/**
 * @brief Initialize servo driver
 * 
 * Configures LEDC PWM for servo control on GPIO5
 * 
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_STATE if already initialized
 *         Other ESP_ERR_* on hardware configuration failure
 */
esp_err_t servo_init(void);

/**
 * @brief Deinitialize servo driver
 * 
 * Stops PWM and frees resources
 * 
 * @return ESP_OK on success
 */
esp_err_t servo_deinit(void);

/**
 * @brief Set servo position by angle
 * 
 * @param angle Angle in degrees (0-180)
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_ARG if angle out of range
 *         ESP_ERR_INVALID_STATE if servo not initialized
 */
esp_err_t servo_set_angle(uint8_t angle);

/**
 * @brief Set servo position by pulse width
 * 
 * @param pulse_us Pulse width in microseconds (1000-2000)
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_ARG if pulse width out of range
 *         ESP_ERR_INVALID_STATE if servo not initialized
 */
esp_err_t servo_set_pulse_width(uint16_t pulse_us);

/**
 * @brief Get current servo angle
 * 
 * @return Current angle in degrees (0-180), or 0xFF if not initialized
 */
uint8_t servo_get_angle(void);

/**
 * @brief Get current servo pulse width
 * 
 * @return Current pulse width in microseconds, or 0 if not initialized
 */
uint16_t servo_get_pulse_width(void);

/**
 * @brief Disable servo PWM output
 * 
 * Stops sending pulses to servo (servo will lose holding torque)
 * 
 * @return ESP_OK on success
 */
esp_err_t servo_disable(void);

/**
 * @brief Enable servo PWM output
 * 
 * Resumes sending pulses to servo at last set position
 * 
 * @return ESP_OK on success
 */
esp_err_t servo_enable(void);

#endif // SERVO_DRIVER_H
