/*
 * TB6612FNG Motor Driver - Hardware Abstraction Layer
 * 
 * Low-level driver for TB6612FNG dual H-bridge motor controller (Pololu #713)
 * Uses ESP32 LEDC peripheral for high-frequency PWM control
 * 
 * Hardware: Pololu Dual TB6612FNG Motor Driver Carrier
 * - 2 motors, 1.2A continuous per channel
 * - PWM frequency: Up to 100 kHz
 * - Direct GPIO control (no I2C overhead)
 * 
 * Configuration: 19 kHz PWM @ 12-bit for silent operation with excellent control
 */

#ifndef TB6612_DRIVER_H
#define TB6612_DRIVER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// Logging tag
#define TAG_TB6612 "TB6612"

// TB6612FNG GPIO Pin Assignments (ESP32-C6 Feather)
// Motor A (Auger Motor)
#define TB6612_AIN1_GPIO        6       // Motor A direction control 1
#define TB6612_AIN2_GPIO        4       // Motor A direction control 2
#define TB6612_PWMA_GPIO        1       // Motor A PWM speed control

// Motor B (Air Pump)
#define TB6612_BIN1_GPIO        3       // Motor B direction control 1
#define TB6612_BIN2_GPIO        4       // Motor B direction control 2
#define TB6612_PWMB_GPIO        5       // Motor B PWM speed control

// Standby control (optional - can be tied to VCC for always-on)
#define TB6612_STBY_GPIO        5       // Standby pin (LOW=standby, HIGH=active)
#define TB6612_USE_STBY_PIN     0       // Set to 0 if STBY is tied to VCC

// LEDC Timer Configuration
// ESP32-C6: Max freq at 12-bit = 80 MHz / 4096 = 19.531 kHz
// Using 19 kHz for 12-bit resolution, matching PCA9685 (no scaling needed)
// Still above human hearing threshold (~17-18 kHz for most adults)
#define TB6612_LEDC_TIMER              LEDC_TIMER_0
#define TB6612_LEDC_MODE               LEDC_LOW_SPEED_MODE
#define TB6612_LEDC_DUTY_RESOLUTION    LEDC_TIMER_10_BIT    // 10-bit resolution (0-1023)
#define TB6612_LEDC_FREQUENCY          (25000)               // 19 kHz (max for 10-bit, inaudible)
#define TB6612_PWM_MAX_DUTY            ((1 << 10) - 1)       // 1023 for 10-bit

// LEDC Channel Assignments
#define TB6612_LEDC_CHANNEL_PWMA       LEDC_CHANNEL_0       // Motor A PWM
#define TB6612_LEDC_CHANNEL_PWMB       LEDC_CHANNEL_1       // Motor B PWM

// Motor Channel Definitions (matches motor_control_task.h interface)
// These are logical channel numbers that map to physical pins
#define TB6612_CHANNEL_A_IN1           0       // Motor A direction control 1
#define TB6612_CHANNEL_A_IN2           1       // Motor A direction control 2
#define TB6612_CHANNEL_A_PWM           2       // Motor A PWM control
#define TB6612_CHANNEL_B_IN1           3       // Motor B direction control 1
#define TB6612_CHANNEL_B_IN2           4       // Motor B direction control 2
#define TB6612_CHANNEL_B_PWM           5       // Motor B PWM control

// Motor Direction Control
typedef enum {
    TB6612_DIRECTION_STOP = 0,      // Both IN pins LOW (coast/brake)
    TB6612_DIRECTION_FORWARD,       // IN1=HIGH, IN2=LOW
    TB6612_DIRECTION_REVERSE,       // IN1=LOW, IN2=HIGH
    TB6612_DIRECTION_BRAKE          // Both IN pins HIGH (short brake)
} tb6612_direction_t;

/**
 * @brief Initialize TB6612FNG GPIO and LEDC peripherals
 * 
 * Configures GPIO pins for direction control and LEDC channels for PWM.
 * Sets up 25 kHz PWM frequency for silent operation.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tb6612_init(void);

/**
 * @brief Deinitialize TB6612FNG driver
 * 
 * Stops all motors, releases GPIO pins and LEDC channels.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tb6612_deinit(void);

/**
 * @brief Set PWM value for a motor channel
 * 
 * This function provides a compatible interface with the PCA9685 driver.
 * It maps logical channel numbers to physical TB6612 control pins.
 * Uses same 12-bit resolution (0-4095) as PCA9685 for direct compatibility.
 * 
 * Channel mapping (for compatibility with motor_control_task.c):
 * - Channels 8, 9, 10 → Motor A (Auger)
 *   - 8: PWMA, 9: AIN2, 10: AIN1
 * - Channels 11, 12, 13 → Motor B (Air Pump)
 *   - 13: PWMB, 12: BIN2, 11: BIN1
 * 
 * @param channel Logical channel number (0-15 for PCA9685 compatibility)
 * @param value PWM duty cycle (0-4095, 12-bit resolution)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tb6612_set_pwm_value(uint8_t channel, uint16_t value);

/**
 * @brief Set motor speed and direction
 * 
 * Higher-level function to control a motor's speed and direction.
 * 
 * @param motor Motor selection (0 = Motor A, 1 = Motor B)
 * @param direction Motor direction (forward, reverse, stop, brake)
 * @param speed PWM duty cycle (0-4095, 12-bit resolution)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tb6612_set_motor(uint8_t motor, tb6612_direction_t direction, uint16_t speed);

/**
 * @brief Enable or disable standby mode
 * 
 * When in standby, all motor outputs are disabled (high-impedance).
 * Only functional if TB6612_USE_STBY_PIN is enabled.
 * 
 * @param enable true = active (motors can run), false = standby (motors disabled)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tb6612_set_standby(bool enable);

#endif // TB6612_DRIVER_H
