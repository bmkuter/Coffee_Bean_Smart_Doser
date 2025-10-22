/*
 * PCA9685 PWM Driver - Hardware Abstraction Layer
 * 
 * Low-level driver for PCA9685 16-channel 12-bit PWM controller
 * Communicates via I2C bus
 */

#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// Logging tag
#define TAG_PCA9685 "PCA9685"

// PCA9685 I2C Address (default, can be modified with address jumpers)
#define PCA9685_I2C_ADDRESS         0x60

// PCA9685 Register Definitions
#define PCA9685_REG_MODE1           0x00    // Mode register 1
#define PCA9685_REG_MODE2           0x01    // Mode register 2
#define PCA9685_REG_SUBADR1         0x02    // I2C subaddress 1
#define PCA9685_REG_SUBADR2         0x03    // I2C subaddress 2
#define PCA9685_REG_SUBADR3         0x04    // I2C subaddress 3
#define PCA9685_REG_ALLCALLADR      0x05    // All call address
#define PCA9685_REG_LED0_ON_L       0x06    // LED0 output ON time, low byte
#define PCA9685_REG_LED0_ON_H       0x07    // LED0 output ON time, high byte
#define PCA9685_REG_LED0_OFF_L      0x08    // LED0 output OFF time, low byte
#define PCA9685_REG_LED0_OFF_H      0x09    // LED0 output OFF time, high byte
#define PCA9685_REG_ALL_LED_ON_L    0xFA    // All LED ON time, low byte
#define PCA9685_REG_ALL_LED_ON_H    0xFB    // All LED ON time, high byte
#define PCA9685_REG_ALL_LED_OFF_L   0xFC    // All LED OFF time, low byte
#define PCA9685_REG_ALL_LED_OFF_H   0xFD    // All LED OFF time, high byte
#define PCA9685_REG_PRESCALE        0xFE    // Prescaler for PWM output frequency
#define PCA9685_REG_TESTMODE        0xFF    // Test mode register

// PCA9685 Mode1 Register Bits
#define PCA9685_MODE1_RESTART       0x80    // Restart enabled
#define PCA9685_MODE1_EXTCLK        0x40    // Use external clock
#define PCA9685_MODE1_AI            0x20    // Auto-increment enabled
#define PCA9685_MODE1_SLEEP         0x10    // Low power mode (oscillator off)
#define PCA9685_MODE1_SUB1          0x08    // Respond to I2C subaddress 1
#define PCA9685_MODE1_SUB2          0x04    // Respond to I2C subaddress 2
#define PCA9685_MODE1_SUB3          0x02    // Respond to I2C subaddress 3
#define PCA9685_MODE1_ALLCALL       0x01    // Respond to all call I2C address

// PCA9685 Mode2 Register Bits
#define PCA9685_MODE2_INVRT         0x10    // Invert output logic
#define PCA9685_MODE2_OCH           0x08    // Outputs change on STOP vs ACK
#define PCA9685_MODE2_OUTDRV        0x04    // Totem pole vs open-drain
#define PCA9685_MODE2_OUTNE1        0x02    // Output enable mode bit 1
#define PCA9685_MODE2_OUTNE0        0x01    // Output enable mode bit 0

// PWM Constants
#define PCA9685_PWM_FREQUENCY       1526    // Maximum PWM frequency in Hz (prescale=3)
#define PCA9685_CLOCK_FREQ          25000000  // Internal oscillator frequency (25 MHz)
#define PCA9685_PWM_RESOLUTION      4096    // 12-bit PWM resolution
#define PCA9685_NUM_CHANNELS        16      // Number of PWM channels

// Motor Channel Definitions (per Adafruit Motor Shield V2 mapping)
// These map to PWM controller channels - specific to current hardware
// DC Motors: M1 (auger motor), M2 (air pump), M3, M4
#define MOTOR_M1_PWM                8       // Motor 1 PWM pin (AUGER MOTOR)
#define MOTOR_M1_IN2                9       // Motor 1 IN2 pin
#define MOTOR_M1_IN1                10      // Motor 1 IN1 pin
#define MOTOR_M2_PWM                13      // Motor 2 PWM pin (AIR PUMP)
#define MOTOR_M2_IN2                12      // Motor 2 IN2 pin
#define MOTOR_M2_IN1                11      // Motor 2 IN1 pin

#define MOTOR_M3_PWM                2       // Motor 3 PWM pin (unused)
#define MOTOR_M3_IN2                3       // Motor 3 IN2 pin
#define MOTOR_M3_IN1                4       // Motor 3 IN1 pin
#define MOTOR_M4_PWM                7       // Motor 4 PWM pin (unused)
#define MOTOR_M4_IN2                6       // Motor 4 IN2 pin
#define MOTOR_M4_IN1                5       // Motor 4 IN1 pin

/**
 * @brief Initialize PCA9685 I2C communication
 * 
 * Adds PCA9685 device to the shared I2C bus
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pca9685_init(void);

/**
 * @brief Deinitialize PCA9685 driver
 * 
 * Removes PCA9685 device from I2C bus and cleans up resources
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pca9685_deinit(void);

/**
 * @brief Reset PCA9685 to default state
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pca9685_reset(void);

/**
 * @brief Set PWM frequency for all channels
 * 
 * @param freq_hz Desired PWM frequency in Hz (24-1526 Hz recommended)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pca9685_set_pwm_freq(uint16_t freq_hz);

/**
 * @brief Set PWM duty cycle for a specific channel
 * 
 * @param channel PWM channel (0-15)
 * @param on_time 12-bit value for ON time (0-4095)
 * @param off_time 12-bit value for OFF time (0-4095)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on_time, uint16_t off_time);

/**
 * @brief Set PWM duty cycle using simplified interface
 * 
 * @param channel PWM channel (0-15)
 * @param value 12-bit PWM value (0-4095), on_time is always 0
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pca9685_set_pwm_value(uint8_t channel, uint16_t value);

/**
 * @brief Turn off all PWM outputs
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pca9685_all_off(void);

/**
 * @brief Read a register from PCA9685
 * 
 * @param reg_addr Register address
 * @param value Pointer to store read value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pca9685_read_register(uint8_t reg_addr, uint8_t* value);

/**
 * @brief Write a register to PCA9685
 * 
 * @param reg_addr Register address
 * @param value Value to write
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t pca9685_write_register(uint8_t reg_addr, uint8_t value);

#endif // PCA9685_DRIVER_H
