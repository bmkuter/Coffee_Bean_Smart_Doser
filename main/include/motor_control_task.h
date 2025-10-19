/*
 * Motor Control Task - PCA9685 PWM Motor Controller
 * 
 * Controls DC motors and air pump via Adafruit Motor FeatherWing
 * Uses PCA9685 I2C PWM controller with TB6612 motor drivers
 */

#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// Logging tag
#define TAG_MOTOR "MOTOR_CTRL"

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
#define PCA9685_PWM_FREQUENCY       1600    // Default PWM frequency in Hz (1.6kHz)
#define PCA9685_CLOCK_FREQ          25000000  // Internal oscillator frequency (25 MHz)
#define PCA9685_PWM_RESOLUTION      4096    // 12-bit PWM resolution

// Motor Channel Definitions (per Adafruit Motor Shield V2 mapping)
// DC Motors: M1, M2, M3 (auger), M4 (vacuum pump)
#define MOTOR_M1_PWM                8       // Motor 1 PWM pin on PCA9685
#define MOTOR_M1_IN2                9       // Motor 1 IN2 pin
#define MOTOR_M1_IN1                10      // Motor 1 IN1 pin
#define MOTOR_M2_PWM                13      // Motor 2 PWM pin
#define MOTOR_M2_IN2                12      // Motor 2 IN2 pin
#define MOTOR_M2_IN1                11      // Motor 2 IN1 pin

#define MOTOR_M3_PWM                2       // Motor 3 PWM pin (auger motor)
#define MOTOR_M3_IN2                3       // Motor 3 IN2 pin
#define MOTOR_M3_IN1                4       // Motor 3 IN1 pin
#define MOTOR_M4_PWM                7       // Motor 4 PWM pin (vacuum pump)
#define MOTOR_M4_IN2                6       // Motor 4 IN2 pin
#define MOTOR_M4_IN1                5       // Motor 4 IN1 pin

// Task Configuration
#define MOTOR_TASK_STACK_SIZE       4096
#define MOTOR_TASK_PRIORITY         2       // Lower priority than ADC/Display
#define MOTOR_TASK_CORE             0       // Run on core 0

// Motor command types
typedef enum {
    MOTOR_CMD_DISPENSE = 0,     // Start dispensing operation (from double-click)
    MOTOR_CMD_STOP,             // Stop current operation
    MOTOR_CMD_HOME,             // Reserved for future use
    MOTOR_CMD_AIR_PUMP_ON,      // Turn on air pump
    MOTOR_CMD_AIR_PUMP_OFF      // Turn off air pump
} motor_command_type_t;

// Motor command message structure
typedef struct {
    motor_command_type_t command;
    uint32_t parameter;         // Optional parameter (e.g., steps to move, weight target)
} motor_command_t;

// Motor state structure
typedef struct {
    bool initialized;
    bool air_pump_enabled;
} motor_state_t;

/**
 * @brief Initialize motor control task
 * 
 * Sets up I2C communication with PCA9685 and starts the motor control task
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_control_task_init(void);

/**
 * @brief Deinitialize motor control task
 * 
 * Stops the motor control task and releases resources
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_control_task_deinit(void);

/**
 * @brief Get current motor state
 * 
 * @param state Pointer to motor_state_t structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_get_state(motor_state_t* state);

/**
 * @brief Set PWM value for a specific PCA9685 channel
 * 
 * @param channel PWM channel (0-15)
 * @param value PWM value (0-4095, 12-bit). Use 4095 for digital HIGH, 0 for LOW
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_set_pwm(uint8_t channel, uint16_t value);

/**
 * @brief Control vacuum pump on M4 (on/off)
 * 
 * @param enable true to turn on pump at full speed, false to turn off
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_air_pump_control(bool enable);

/**
 * @brief Send a command to the motor control task
 * 
 * @param command Command type to send
 * @param parameter Optional parameter for the command
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_send_command(motor_command_type_t command, uint32_t parameter);

#endif // MOTOR_CONTROL_TASK_H
