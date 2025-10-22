/*
 * Motor Control Task - Application Layer
 * 
 * Controls DC motors and air pump via hardware abstraction layer
 * Hardware-agnostic motor control for coffee bean dispensing
 */

#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "pca9685_driver.h"

// Logging tag
#define TAG_MOTOR "MOTOR_CTRL"

// Motor Performance Parameters
#define MOTOR_MIN_SPEED_PERCENT     1      // Minimum speed to overcome static friction

// Motor Assignment Macros (for readability)
#define MOTOR_AUGER_PWM             MOTOR_M1_PWM
#define MOTOR_AUGER_IN1             MOTOR_M1_IN1
#define MOTOR_AUGER_IN2             MOTOR_M1_IN2

#define MOTOR_AIR_PUMP_PWM          MOTOR_M2_PWM
#define MOTOR_AIR_PUMP_IN1          MOTOR_M2_IN1
#define MOTOR_AIR_PUMP_IN2          MOTOR_M2_IN2

// Task Configuration
#define MOTOR_TASK_STACK_SIZE       4096
#define MOTOR_TASK_PRIORITY         2       // Lower priority than ADC/Display
#define MOTOR_TASK_CORE             0       // Run on core 0

// Motor command types
typedef enum {
    MOTOR_CMD_DISPENSE = 0,     // Start dispensing operation (from double-click)
    MOTOR_CMD_STOP,             // Stop current operation
    MOTOR_CMD_HOME,             // Reserved for future use
    MOTOR_CMD_AIR_PUMP_SET,     // Set air pump speed (parameter = percent 0-100)
    MOTOR_CMD_AUGER_SET         // Set auger motor speed (parameter = percent 0-100)
} motor_command_type_t;

// Motor command message structure
typedef struct {
    motor_command_type_t command;
    uint32_t parameter;         // Optional parameter (e.g., speed percent 0-100, weight target)
} motor_command_t;

// Motor state structure
typedef struct {
    bool initialized;
    uint16_t air_pump_speed;    // Current air pump PWM speed (0-4095)
    uint16_t auger_speed;       // Current auger motor PWM speed (0-4095)
} motor_state_t;

/**
 * @brief Initialize motor control task
 * 
 * Sets up hardware PWM controller and starts the motor control task
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
 * @brief Set PWM value for a specific channel (hardware abstraction)
 * 
 * @param channel PWM channel (hardware-specific)
 * @param value PWM value (0-4095, 12-bit). Use 4095 for digital HIGH, 0 for LOW
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_set_pwm(uint8_t channel, uint16_t value);

/**
 * @brief Control air pump with PWM speed control
 * 
 * @param speed_percent Speed as percentage (0-100). 0 = off, 100 = full speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_air_pump_set_speed(uint8_t speed_percent);

/**
 * @brief Control auger motor with PWM speed control
 * 
 * @param speed_percent Speed as percentage (0-100). 0 = off, 100 = full speed
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_auger_set_speed(uint8_t speed_percent);

/**
 * @brief Send a command to the motor control task
 * 
 * @param command Command type to send
 * @param parameter Optional parameter for the command
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_send_command(motor_command_type_t command, uint32_t parameter);

#endif // MOTOR_CONTROL_TASK_H
