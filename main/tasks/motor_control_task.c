/*
 * Motor Control Task - Application Layer
 * 
 * Controls DC motors and air pump via hardware abstraction layer
 * Hardware-agnostic motor control for coffee bean dispensing
 */

#include "motor_control_task.h"
#include "pca9685_driver.h"
#include "coffee_doser_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

// Motor command queue configuration
#define MOTOR_COMMAND_QUEUE_SIZE    10  // Maximum pending commands

// Task variables
static TaskHandle_t motor_task_handle = NULL;
static bool motor_task_running = false;
static SemaphoreHandle_t motor_state_mutex = NULL;
static motor_state_t motor_state = {0};
static QueueHandle_t motor_command_queue = NULL;

// Internal function prototypes
static void motor_task_function(void* pvParameters);
static void motor_process_command(motor_command_t* cmd);

esp_err_t motor_control_task_init(void)
{
    if (motor_task_running) {
        ESP_LOGW(TAG_MOTOR, "Motor control task already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG_MOTOR, "Initializing motor control task");

    // Initialize PWM hardware driver (PCA9685)
    esp_err_t ret = pca9685_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "Failed to initialize PWM hardware: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create state mutex
    motor_state_mutex = xSemaphoreCreateMutex();
    if (motor_state_mutex == NULL) {
        ESP_LOGE(TAG_MOTOR, "Failed to create motor state mutex");
        pca9685_deinit();
        return ESP_ERR_NO_MEM;
    }

    // Create command queue
    motor_command_queue = xQueueCreate(MOTOR_COMMAND_QUEUE_SIZE, sizeof(motor_command_t));
    if (motor_command_queue == NULL) {
        ESP_LOGE(TAG_MOTOR, "Failed to create motor command queue");
        vSemaphoreDelete(motor_state_mutex);
        pca9685_deinit();
        return ESP_ERR_NO_MEM;
    }

    // Initialize motor state
    if (xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        motor_state.initialized = true;
        motor_state.air_pump_speed = 0;
        motor_state.auger_speed = 0;
        xSemaphoreGive(motor_state_mutex);
    }

    // Create motor control task
    BaseType_t task_created = xTaskCreatePinnedToCore(
        motor_task_function,
        "motor_ctrl_task",
        MOTOR_TASK_STACK_SIZE,
        NULL,
        MOTOR_TASK_PRIORITY,
        &motor_task_handle,
        MOTOR_TASK_CORE
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG_MOTOR, "Failed to create motor control task");
        vSemaphoreDelete(motor_state_mutex);
        vQueueDelete(motor_command_queue);
        pca9685_deinit();
        return ESP_FAIL;
    }

    motor_task_running = true;
    ESP_LOGI(TAG_MOTOR, "Motor control task initialized successfully");
    return ESP_OK;
}

esp_err_t motor_control_task_deinit(void)
{
    if (!motor_task_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG_MOTOR, "Deinitializing motor control task");

    // Delete task
    if (motor_task_handle != NULL) {
        vTaskDelete(motor_task_handle);
        motor_task_handle = NULL;
    }

    // Turn off all motors
    motor_air_pump_set_speed(0);
    motor_auger_set_speed(0);

    // Delete command queue
    if (motor_command_queue != NULL) {
        vQueueDelete(motor_command_queue);
        motor_command_queue = NULL;
    }

    // Delete mutex
    if (motor_state_mutex != NULL) {
        vSemaphoreDelete(motor_state_mutex);
        motor_state_mutex = NULL;
    }

    // Deinitialize PWM hardware
    pca9685_deinit();

    motor_task_running = false;
    ESP_LOGI(TAG_MOTOR, "Motor control task deinitialized");
    return ESP_OK;
}

static void motor_task_function(void* pvParameters)
{
    (void)pvParameters;
    
    ESP_LOGI(TAG_MOTOR, "Motor control task started");

    uint32_t iteration = 0;
    motor_command_t cmd;
    
    while (1) {
        // Check for commands in the queue (with 100ms timeout)
        if (xQueueReceive(motor_command_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Process the received command
            motor_process_command(&cmd);
        }
        
        // Periodic "hello" message every 5 seconds
        if (iteration % 50 == 0) {
            ESP_LOGD(TAG_MOTOR, "Motor control task running - iteration %lu", iteration);
            
            // Log current state
            motor_state_t state;
            if (motor_get_state(&state) == ESP_OK) {
                ESP_LOGD(TAG_MOTOR, "State: air_pump_speed=%u, auger_speed=%u", 
                         state.air_pump_speed, state.auger_speed);
            }
        }
        
        iteration++;
    }
}

static void motor_process_command(motor_command_t* cmd)
{
    if (cmd == NULL) {
        return;
    }
    
    ESP_LOGI(TAG_MOTOR, "Received command: type=%d, parameter=%lu", cmd->command, cmd->parameter);
    
    switch (cmd->command) {
        case MOTOR_CMD_DISPENSE:
            ESP_LOGI(TAG_MOTOR, "========================================");
            ESP_LOGI(TAG_MOTOR, "DISPENSE command - parameter=%lu grams", cmd->parameter);
            ESP_LOGI(TAG_MOTOR, "Using DC motor on M1 for auger");
            
            // DC Motor Configuration
            const uint16_t auger_speed = 2048;  // 50% PWM (~0.05-0.1A)
            const uint32_t run_time_ms = 10000;  // 10 seconds
            
            // Ramp up
            ESP_LOGI(TAG_MOTOR, "Ramping up...");
            motor_set_pwm(MOTOR_AUGER_IN1, 4095);  // Forward direction
            motor_set_pwm(MOTOR_AUGER_IN2, 0);
            for (int s = 0; s <= auger_speed; s += 256) {
                motor_set_pwm(MOTOR_AUGER_PWM, s);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            motor_set_pwm(MOTOR_AUGER_PWM, auger_speed);
            
            // Run
            ESP_LOGI(TAG_MOTOR, "Dispensing...");
            vTaskDelay(pdMS_TO_TICKS(run_time_ms));
            
            // Ramp down
            ESP_LOGI(TAG_MOTOR, "Ramping down...");
            for (int s = auger_speed; s >= 0; s -= 256) {
                motor_set_pwm(MOTOR_AUGER_PWM, s);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            
            // Stop (coast mode: all pins low)
            motor_set_pwm(MOTOR_AUGER_PWM, 0);
            motor_set_pwm(MOTOR_AUGER_IN1, 0);
            motor_set_pwm(MOTOR_AUGER_IN2, 0);
            
            ESP_LOGI(TAG_MOTOR, "Dispense complete");
            ESP_LOGI(TAG_MOTOR, "========================================");
            break;
            
        case MOTOR_CMD_STOP:
            ESP_LOGI(TAG_MOTOR, "STOP command received - stopping all motors");
            // Turn off all motors
            motor_auger_set_speed(0);
            motor_air_pump_set_speed(0);
            break;
            
        case MOTOR_CMD_HOME:
            ESP_LOGI(TAG_MOTOR, "HOME command received");
            // Not applicable for DC motors
            break;
            
        case MOTOR_CMD_AIR_PUMP_SET:
            ESP_LOGI(TAG_MOTOR, "AIR_PUMP_SET command received - speed=%lu%%", cmd->parameter);
            motor_air_pump_set_speed((uint8_t)(cmd->parameter & 0xFF));
            break;
            
        case MOTOR_CMD_AUGER_SET:
            ESP_LOGI(TAG_MOTOR, "AUGER_SET command received - speed=%lu%%", cmd->parameter);
            motor_auger_set_speed((uint8_t)(cmd->parameter & 0xFF));
            break;
            
        default:
            ESP_LOGW(TAG_MOTOR, "Unknown command type: %d", cmd->command);
            break;
    }
}

esp_err_t motor_get_state(motor_state_t* state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(state, &motor_state, sizeof(motor_state_t));
        xSemaphoreGive(motor_state_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t motor_set_pwm(uint8_t channel, uint16_t value)
{
    if (!motor_task_running) {
        ESP_LOGE(TAG_MOTOR, "Motor control task not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Use hardware abstraction layer
    return pca9685_set_pwm_value(channel, value);
}

esp_err_t motor_air_pump_set_speed(uint8_t speed_percent)
{
    if (!motor_task_running) {
        ESP_LOGE(TAG_MOTOR, "Motor control task not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp to 0-100 range
    if (speed_percent > 100) {
        speed_percent = 100;
    }
    
    // Apply minimum speed threshold to overcome static friction
    // If speed is between 1-MOTOR_MIN_SPEED_PERCENT, boost to minimum
    uint8_t actual_speed = speed_percent;
    if (speed_percent > 0 && speed_percent < MOTOR_MIN_SPEED_PERCENT) {
        actual_speed = MOTOR_MIN_SPEED_PERCENT;
        ESP_LOGD(TAG_MOTOR, "Air pump: Boosting %u%% to minimum %u%% for startup", 
                 speed_percent, actual_speed);
    }
    
    // Convert percentage to 12-bit PWM value (0-4095)
    uint16_t pwm_value = (uint16_t)((actual_speed * 4095) / 100);
    
    ESP_LOGI(TAG_MOTOR, "Air pump (M2): %u%% requested, %u%% applied (PWM=%u)", 
             speed_percent, actual_speed, pwm_value);
    
    // Use M2 for air pump control
    esp_err_t ret = ESP_OK;
    if (pwm_value > 0) {
        // Set direction to forward
        motor_set_pwm(MOTOR_AIR_PUMP_IN1, 4095);
        motor_set_pwm(MOTOR_AIR_PUMP_IN2, 0);
        ret = motor_set_pwm(MOTOR_AIR_PUMP_PWM, pwm_value);
    } else {
        // Turn off M2 (coast mode)
        motor_set_pwm(MOTOR_AIR_PUMP_PWM, 0);
        motor_set_pwm(MOTOR_AIR_PUMP_IN1, 0);
        ret = motor_set_pwm(MOTOR_AIR_PUMP_IN2, 0);
    }
    
    if (ret == ESP_OK && xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        motor_state.air_pump_speed = pwm_value;
        xSemaphoreGive(motor_state_mutex);
    }
    
    return ret;
}

esp_err_t motor_auger_set_speed(uint8_t speed_percent)
{
    if (!motor_task_running) {
        ESP_LOGE(TAG_MOTOR, "Motor control task not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp to 0-100 range
    if (speed_percent > 100) {
        speed_percent = 100;
    }
    
    // Apply minimum speed threshold to overcome static friction
    // If speed is between 1-MOTOR_MIN_SPEED_PERCENT, boost to minimum
    uint8_t actual_speed = speed_percent;
    if (speed_percent > 0 && speed_percent < MOTOR_MIN_SPEED_PERCENT) {
        actual_speed = MOTOR_MIN_SPEED_PERCENT;
        ESP_LOGD(TAG_MOTOR, "Auger motor: Boosting %u%% to minimum %u%% for startup", 
                 speed_percent, actual_speed);
    }
    
    // Convert percentage to 12-bit PWM value (0-4095)
    uint16_t pwm_value = (uint16_t)((actual_speed * 4095) / 100);
    
    ESP_LOGI(TAG_MOTOR, "Auger motor (M1): %u%% requested, %u%% applied (PWM=%u)", 
             speed_percent, actual_speed, pwm_value);
    
    // Use M1 for auger motor control
    esp_err_t ret = ESP_OK;
    if (pwm_value > 0) {
        // Set direction to forward
        motor_set_pwm(MOTOR_AUGER_IN1, 4095);
        motor_set_pwm(MOTOR_AUGER_IN2, 0);
        ret = motor_set_pwm(MOTOR_AUGER_PWM, pwm_value);
    } else {
        // Turn off M1 (coast mode)
        motor_set_pwm(MOTOR_AUGER_PWM, 0);
        motor_set_pwm(MOTOR_AUGER_IN1, 0);
        ret = motor_set_pwm(MOTOR_AUGER_IN2, 0);
    }
    
    if (ret == ESP_OK && xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        motor_state.auger_speed = pwm_value;
        xSemaphoreGive(motor_state_mutex);
    }
    
    return ret;
}

esp_err_t motor_send_command(motor_command_type_t command, uint32_t parameter)
{
    if (!motor_task_running) {
        ESP_LOGE(TAG_MOTOR, "Motor control task not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (motor_command_queue == NULL) {
        ESP_LOGE(TAG_MOTOR, "Motor command queue not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    motor_command_t cmd = {
        .command = command,
        .parameter = parameter
    };
    
    // Send command to queue (with 100ms timeout)
    if (xQueueSend(motor_command_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG_MOTOR, "Failed to send command to queue (queue full?)");
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGD(TAG_MOTOR, "Command queued: type=%d, parameter=%lu", command, parameter);
    return ESP_OK;
}
