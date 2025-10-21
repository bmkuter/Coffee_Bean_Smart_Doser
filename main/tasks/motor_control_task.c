/*
 * Motor Control Task - PCA9685 PWM Motor Controller
 * 
 * Controls DC motors and air pump via Adafruit Motor FeatherWing
 * Uses PCA9685 I2C PWM controller with TB6612 motor drivers
 */

#include "motor_control_task.h"
#include "shared_i2c_bus.h"
#include "coffee_doser_config.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

// I2C timeout constant
#define I2C_TIMEOUT_MS          1000    // 1 second timeout for I2C operations

// Motor command queue configuration
#define MOTOR_COMMAND_QUEUE_SIZE    10  // Maximum pending commands

// Task variables
static TaskHandle_t motor_task_handle = NULL;
static bool motor_task_running = false;
static SemaphoreHandle_t motor_state_mutex = NULL;
static motor_state_t motor_state = {0};
static QueueHandle_t motor_command_queue = NULL;

// I2C device handle
static i2c_master_dev_handle_t pca9685_i2c_dev_handle = NULL;

// Internal function prototypes
static void motor_task_function(void* pvParameters);
static void motor_process_command(motor_command_t* cmd);
static esp_err_t pca9685_i2c_init(void);
static esp_err_t pca9685_device_init(void);
static esp_err_t pca9685_write_register(uint8_t reg_addr, uint8_t value);
static esp_err_t pca9685_read_register(uint8_t reg_addr, uint8_t* value);
static esp_err_t pca9685_set_pwm_freq(uint16_t freq_hz);
static esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on_time, uint16_t off_time);
static esp_err_t pca9685_reset(void);

esp_err_t motor_control_task_init(void)
{
    if (motor_task_running) {
        ESP_LOGW(TAG_MOTOR, "Motor control task already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG_MOTOR, "Initializing motor control task");

    // Initialize I2C for PCA9685
    esp_err_t ret = pca9685_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create state mutex
    motor_state_mutex = xSemaphoreCreateMutex();
    if (motor_state_mutex == NULL) {
        ESP_LOGE(TAG_MOTOR, "Failed to create motor state mutex");
        return ESP_ERR_NO_MEM;
    }

    // Create command queue
    motor_command_queue = xQueueCreate(MOTOR_COMMAND_QUEUE_SIZE, sizeof(motor_command_t));
    if (motor_command_queue == NULL) {
        ESP_LOGE(TAG_MOTOR, "Failed to create motor command queue");
        vSemaphoreDelete(motor_state_mutex);
        return ESP_ERR_NO_MEM;
    }

    // Initialize PCA9685 device
    ret = pca9685_device_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "Failed to initialize PCA9685 device: %s", esp_err_to_name(ret));
        vSemaphoreDelete(motor_state_mutex);
        return ret;
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

    // Delete I2C device
    if (pca9685_i2c_dev_handle != NULL) {
        i2c_master_bus_rm_device(pca9685_i2c_dev_handle);
        pca9685_i2c_dev_handle = NULL;
    }

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
            ESP_LOGI(TAG_MOTOR, "Using DC motor on M2 for auger");
            
            // DC Motor Configuration
            const uint16_t auger_speed = 2048;  // 50% PWM (~0.05-0.1A)
            const uint32_t run_time_ms = 10000;  // 10 seconds
            
            // Ramp up
            ESP_LOGI(TAG_MOTOR, "Ramping up...");
            motor_set_pwm(MOTOR_M2_IN1, 4095);  // Forward direction
            motor_set_pwm(MOTOR_M2_IN2, 0);
            for (int s = 0; s <= auger_speed; s += 256) {
                motor_set_pwm(MOTOR_M2_PWM, s);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            motor_set_pwm(MOTOR_M2_PWM, auger_speed);
            
            // Run
            ESP_LOGI(TAG_MOTOR, "Dispensing...");
            vTaskDelay(pdMS_TO_TICKS(run_time_ms));
            
            // Ramp down
            ESP_LOGI(TAG_MOTOR, "Ramping down...");
            for (int s = auger_speed; s >= 0; s -= 256) {
                motor_set_pwm(MOTOR_M2_PWM, s);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            
            // Stop (coast mode: all pins low)
            motor_set_pwm(MOTOR_M2_PWM, 0);
            motor_set_pwm(MOTOR_M2_IN1, 0);
            motor_set_pwm(MOTOR_M2_IN2, 0);
            
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

static esp_err_t pca9685_i2c_init(void)
{
    ESP_LOGI(TAG_MOTOR, "Initializing I2C for PCA9685 at address 0x%02X", PCA9685_I2C_ADDRESS);

    // Get shared I2C bus handle
    i2c_master_bus_handle_t bus_handle = shared_i2c_get_bus_handle();
    if (bus_handle == NULL) {
        ESP_LOGE(TAG_MOTOR, "Failed to get shared I2C bus handle");
        return ESP_ERR_INVALID_STATE;
    }

    // Configure I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9685_I2C_ADDRESS,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &pca9685_i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "Failed to add PCA9685 device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_MOTOR, "PCA9685 I2C device initialized successfully");
    return ESP_OK;
}

static esp_err_t pca9685_device_init(void)
{
    ESP_LOGI(TAG_MOTOR, "Initializing PCA9685 PWM controller");
    
    // Reset PCA9685 to default state
    esp_err_t ret = pca9685_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "Failed to reset PCA9685");
        return ret;
    }
    
    // Wait for oscillator to stabilize
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set PWM frequency to 1.6kHz (default for motor shield)
    ret = pca9685_set_pwm_freq(PCA9685_PWM_FREQUENCY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "Failed to set PWM frequency");
        return ret;
    }
    
    // Configure MODE2 register for totem-pole output (better for motor drivers)
    ret = pca9685_write_register(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "Failed to configure MODE2 register");
        return ret;
    }
    
    // Turn off all PWM outputs initially
    ret = pca9685_write_register(PCA9685_REG_ALL_LED_OFF_L, 0x00);
    ret |= pca9685_write_register(PCA9685_REG_ALL_LED_OFF_H, 0x10);  // Full OFF
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MOTOR, "Failed to turn off all PWM outputs");
        return ret;
    }
    
    ESP_LOGI(TAG_MOTOR, "PCA9685 device initialized successfully");
    return ESP_OK;
}

static esp_err_t pca9685_reset(void)
{
    ESP_LOGI(TAG_MOTOR, "Resetting PCA9685");
    
    // Send software reset to MODE1 register
    esp_err_t ret = pca9685_write_register(PCA9685_REG_MODE1, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for reset to complete
    return ESP_OK;
}

static esp_err_t pca9685_write_register(uint8_t reg_addr, uint8_t value)
{
    if (pca9685_i2c_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t write_buf[2] = {reg_addr, value};
    esp_err_t ret = i2c_master_transmit(pca9685_i2c_dev_handle, write_buf, sizeof(write_buf), 
                                        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_MOTOR, "Failed to write register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t pca9685_read_register(uint8_t reg_addr, uint8_t* value)
{
    if (pca9685_i2c_dev_handle == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_transmit_receive(pca9685_i2c_dev_handle, 
                                                &reg_addr, 1, 
                                                value, 1, 
                                                I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_MOTOR, "Failed to read register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t pca9685_set_pwm_freq(uint16_t freq_hz)
{
    ESP_LOGI(TAG_MOTOR, "Setting PWM frequency to %d Hz", freq_hz);
    
    // Calculate prescale value: prescale = round(25MHz / (4096 * freq)) - 1
    float prescale_val = ((float)PCA9685_CLOCK_FREQ / (PCA9685_PWM_RESOLUTION * (float)freq_hz)) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_val + 0.5f);  // Round to nearest integer
    
    ESP_LOGI(TAG_MOTOR, "Calculated prescale value: %d (from %.2f)", prescale, prescale_val);
    
    // Read current MODE1 register
    uint8_t old_mode;
    esp_err_t ret = pca9685_read_register(PCA9685_REG_MODE1, &old_mode);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Enter sleep mode to change prescale (oscillator must be off)
    uint8_t new_mode = (old_mode & 0x7F) | PCA9685_MODE1_SLEEP;
    ret = pca9685_write_register(PCA9685_REG_MODE1, new_mode);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Write prescale value
    ret = pca9685_write_register(PCA9685_REG_PRESCALE, prescale);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Restore previous MODE1 register (exit sleep)
    ret = pca9685_write_register(PCA9685_REG_MODE1, old_mode);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for oscillator to stabilize
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Enable restart if it was set before
    ret = pca9685_write_register(PCA9685_REG_MODE1, old_mode | PCA9685_MODE1_RESTART);
    
    return ret;
}

static esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on_time, uint16_t off_time)
{
    if (channel >= 16) {
        ESP_LOGE(TAG_MOTOR, "Invalid PWM channel: %d (must be 0-15)", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate times are within 12-bit range
    on_time &= 0x0FFF;
    off_time &= 0x0FFF;
    
    // Calculate register addresses for this channel
    uint8_t reg_base = PCA9685_REG_LED0_ON_L + (4 * channel);
    
    esp_err_t ret = ESP_OK;
    ret |= pca9685_write_register(reg_base, on_time & 0xFF);          // ON_L
    ret |= pca9685_write_register(reg_base + 1, on_time >> 8);        // ON_H
    ret |= pca9685_write_register(reg_base + 2, off_time & 0xFF);     // OFF_L
    ret |= pca9685_write_register(reg_base + 3, off_time >> 8);       // OFF_H
    
    return ret;
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
    
    // Clamp value to 12-bit range
    if (value > 4095) {
        value = 4095;
    }
    
    // Set PWM with on_time=0 and off_time=value for normal PWM operation
    return pca9685_set_pwm(channel, 0, value);
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
    
    ESP_LOGI(TAG_MOTOR, "Air pump (M1): %u%% requested, %u%% applied (PWM=%u)", 
             speed_percent, actual_speed, pwm_value);
    
    // Use M1 for air pump control
    esp_err_t ret = ESP_OK;
    if (pwm_value > 0) {
        // Set direction to forward
        motor_set_pwm(MOTOR_M1_IN1, 4095);
        motor_set_pwm(MOTOR_M1_IN2, 0);
        ret = motor_set_pwm(MOTOR_M1_PWM, pwm_value);
    } else {
        // Turn off M1 (coast mode)
        motor_set_pwm(MOTOR_M1_PWM, 0);
        motor_set_pwm(MOTOR_M1_IN1, 0);
        ret = motor_set_pwm(MOTOR_M1_IN2, 0);
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
    
    ESP_LOGI(TAG_MOTOR, "Auger motor (M2): %u%% requested, %u%% applied (PWM=%u)", 
             speed_percent, actual_speed, pwm_value);
    
    // Use M2 for auger motor control
    esp_err_t ret = ESP_OK;
    if (pwm_value > 0) {
        // Set direction to forward
        motor_set_pwm(MOTOR_M2_IN1, 4095);
        motor_set_pwm(MOTOR_M2_IN2, 0);
        ret = motor_set_pwm(MOTOR_M2_PWM, pwm_value);
    } else {
        // Turn off M2 (coast mode)
        motor_set_pwm(MOTOR_M2_PWM, 0);
        motor_set_pwm(MOTOR_M2_IN1, 0);
        ret = motor_set_pwm(MOTOR_M2_IN2, 0);
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
