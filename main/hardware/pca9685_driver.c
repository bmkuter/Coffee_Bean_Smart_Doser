/*
 * PCA9685 PWM Driver - Hardware Abstraction Layer
 * 
 * Low-level driver for PCA9685 16-channel 12-bit PWM controller
 * Communicates via I2C bus
 */

#include "pca9685_driver.h"
#include "shared_i2c_bus.h"
#include "coffee_doser_config.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// I2C timeout constant
#define I2C_TIMEOUT_MS          1000    // 1 second timeout for I2C operations

// I2C device handle
static i2c_master_dev_handle_t pca9685_i2c_dev_handle = NULL;
static bool pca9685_initialized = false;

esp_err_t pca9685_init(void)
{
    if (pca9685_initialized) {
        ESP_LOGW(TAG_PCA9685, "PCA9685 already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG_PCA9685, "Initializing PCA9685 at I2C address 0x%02X", PCA9685_I2C_ADDRESS);

    // Get shared I2C bus handle
    i2c_master_bus_handle_t bus_handle = shared_i2c_get_bus_handle();
    if (bus_handle == NULL) {
        ESP_LOGE(TAG_PCA9685, "Failed to get shared I2C bus handle");
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
        ESP_LOGE(TAG_PCA9685, "Failed to add PCA9685 device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Reset PCA9685 to default state
    ret = pca9685_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PCA9685, "Failed to reset PCA9685");
        i2c_master_bus_rm_device(pca9685_i2c_dev_handle);
        pca9685_i2c_dev_handle = NULL;
        return ret;
    }
    
    // Wait for oscillator to stabilize
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set PWM frequency to 1.6kHz (default for motor shield)
    ret = pca9685_set_pwm_freq(PCA9685_PWM_FREQUENCY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PCA9685, "Failed to set PWM frequency");
        i2c_master_bus_rm_device(pca9685_i2c_dev_handle);
        pca9685_i2c_dev_handle = NULL;
        return ret;
    }
    
    // Configure MODE2 register for totem-pole output (better for motor drivers)
    ret = pca9685_write_register(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PCA9685, "Failed to configure MODE2 register");
        i2c_master_bus_rm_device(pca9685_i2c_dev_handle);
        pca9685_i2c_dev_handle = NULL;
        return ret;
    }
    
    // Turn off all PWM outputs initially
    ret = pca9685_all_off();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PCA9685, "Failed to turn off all PWM outputs");
        i2c_master_bus_rm_device(pca9685_i2c_dev_handle);
        pca9685_i2c_dev_handle = NULL;
        return ret;
    }

    pca9685_initialized = true;
    ESP_LOGI(TAG_PCA9685, "PCA9685 initialized successfully");
    return ESP_OK;
}

esp_err_t pca9685_deinit(void)
{
    if (!pca9685_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG_PCA9685, "Deinitializing PCA9685");

    // Turn off all outputs
    pca9685_all_off();

    // Remove I2C device
    if (pca9685_i2c_dev_handle != NULL) {
        i2c_master_bus_rm_device(pca9685_i2c_dev_handle);
        pca9685_i2c_dev_handle = NULL;
    }

    pca9685_initialized = false;
    ESP_LOGI(TAG_PCA9685, "PCA9685 deinitialized");
    return ESP_OK;
}

esp_err_t pca9685_reset(void)
{
    ESP_LOGI(TAG_PCA9685, "Resetting PCA9685");
    
    // Send software reset to MODE1 register
    esp_err_t ret = pca9685_write_register(PCA9685_REG_MODE1, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for reset to complete
    return ESP_OK;
}

esp_err_t pca9685_write_register(uint8_t reg_addr, uint8_t value)
{
    if (pca9685_i2c_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t write_buf[2] = {reg_addr, value};
    esp_err_t ret = i2c_master_transmit(pca9685_i2c_dev_handle, write_buf, sizeof(write_buf), 
                                        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_PCA9685, "Failed to write register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t pca9685_read_register(uint8_t reg_addr, uint8_t* value)
{
    if (pca9685_i2c_dev_handle == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_transmit_receive(pca9685_i2c_dev_handle, 
                                                &reg_addr, 1, 
                                                value, 1, 
                                                I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_PCA9685, "Failed to read register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t pca9685_set_pwm_freq(uint16_t freq_hz)
{
    ESP_LOGI(TAG_PCA9685, "Setting PWM frequency to %d Hz", freq_hz);
    
    // Calculate prescale value: prescale = round(25MHz / (4096 * freq)) - 1
    float prescale_val = ((float)PCA9685_CLOCK_FREQ / (PCA9685_PWM_RESOLUTION * (float)freq_hz)) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_val + 0.5f);  // Round to nearest integer
    
    ESP_LOGI(TAG_PCA9685, "Calculated prescale value: %d (from %.2f)", prescale, prescale_val);
    
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

esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on_time, uint16_t off_time)
{
    if (channel >= PCA9685_NUM_CHANNELS) {
        ESP_LOGE(TAG_PCA9685, "Invalid PWM channel: %d (must be 0-%d)", channel, PCA9685_NUM_CHANNELS - 1);
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

esp_err_t pca9685_set_pwm_value(uint8_t channel, uint16_t value)
{
    // Clamp value to 12-bit range
    if (value > 4095) {
        value = 4095;
    }
    
    // Set PWM with on_time=0 and off_time=value for normal PWM operation
    return pca9685_set_pwm(channel, 0, value);
}

esp_err_t pca9685_all_off(void)
{
    ESP_LOGI(TAG_PCA9685, "Turning off all PWM outputs");
    
    esp_err_t ret = pca9685_write_register(PCA9685_REG_ALL_LED_OFF_L, 0x00);
    ret |= pca9685_write_register(PCA9685_REG_ALL_LED_OFF_H, 0x10);  // Full OFF
    
    return ret;
}
