/*
 * NAU7802 ADC Task - 24-bit Strain Gauge ADC Implementation
 * 
 * Manages dual-channel 24-bit ADC for precision weight measurements.
 * Provides calibrated weight data to the display task via message queues.
 */

#include "nau7802_task.h"
#include "display_task.h"
#include "shared_i2c_bus.h"
#include "coffee_doser_config.h"
#include "rotary_encoder_task.h"
#include "motor_control_task.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <math.h>

// NVS Storage constants
#define NVS_NAMESPACE "nau7802"
#define NVS_KEY_TARE_A "tare_a"
#define NVS_KEY_TARE_B "tare_b"

// Task variables
static TaskHandle_t nau7802_task_handle = NULL;
static bool nau7802_task_running = false;
static SemaphoreHandle_t nau7802_data_mutex = NULL;
static nau7802_data_t nau7802_data = {0};

// Connection status tracking
static bool channel_a_connected = false;
static bool channel_b_connected = false;
static uint32_t channel_a_consecutive_errors = 0;
static uint32_t channel_b_consecutive_errors = 0;
#define MAX_CONSECUTIVE_ERRORS 5  // Consider disconnected after 5 consecutive errors

// Tare operation tracking
static volatile bool tare_in_progress = false;  // Flag to indicate tare operation in progress

// I2C device handle
static i2c_master_dev_handle_t nau7802_i2c_dev_handle = NULL;

// Calibration data for both channels  
static nau7802_calibration_t channel_a_cal = {0, 10.44f, false};  // Corrected based on 358g weight test
static nau7802_calibration_t channel_b_cal = {0, 10.44f, false};  // Corrected based on 358g weight test

// Configuration
static nau7802_gain_t current_gain = NAU7802_GAIN_1X;  // Maximum stability with lowest gain
static nau7802_rate_t current_rate = NAU7802_RATE_40SPS;

// Dynamic timeout optimization
static uint32_t optimal_timeout_ms = 100;  // Default timeout, will be calibrated
static bool timeout_calibrated = false;

// Internal function prototypes
static void nau7802_task_function(void* pvParameters);
static esp_err_t nau7802_i2c_init(void);
static esp_err_t nau7802_device_init(void);
static esp_err_t nau7802_write_register(uint8_t reg_addr, uint8_t value);
static esp_err_t nau7802_read_register(uint8_t reg_addr, uint8_t* value);
static esp_err_t nau7802_read_adc_raw(int32_t* raw_value);
static esp_err_t nau7802_select_channel(nau7802_channel_t channel);
static esp_err_t nau7802_wait_for_data_ready(uint32_t timeout_ms);
static bool nau7802_is_data_ready(void);
static float nau7802_raw_to_weight(int32_t raw_value, nau7802_calibration_t* cal);
static esp_err_t nau7802_perform_calibration(void);
static void nau7802_calibrate_timing(int num_samples);
static void nau7802_rotary_event_handler(rotary_event_t event, int32_t position, int32_t delta);
static esp_err_t nau7802_save_tare_values(void);
static esp_err_t nau7802_load_tare_values(void);
static esp_err_t nau7802_init_nvs(void);
static float nau7802_raw_to_weight_filtered(int32_t raw_value, nau7802_calibration_t* cal, nau7802_channel_data_t* ch_data, uint32_t timestamp_ms, const char* channel_name);


esp_err_t nau7802_task_init(void)
{
    if (nau7802_task_running) {
        ESP_LOGW(TAG_NAU7802, "NAU7802 task already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG_NAU7802, "Initializing NAU7802 24-bit ADC task");

    // Initialize I2C for NAU7802
    esp_err_t ret = nau7802_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create data mutex
    nau7802_data_mutex = xSemaphoreCreateMutex();
    if (nau7802_data_mutex == NULL) {
        ESP_LOGE(TAG_NAU7802, "Failed to create NAU7802 data mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize device hardware
    ret = nau7802_device_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to initialize NAU7802 device: %s", esp_err_to_name(ret));
        vSemaphoreDelete(nau7802_data_mutex);
        return ret;
    }

    // Initialize NVS for persistent tare storage
    ret = nau7802_init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_NAU7802, "Failed to initialize NVS - tare values won't persist");
    } else {
        // Load saved tare values from flash
        ret = nau7802_load_tare_values();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG_NAU7802, "Restored tare values from flash storage");
        } else {
            ESP_LOGW(TAG_NAU7802, "Using default tare values (no saved values found)");
        }
    }

    // Initialize data structure
    memset(&nau7802_data, 0, sizeof(nau7802_data_t));
    nau7802_data.device_ready = true;

    // Send initial display update AFTER loading tare values
    // Channel B disabled (NO_CONN), Channel A will show 0g initially
    display_send_coffee_info(WEIGHT_DISPLAY_NO_CONN, 0.0f, false);
    ESP_LOGI(TAG_NAU7802, "Sent initial display update - weights will update once measurements begin");

    // Register callback with rotary encoder for tare functionality
    ret = rotary_encoder_register_callback(nau7802_rotary_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_NAU7802, "Failed to register rotary encoder callback: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG_NAU7802, "Tare-on-click functionality will not be available");
    } else {
        ESP_LOGI(TAG_NAU7802, "Rotary encoder callback registered - click to tare scale");
    }

    ESP_LOGI(TAG_NAU7802, "NAU7802 task initialized successfully");
    return ESP_OK;
}

esp_err_t nau7802_task_start(void)
{
    if (nau7802_task_running) {
        ESP_LOGW(TAG_NAU7802, "NAU7802 task already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG_NAU7802, "Starting NAU7802 task");

    // Set flag before creating task to prevent race condition
    nau7802_task_running = true;
    
    // Create the task
    BaseType_t ret = xTaskCreate(
        nau7802_task_function,
        "nau7802_task",
        NAU7802_TASK_STACK_SIZE,
        NULL,
        NAU7802_TASK_PRIORITY,
        &nau7802_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG_NAU7802, "Failed to create NAU7802 task");
        nau7802_task_running = false;  // Reset flag on failure
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG_NAU7802, "NAU7802 task started successfully");
    return ESP_OK;
}

static esp_err_t nau7802_i2c_init(void)
{
    ESP_LOGI(TAG_NAU7802, "Adding NAU7802 to shared I2C bus");

    // Add NAU7802 device to the shared bus
    esp_err_t ret = shared_i2c_add_device(NAU7802_ADDR, 100000, &nau7802_i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to add NAU7802 to shared I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_NAU7802, "NAU7802 added to shared I2C bus successfully");
    return ESP_OK;
}

static void nau7802_task_function(void* pvParameters)
{
    ESP_LOGI(TAG_NAU7802, "NAU7802 task running - monitoring dual-channel ADC");

    uint32_t channel_a_samples = 0;
    uint32_t channel_b_samples = 0;
    uint32_t error_count = 0;
    uint32_t last_error_log = 0;

    while (nau7802_task_running) {
        esp_err_t ret;
        bool channel_a_success = false;
        bool channel_b_success = false;
        
        // Read Channel A (Container Weight)
        ret = nau7802_select_channel(NAU7802_CHANNEL_1);
        if (ret == ESP_OK) {
            ret = nau7802_wait_for_data_ready(250);  // Use calibrated timeout
            if (ret == ESP_OK) {
                int32_t raw_value;
                ret = nau7802_read_adc_raw(&raw_value);
                if (ret == ESP_OK) {
                    // Update Channel A data
                    if (xSemaphoreTake(nau7802_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        uint32_t timestamp_ms = esp_timer_get_time() / 1000;
                        
                        nau7802_data.channel_a.raw_value = raw_value;
                        nau7802_data.channel_a.weight_grams = nau7802_raw_to_weight_filtered(raw_value, &channel_a_cal, &nau7802_data.channel_a, timestamp_ms, "Channel A");
                        nau7802_data.channel_a.data_ready = true;
                        nau7802_data.channel_a.timestamp = timestamp_ms;
                        nau7802_data.channel_a.sample_count = ++channel_a_samples;
                        nau7802_data.total_conversions++;
                        xSemaphoreGive(nau7802_data_mutex);

                        // Check if the weight value indicates a disconnected sensor
                        // Temporarily disabled to allow re-taring after gain change
                        if (false && nau7802_data.channel_a.weight_grams < -900.0f) {
                            ESP_LOGW(TAG_NAU7802, "Channel A: invalid weight %.2fg indicates disconnection", 
                                     nau7802_data.channel_a.weight_grams);
                            channel_a_consecutive_errors++;
                        } else {
                            ESP_LOGD(TAG_NAU7802, "Channel A: raw=%ld, filtered=%.0fg, velocity=%.3fg/s, accel=%.3fg/s², conf=%.2f", 
                                     raw_value, nau7802_data.channel_a.filtered_weight,
                                     nau7802_data.channel_a.velocity, nau7802_data.channel_a.acceleration,
                                     nau7802_data.channel_a.confidence);
                            channel_a_success = true;
                            
                            // Reset error count and mark as connected
                            channel_a_consecutive_errors = 0;
                            if (!channel_a_connected) {
                                channel_a_connected = true;
                                ESP_LOGI(TAG_NAU7802, "Channel A strain gauge connected and responding");
                            }
                        }
                    }
                } else {
                    ESP_LOGW(TAG_NAU7802, "Failed to read Channel A ADC data: %s", esp_err_to_name(ret));
                    channel_a_consecutive_errors++;
                }
            } else {
                ESP_LOGD(TAG_NAU7802, "Channel A data ready timeout: %s", esp_err_to_name(ret));
                channel_a_consecutive_errors++;
            }
        } else {
            ESP_LOGW(TAG_NAU7802, "Failed to select Channel A: %s", esp_err_to_name(ret));
            channel_a_consecutive_errors++;
        }
        
        // Check if Channel A should be considered disconnected
        if (channel_a_consecutive_errors >= MAX_CONSECUTIVE_ERRORS && channel_a_connected) {
            channel_a_connected = false;
            ESP_LOGW(TAG_NAU7802, "Channel A strain gauge appears disconnected (errors: %lu)", channel_a_consecutive_errors);
        }

        // Send display update with current weight readings (rounded to integers)
        // Show TARING indicator during tare operation to avoid confusing fluctuations
        int display_container_weight, display_dosage_weight;
        
        if (tare_in_progress) {
            // During tare, show "----" instead of fluctuating values
            display_container_weight = (int)WEIGHT_DISPLAY_TARING;
            display_dosage_weight = (int)WEIGHT_DISPLAY_TARING;
        } else {
            // Normal operation - show actual weights or NO_CONN if disconnected
            display_container_weight = channel_a_connected ? (int)round(nau7802_data.channel_a.weight_grams) : (int)WEIGHT_DISPLAY_NO_CONN;
            display_dosage_weight = channel_b_connected ? (int)round(nau7802_data.channel_b.weight_grams) : (int)WEIGHT_DISPLAY_NO_CONN;
        }
        
        // Send display update to show current weights
        display_send_coffee_info((float)display_dosage_weight, (float)display_container_weight, false);

        // Small delay between channel reads (important for strain gauge settling)
        vTaskDelay(pdMS_TO_TICKS(10));  // Increased for better strain gauge stability

        // Channel B (Dosage Cup Weight) - DISABLED
        // Force Channel B to always show as disconnected
        channel_b_connected = false;
        
        /* DISABLED - Channel B reading code
        // Read Channel B (Dosage Cup Weight)
        ret = nau7802_select_channel(NAU7802_CHANNEL_2);
        if (ret == ESP_OK) {
            ret = nau7802_wait_for_data_ready(250);  // Use same timeout as Channel A
            if (ret == ESP_OK) {
                int32_t raw_value;
                ret = nau7802_read_adc_raw(&raw_value);
                if (ret == ESP_OK) {
                    // Update Channel B data with filtered weight
                    if (xSemaphoreTake(nau7802_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        uint32_t timestamp_ms = esp_timer_get_time() / 1000;
                        
                        nau7802_data.channel_b.raw_value = raw_value;
                        nau7802_data.channel_b.weight_grams = nau7802_raw_to_weight_filtered(raw_value, &channel_b_cal, &nau7802_data.channel_b, timestamp_ms, "Channel B");
                        nau7802_data.channel_b.data_ready = true;
                        nau7802_data.channel_b.timestamp = timestamp_ms;
                        nau7802_data.channel_b.sample_count = ++channel_b_samples;
                        nau7802_data.total_conversions++;
                        xSemaphoreGive(nau7802_data_mutex);

                        // Check if the weight value indicates a disconnected sensor
                        // Temporarily disabled to allow re-taring after gain change
                        if (false && nau7802_data.channel_b.weight_grams < -900.0f) {
                            ESP_LOGW(TAG_NAU7802, "Channel B: invalid weight %.2fg indicates disconnection", 
                                     nau7802_data.channel_b.weight_grams);
                            channel_b_consecutive_errors++;
                        } else {
                            ESP_LOGD(TAG_NAU7802, "Channel B: raw=%ld, filtered=%.0fg, velocity=%.3fg/s, accel=%.3fg/s², conf=%.2f", 
                                     raw_value, nau7802_data.channel_b.filtered_weight,
                                     nau7802_data.channel_b.velocity, nau7802_data.channel_b.acceleration,
                                     nau7802_data.channel_b.confidence);
                            channel_b_success = true;
                            
                            // Reset error count and mark as connected
                            channel_b_consecutive_errors = 0;
                            if (!channel_b_connected) {
                                channel_b_connected = true;
                                ESP_LOGI(TAG_NAU7802, "Channel B strain gauge connected and responding");
                            }
                        }
                    }
                } else {
                    ESP_LOGW(TAG_NAU7802, "Failed to read Channel B ADC data: %s", esp_err_to_name(ret));
                    channel_b_consecutive_errors++;
                }
            } else {
                ESP_LOGD(TAG_NAU7802, "Channel B data ready timeout: %s", esp_err_to_name(ret));
                channel_b_consecutive_errors++;
            }
        } else {
            ESP_LOGW(TAG_NAU7802, "Failed to select Channel B: %s", esp_err_to_name(ret));
            channel_b_consecutive_errors++;
        }
        
        // Check if Channel B should be considered disconnected
        if (channel_b_consecutive_errors >= MAX_CONSECUTIVE_ERRORS && channel_b_connected) {
            channel_b_connected = false;
            ESP_LOGW(TAG_NAU7802, "Channel B strain gauge appears disconnected (errors: %lu)", channel_b_consecutive_errors);
        }
        */

        // Error counting and periodic logging for overall status
        if (!channel_a_connected && !channel_b_connected) {
            error_count++;
            uint32_t current_time = esp_timer_get_time() / 1000000;  // Convert to seconds
            if (current_time - last_error_log >= 5) {  // Log every 5 seconds
                ESP_LOGW(TAG_NAU7802, "No strain gauges connected (total errors: %lu). Check connections.", error_count);
                last_error_log = current_time;
            }
        } else {
            // Reset error count when at least one channel is connected
            if (error_count > 0 && (channel_a_connected || channel_b_connected)) {
                ESP_LOGI(TAG_NAU7802, "At least one strain gauge connected after %lu errors", error_count);
                error_count = 0;
            }
        }

        // Task delay between complete dual-channel cycles
        vTaskDelay(pdMS_TO_TICKS(NAU7802_POLL_RATE_MS));
    }

    ESP_LOGI(TAG_NAU7802, "NAU7802 task stopped");
    vTaskDelete(NULL);
}

static esp_err_t nau7802_device_init(void)
{
    ESP_LOGI(TAG_NAU7802, "Initializing NAU7802 device");
    
    // First, probe the device to check basic I2C communication
    esp_err_t probe_ret = i2c_master_probe(shared_i2c_get_bus_handle(), NAU7802_ADDR, 1000);
    if (probe_ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "NAU7802 device not responding at address 0x%02X: %s", NAU7802_ADDR, esp_err_to_name(probe_ret));
        ESP_LOGE(TAG_NAU7802, "Check NAU7802 wiring and power connections");
        return probe_ret;
    }
    ESP_LOGI(TAG_NAU7802, "NAU7802 device detected at address 0x%02X", NAU7802_ADDR);
    
    // Reset the device (Adafruit reset procedure)
    esp_err_t ret = nau7802_write_register(NAU7802_REG_PU_CTRL, NAU7802_PU_CTRL_RR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to reset device");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for reset
    
    // Clear reset bit and power up digital
    ret = nau7802_write_register(NAU7802_REG_PU_CTRL, NAU7802_PU_CTRL_PUD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to power up digital");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Check if device is ready (power up ready bit)
    uint8_t status;
    ret = nau7802_read_register(NAU7802_REG_PU_CTRL, &status);
    if (ret != ESP_OK || !(status & NAU7802_PU_CTRL_PUR)) {
        ESP_LOGE(TAG_NAU7802, "Device not ready after digital power up");
        return ESP_ERR_TIMEOUT;
    }
    
    // Power up analog
    uint8_t pu_ctrl = NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA;
    ret = nau7802_write_register(NAU7802_REG_PU_CTRL, pu_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to power up analog");
        return ret;
    }
    
    // Wait for analog power up (600ms as per Adafruit)
    ESP_LOGI(TAG_NAU7802, "Waiting for analog power up stabilization...");
    vTaskDelay(pdMS_TO_TICKS(600));
    
    // Set LDO to 3.0V and enable internal AVDD source (critical for operation)
    pu_ctrl |= NAU7802_PU_CTRL_AVDDS;  // Enable internal AVDD source
    ret = nau7802_write_register(NAU7802_REG_PU_CTRL, pu_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to enable internal AVDD");
        return ret;
    }
    
    // Configure LDO voltage (3.0V)
    uint8_t ctrl1_val;
    ret = nau7802_read_register(NAU7802_REG_CTRL1, &ctrl1_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to read CTRL1 register");
        return ret;
    }
    
    ctrl1_val &= ~NAU7802_CTRL1_VLDO;  // Clear LDO bits
    ctrl1_val |= (5 << 3);  // Set to 3.0V (NAU7802_3V0 = 5)
    ret = nau7802_write_register(NAU7802_REG_CTRL1, ctrl1_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to set LDO voltage");
        return ret;
    }
    
    // Now check revision ID after device is fully powered and stable
    uint8_t revision_id;
    ret = nau7802_read_register(NAU7802_REG_REVISION_ID, &revision_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to read revision ID: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG_NAU7802, "NAU7802 Revision ID: 0x%02X (after full power up)", revision_id);
    
    // Check if the low nibble is 0xF (as per Adafruit implementation)
    if ((revision_id & 0x0F) != 0x0F) {
        ESP_LOGW(TAG_NAU7802, "Unexpected revision ID low nibble: 0x%X (expected 0xF)", revision_id & 0x0F);
        ESP_LOGW(TAG_NAU7802, "This may be a different NAU7802 variant or firmware version");
    } else {
        ESP_LOGI(TAG_NAU7802, "NAU7802 revision ID check passed");
    }
    
    // Read a few more registers for diagnostics
    uint8_t pu_ctrl_readback;
    ret = nau7802_read_register(NAU7802_REG_PU_CTRL, &pu_ctrl_readback);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_NAU7802, "PU_CTRL register: 0x%02X", pu_ctrl_readback);
    }
    
    uint8_t ctrl1_readback;
    ret = nau7802_read_register(NAU7802_REG_CTRL1, &ctrl1_readback);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_NAU7802, "CTRL1 register: 0x%02X", ctrl1_readback);
    }
    
    // Set gain to 4x
    ret = nau7802_set_gain(current_gain);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to set gain");
        return ret;
    }
    
    // Set sample rate to 10 SPS for maximum noise reduction
    ret = nau7802_set_sample_rate(current_rate);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to set sample rate");
        return ret;
    }
    
    // Disable ADC chopper clock (as per Adafruit)
    uint8_t adc_reg;
    ret = nau7802_read_register(NAU7802_REG_ADC, &adc_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to read ADC register");
        return ret;
    }
    
    adc_reg &= ~0x30;  // Clear chopper clock bits
    adc_reg |= 0x30;   // Set to disable chopper (0x3 in bits 4-5)
    ret = nau7802_write_register(NAU7802_REG_ADC, adc_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to configure ADC chopper");
        return ret;
    }
    
    // Configure for low ESR capacitors (as per Adafruit)
    uint8_t pga_reg;
    ret = nau7802_read_register(NAU7802_REG_PGA, &pga_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to read PGA register");
        return ret;
    }
    
    pga_reg &= ~0x40;  // Clear bit 6 for low ESR mode
    ret = nau7802_write_register(NAU7802_REG_PGA, pga_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to configure PGA for low ESR");
        return ret;
    }
    
    // Start conversions
    ret = nau7802_read_register(NAU7802_REG_PU_CTRL, &pu_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to read PU_CTRL for conversion start");
        return ret;
    }
    
    pu_ctrl |= NAU7802_PU_CTRL_CS;  // Start conversions
    ret = nau7802_write_register(NAU7802_REG_PU_CTRL, pu_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to start conversions");
        return ret;
    }
    
    // Perform initial calibration
    ret = nau7802_perform_calibration();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_NAU7802, "Initial calibration failed, continuing anyway");
    }
    
    ESP_LOGI(TAG_NAU7802, "NAU7802 device initialized successfully");
    return ESP_OK;
}

static esp_err_t nau7802_write_register(uint8_t reg_addr, uint8_t value)
{
    uint8_t write_data[2] = {reg_addr, value};
    return i2c_master_transmit(nau7802_i2c_dev_handle, write_data, 2, 1000);
}

static esp_err_t nau7802_read_register(uint8_t reg_addr, uint8_t* value)
{
    esp_err_t ret = i2c_master_transmit(nau7802_i2c_dev_handle, &reg_addr, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to read register 0x%02X with error: %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_master_receive(nau7802_i2c_dev_handle, value, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to receive data for register 0x%02X with error: %s", reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t nau7802_read_adc_raw(int32_t* raw_value)
{
    uint8_t data[3];
    
    // Read 3 bytes of ADC data
    esp_err_t ret = nau7802_read_register(NAU7802_REG_ADCO_B2, &data[0]);
    if (ret != ESP_OK) return ret;
    
    ret = nau7802_read_register(NAU7802_REG_ADCO_B1, &data[1]);
    if (ret != ESP_OK) return ret;
    
    ret = nau7802_read_register(NAU7802_REG_ADCO_B0, &data[2]);
    if (ret != ESP_OK) return ret;
    
    // Combine bytes into 24-bit signed value
    int32_t result = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | (int32_t)data[2];
    
    // Sign extend from 24-bit to 32-bit
    if (result & 0x800000) {
        result |= 0xFF000000;
    }
    
    // Range validation for disconnected strain gauge detection
    // NAU7802 with 128x gain should produce readings roughly in range ±2M for connected load cells
    // Readings near the rails (±8M) or exactly 0x000000/0xFFFFFF often indicate floating inputs
    if (result == 0x000000 || result == 0xFFFFFF || result == -1 || 
        result > 6000000 || result < -6000000) {
        ESP_LOGD(TAG_NAU7802, "ADC reading out of expected range: %ld (possible disconnect)", result);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    *raw_value = result;
    return ESP_OK;
}

static esp_err_t nau7802_select_channel(nau7802_channel_t channel)
{
    uint8_t ctrl2_val;
    esp_err_t ret = nau7802_read_register(NAU7802_REG_CTRL2, &ctrl2_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to read CTRL2 register with error: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (channel == NAU7802_CHANNEL_2) {
        ctrl2_val |= NAU7802_CTRL2_CHS;  // Set channel select bit for channel 2
    } else {
        ctrl2_val &= ~NAU7802_CTRL2_CHS; // Clear channel select bit for channel 1
    }
    ret = nau7802_write_register(NAU7802_REG_CTRL2, ctrl2_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to select channel %d with error: %s", channel, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t nau7802_wait_for_data_ready(uint32_t timeout_ms)
{
    uint32_t start_time = esp_timer_get_time() / 1000;
    
    while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
        if (nau7802_is_data_ready()) {
            uint32_t actual_time = esp_timer_get_time() / 1000 - start_time;
            // Enable timing diagnostics for calibration
            ESP_LOGD(TAG_NAU7802, "Data ready in %lu ms", actual_time);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    return ESP_ERR_TIMEOUT;
}

static bool nau7802_is_data_ready(void)
{
    uint8_t status;
    if (nau7802_read_register(NAU7802_REG_PU_CTRL, &status) == ESP_OK) {
        return (status & NAU7802_PU_CTRL_CR) != 0;
    }
    return false;
}

static float nau7802_raw_to_weight(int32_t raw_value, nau7802_calibration_t* cal)
{
    // Additional validation - raw values should be reasonable for strain gauges
    // Extreme values near ADC rails often indicate disconnected sensors
    if (raw_value > 6000000 || raw_value < -6000000) {
        ESP_LOGD(TAG_NAU7802, "Raw value %ld out of operational range", raw_value);
        return -999.0f; // Invalid weight marker
    }
    
    if (!cal->is_calibrated) {
        // Return raw value scaled down if not calibrated
        return (float)(raw_value - cal->zero_offset) / 1000.0f;
    }
    
    // Apply calibration: weight = (raw - offset) / scale_factor
    float weight = (float)(raw_value - cal->zero_offset) / cal->scale_factor;
    
    // Sanity check on calculated weight - should be reasonable for coffee applications
    // Negative weights beyond reasonable tare range or extremely large weights suggest problems
    if (weight < -1000.0f || weight > 10000.0f) {
        ESP_LOGD(TAG_NAU7802, "Calculated weight %.2fg out of reasonable range", weight);
        return -999.0f; // Invalid weight marker
    }
    
    return weight;
}

static esp_err_t nau7802_perform_calibration(void)
{
    ESP_LOGI(TAG_NAU7802, "Performing internal calibration");
    
    // Start internal calibration
    uint8_t ctrl2_val;
    esp_err_t ret = nau7802_read_register(NAU7802_REG_CTRL2, &ctrl2_val);
    if (ret != ESP_OK) return ret;
    
    ctrl2_val |= NAU7802_CTRL2_CALS;  // Start calibration
    ret = nau7802_write_register(NAU7802_REG_CTRL2, ctrl2_val);
    if (ret != ESP_OK) return ret;
    
    // Wait for calibration to complete
    uint32_t timeout = 1000;  // 1 second timeout
    while (timeout--) {
        ret = nau7802_read_register(NAU7802_REG_CTRL2, &ctrl2_val);
        if (ret == ESP_OK && !(ctrl2_val & NAU7802_CTRL2_CALS)) {
            // Check for calibration error
            if (ctrl2_val & NAU7802_CTRL2_CAL_ERROR) {
                ESP_LOGE(TAG_NAU7802, "Calibration error detected");
                return ESP_ERR_INVALID_RESPONSE;
            }
            ESP_LOGI(TAG_NAU7802, "Internal calibration completed successfully");
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_LOGE(TAG_NAU7802, "Calibration timeout");
    return ESP_ERR_TIMEOUT;
}

// Rotary encoder event handler for tare functionality and motor control
static void nau7802_rotary_event_handler(rotary_event_t event, int32_t position, int32_t delta)
{
    (void)position;  // Unused parameter
    (void)delta;     // Unused parameter
    
    if (event == ROTARY_EVENT_DOUBLE_CLICK) {
        ESP_LOGI(TAG_NAU7802, "Double-click detected - sending dispense command to motor task");
        
        // Show "DISPENSING" notification on display for 2 seconds
        display_send_system_status("  DISPENSING", false, 2000);
        
        // Send dispense command to motor control task
        // Parameter could be target weight from encoder position or a fixed amount
        uint32_t target_grams = 20;  // Example: dispense 20 grams
        esp_err_t ret = motor_send_command(MOTOR_CMD_DISPENSE, target_grams);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG_NAU7802, "Dispense command sent to motor task (target: %lu grams)", target_grams);
        } else {
            ESP_LOGW(TAG_NAU7802, "Failed to send dispense command: %s", esp_err_to_name(ret));
        }
    } else if (event == ROTARY_EVENT_LONG_PRESS) {
        ESP_LOGI(TAG_NAU7802, "Rotary encoder long press detected - taring scale...");
        
        bool any_tared = false;
        
        // Only tare connected channels
        if (channel_a_connected) {
            esp_err_t ret_a = nau7802_tare_channel(NAU7802_CHANNEL_1);
            if (ret_a == ESP_OK) {
                ESP_LOGI(TAG_NAU7802, "Channel A (Container) tared successfully");
                any_tared = true;
            } else {
                ESP_LOGW(TAG_NAU7802, "Failed to tare Channel A: %s", esp_err_to_name(ret_a));
            }
        }
        
        if (channel_b_connected) {
            esp_err_t ret_b = nau7802_tare_channel(NAU7802_CHANNEL_2);
            if (ret_b == ESP_OK) {
                ESP_LOGI(TAG_NAU7802, "Channel B (Dosage Cup) tared successfully");
                any_tared = true;
            } else {
                ESP_LOGW(TAG_NAU7802, "Failed to tare Channel B: %s", esp_err_to_name(ret_b));
            }
        }
        
        if (any_tared) {
            ESP_LOGI(TAG_NAU7802, "✓ Scale tared - all connected channels zeroed");
            
            // Reset rotary encoder position to 0 as part of the tare/reset operation
            esp_err_t encoder_ret = rotary_encoder_set_position(0);
            if (encoder_ret == ESP_OK) {
                ESP_LOGI(TAG_NAU7802, "Rotary encoder position reset to 0");
            } else {
                ESP_LOGW(TAG_NAU7802, "Failed to reset encoder position: %s", esp_err_to_name(encoder_ret));
            }
            
            // Note: Display will automatically update once tare_in_progress flag is cleared by tare function
            // No need for manual display update here - the main loop will handle it
        } else if (!channel_a_connected && !channel_b_connected) {
            ESP_LOGW(TAG_NAU7802, "No strain gauges connected - cannot tare scale");
            // Send error message to display (3 seconds)
            display_send_system_status("NO SCALE", true, 3000);
        } else {
            ESP_LOGW(TAG_NAU7802, "Failed to tare any connected channels");
            // Send error message to display (3 seconds)
            display_send_system_status("TARE FAILED", true, 3000);
        }
    }
}

// NVS Storage Functions
static esp_err_t nau7802_init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG_NAU7802, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG_NAU7802, "NVS initialized successfully");
    return ESP_OK;
}

static esp_err_t nau7802_save_tare_values(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to open NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save Channel A tare value
    ret = nvs_set_i32(nvs_handle, NVS_KEY_TARE_A, channel_a_cal.zero_offset);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to save Channel A tare: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Save Channel B tare value
    ret = nvs_set_i32(nvs_handle, NVS_KEY_TARE_B, channel_b_cal.zero_offset);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to save Channel B tare: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NAU7802, "Failed to commit NVS changes: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG_NAU7802, "Tare values saved - Channel A: %ld, Channel B: %ld", 
             channel_a_cal.zero_offset, channel_b_cal.zero_offset);
    return ESP_OK;
}

static esp_err_t nau7802_load_tare_values(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_NAU7802, "Failed to open NVS handle for reading: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load Channel A tare value
    size_t required_size = sizeof(int32_t);
    ret = nvs_get_i32(nvs_handle, NVS_KEY_TARE_A, &channel_a_cal.zero_offset);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG_NAU7802, "No saved Channel A tare value found, using default (0)");
        channel_a_cal.zero_offset = 0;
    } else if (ret != ESP_OK) {
        ESP_LOGW(TAG_NAU7802, "Failed to load Channel A tare: %s", esp_err_to_name(ret));
        channel_a_cal.zero_offset = 0;
    } else {
        ESP_LOGI(TAG_NAU7802, "Loaded Channel A tare value: %ld", channel_a_cal.zero_offset);
    }

    // Load Channel B tare value
    ret = nvs_get_i32(nvs_handle, NVS_KEY_TARE_B, &channel_b_cal.zero_offset);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG_NAU7802, "No saved Channel B tare value found, using default (0)");
        channel_b_cal.zero_offset = 0;
    } else if (ret != ESP_OK) {
        ESP_LOGW(TAG_NAU7802, "Failed to load Channel B tare: %s", esp_err_to_name(ret));
        channel_b_cal.zero_offset = 0;
    } else {
        ESP_LOGI(TAG_NAU7802, "Loaded Channel B tare value: %ld", channel_b_cal.zero_offset);
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}

// Public API Functions
esp_err_t nau7802_get_data(nau7802_data_t* data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(nau7802_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(data, &nau7802_data, sizeof(nau7802_data_t));
        xSemaphoreGive(nau7802_data_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t nau7802_get_channel_data(nau7802_channel_t channel, nau7802_channel_data_t* data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(nau7802_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (channel == NAU7802_CHANNEL_1) {
            memcpy(data, &nau7802_data.channel_a, sizeof(nau7802_channel_data_t));
        } else {
            memcpy(data, &nau7802_data.channel_b, sizeof(nau7802_channel_data_t));
        }
        xSemaphoreGive(nau7802_data_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t nau7802_tare_channel(nau7802_channel_t channel)
{
    ESP_LOGI(TAG_NAU7802, "Taring channel %d - collecting 500ms of stable readings...", channel);
    
    // Set flag to prevent display updates during tare
    tare_in_progress = true;
    
    // Show "Scale Taring..." message on display (permanent until cleared)
    display_send_system_status("Scale Taring...", false, 0);
    
    // Force display to show "----" during tare by sending taring indicator
    display_send_coffee_info(WEIGHT_DISPLAY_TARING, WEIGHT_DISPLAY_TARING, false);

    // Delay for physical stability 
    vTaskDelay(pdMS_TO_TICKS(500));

    nau7802_calibration_t* cal = (channel == NAU7802_CHANNEL_1) ? &channel_a_cal : &channel_b_cal;
    nau7802_channel_data_t* ch_data = (channel == NAU7802_CHANNEL_1) ? &nau7802_data.channel_a : &nau7802_data.channel_b;
    
    // Collect multiple readings over 1 sec for stable tare
    const int num_samples = 20;  // 20 samples over 1 sec = 50ms per sample
    int32_t tare_samples[num_samples];
    int valid_samples = 0;
    
    for (int i = 0; i < num_samples; i++) {
        // Wait for data ready with timeout
        esp_err_t ret = nau7802_select_channel(channel);
        if (ret == ESP_OK) {
            ret = nau7802_wait_for_data_ready(100);  // 100ms timeout per sample
            if (ret == ESP_OK) {
                int32_t raw_value;
                ret = nau7802_read_adc_raw(&raw_value);
                if (ret == ESP_OK) {
                    tare_samples[valid_samples] = raw_value;
                    valid_samples++;
                    ESP_LOGD(TAG_NAU7802, "Tare sample %d: %ld", valid_samples, raw_value);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms between samples
    }
    
    if (valid_samples < 5) {
        ESP_LOGE(TAG_NAU7802, "Insufficient stable readings for tare (%d/10)", valid_samples);
        return ESP_ERR_TIMEOUT;
    }
    
    // Calculate average of collected samples
    int64_t sum = 0;
    for (int i = 0; i < valid_samples; i++) {
        sum += tare_samples[i];
    }
    int32_t averaged_tare = (int32_t)(sum / valid_samples);
    
    ESP_LOGI(TAG_NAU7802, "Tare average from %d samples: %ld", valid_samples, averaged_tare);
    
    if (xSemaphoreTake(nau7802_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        cal->zero_offset = averaged_tare;  // Use averaged value instead of single reading
        
        // Force complete filter reinitialization when taring
        ch_data->kf.initialized = false;  // Force Kalman filter to reinitialize
        
        // Reset moving average buffer 
        for (int i = 0; i < 20; i++) {  // Updated to match new buffer size
            ch_data->avg_buffer[i] = 0.0f;  // Reset to zero weight
        }
        ch_data->avg_index = 0;
        ch_data->avg_filled = false;
        
        // Reset deadband filter
        ch_data->last_stable_weight = 0.0f;
        
        // Reset filtered weight and motion estimates
        ch_data->filtered_weight = 0.0f;
        ch_data->velocity = 0.0f;
        ch_data->acceleration = 0.0f;
        ch_data->confidence = 0.5f;
        
        ESP_LOGI(TAG_NAU7802, "Channel %d all filters reset for clean tare", channel);
        
        xSemaphoreGive(nau7802_data_mutex);
        ESP_LOGI(TAG_NAU7802, "Channel %d tared with averaged offset %ld", channel, cal->zero_offset);
        
        // Save tare values to NVS for persistence across reboots
        esp_err_t save_ret = nau7802_save_tare_values();
        if (save_ret == ESP_OK) {
            ESP_LOGI(TAG_NAU7802, "Tare values saved to flash storage");
        } else {
            ESP_LOGW(TAG_NAU7802, "Failed to save tare values to flash: %s", esp_err_to_name(save_ret));
        }
        
        // Show "Scale Tared!" success message for 2 seconds
        // This will replace the infinite "Scale Taring..." message immediately
        display_send_system_status("Scale Tared!", false, 2000);
        
        // Clear tare-in-progress flag to resume normal display updates
        tare_in_progress = false;
        
        return ESP_OK;
    }
    
    // Clear tare-in-progress flag on error too
    tare_in_progress = false;
    
    // Show error message if tare failed (3 seconds)
    // This will replace the infinite "Scale Taring..." message immediately
    display_send_system_status("Tare Failed!", true, 3000);
    return ESP_ERR_TIMEOUT;
}

esp_err_t nau7802_set_gain(nau7802_gain_t gain)
{
    uint8_t ctrl1_val;
    esp_err_t ret = nau7802_read_register(NAU7802_REG_CTRL1, &ctrl1_val);
    if (ret != ESP_OK) return ret;
    
    ctrl1_val = (ctrl1_val & ~NAU7802_CTRL1_GAINS) | ((uint8_t)gain & NAU7802_CTRL1_GAINS);
    ret = nau7802_write_register(NAU7802_REG_CTRL1, ctrl1_val);
    
    if (ret == ESP_OK) {
        current_gain = gain;
        ESP_LOGI(TAG_NAU7802, "Gain set to %dx", 1 << gain);
    }
    
    return ret;
}

esp_err_t nau7802_set_sample_rate(nau7802_rate_t rate)
{
    uint8_t ctrl2_val;
    esp_err_t ret = nau7802_read_register(NAU7802_REG_CTRL2, &ctrl2_val);
    if (ret != ESP_OK) return ret;
    
    ctrl2_val = (ctrl2_val & ~NAU7802_CTRL2_CRS) | (((uint8_t)rate << 4) & NAU7802_CTRL2_CRS);
    ret = nau7802_write_register(NAU7802_REG_CTRL2, ctrl2_val);
    
    if (ret == ESP_OK) {
        current_rate = rate;
        const uint16_t rates[] = {10, 20, 40, 80};
        ESP_LOGI(TAG_NAU7802, "Sample rate set to %d SPS", rates[rate]);
    }
    
    return ret;
}

// Kalman Filter Implementation
esp_err_t nau7802_kalman_init(kalman_filter_t* kf, float process_noise, float measurement_noise, float dt)
{
    if (kf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize state vector [weight, velocity, acceleration]
    kf->state[0] = 0.0f;  // Initial weight
    kf->state[1] = 0.0f;  // Initial velocity
    kf->state[2] = 0.0f;  // Initial acceleration
    
    // Initialize error covariance matrix P (3x3)
    // Start with high uncertainty
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = (i == j) ? 10.0f : 0.0f;  // Diagonal matrix with high initial uncertainty
        }
    }
    
    // Process noise covariance Q (models system dynamics uncertainty)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->Q[i][j] = 0.0f;
        }
    }
    // Modified Q matrix for kitchen scale - heavily damped velocity/acceleration
    float dt2 = dt * dt;
    
    // Weight changes slowly and independently
    kf->Q[0][0] = process_noise;                    // Weight variance only
    kf->Q[0][1] = 0.0f;                            // No weight-velocity coupling
    kf->Q[0][2] = 0.0f;                            // No weight-acceleration coupling
    kf->Q[1][0] = 0.0f;                            // No velocity-weight coupling  
    kf->Q[1][1] = process_noise * 0.001f;          // Very small velocity variance
    kf->Q[1][2] = 0.0f;                            // No velocity-acceleration coupling
    kf->Q[2][0] = 0.0f;                            // No acceleration-weight coupling
    kf->Q[2][1] = 0.0f;                            // No acceleration-velocity coupling
    kf->Q[2][2] = process_noise * 0.0001f;         // Very small acceleration variance
    
    // Measurement noise variance R (strain gauge noise)
    kf->R = measurement_noise;
    
    kf->dt = dt;
    kf->initialized = true;
    kf->last_update_ms = 0;
    
    ESP_LOGI(TAG_NAU7802, "Kalman filter initialized - dt: %.3fs, process_noise: %.4f, measurement_noise: %.4f", 
             dt, process_noise, measurement_noise);
    
    return ESP_OK;
}

esp_err_t nau7802_kalman_update(kalman_filter_t* kf, float measurement, uint32_t timestamp_ms)
{
    if (kf == NULL || !kf->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update time step if this isn't the first measurement
    if (kf->last_update_ms > 0) {
        float actual_dt = (timestamp_ms - kf->last_update_ms) / 1000.0f;
        if (actual_dt > 0.001f && actual_dt < 1.0f) {  // Reasonable time step bounds
            kf->dt = actual_dt;
        }
    }
    kf->last_update_ms = timestamp_ms;
    
    float dt = kf->dt;
    float dt2 = dt * dt;
    
    // State transition matrix F (heavily damped for kitchen scale)
    float F[3][3] = {
        {1.0f, dt * 0.1f,   0.0f},        // weight changes slowly, minimal velocity coupling
        {0.0f, 0.8f,        dt * 0.1f},   // velocity decays quickly (0.8 damping)
        {0.0f, 0.0f,        0.5f}         // acceleration decays very quickly (0.5 damping)
    };
    
    // Prediction step
    float x_pred[3];
    float P_pred[3][3];
    
    // Predict state: x_pred = F * x
    for (int i = 0; i < 3; i++) {
        x_pred[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            x_pred[i] += F[i][j] * kf->state[j];
        }
    }
    
    // Predict covariance: P_pred = F * P * F' + Q
    float FP[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                FP[i][j] += F[i][k] * kf->P[k][j];
            }
        }
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_pred[i][j] = kf->Q[i][j];  // Start with process noise
            for (int k = 0; k < 3; k++) {
                P_pred[i][j] += FP[i][k] * F[j][k];  // F * P * F'
            }
        }
    }
    
    // Update step
    // Measurement matrix H = [1, 0, 0] (we only observe weight directly)
    float H[3] = {1.0f, 0.0f, 0.0f};
    
    // Innovation (measurement residual): y = z - H * x_pred
    float innovation = measurement - x_pred[0];  // Only weight is measured
    
    // Innovation covariance: S = H * P_pred * H' + R
    float S = P_pred[0][0] + kf->R;
    
    // Kalman gain: K = P_pred * H' * S^(-1)
    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = P_pred[i][0] / S;  // P_pred * H' / S
    }
    
    // Update state: x = x_pred + K * innovation
    for (int i = 0; i < 3; i++) {
        kf->state[i] = x_pred[i] + K[i] * innovation;
    }
    
    // Update covariance: P = (I - K * H) * P_pred
    float KH[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            KH[i][j] = K[i] * H[j];
        }
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float I_KH = (i == j) ? 1.0f - KH[i][j] : -KH[i][j];
            kf->P[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                float I_KH_k = (i == k) ? 1.0f - KH[i][k] : -KH[i][k];
                kf->P[i][j] += I_KH_k * P_pred[k][j];
            }
        }
    }
    
    return ESP_OK;
}

esp_err_t nau7802_kalman_reset(kalman_filter_t* kf)
{
    if (kf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Reset state to zero
    kf->state[0] = 0.0f;
    kf->state[1] = 0.0f;
    kf->state[2] = 0.0f;
    
    // Reset covariance to high uncertainty
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = (i == j) ? 10.0f : 0.0f;
        }
    }
    
    kf->last_update_ms = 0;
    
    ESP_LOGI(TAG_NAU7802, "Kalman filter reset");
    return ESP_OK;
}

esp_err_t nau7802_kalman_set_parameters(kalman_filter_t* kf, float process_noise, float measurement_noise)
{
    if (kf == NULL || !kf->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update process noise covariance Q
    float dt = kf->dt;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    
    kf->Q[0][0] = (dt4 / 4.0f) * process_noise;
    kf->Q[0][1] = (dt3 / 2.0f) * process_noise;
    kf->Q[0][2] = (dt2 / 2.0f) * process_noise;
    kf->Q[1][0] = (dt3 / 2.0f) * process_noise;
    kf->Q[1][1] = dt2 * process_noise;
    kf->Q[1][2] = dt * process_noise;
    kf->Q[2][0] = (dt2 / 2.0f) * process_noise;
    kf->Q[2][1] = dt * process_noise;
    kf->Q[2][2] = process_noise;
    
    // Update measurement noise variance R
    kf->R = measurement_noise;
    
    ESP_LOGI(TAG_NAU7802, "Kalman filter parameters updated - process_noise: %.4f, measurement_noise: %.4f", 
             process_noise, measurement_noise);
    
    return ESP_OK;
}

// Enhanced weight conversion with Kalman filtering
static float nau7802_raw_to_weight_filtered(int32_t raw_value, nau7802_calibration_t* cal, nau7802_channel_data_t* ch_data, uint32_t timestamp_ms, const char* channel_name)
{
    // Basic weight conversion
    float raw_weight = (float)(raw_value - cal->zero_offset) / cal->scale_factor;
    
    // Apply simple deadband filter to reduce noise before Kalman filtering
    float weight_change = fabsf(raw_weight - ch_data->last_stable_weight);
    if (weight_change < 0.5f) {  // Only ignore changes smaller than 0.5g to maintain gram accuracy
        raw_weight = ch_data->last_stable_weight;
    } else {
        ch_data->last_stable_weight = raw_weight;
    }
    
    // Apply simple moving average for additional smoothing
    ch_data->avg_buffer[ch_data->avg_index] = raw_weight;
    ch_data->avg_index = (ch_data->avg_index + 1) % 10;
    if (!ch_data->avg_filled && ch_data->avg_index == 0) {
        ch_data->avg_filled = true;
    }
    
    // Calculate moving average if buffer has enough samples
    if (ch_data->avg_filled) {
        float sum = 0.0f;
        for (int i = 0; i < 10; i++) {
            sum += ch_data->avg_buffer[i];
        }
        raw_weight = sum / 10.0f;  // Use 10-sample moving average for balanced performance
    }
    
    // // Debug logging for calibration (remove once calibrated)
    // static uint32_t debug_counter = 0;
    // if (++debug_counter % 20 == 0) {  // Log every 20 samples (once per second)
    //     ESP_LOGI(TAG_NAU7802, "%s calibration debug: raw=%ld, offset=%ld, scale=%.1f, raw_weight=%.2fg", 
    //              channel_name, raw_value, cal->zero_offset, cal->scale_factor, raw_weight);
    // }

    // Initialize Kalman filter if not done yet
    if (!ch_data->kf.initialized) {
        // More responsive parameters for faster settling during weight changes
        float process_noise = 25.0f;       // Higher process noise for faster response
        float measurement_noise = 2.0f;    // Still trust measurements, but allow more adaptation
        float dt = 0.025f;                 // 40 SPS = 25ms intervals
        
        nau7802_kalman_init(&ch_data->kf, process_noise, measurement_noise, dt);
        
        // Initialize with first measurement
        ch_data->kf.state[0] = raw_weight;
        ch_data->filtered_weight = raw_weight;
        ch_data->velocity = 0.0f;
        ch_data->acceleration = 0.0f;
        ch_data->confidence = 0.5f;  // Medium confidence initially
        ch_data->last_stable_weight = raw_weight;  // Initialize deadband filter
        
        // Initialize moving average buffer
        for (int i = 0; i < 10; i++) {
            ch_data->avg_buffer[i] = raw_weight;
        }
        ch_data->avg_index = 0;
        ch_data->avg_filled = true;
        
        ESP_LOGI(TAG_NAU7802, "Kalman filter initialized - dt: %.3fs, process_noise: %.1f, measurement_noise: %.1f", 
                 dt, process_noise, measurement_noise);
        return raw_weight;
    }
    
    // Update Kalman filter with moving averaged measurement
    esp_err_t ret = nau7802_kalman_update(&ch_data->kf, raw_weight, timestamp_ms);
    if (ret == ESP_OK) {
        // Extract filtered values from Kalman filter
        ch_data->filtered_weight = ch_data->kf.state[0];
        ch_data->velocity = ch_data->kf.state[1];
        ch_data->acceleration = ch_data->kf.state[2];
        
        // Calculate confidence based on innovation and measurement consistency
        float innovation = fabsf(raw_weight - ch_data->filtered_weight);
        float max_innovation = 5.0f;  // 5g maximum expected innovation for good confidence (tighter with better scale factor)
        ch_data->confidence = fmaxf(0.0f, fminf(1.0f, 1.0f - (innovation / max_innovation)));
        
        // Ground the scale at 0g - prevent any negative readings and small positive noise
        float final_weight = fmaxf(0.0f, floorf(ch_data->filtered_weight));
        
        return final_weight;  // Return grounded Kalman-filtered weight
    } else {
        // Fallback to moving average if Kalman update fails
        float final_weight = fmaxf(0.0f, floorf(raw_weight));
        return final_weight;
    }
}

esp_err_t nau7802_task_stop(void)
{
    if (!nau7802_task_running) {
        return ESP_OK;
    }

    nau7802_task_running = false;

    // Wait for task to finish
    if (nau7802_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
        nau7802_task_handle = NULL;
    }

    // Clean up mutex
    if (nau7802_data_mutex != NULL) {
        vSemaphoreDelete(nau7802_data_mutex);
        nau7802_data_mutex = NULL;
    }

    ESP_LOGI(TAG_NAU7802, "NAU7802 task stopped");
    return ESP_OK;
}

// Timing calibration function - call this to find optimal timeout values
static void nau7802_calibrate_timing(int num_samples)
{
    ESP_LOGI(TAG_NAU7802, "Starting timing calibration with %d samples...", num_samples);
    
    uint32_t total_time_ch1 = 0;
    uint32_t total_time_ch2 = 0;
    uint32_t successful_ch1 = 0;
    uint32_t successful_ch2 = 0;
    uint32_t max_time_ch1 = 0;
    uint32_t max_time_ch2 = 0;
    
    for (int i = 0; i < num_samples; i++) {
        // Test Channel 1
        esp_err_t ret = nau7802_select_channel(NAU7802_CHANNEL_1);
        if (ret == ESP_OK) {
            uint32_t start_time = esp_timer_get_time() / 1000;
            ret = nau7802_wait_for_data_ready(100);  // Use generous timeout for calibration
            if (ret == ESP_OK) {
                uint32_t time_taken = esp_timer_get_time() / 1000 - start_time;
                total_time_ch1 += time_taken;
                successful_ch1++;
                if (time_taken > max_time_ch1) max_time_ch1 = time_taken;
                
                int32_t raw_value;
                nau7802_read_adc_raw(&raw_value);  // Actually read the value
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
        
        // Test Channel 2
        ret = nau7802_select_channel(NAU7802_CHANNEL_2);
        if (ret == ESP_OK) {
            uint32_t start_time = esp_timer_get_time() / 1000;
            ret = nau7802_wait_for_data_ready(100);  // Use generous timeout for calibration
            if (ret == ESP_OK) {
                uint32_t time_taken = esp_timer_get_time() / 1000 - start_time;
                total_time_ch2 += time_taken;
                successful_ch2++;
                if (time_taken > max_time_ch2) max_time_ch2 = time_taken;
                
                int32_t raw_value;
                nau7802_read_adc_raw(&raw_value);  // Actually read the value
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Brief pause between samples
    }
    
    ESP_LOGI(TAG_NAU7802, "=== TIMING CALIBRATION RESULTS ===");
    ESP_LOGI(TAG_NAU7802, "Channel 1: %lu/%d successful reads", successful_ch1, num_samples);
    if (successful_ch1 > 0) {
        ESP_LOGI(TAG_NAU7802, "  Average time: %lu ms", total_time_ch1 / successful_ch1);
        ESP_LOGI(TAG_NAU7802, "  Maximum time: %lu ms", max_time_ch1);
        ESP_LOGI(TAG_NAU7802, "  Recommended timeout: %lu ms", max_time_ch1 + 10);
    }
    
    ESP_LOGI(TAG_NAU7802, "Channel 2: %lu/%d successful reads", successful_ch2, num_samples);
    if (successful_ch2 > 0) {
        ESP_LOGI(TAG_NAU7802, "  Average time: %lu ms", total_time_ch2 / successful_ch2);
        ESP_LOGI(TAG_NAU7802, "  Maximum time: %lu ms", max_time_ch2);
        ESP_LOGI(TAG_NAU7802, "  Recommended timeout: %lu ms", max_time_ch2 + 10);
    }
    
    uint32_t overall_max = (max_time_ch1 > max_time_ch2) ? max_time_ch1 : max_time_ch2;
    ESP_LOGI(TAG_NAU7802, "Overall recommended timeout: %lu ms", overall_max + 10);
    ESP_LOGI(TAG_NAU7802, "=== END CALIBRATION RESULTS ===");
}

// Public function to run timing calibration - add to header if you want to call it externally
esp_err_t nau7802_run_timing_calibration(int num_samples)
{
    if (!nau7802_task_running) {
        ESP_LOGE(TAG_NAU7802, "NAU7802 task not running, cannot calibrate");
        return ESP_ERR_INVALID_STATE;
    }
    
    nau7802_calibrate_timing(num_samples);
    return ESP_OK;
}

// Test different timeout values to find optimal settings
esp_err_t nau7802_test_timeout_values(uint32_t min_timeout, uint32_t max_timeout, uint32_t step_timeout, uint32_t* optimal_timeout)
{
    if (!nau7802_task_running) {
        ESP_LOGE(TAG_NAU7802, "NAU7802 task not running, cannot test");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (optimal_timeout == NULL) {
        ESP_LOGE(TAG_NAU7802, "optimal_timeout parameter cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG_NAU7802, "Testing timeout values from %lu to %lu ms (step: %lu ms)", 
             min_timeout, max_timeout, step_timeout);
    
    uint32_t best_timeout = max_timeout;  // Default to max if no optimal found
    float best_avg_time = 999.0f;  // Track the fastest successful timeout
    
    for (uint32_t timeout = min_timeout; timeout <= max_timeout; timeout += step_timeout) {
        uint32_t successful_reads = 0;
        uint32_t total_attempts = 10;
        uint32_t total_time = 0;
        
        ESP_LOGI(TAG_NAU7802, "Testing timeout: %lu ms", timeout);
        
        for (uint32_t i = 0; i < total_attempts; i++) {
            // Test Channel A
            esp_err_t ret = nau7802_select_channel(NAU7802_CHANNEL_1);
            if (ret == ESP_OK) {
                uint32_t start_time = esp_timer_get_time() / 1000;
                ret = nau7802_wait_for_data_ready(timeout);
                if (ret == ESP_OK) {
                    uint32_t actual_time = esp_timer_get_time() / 1000 - start_time;
                    total_time += actual_time;
                    successful_reads++;
                    
                    int32_t raw_value;
                    nau7802_read_adc_raw(&raw_value);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(30));  // Wait for next sample cycle
        }
        
        float success_rate = (float)successful_reads / total_attempts * 100.0f;
        float avg_time = successful_reads > 0 ? (float)total_time / successful_reads : 999.0f;
        
        ESP_LOGI(TAG_NAU7802, "  Timeout %lu ms: %.1f%% success, avg time: %.1f ms", 
                 timeout, success_rate, avg_time);
        
        // If we achieve 100% success, this is a candidate for optimal timeout
        if (success_rate >= 100.0f) {
            if (avg_time < best_avg_time) {
                best_timeout = timeout;
                best_avg_time = avg_time;
            }
            ESP_LOGI(TAG_NAU7802, "  ✓ 100%% success with %.1f ms avg time", avg_time);
            
            // If this timeout is very close to the actual timing, we found our optimum
            if (timeout <= (uint32_t)(avg_time * 1.5f)) {
                ESP_LOGI(TAG_NAU7802, "  ✓ OPTIMAL: %lu ms is ideal (only %.1fx actual time)", 
                         timeout, (float)timeout / avg_time);
                best_timeout = timeout;
                break;
            }
        }
    }
    
    *optimal_timeout = best_timeout;
    ESP_LOGI(TAG_NAU7802, "=== OPTIMAL TIMEOUT FOUND: %lu ms (avg actual: %.1f ms) ===", 
             best_timeout, best_avg_time);
    
    return ESP_OK;
}

// Get the current optimal timeout value
uint32_t nau7802_get_optimal_timeout(void)
{
    return optimal_timeout_ms;
}

// Recalibrate the optimal timeout during runtime
esp_err_t nau7802_recalibrate_timeout(void)
{
    if (!nau7802_task_running) {
        ESP_LOGE(TAG_NAU7802, "NAU7802 task not running, cannot recalibrate");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG_NAU7802, "Recalibrating timeout values...");
    
    uint32_t new_timeout = 100;  // Default fallback
    esp_err_t ret = nau7802_test_timeout_values(20, 200, 10, &new_timeout);
    
    if (ret == ESP_OK) {
        optimal_timeout_ms = new_timeout;
        timeout_calibrated = true;
        ESP_LOGI(TAG_NAU7802, "Timeout recalibrated to: %lu ms", optimal_timeout_ms);
    } else {
        ESP_LOGW(TAG_NAU7802, "Timeout recalibration failed, keeping current: %lu ms", optimal_timeout_ms);
    }
    
    return ret;
}