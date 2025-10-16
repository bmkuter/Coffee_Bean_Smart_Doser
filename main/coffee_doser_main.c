/*
 * Coffee Bean Smart Doser - Main Application
 * 
 * An automated coffee bean dispensing system with:
 * - Weight monitoring via strain gauges
 * - Servo-controlled trap door
 * - Auger-controlled bean flow
 * - Rotary encoder interface
 * - OLED status display
 * - NeoPixel status indicator
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

// Project headers
#include "coffee_doser_config.h"
#include "led_task.h"
#include "rotary_encoder_task.h"
#include "display_task.h"
#include "nau7802_task.h"
#include "motor_control_task.h"

static void initialize_system(void)
{
    // Initialize NVS for configuration storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG_MAIN, "NVS initialized");
}

static void initialize_peripherals(void)
{
    ESP_LOGI(TAG_MAIN, "Initializing Coffee Bean Smart Doser peripherals...");
    
    // Initialize rotary encoder task (manages its own I2C)
    esp_err_t ret = rotary_encoder_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize rotary encoder task: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize display task (manages its own I2C)
    ret = display_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize display task: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize NAU7802 ADC task (manages its own I2C)
    ret = nau7802_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize NAU7802 task: %s", esp_err_to_name(ret));
        return;
    }
    
    // Start rotary encoder task
    ret = rotary_encoder_task_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to start rotary encoder task: %s", esp_err_to_name(ret));
        return;
    }

    // Start display task
    ret = display_task_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to start display task: %s", esp_err_to_name(ret));
        return;
    }

    // Start NAU7802 task
    ret = nau7802_task_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to start NAU7802 task: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize motor control task
    ret = motor_control_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Failed to initialize motor control task: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG_MAIN, "Peripheral initialization complete");
    ESP_LOGI(TAG_MAIN, "Direct I2C hardware access with rotary encoder, display, and NAU7802 ADC tasks running");
}

static void log_system_info(void)
{
    ESP_LOGI(TAG_MAIN, "=== Coffee Bean Smart Doser ===");
    ESP_LOGI(TAG_MAIN, "Hardware: Adafruit ESP32-C6 Feather");
    ESP_LOGI(TAG_MAIN, "Core peripherals (Phase 1):");
    ESP_LOGI(TAG_MAIN, "  - NAU7802 Dual Strain Gauge ADC (0x%02X)", NAU7802_ADDR);
    ESP_LOGI(TAG_MAIN, "    * Channel A: Container weight monitoring");
    ESP_LOGI(TAG_MAIN, "    * Channel B: Dosage cup weight monitoring");
    ESP_LOGI(TAG_MAIN, "  - Seesaw Rotary Encoder (0x%02X)", ROTARY_ENCODER_ADDR);
    ESP_LOGI(TAG_MAIN, "  - SSD1306 OLED Display (0x%02X)", OLED_DISPLAY_ADDR);
    ESP_LOGI(TAG_MAIN, "Motor control peripherals (Phase 2):");
    ESP_LOGI(TAG_MAIN, "  - PCA9685 PWM Motor Controller (0x60)");
    ESP_LOGI(TAG_MAIN, "    * Stepper motor on M1+M2 (Ports 1)");
    ESP_LOGI(TAG_MAIN, "    * Air pump on M3 (Port 3)");
    ESP_LOGI(TAG_MAIN, "===============================");
}

void app_main(void)
{
    // // Pause for 100 ms to allow system stabilization
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Log system information
    log_system_info();
    
    // Initialize system components
    initialize_system();
    
    // Initialize peripherals first (now handles I2C power)
    initialize_peripherals();

    ESP_LOGI(TAG_MAIN, "Coffee Bean Smart Doser started successfully!");
    ESP_LOGI(TAG_MAIN, "System is ready for peripheral integration...");
    
    // Main application loop
    while (1) {
        // Suspend Task
        vTaskSuspend(NULL); // Uncomment to suspend main task if needed
        ESP_LOGI(TAG_MAIN, "System running - Ready for coffee bean dosing!");
        
        // Main loop delay
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
