/*
 * Display Task - OLED Status Display (New Implementation)
 * 
 * Clean implementation using the ESP-IDF SSD1306 library by nopnop2002:
 * https://github.com/nopnop2002/esp-idf-ssd1306
 * 
 * Manages SSD1306 OLED display with FreeRTOS message queue for thread-safe updates.
 * Displays encoder position, system status, and coffee dosing information.
 */

#include "display_task.h"
#include "shared_i2c_bus.h"
#include "coffee_doser_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ssd1306.h"  // Official ESP-IDF SSD1306 component
#include <string.h>
#include <stdio.h>

// Display configuration
#define SSD1306_WIDTH               128
#define SSD1306_HEIGHT              64
#define SSD1306_I2C_ADDR            0x3D

// Task variables
static TaskHandle_t display_task_handle = NULL;
static QueueHandle_t display_message_queue = NULL;
static bool display_task_running = false;
static uint32_t messages_processed = 0;
static uint32_t queue_overflows = 0;

// SSD1306 device handle for the official component
static SSD1306_t ssd1306_dev;
static i2c_master_dev_handle_t display_i2c_dev_handle = NULL;

// Internal functions
static void display_task_function(void* pvParameters);
static esp_err_t ssd1306_display_init(void);
static void display_process_encoder_message(const display_message_t* msg);
static void display_process_status_message(const display_message_t* msg);
static void display_process_coffee_message(const display_message_t* msg);

esp_err_t display_task_init(void)
{
    if (display_task_running) {
        ESP_LOGW(TAG_DISPLAY, "Display task already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG_DISPLAY, "Initializing display task with SSD1306 library");

    // Create message queue
    display_message_queue = xQueueCreate(DISPLAY_MESSAGE_QUEUE_SIZE, sizeof(display_message_t));
    if (display_message_queue == NULL) {
        ESP_LOGE(TAG_DISPLAY, "Failed to create display message queue");
        return ESP_ERR_NO_MEM;
    }

    // Initialize SSD1306 display using the official component
    esp_err_t ret = ssd1306_display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_DISPLAY, "Failed to initialize SSD1306 display: %s", esp_err_to_name(ret));
        vQueueDelete(display_message_queue);
        return ret;
    }

    // Clear display and show initial encoder display layout
    ssd1306_clear_screen(&ssd1306_dev, false);
    ssd1306_display_text(&ssd1306_dev, 0, "Rotary Encoder", 14, false);
    ssd1306_display_text(&ssd1306_dev, 1, "Hold btn=reset", 14, false);
    ssd1306_display_text(&ssd1306_dev, 2, "Pos: 0       ", 13, false);
    ssd1306_display_text(&ssd1306_dev, 4, "Delta: 0     ", 13, false);
    ssd1306_display_text(&ssd1306_dev, 6, "Btn: ---     ", 13, false);

    ESP_LOGI(TAG_DISPLAY, "Display task initialized successfully");
    return ESP_OK;
}

esp_err_t display_task_start(void)
{
    if (display_task_running) {
        ESP_LOGW(TAG_DISPLAY, "Display task already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG_DISPLAY, "Starting display task");

    // Create the task
    display_task_running = true;
    BaseType_t ret = xTaskCreate(
        display_task_function,
        "display_task",
        DISPLAY_TASK_STACK_SIZE,
        NULL,
        DISPLAY_TASK_PRIORITY,
        &display_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG_DISPLAY, "Failed to create display task");
        display_task_running = false;  // Reset flag on failure
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG_DISPLAY, "Display task started successfully");
    return ESP_OK;
}

static void display_task_function(void* pvParameters)
{
    ESP_LOGI(TAG_DISPLAY, "Display task running with SSD1306 library");
    
    display_message_t message;
    TickType_t last_update = xTaskGetTickCount();
    
    while (display_task_running) {
        // Check for messages with timeout
        if (xQueueReceive(display_message_queue, &message, pdMS_TO_TICKS(DISPLAY_UPDATE_RATE_MS)) == pdTRUE) {
            messages_processed++;
            
            // Process message based on type
            switch (message.type) {
                case DISPLAY_MSG_ENCODER_UPDATE:
                    display_process_encoder_message(&message);
                    break;
                    
                case DISPLAY_MSG_SYSTEM_STATUS:
                    display_process_status_message(&message);
                    break;
                    
                case DISPLAY_MSG_COFFEE_INFO:
                    display_process_coffee_message(&message);
                    break;
                    
                case DISPLAY_MSG_CLEAR_SCREEN:
                    ssd1306_clear_screen(&ssd1306_dev, false);
                    break;
                    
                case DISPLAY_MSG_SHUTDOWN:
                    display_task_running = false;
                    break;
                    
                default:
                    ESP_LOGW(TAG_DISPLAY, "Unknown message type: %d", message.type);
                    break;
            }
        }
        
        // Periodic refresh even without messages (every 1 second)
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_update) > pdMS_TO_TICKS(1000)) {
            // Show timestamp or other periodic info
            last_update = current_time;
        }
    }
    
    ESP_LOGI(TAG_DISPLAY, "Display task stopped");
    vTaskDelete(NULL);
}

static void display_process_encoder_message(const display_message_t* msg)
{
    char position_text[32];
    char delta_text[32];
    char button_text[16];
    
    // Don't clear the entire screen - just update the changing values
    // Title remains static, so no need to redraw it every time
    
    // Show position (clear line first by overwriting with spaces, then write new value)
    snprintf(position_text, sizeof(position_text), "Pos: %-8ld", msg->data.encoder_data.position);
    ssd1306_display_text(&ssd1306_dev, 2, position_text, strlen(position_text), false);
    
    // Show delta (always show, even if zero, to clear previous values)
    // Handle special case where delta = -999 indicates a reset operation
    if (msg->data.encoder_data.delta == -999) {
        // Don't display the special delta value, show 0 instead for reset
        snprintf(delta_text, sizeof(delta_text), "Delta: 0     ");
    } else {
        snprintf(delta_text, sizeof(delta_text), "Delta: %s%-6ld", 
                 msg->data.encoder_data.delta > 0 ? "+" : "", msg->data.encoder_data.delta);
    }
    ssd1306_display_text(&ssd1306_dev, 4, delta_text, strlen(delta_text), false);
    
    // Show button state with special indication for reset
    // Look for special delta value (-999) which indicates an actual reset operation
    if (msg->data.encoder_data.button_pressed && msg->data.encoder_data.delta == -999) {
        snprintf(button_text, sizeof(button_text), "Btn: RESET!");
    } else {
        snprintf(button_text, sizeof(button_text), "Btn: %-6s", 
                 msg->data.encoder_data.button_pressed ? "PRESS" : "---");
    }
    ssd1306_display_text(&ssd1306_dev, 6, button_text, strlen(button_text), false);
}

static void display_process_status_message(const display_message_t* msg)
{
    // Add status message to display (could be overlayed or on separate area)
    ssd1306_display_text(&ssd1306_dev, 7, msg->data.system_status.status_text, 
                        strlen(msg->data.system_status.status_text), false);
}

static void display_process_coffee_message(const display_message_t* msg)
{
    char target_text[32];
    char current_text[32];
    
    ssd1306_clear_screen(&ssd1306_dev, false);
    ssd1306_display_text(&ssd1306_dev, 0, "Coffee Dosing", 13, false);
    
    snprintf(target_text, sizeof(target_text), "Target: %.1fg", msg->data.coffee_info.target_weight);
    ssd1306_display_text(&ssd1306_dev, 2, target_text, strlen(target_text), false);
    
    snprintf(current_text, sizeof(current_text), "Current: %.1fg", msg->data.coffee_info.current_weight);
    ssd1306_display_text(&ssd1306_dev, 4, current_text, strlen(current_text), false);
    
    if (msg->data.coffee_info.dosing_active) {
        ssd1306_display_text(&ssd1306_dev, 6, "DOSING...", 9, false);
    }
}

// SSD1306 Hardware Functions using the shared I2C bus
static esp_err_t ssd1306_display_init(void)
{
    ESP_LOGI(TAG_DISPLAY, "Initializing SSD1306 OLED display using shared I2C bus");
    
    // Add SSD1306 display to the shared I2C bus
    // Use conservative speed for better reliability during initialization
    esp_err_t ret = shared_i2c_add_device(SSD1306_I2C_ADDR, 100000, &display_i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_DISPLAY, "Failed to add SSD1306 to shared I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Critical: Allow extra time for SSD1306 to fully stabilize after power-on
    // SSD1306 needs time to initialize its internal state machine
    ESP_LOGI(TAG_DISPLAY, "Waiting for SSD1306 power stabilization...");
    vTaskDelay(pdMS_TO_TICKS(100));  // Increased from implicit timing to explicit 100ms

    // Initialize device structure for the SSD1306 library
    ssd1306_dev._address = SSD1306_I2C_ADDR;
    ssd1306_dev._width = SSD1306_WIDTH;
    ssd1306_dev._height = SSD1306_HEIGHT;
    ssd1306_dev._flip = false;
    ssd1306_dev._i2c_bus_handle = shared_i2c_get_bus_handle();
    ssd1306_dev._i2c_dev_handle = display_i2c_dev_handle;

    // Use the library's device add function instead of master init
    // This uses our existing I2C bus
    i2c_device_add(&ssd1306_dev, I2C_NUM_0, -1, SSD1306_I2C_ADDR);
    
    // Add small delay before attempting communication
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Initialize the SSD1306 with 128x64 dimensions
    i2c_init(&ssd1306_dev, SSD1306_WIDTH, SSD1306_HEIGHT);
    
    ESP_LOGI(TAG_DISPLAY, "SSD1306 initialized successfully using shared I2C bus");
    return ESP_OK;
}

// Public API Functions
esp_err_t display_send_encoder_data(int32_t position, int32_t delta, bool button_pressed)
{
    display_message_t msg = {
        .type = DISPLAY_MSG_ENCODER_UPDATE,
        .timestamp = esp_timer_get_time() / 1000,
        .data.encoder_data = {
            .position = position,
            .delta = delta,
            .button_pressed = button_pressed
        }
    };
    
    if (xQueueSend(display_message_queue, &msg, 0) != pdTRUE) {
        ESP_LOGE(TAG_DISPLAY, "Display queue overflow on encoder data");
        queue_overflows++;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG_DISPLAY, "Encoder data sent: pos=%ld, delta=%ld, button=%s", 
             position, delta, button_pressed ? "PRESSED" : "RELEASED");
    return ESP_OK;
}

esp_err_t display_send_system_status(const char* status_text, bool is_error)
{
    display_message_t msg = {
        .type = DISPLAY_MSG_SYSTEM_STATUS,
        .timestamp = esp_timer_get_time() / 1000,
    };
    
    strncpy(msg.data.system_status.status_text, status_text, sizeof(msg.data.system_status.status_text) - 1);
    msg.data.system_status.status_text[sizeof(msg.data.system_status.status_text) - 1] = '\0';
    msg.data.system_status.is_error = is_error;
    
    if (xQueueSend(display_message_queue, &msg, 0) != pdTRUE) {
        queue_overflows++;
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t display_send_coffee_info(float target_weight, float current_weight, bool dosing_active)
{
    display_message_t msg = {
        .type = DISPLAY_MSG_COFFEE_INFO,
        .timestamp = esp_timer_get_time() / 1000,
        .data.coffee_info = {
            .target_weight = target_weight,
            .current_weight = current_weight,
            .dosing_active = dosing_active
        }
    };
    
    if (xQueueSend(display_message_queue, &msg, 0) != pdTRUE) {
        queue_overflows++;
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t display_clear_screen(void)
{
    display_message_t msg = {
        .type = DISPLAY_MSG_CLEAR_SCREEN,
        .timestamp = esp_timer_get_time() / 1000
    };
    
    if (xQueueSend(display_message_queue, &msg, 0) != pdTRUE) {
        queue_overflows++;
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t display_get_stats(uint32_t* messages_processed_out, uint32_t* queue_overflows_out)
{
    if (messages_processed_out) *messages_processed_out = messages_processed;
    if (queue_overflows_out) *queue_overflows_out = queue_overflows;
    return ESP_OK;
}

esp_err_t display_task_stop(void)
{
    if (!display_task_running) {
        return ESP_OK;
    }
    
    // Send shutdown message
    display_message_t msg = {
        .type = DISPLAY_MSG_SHUTDOWN,
        .timestamp = esp_timer_get_time() / 1000
    };
    
    xQueueSend(display_message_queue, &msg, pdMS_TO_TICKS(100));
    
    // Wait for task to finish
    if (display_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(200));
        display_task_handle = NULL;
    }
    
    // Clean up resources
    if (display_message_queue) {
        vQueueDelete(display_message_queue);
        display_message_queue = NULL;
    }
    
    display_task_running = false;
    ESP_LOGI(TAG_DISPLAY, "Display task stopped");
    
    return ESP_OK;
}