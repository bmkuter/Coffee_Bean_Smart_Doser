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
#include <math.h>

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

// Status notification queue (circular buffer)
// Each notification has text, error flag, and duration:
//   - duration_ms = 0: Stay until replaced by next message (infinite)
//   - duration_ms > 0: Auto-clear after duration expires
#define STATUS_NOTIFICATION_QUEUE_SIZE 5
typedef struct {
    char text[64];
    bool is_error;
    uint32_t duration_ms;  // 0 = stay until replaced, >0 = auto-clear after duration
} status_notification_t;

static status_notification_t notification_queue[STATUS_NOTIFICATION_QUEUE_SIZE];
static uint8_t notification_queue_head = 0;
static uint8_t notification_queue_tail = 0;
static uint8_t notification_queue_count = 0;

// Current displayed notification tracking
static bool status_message_active = false;
static TickType_t status_message_start_time = 0;
static uint32_t current_message_duration_ms = 0;  // 0 = infinite, stays until replaced

// Latest data for display (all display state in one place)
static int32_t latest_encoder_position = 0;
static int32_t latest_encoder_delta = 0;
static bool latest_button_pressed = false;
static bool encoder_data_available = false;

// Latest weight data for display on encoder screen
static float latest_container_weight = 0.0f;
static float latest_dosage_weight = 0.0f;
static bool weight_data_available = false;

// SSD1306 device handle for the official component
static SSD1306_t ssd1306_dev;
static i2c_master_dev_handle_t display_i2c_dev_handle = NULL;

// Internal functions
static void display_task_function(void* pvParameters);
static esp_err_t ssd1306_display_init(void);
static void display_process_encoder_message(const display_message_t* msg);
static void display_process_status_message(const display_message_t* msg);
static void display_process_coffee_message(const display_message_t* msg);
static void display_process_individual_weight_message(const display_message_t* msg);
static void display_draw_status_overlay(const char* message);
static void display_clear_status_overlay(void);
static void display_redraw_header_lines(void);
static void display_refresh_main_screen(void);

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

    // Draw ASCII Coffee Bean Startup Logo
    ssd1306_display_text(&ssd1306_dev, 1, "   .-\"\"\"\"\"-.", 11, false);
    ssd1306_display_text(&ssd1306_dev, 2, "  /  Coffee \\", 13, false);
    ssd1306_display_text(&ssd1306_dev, 3, " |   Bean   |", 13, false);
    ssd1306_display_text(&ssd1306_dev, 4, " |   Smart  |", 13, false);
    ssd1306_display_text(&ssd1306_dev, 5, "  \\ Doser  /", 12, false);
    ssd1306_display_text(&ssd1306_dev, 6, "   '-.....-'", 12, false);

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
    
    vTaskDelay(1500);  // Wait for 1.5 seconds to show startup logo & let ADC settle

    // Clear display and show initial encoder display layout with actual current values
    ssd1306_clear_screen(&ssd1306_dev, false);
    
    // Small delay to allow other I2C devices (NAU7802) to access the bus
    // This prevents I2C contention during the burst of display updates
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ssd1306_display_text(&ssd1306_dev, 0, "Rotary Encoder", 14, false);
    ssd1306_display_text(&ssd1306_dev, 1, "Long press=tare", 15, false);
    
    // Populate with actual current values instead of template placeholders
    display_refresh_main_screen();

    vTaskDelay(pdMS_TO_TICKS(10));

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
                    
                case DISPLAY_MSG_INDIVIDUAL_WEIGHT:
                    display_process_individual_weight_message(&message);
                    break;
                    
                case DISPLAY_MSG_CLEAR_SCREEN:
                    ssd1306_clear_screen(&ssd1306_dev, false);
                    // After clearing, redraw everything
                    display_redraw_header_lines();
                    display_refresh_main_screen();
                    break;
                    
                case DISPLAY_MSG_SHUTDOWN:
                    display_task_running = false;
                    break;
                    
                default:
                    ESP_LOGW(TAG_DISPLAY, "Unknown message type: %d", message.type);
                    break;
            }
        }
        
        // Check if current message duration has expired (only for finite duration messages)
        if (status_message_active && current_message_duration_ms > 0) {
            TickType_t current_time = xTaskGetTickCount();
            TickType_t elapsed_time = (current_time - status_message_start_time) * portTICK_PERIOD_MS;
            
            if (elapsed_time >= current_message_duration_ms) {
                // Message duration expired
                ESP_LOGI(TAG_DISPLAY, "Status message expired after %lums", current_message_duration_ms);
                
                // Check if there are queued notifications to display
                if (notification_queue_count > 0) {
                    // Display next queued message
                    status_notification_t next = notification_queue[notification_queue_head];
                    notification_queue_head = (notification_queue_head + 1) % STATUS_NOTIFICATION_QUEUE_SIZE;
                    notification_queue_count--;
                    
                    display_draw_status_overlay(next.text);
                    status_message_start_time = xTaskGetTickCount();
                    current_message_duration_ms = next.duration_ms;
                    
                    ESP_LOGI(TAG_DISPLAY, "Displaying queued: '%s' (duration: %lums, queue: %d/%d)", 
                             next.text, next.duration_ms, notification_queue_count, STATUS_NOTIFICATION_QUEUE_SIZE);
                } else {
                    // No more messages, clear the banner
                    display_clear_status_overlay();
                    status_message_active = false;
                    ESP_LOGI(TAG_DISPLAY, "Cleared banner, no more messages");
                }
            }
        }
        
        // Periodic refresh even without messages (every 1 second)
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_update) > pdMS_TO_TICKS(333)) {
            // Show timestamp or other periodic info
            last_update = current_time;
        }
    }
    
    ESP_LOGI(TAG_DISPLAY, "Display task stopped");
    vTaskDelete(NULL);
}

static void display_process_encoder_message(const display_message_t* msg)
{
    // Update encoder data
    latest_encoder_position = msg->data.encoder_data.position;
    latest_encoder_delta = msg->data.encoder_data.delta;
    latest_button_pressed = msg->data.encoder_data.button_pressed;
    encoder_data_available = true;
    
    // Refresh the entire main screen
    display_refresh_main_screen();
}

static void display_process_status_message(const display_message_t* msg)
{
    // Create notification from incoming message
    status_notification_t notif = {
        .is_error = msg->data.system_status.is_error,
        .duration_ms = msg->data.system_status.duration_ms
    };
    strncpy(notif.text, msg->data.system_status.status_text, sizeof(notif.text) - 1);
    notif.text[sizeof(notif.text) - 1] = '\0';
    
    // Check current message duration:
    // - If current message has duration_ms = 0 (infinite), replace it immediately
    // - If current message has duration_ms > 0 (finite), queue the new message
    
    if (!status_message_active) {
        // No message currently showing, display this one immediately
        display_draw_status_overlay(notif.text);
        status_message_active = true;
        status_message_start_time = xTaskGetTickCount();
        current_message_duration_ms = notif.duration_ms;
        ESP_LOGI(TAG_DISPLAY, "Displaying: '%s' (duration: %lums)", 
                 notif.text, notif.duration_ms);
    }
    else if (current_message_duration_ms == 0) {
        // Current message is infinite (duration = 0)
        if (notification_queue_count == 0) {
            // No queue, replace current message immediately
            display_draw_status_overlay(notif.text);
            status_message_start_time = xTaskGetTickCount();
            current_message_duration_ms = notif.duration_ms;
            ESP_LOGI(TAG_DISPLAY, "Replaced infinite message with: '%s' (duration: %lums)", 
                     notif.text, notif.duration_ms);
        } else {
            // Queue exists, add new message to end and display next queued message
            if (notification_queue_count < STATUS_NOTIFICATION_QUEUE_SIZE) {
                notification_queue[notification_queue_tail] = notif;
                notification_queue_tail = (notification_queue_tail + 1) % STATUS_NOTIFICATION_QUEUE_SIZE;
                notification_queue_count++;
                ESP_LOGI(TAG_DISPLAY, "Queued new message, displaying next from queue");
            } else {
                ESP_LOGW(TAG_DISPLAY, "Queue full, dropping: '%s'", notif.text);
            }
            
            // Display next message from queue
            status_notification_t next = notification_queue[notification_queue_head];
            notification_queue_head = (notification_queue_head + 1) % STATUS_NOTIFICATION_QUEUE_SIZE;
            notification_queue_count--;
            
            display_draw_status_overlay(next.text);
            status_message_start_time = xTaskGetTickCount();
            current_message_duration_ms = next.duration_ms;
            ESP_LOGI(TAG_DISPLAY, "Displaying queued: '%s' (duration: %lums, queue: %d/%d)", 
                     next.text, next.duration_ms, notification_queue_count, STATUS_NOTIFICATION_QUEUE_SIZE);
        }
    }
    else {
        // Current message has finite duration, queue the new message
        if (notification_queue_count < STATUS_NOTIFICATION_QUEUE_SIZE) {
            notification_queue[notification_queue_tail] = notif;
            notification_queue_tail = (notification_queue_tail + 1) % STATUS_NOTIFICATION_QUEUE_SIZE;
            notification_queue_count++;
            ESP_LOGI(TAG_DISPLAY, "Queued notification: '%s' (queue: %d/%d)", 
                     notif.text, notification_queue_count, STATUS_NOTIFICATION_QUEUE_SIZE);
        } else {
            ESP_LOGW(TAG_DISPLAY, "Notification queue full, dropping: '%s'", notif.text);
        }
    }
}

static void display_process_coffee_message(const display_message_t* msg)
{
    // Store the latest weight data for display on encoder screen
    latest_container_weight = msg->data.coffee_info.container_weight;  // Channel A (container)
    latest_dosage_weight = msg->data.coffee_info.dosage_weight;        // Channel B (dosage cup)
    weight_data_available = true;
    
    ESP_LOGD(TAG_DISPLAY, "Coffee Info - Container: %.1fg, Dosage: %.1fg, Dosing: %s",
             msg->data.coffee_info.container_weight,
             msg->data.coffee_info.dosage_weight,
             msg->data.coffee_info.dosing_active ? "YES" : "NO");

    // Refresh the entire main screen to show updated weights
    display_refresh_main_screen();
}

static void display_process_individual_weight_message(const display_message_t* msg)
{
    // Update the specific channel's weight value
    switch (msg->data.individual_weight.channel) {
        case 0:  // Channel A - Container
            latest_container_weight = msg->data.individual_weight.weight;
            weight_data_available = true;
            ESP_LOGD(TAG_DISPLAY, "Channel %d (Container): %.1fg", 
                     msg->data.individual_weight.channel, msg->data.individual_weight.weight);
            break;
            
        case 1:  // Channel B - Dosage cup
            latest_dosage_weight = msg->data.individual_weight.weight;
            weight_data_available = true;
            ESP_LOGD(TAG_DISPLAY, "Channel %d (Dosage): %.1fg", 
                     msg->data.individual_weight.channel, msg->data.individual_weight.weight);
            break;
            
        default:
            ESP_LOGE(TAG_DISPLAY, "Unknown channel %d with weight %.1fg", 
                     msg->data.individual_weight.channel, msg->data.individual_weight.weight);
            return;
    }
    
    // Refresh the main screen to show the updated weight
    display_refresh_main_screen();
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

static void display_refresh_main_screen(void)
{
    char position_text[32];
    char delta_text[32];
    char button_text[16];
    char container_text[20];
    char dosage_text[20];
    
    // Update encoder position (use actual value or 0 if no data yet)
    snprintf(position_text, sizeof(position_text), "Pos: %-8ld", 
             encoder_data_available ? latest_encoder_position : 0);
    ssd1306_display_text(&ssd1306_dev, 2, position_text, strlen(position_text), false);
    
    // Update encoder delta (use actual value or 0 if no data yet)
    if (encoder_data_available && latest_encoder_delta == -999) {
        // Special case for reset operation
        snprintf(delta_text, sizeof(delta_text), "Delta: 0     ");
    } else {
        int32_t delta_value = encoder_data_available ? latest_encoder_delta : 0;
        snprintf(delta_text, sizeof(delta_text), "Delta: %s%-6ld", 
                 delta_value > 0 ? "+" : "", delta_value);
    }
    ssd1306_display_text(&ssd1306_dev, 4, delta_text, strlen(delta_text), false);
    
    // Update button state (use actual value or default to "---")
    if (encoder_data_available && latest_button_pressed && latest_encoder_delta == -999) {
        snprintf(button_text, sizeof(button_text), "Btn: RESET!");
    } else {
        snprintf(button_text, sizeof(button_text), "Btn: %-6s", 
                 (encoder_data_available && latest_button_pressed) ? "PRESS" : "---");
    }
    ssd1306_display_text(&ssd1306_dev, 5, button_text, strlen(button_text), false);
    
    // Update weight measurements with new error code handling
    // Container weight (Channel A)
    if (latest_container_weight == WEIGHT_DISPLAY_NO_CONN) {
        snprintf(container_text, sizeof(container_text), "Cont: NO CONN");
    } else if (latest_container_weight == WEIGHT_DISPLAY_TARING) {
        snprintf(container_text, sizeof(container_text), "Cont:  ----g ");
    } else if (latest_container_weight < 0) {
        // Any other negative value is an unknown error
        snprintf(container_text, sizeof(container_text), "Cont: ERROR  ");
    } else if (!weight_data_available) {
        // No weight data received yet - show actual value or 0
        snprintf(container_text, sizeof(container_text), "Cont:%6.1fg", 0.0f);
    } else {
        // Normal weight display with 0.1g precision
        snprintf(container_text, sizeof(container_text), "Cont:%6.1fg", latest_container_weight);
    }
    
    // Dosage weight (Channel B)
    if (latest_dosage_weight == WEIGHT_DISPLAY_NO_CONN) {
        snprintf(dosage_text, sizeof(dosage_text), "Dose: NO CONN");
    } else if (latest_dosage_weight == WEIGHT_DISPLAY_TARING) {
        snprintf(dosage_text, sizeof(dosage_text), "Dose:  ----g ");
    } else if (latest_dosage_weight < 0) {
        // Any other negative value is an unknown error
        snprintf(dosage_text, sizeof(dosage_text), "Dose: ERROR  ");
    } else if (!weight_data_available) {
        // No weight data received yet - show "------" for disabled channel
        snprintf(dosage_text, sizeof(dosage_text), "Dose: ------");
    } else {
        // Normal weight display with 0.1g precision
        snprintf(dosage_text, sizeof(dosage_text), "Dose:%6.1fg", latest_dosage_weight);
    }
    
    ssd1306_display_text(&ssd1306_dev, 6, container_text, strlen(container_text), false);
    ssd1306_display_text(&ssd1306_dev, 7, dosage_text, strlen(dosage_text), false);
    
    ESP_LOGD(TAG_DISPLAY, "Screen refreshed - Pos: %ld, Delta: %ld, Btn: %s, Cont: %.1fg, Dose: %.1fg",
             encoder_data_available ? latest_encoder_position : 0, 
             encoder_data_available ? latest_encoder_delta : 0, 
             (encoder_data_available && latest_button_pressed) ? "PRESS" : "---",
             latest_container_weight, latest_dosage_weight);
}

// Status message overlay functions
static void display_draw_status_overlay(const char* message)
{
    // Calculate message dimensions and centering
    int msg_len = strlen(message);
    int start_x = (SSD1306_WIDTH - (msg_len * 6)) / 2;  // Center horizontally (6px per char)
    int start_y = 0;  // Start at top
    int box_width = msg_len * 6 + 8;  // Message width + padding
    int box_height = 16;  // Two lines of 8px text
    int box_x = (SSD1306_WIDTH - box_width) / 2;
    
    // Clear the area first using clear_line for top two rows
    ssd1306_clear_line(&ssd1306_dev, 0, false);
    ssd1306_clear_line(&ssd1306_dev, 1, false);
    
    // Draw border using lines
    // Top and bottom borders
    _ssd1306_line(&ssd1306_dev, box_x, start_y, box_x + box_width - 1, start_y, true);
    _ssd1306_line(&ssd1306_dev, box_x, start_y + box_height - 1, box_x + box_width - 1, start_y + box_height - 1, true);
    // Left and right borders
    _ssd1306_line(&ssd1306_dev, box_x, start_y, box_x, start_y + box_height - 1, true);
    _ssd1306_line(&ssd1306_dev, box_x + box_width - 1, start_y, box_x + box_width - 1, start_y + box_height - 1, true);
    
    // Draw white text in the center of the box
    // Use regular text size and center it within the box
    ssd1306_display_text(&ssd1306_dev, 0, message, msg_len, true);  // Inverted text (white on black)
}

static void display_clear_status_overlay(void)
{
    // Redraw the header lines to clear the overlay
    display_redraw_header_lines();
    
    // Refresh the main screen content after clearing overlay
    display_refresh_main_screen();
}

static void display_redraw_header_lines(void)
{
    // Clear the top two lines
    ssd1306_clear_line(&ssd1306_dev, 0, false);
    ssd1306_clear_line(&ssd1306_dev, 1, false);
    
    // Redraw header text
    ssd1306_display_text(&ssd1306_dev, 0, "Rotary Encoder", 14, false);
    ssd1306_display_text(&ssd1306_dev, 1, "Long press=tare", 15, false);
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

esp_err_t display_send_system_status(const char* status_text, bool is_error, uint32_t duration_ms)
{
    display_message_t msg = {
        .type = DISPLAY_MSG_SYSTEM_STATUS,
        .timestamp = esp_timer_get_time() / 1000,
    };
    
    strncpy(msg.data.system_status.status_text, status_text, sizeof(msg.data.system_status.status_text) - 1);
    msg.data.system_status.status_text[sizeof(msg.data.system_status.status_text) - 1] = '\0';
    msg.data.system_status.is_error = is_error;
    msg.data.system_status.duration_ms = duration_ms;
    
    if (xQueueSend(display_message_queue, &msg, 0) != pdTRUE) {
        queue_overflows++;
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t display_send_coffee_info(float container_weight, float dosage_weight, bool dosing_active)
{
    display_message_t msg = {
        .type = DISPLAY_MSG_COFFEE_INFO,
        .timestamp = esp_timer_get_time() / 1000,
        .data.coffee_info = {
            .container_weight = container_weight,
            .dosage_weight = dosage_weight,
            .dosing_active = dosing_active
        }
    };
    
    if (xQueueSend(display_message_queue, &msg, 0) != pdTRUE) {
        queue_overflows++;
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t display_send_individual_weight(float weight, int channel)
{
    display_message_t msg = {
        .type = DISPLAY_MSG_INDIVIDUAL_WEIGHT,
        .timestamp = esp_timer_get_time() / 1000,
        .data.individual_weight = {
            .weight = weight,
            .channel = channel
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