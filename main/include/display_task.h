/*
 * Display Task Header - OLED Status Display
 * 
 * Handles SSD1306 OLED display updates showing:
 * - Current encoder position
 * - System status
 * - Coffee dosing information
 * 
 * Uses FreeRTOS message queues for thread-safe data sharing
 */

#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Display Task Configuration
#define DISPLAY_TASK_STACK_SIZE     4096
#define DISPLAY_TASK_PRIORITY       4        // Higher priority to handle queue better (was 2)
#define DISPLAY_UPDATE_RATE_MS      100      // 10 FPS update rate
#define DISPLAY_MESSAGE_QUEUE_SIZE  64       // Larger buffer for display updates (was 0x20/32)

// Weight Display Error Codes (float values, grams can never be negative in normal operation)
#define WEIGHT_DISPLAY_NO_CONN      -999.0f  // Strain gauge not connected
#define WEIGHT_DISPLAY_TARING       -998.0f  // Tare operation in progress
// Note: Any negative weight indicates an error state since grams cannot be negative

// Display Message Types
typedef enum {
    DISPLAY_MSG_ENCODER_UPDATE,
    DISPLAY_MSG_SYSTEM_STATUS,
    DISPLAY_MSG_COFFEE_INFO,
    DISPLAY_MSG_INDIVIDUAL_WEIGHT,  // New: single weight from one channel
    DISPLAY_MSG_CLEAR_SCREEN,
    DISPLAY_MSG_SHUTDOWN
} display_message_type_t;

// Display Message Structure
typedef struct {
    display_message_type_t type;
    uint32_t timestamp;
    union {
        struct {
            int32_t position;
            int32_t delta;
            bool button_pressed;
        } encoder_data;
        
        struct {
            char status_text[32];
            bool is_error;
            uint32_t duration_ms;  // Duration to show message (0 = permanent until cleared)
        } system_status;
        
        struct {
            float container_weight;  // Channel A weight in grams
            float dosage_weight;     // Channel B weight in grams
            bool dosing_active;
        } coffee_info;
        
        struct {
            float weight;    // Weight value in grams (or error code)
            int channel;     // Channel identifier (0=container/A, 1=dosage/B, etc.)
        } individual_weight;
    } data;
} display_message_t;

/**
 * @brief Initialize the display task system
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_task_init(void);

/**
 * @brief Start the display task
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_task_start(void);

/**
 * @brief Stop the display task
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_task_stop(void);

/**
 * @brief Send encoder data to display (non-blocking)
 * @param position Current encoder position
 * @param delta Change in position
 * @param button_pressed Button state
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t display_send_encoder_data(int32_t position, int32_t delta, bool button_pressed);

/**
 * @brief Send system status message to display
 * @param status_text Status message text
 * @param is_error True if this is an error message
 * @param duration_ms Duration to show message in milliseconds (0 = permanent until cleared)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t display_send_system_status(const char* status_text, bool is_error, uint32_t duration_ms);

/**
 * @brief Send weight measurements to display (legacy - kept for compatibility)
 * @param container_weight Container weight in grams (Channel A)
 * @param dosage_weight Dosage cup weight in grams (Channel B)
 * @param dosing_active True if dosing is in progress
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t display_send_coffee_info(float container_weight, float dosage_weight, bool dosing_active);

/**
 * @brief Send individual weight measurement from a single channel (scalable architecture)
 * @param weight Weight value in grams (or error code like WEIGHT_DISPLAY_NO_CONN)
 * @param channel Channel identifier (0=container/Channel A, 1=dosage/Channel B, etc.)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t display_send_individual_weight(float weight, int channel);

/**
 * @brief Clear the display screen
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t display_clear_screen(void);

/**
 * @brief Get display task statistics
 * @param messages_processed Total messages processed
 * @param queue_overflows Number of queue overflow events
 * @return ESP_OK on success
 */
esp_err_t display_get_stats(uint32_t* messages_processed, uint32_t* queue_overflows);

#ifdef __cplusplus
}
#endif

#endif // DISPLAY_TASK_H