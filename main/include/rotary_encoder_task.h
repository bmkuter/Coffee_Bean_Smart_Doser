/*
 * Rotary Encoder Task Header - User Input Interface
 * 
 * Handles the Seesaw rotary encoder for coffee dosage adjustment.
 * Uses the I2C thread system for communication.
 */

#ifndef ROTARY_ENCODER_TASK_H
#define ROTARY_ENCODER_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Rotary Encoder Task Configuration
#define ROTARY_ENCODER_TASK_STACK_SIZE  3072
#define ROTARY_ENCODER_TASK_PRIORITY    5       // High priority for responsiveness
#define ROTARY_ENCODER_POLL_RATE_MS     25      // Fast polling for responsive encoder

// Seesaw Hardware ID Codes (from Adafruit seesaw library)
#define SEESAW_HW_ID_CODE_SAMD09        0x55
#define SEESAW_HW_ID_CODE_TINY817       0x84
#define SEESAW_HW_ID_CODE_TINY807       0x85
#define SEESAW_HW_ID_CODE_TINY816       0x86
#define SEESAW_HW_ID_CODE_TINY806       0x87
#define SEESAW_HW_ID_CODE_TINY1616      0x88
#define SEESAW_HW_ID_CODE_TINY1617      0x89

// Seesaw Status Register Map
#define SEESAW_STATUS_HW_ID             0x01
#define SEESAW_STATUS_VERSION           0x02
#define SEESAW_STATUS_OPTIONS           0x03
#define SEESAW_STATUS_TEMP              0x04

// Seesaw Rotary Encoder Register Map
#define SEESAW_STATUS_BASE              0x00
#define SEESAW_STATUS_SWRST             0x7F

#define SEESAW_GPIO_BASE                0x01
#define SEESAW_GPIO_DIRSET_BULK         0x02
#define SEESAW_GPIO_DIRCLR_BULK         0x03
#define SEESAW_GPIO_BULK                0x04
#define SEESAW_GPIO_BULK_SET            0x05
#define SEESAW_GPIO_BULK_CLR            0x06
#define SEESAW_GPIO_BULK_TOGGLE         0x07
#define SEESAW_GPIO_INTENSET            0x08
#define SEESAW_GPIO_INTENCLR            0x09
#define SEESAW_GPIO_INTFLAG             0x0A
#define SEESAW_GPIO_PULLENSET           0x0B
#define SEESAW_GPIO_PULLENCLR           0x0C

#define SEESAW_TIMER_BASE               0x08
#define SEESAW_TIMER_STATUS             0x00
#define SEESAW_TIMER_PWM                0x01
#define SEESAW_TIMER_FREQ               0x02

#define SEESAW_ADC_BASE                 0x09
#define SEESAW_ADC_STATUS               0x00
#define SEESAW_ADC_INTEN                0x02
#define SEESAW_ADC_INTENCLR             0x03
#define SEESAW_ADC_WINMODE              0x04
#define SEESAW_ADC_WINTHRESH            0x05
#define SEESAW_ADC_CHANNEL_OFFSET       0x07

#define SEESAW_ENCODER_BASE             0x11
#define SEESAW_ENCODER_STATUS           0x00
#define SEESAW_ENCODER_INTENSET         0x10
#define SEESAW_ENCODER_INTENCLR         0x20
#define SEESAW_ENCODER_POSITION         0x30
#define SEESAW_ENCODER_DELTA            0x40

#define SEESAW_NEOPIXEL_BASE            0x0E
#define SEESAW_NEOPIXEL_STATUS          0x00
#define SEESAW_NEOPIXEL_PIN             0x01
#define SEESAW_NEOPIXEL_SPEED           0x02
#define SEESAW_NEOPIXEL_BUF_LENGTH      0x03
#define SEESAW_NEOPIXEL_BUF             0x04
#define SEESAW_NEOPIXEL_SHOW            0x05

// Button pin on rotary encoder
#define SEESAW_BUTTON_PIN               24

// Rotary Encoder Events
typedef enum {
    ROTARY_EVENT_NONE,
    ROTARY_EVENT_CLOCKWISE,
    ROTARY_EVENT_COUNTERCLOCKWISE,
    ROTARY_EVENT_BUTTON_PRESS,
    ROTARY_EVENT_BUTTON_RELEASE,
    ROTARY_EVENT_LONG_PRESS
} rotary_event_t;

// Rotary Encoder Data
typedef struct {
    int32_t position;
    int32_t delta;
    bool button_pressed;
    bool button_changed;
    rotary_event_t last_event;
    uint32_t event_count;
} rotary_encoder_data_t;

// Event callback function type
typedef void (*rotary_event_callback_t)(rotary_event_t event, int32_t position, int32_t delta);

/**
 * @brief Initialize the rotary encoder task
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rotary_encoder_task_init(void);

/**
 * @brief Start the rotary encoder task
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rotary_encoder_task_start(void);

/**
 * @brief Stop the rotary encoder task
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rotary_encoder_task_stop(void);

/**
 * @brief Get current rotary encoder data
 * @param data Pointer to structure to fill with current data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rotary_encoder_get_data(rotary_encoder_data_t* data);

/**
 * @brief Set the rotary encoder position
 * @param position New position value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rotary_encoder_set_position(int32_t position);

/**
 * @brief Register event callback function
 * @param callback Function to call on encoder events
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rotary_encoder_register_callback(rotary_event_callback_t callback);

/**
 * @brief Set NeoPixel color on the rotary encoder
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rotary_encoder_set_neopixel(uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif

#endif // ROTARY_ENCODER_TASK_H