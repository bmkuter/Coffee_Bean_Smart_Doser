/*
 * Rotary Encoder Task - User Input Interface
 * 
 * Handles communication with the Seesaw rotary encoder using the I2C thread system.
 * Provides rotary position tracking, button detection, and NeoPixel control.
 */

#include "rotary_encoder_task.h"
#include "display_task.h"
#include "shared_i2c_bus.h"
#include "coffee_doser_config.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include <string.h>

static TaskHandle_t rotary_task_handle = NULL;
static bool rotary_task_running = false;
static SemaphoreHandle_t rotary_data_mutex = NULL;
static rotary_encoder_data_t rotary_data = {0};
static rotary_event_callback_t event_callback = NULL;

// I2C device handle for rotary encoder
static i2c_master_dev_handle_t rotary_i2c_dev_handle = NULL;

// Internal functions
static void rotary_encoder_task(void* pvParameters);
static esp_err_t rotary_i2c_init(void);
static esp_err_t seesaw_probe_device(void);
static esp_err_t seesaw_soft_reset(void);
static esp_err_t seesaw_verify_hardware_id(void);
static esp_err_t seesaw_init_encoder(void);
static esp_err_t seesaw_init_button(void);
// static esp_err_t seesaw_init_neopixel(void);  // Commented out - unused
static esp_err_t seesaw_read_encoder_position(int32_t* position);
static esp_err_t seesaw_read_encoder_delta(int32_t* delta);
static esp_err_t seesaw_read_button_state(bool* pressed);
static esp_err_t seesaw_write_register(uint8_t reg_base, uint8_t reg_addr, uint8_t* data, size_t length);
static esp_err_t seesaw_read_register(uint8_t reg_base, uint8_t reg_addr, uint8_t* data, size_t length, uint16_t delay_us);

esp_err_t rotary_encoder_task_init(void)
{
    if (rotary_task_running) {
        ESP_LOGW(TAG_ROTARY, "Rotary encoder task already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG_ROTARY, "Initializing rotary encoder task");

    // Initialize I2C for rotary encoder
    esp_err_t ret = rotary_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ROTARY, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create data mutex
    rotary_data_mutex = xSemaphoreCreateMutex();
    if (rotary_data_mutex == NULL) {
        ESP_LOGE(TAG_ROTARY, "Failed to create rotary data mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize rotary encoder data
    memset(&rotary_data, 0, sizeof(rotary_encoder_data_t));

    ESP_LOGI(TAG_ROTARY, "Rotary encoder task initialized");
    return ESP_OK;
}

esp_err_t rotary_encoder_task_start(void)
{
    if (rotary_task_running) {
        ESP_LOGW(TAG_ROTARY, "Rotary encoder task already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG_ROTARY, "Starting rotary encoder task");

    // Set flag before creating task to prevent race condition
    rotary_task_running = true;
    
    // Create the task
    BaseType_t ret = xTaskCreate(
        rotary_encoder_task,
        "rotary_encoder",
        ROTARY_ENCODER_TASK_STACK_SIZE,
        NULL,
        ROTARY_ENCODER_TASK_PRIORITY,
        &rotary_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG_ROTARY, "Failed to create rotary encoder task");
        rotary_task_running = false;  // Reset flag on failure
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG_ROTARY, "Rotary encoder task started");

    return ESP_OK;
}

static esp_err_t rotary_i2c_init(void)
{
    ESP_LOGI(TAG_ROTARY, "Adding rotary encoder to shared I2C bus");

    // Initialize the shared I2C bus
    esp_err_t ret = shared_i2c_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ROTARY, "Failed to initialize shared I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add rotary encoder device to the shared bus
    ret = shared_i2c_add_device(ROTARY_ENCODER_ADDR, 100000, &rotary_i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ROTARY, "Failed to add rotary encoder to shared I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_ROTARY, "Rotary encoder added to shared I2C bus successfully");
    return ESP_OK;
}

static void rotary_encoder_task(void* pvParameters)
{
    ESP_LOGI(TAG_ROTARY, "Rotary encoder task running");

    // First, probe the device to see if it's reachable
    ESP_LOGI(TAG_ROTARY, "Probing Seesaw device at address 0x%02X...", ROTARY_ENCODER_ADDR);
    esp_err_t ret = seesaw_probe_device();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ROTARY, "Seesaw device not found at 0x%02X: %s", ROTARY_ENCODER_ADDR, esp_err_to_name(ret));
        ESP_LOGE(TAG_ROTARY, "HALTING: Rotary encoder is required for operation!");
        rotary_task_running = false;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_ROTARY, "Seesaw device found and responding at 0x%02X", ROTARY_ENCODER_ADDR);

    // Initialize the Seesaw device with reset
    ret = seesaw_soft_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ROTARY, "Failed to reset Seesaw device: %s", esp_err_to_name(ret));
        rotary_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify hardware ID after reset (with retry logic like Arduino)
    ret = seesaw_verify_hardware_id();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ROTARY, "Failed to verify Seesaw hardware ID: %s", esp_err_to_name(ret));
        rotary_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    // Initialize encoder
    ret = seesaw_init_encoder();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ROTARY, "Failed to initialize encoder: %s", esp_err_to_name(ret));
        rotary_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    // Initialize button
    ret = seesaw_init_button();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ROTARY, "Failed to initialize button: %s", esp_err_to_name(ret));
        rotary_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    // // Initialize NeoPixel
    // ret = seesaw_init_neopixel();
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG_ROTARY, "Failed to initialize NeoPixel: %s", esp_err_to_name(ret));
    //     // Continue without NeoPixel (non-critical)
    // }

    // // Set initial NeoPixel color (dim blue to indicate ready)
    // rotary_encoder_set_neopixel(0, 0, 32);

    ESP_LOGI(TAG_ROTARY, "Seesaw rotary encoder initialized successfully");

    bool last_button_state = false;
    bool last_raw_button_state = false;
    TickType_t button_change_time = 0;
    const TickType_t button_debounce_ticks = pdMS_TO_TICKS(20);  // 250ms debounce
    
    // Long press detection for reset functionality
    TickType_t button_press_start = 0;
    const TickType_t long_press_duration = pdMS_TO_TICKS(1250);  // 1250ms long press
    bool long_press_triggered = false;
    bool showing_long_press_feedback = false;
    
    // Multi-click detection (double-click)
    TickType_t first_click_time = 0;
    const TickType_t click_window = pdMS_TO_TICKS(450);  // 450ms window between clicks
    int click_count = 0;  // Track number of clicks
    bool waiting_for_next_click = false;
    
    // Rate limiting for display updates
    TickType_t last_display_update = 0;
    const TickType_t display_update_interval = pdMS_TO_TICKS(100);  // Max 10Hz display updates (was 50ms/20Hz)
    bool pending_display_update = false;
    int32_t latest_position = 0;
    int32_t cumulative_delta = 0;

    while (1) {
        // Read encoder delta first (more efficient than position for change detection)
        int32_t delta;
        ret = seesaw_read_encoder_delta(&delta);
        if (ret == ESP_OK && delta != 0) {
            // Only read position if there was a change
            int32_t position;
            ret = seesaw_read_encoder_position(&position);
            if (ret == ESP_OK) {
                // Update data (reverse direction: negate values for intuitive operation)
                if (xSemaphoreTake(rotary_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    rotary_data.position = -position;  // Reverse position
                    rotary_data.delta = -delta;        // Reverse delta (clockwise = positive)
                    rotary_data.event_count++;

                    // Determine rotation direction (using corrected delta)
                    rotary_event_t event = (rotary_data.delta > 0) ? ROTARY_EVENT_CLOCKWISE : ROTARY_EVENT_COUNTERCLOCKWISE;
                    rotary_data.last_event = event;

                    xSemaphoreGive(rotary_data_mutex);

                    // Call callback if registered (use corrected values)
                    if (event_callback != NULL) {
                        event_callback(event, rotary_data.position, rotary_data.delta);
                    }

                    ESP_LOGI(TAG_ROTARY, "Encoder: pos=%ld, delta=%ld (%s)",
                             rotary_data.position, rotary_data.delta, (rotary_data.delta > 0) ? "CW" : "CCW");
                    
                    // Accumulate changes for rate-limited display updates (use corrected values)
                    latest_position = rotary_data.position;
                    cumulative_delta += rotary_data.delta;
                    pending_display_update = true;
                }
            }
        }

        // Read button state with debouncing and long press detection
        bool raw_button_pressed;
        ret = seesaw_read_button_state(&raw_button_pressed);
        if (ret == ESP_OK) {
            TickType_t current_time = xTaskGetTickCount();
            
            // Check if multi-click window has expired - fire single click if needed
            if (waiting_for_next_click && click_count == 1) {
                TickType_t time_since_first_click = current_time - first_click_time;
                    
                if (time_since_first_click >= click_window) {
                    // Window expired with only one click - fire single click event
                    ESP_LOGI(TAG_ROTARY, "Single click detected (window expired)");
                    waiting_for_next_click = false;
                    click_count = 0;
                    
                    // Fire single click callback
                    if (event_callback != NULL) {
                        event_callback(ROTARY_EVENT_BUTTON_RELEASE, rotary_data.position, 0);
                    }
                }
            }
            
            // Check if raw button state has changed
            if (raw_button_pressed != last_raw_button_state) {
                // Raw state changed, record the time and update raw state
                button_change_time = current_time;
                last_raw_button_state = raw_button_pressed;
                
                // Reset long press tracking on any state change
                if (raw_button_pressed) {
                    // Button just pressed - start tracking for long press
                    button_press_start = current_time;
                    long_press_triggered = false;
                    showing_long_press_feedback = false;
                } else {
                    // Button just released - clear long press tracking
                    button_press_start = 0;
                    long_press_triggered = false;
                    if (showing_long_press_feedback) {
                        showing_long_press_feedback = false;
                        // Mark for rate-limited display update to clear long press feedback
                        latest_position = rotary_data.position;
                        pending_display_update = true;
                    }
                }
            } else if (raw_button_pressed != last_button_state) {
                // Raw state is stable and different from debounced state
                // Check if enough time has passed for debouncing
                if ((current_time - button_change_time) >= button_debounce_ticks) {
                    // Button state change is valid after debounce period
                    if (xSemaphoreTake(rotary_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        rotary_data.button_pressed = raw_button_pressed;
                        rotary_data.button_changed = true;
                        rotary_data.event_count++;

                        rotary_event_t event = raw_button_pressed ? ROTARY_EVENT_BUTTON_PRESS : ROTARY_EVENT_BUTTON_RELEASE;
                        rotary_data.last_event = event;

                        xSemaphoreGive(rotary_data_mutex);

                        // Multi-click detection on button release
                        if (!raw_button_pressed) {  // Button was just released
                            click_count++;
                            
                            if (click_count == 1) {
                                // First click - start waiting to see if more clicks come
                                first_click_time = current_time;
                                waiting_for_next_click = true;
                                ESP_LOGD(TAG_ROTARY, "First click - waiting for potential multi-click");
                                // Don't call callback yet - wait for window to expire
                            } else if (click_count == 2) {
                                // Second click - check if within window
                                if ((current_time - first_click_time) < click_window) {
                                    // Valid double-click - fire immediately
                                    ESP_LOGI(TAG_ROTARY, "Double-click detected!");
                                    waiting_for_next_click = false;
                                    click_count = 0;
                                    
                                    event = ROTARY_EVENT_DOUBLE_CLICK;
                                    if (xSemaphoreTake(rotary_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                        rotary_data.last_event = event;
                                        xSemaphoreGive(rotary_data_mutex);
                                    }
                                    
                                    // Call callback for double-click
                                    if (event_callback != NULL) {
                                        event_callback(event, rotary_data.position, 0);
                                    }
                                } else {
                                    // Second click too slow - treat first click as single, start new sequence
                                    ESP_LOGI(TAG_ROTARY, "Single click detected (timeout)");
                                    
                                    // Fire single click callback for the first click
                                    event = ROTARY_EVENT_BUTTON_RELEASE;
                                    if (event_callback != NULL) {
                                        event_callback(event, rotary_data.position, 0);
                                    }
                                    
                                    // Start new sequence with this click
                                    click_count = 1;
                                    first_click_time = current_time;
                                    waiting_for_next_click = true;
                                }
                            } else {
                                // More than 2 clicks - reset (we don't use triple-click anymore)
                                click_count = 1;
                                first_click_time = current_time;
                                waiting_for_next_click = true;
                            }
                        } else {
                            // Button press event - only call callback for press events
                            ESP_LOGI(TAG_ROTARY, "Button: PRESSED (debounced)");
                            if (event_callback != NULL) {
                                event_callback(ROTARY_EVENT_BUTTON_PRESS, rotary_data.position, 0);
                            }
                        }

                        // Mark for rate-limited display update instead of immediate send
                        latest_position = rotary_data.position;
                        pending_display_update = true;
                    }

                    last_button_state = raw_button_pressed;
                }
            }
            
            // Check for long press while button is held down
            if (raw_button_pressed && last_button_state && !long_press_triggered && button_press_start > 0) {
                if ((current_time - button_press_start) >= long_press_duration) {
                    // Long press detected - trigger callback
                    long_press_triggered = true;
                    showing_long_press_feedback = true;
                    
                    ESP_LOGI(TAG_ROTARY, "Long press detected - triggering callback");
                    
                    // Call callback if registered
                    if (event_callback != NULL) {
                        event_callback(ROTARY_EVENT_LONG_PRESS, rotary_data.position, 0);
                    }
                    
                    // Mark for rate-limited display update instead of immediate send
                    latest_position = rotary_data.position;
                    pending_display_update = true;
                }
            }
        }

        // Rate-limited display updates to prevent queue overflow
        TickType_t current_time = xTaskGetTickCount();
        if (pending_display_update && 
            (current_time - last_display_update) >= display_update_interval) {
            
            // Send accumulated encoder data to display
            esp_err_t display_ret = display_send_encoder_data(latest_position, cumulative_delta, rotary_data.button_pressed);
            if (display_ret == ESP_OK) {
                // Successfully sent, reset accumulation
                last_display_update = current_time;
                cumulative_delta = 0;
                pending_display_update = false;
            }
            // If display queue is still full, we'll try again next cycle
        }

        // Task delay
        vTaskDelay(pdMS_TO_TICKS(ROTARY_ENCODER_POLL_RATE_MS));
    }

    ESP_LOGI(TAG_ROTARY, "Rotary encoder task stopped");
    vTaskDelete(NULL);
}

static esp_err_t seesaw_soft_reset(void)
{
    uint8_t reset_cmd = 0xFF;
    return seesaw_write_register(SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, &reset_cmd, 1);
}

static esp_err_t seesaw_verify_hardware_id(void)
{
    // Retry logic similar to Arduino implementation
    const int max_retries = 10;
    bool found = false;
    
    for (int retry = 0; retry < max_retries && !found; retry++) {
        uint8_t hw_id = 0;
        esp_err_t ret = seesaw_read_register(SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID, &hw_id, 1, 125);
        
        if (ret == ESP_OK) {
            // Check if this is a known Seesaw hardware ID
            if (hw_id == SEESAW_HW_ID_CODE_SAMD09 || 
                hw_id == SEESAW_HW_ID_CODE_TINY817 ||
                hw_id == SEESAW_HW_ID_CODE_TINY807 || 
                hw_id == SEESAW_HW_ID_CODE_TINY816 ||
                hw_id == SEESAW_HW_ID_CODE_TINY806 || 
                hw_id == SEESAW_HW_ID_CODE_TINY1616 ||
                hw_id == SEESAW_HW_ID_CODE_TINY1617) {
                    
                ESP_LOGI(TAG_ROTARY, "Seesaw hardware ID verified: 0x%02X", hw_id);
                found = true;
                return ESP_OK;
            } else {
                ESP_LOGW(TAG_ROTARY, "Unknown hardware ID: 0x%02X (retry %d/%d)", hw_id, retry + 1, max_retries);
            }
        } else {
            ESP_LOGW(TAG_ROTARY, "Failed to read hardware ID (retry %d/%d): %s", retry + 1, max_retries, esp_err_to_name(ret));
        }
        
        // Delay between retries
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGE(TAG_ROTARY, "Failed to verify Seesaw hardware ID after %d retries", max_retries);
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t seesaw_init_encoder(void)
{
    // Enable encoder interrupt - this is critical for responsiveness
    uint8_t enable_cmd = 0x01;
    esp_err_t ret = seesaw_write_register(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET, &enable_cmd, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ROTARY, "Failed to enable encoder interrupt: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG_ROTARY, "Encoder interrupt enabled");
    return ESP_OK;
}

static esp_err_t seesaw_init_button(void)
{
    // Set button pin as input
    uint32_t pin_mask = 1UL << SEESAW_BUTTON_PIN;
    uint8_t pin_data[4];
    pin_data[0] = (pin_mask >> 24) & 0xFF;
    pin_data[1] = (pin_mask >> 16) & 0xFF;
    pin_data[2] = (pin_mask >> 8) & 0xFF;
    pin_data[3] = pin_mask & 0xFF;

    esp_err_t ret = seesaw_write_register(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, pin_data, 4);
    if (ret != ESP_OK) {
        return ret;
    }

    // Enable pull-up
    return seesaw_write_register(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, pin_data, 4);
}

// Currently unused - keep for future NeoPixel support
/*
static esp_err_t seesaw_init_neopixel(void)
{
    // Set NeoPixel pin
    uint8_t pin = 6;  // NeoPixel pin on Seesaw
    esp_err_t ret = seesaw_write_register(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_PIN, &pin, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set buffer length to 3 bytes (1 pixel * 3 colors)
    uint8_t buf_len[2] = {0x00, 0x03};
    ret = seesaw_write_register(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF_LENGTH, buf_len, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}
*/

static esp_err_t seesaw_read_encoder_position(int32_t* position)
{
    uint8_t data[4];
    esp_err_t ret = seesaw_read_register(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION, data, 4, 125);
    if (ret == ESP_OK) {
        *position = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
    }
    return ret;
}

static esp_err_t seesaw_read_encoder_delta(int32_t* delta)
{
    uint8_t data[4];
    esp_err_t ret = seesaw_read_register(SEESAW_ENCODER_BASE, SEESAW_ENCODER_DELTA, data, 4, 125);
    if (ret == ESP_OK) {
        *delta = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
    }
    return ret;
}

static esp_err_t seesaw_read_button_state(bool* pressed)
{
    uint8_t data[4];
    esp_err_t ret = seesaw_read_register(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK, data, 4, 125);
    if (ret == ESP_OK) {
        uint32_t gpio_state = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
        *pressed = !(gpio_state & (1UL << SEESAW_BUTTON_PIN));  // Button is active low
    }
    return ret;
}

static esp_err_t seesaw_write_register(uint8_t reg_base, uint8_t reg_addr, uint8_t* data, size_t length)
{
    // For Seesaw protocol: [reg_base][reg_addr][data...]
    uint8_t* write_buffer = malloc(length + 2);
    if (write_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    write_buffer[0] = reg_base;
    write_buffer[1] = reg_addr;
    if (data != NULL && length > 0) {
        memcpy(write_buffer + 2, data, length);
    }
    
    // Use our dedicated I2C device handle
    esp_err_t ret = i2c_master_transmit(rotary_i2c_dev_handle, write_buffer, length + 2, 1000);
    
    free(write_buffer);
    return ret;
}

static esp_err_t seesaw_read_register(uint8_t reg_base, uint8_t reg_addr, uint8_t* data, size_t length, uint16_t delay_us)
{
    // First write the register address (Seesaw protocol)
    uint8_t reg_command[2] = {reg_base, reg_addr};
    
    // First transmit register address
    esp_err_t ret = i2c_master_transmit(rotary_i2c_dev_handle, reg_command, 2, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    // Add configurable delay for register access (microseconds)
    if (delay_us > 0) {
        if (delay_us >= 1000) {
            vTaskDelay(pdMS_TO_TICKS(delay_us / 1000));
        } else {
            // For very short delays, use a busy wait
            ets_delay_us(delay_us);
        }
    }

    // Then read the data with retry logic similar to Arduino implementation
    const int max_retries = 3;
    for (int retry = 0; retry < max_retries; retry++) {
        ret = i2c_master_receive(rotary_i2c_dev_handle, data, length, 1000);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        
        // Small delay between retries
        if (retry < max_retries - 1) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    
    return ret;
}

esp_err_t rotary_encoder_get_data(rotary_encoder_data_t* data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(rotary_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(data, &rotary_data, sizeof(rotary_encoder_data_t));
        // Clear the button changed flag after reading
        rotary_data.button_changed = false;
        xSemaphoreGive(rotary_data_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t rotary_encoder_set_position(int32_t position)
{
    uint8_t data[4];
    data[0] = (position >> 24) & 0xFF;
    data[1] = (position >> 16) & 0xFF;
    data[2] = (position >> 8) & 0xFF;
    data[3] = position & 0xFF;

    // Update hardware position
    esp_err_t ret = seesaw_write_register(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION, data, 4);
    
    if (ret == ESP_OK) {
        // Update internal position state (apply same reversal as in the main loop)
        if (xSemaphoreTake(rotary_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            rotary_data.position = -position;  // Apply same reversal as main loop
            xSemaphoreGive(rotary_data_mutex);
            
            // Immediately send display update to show new position
            display_send_encoder_data(rotary_data.position, 0, rotary_data.button_pressed);
            
            ESP_LOGI(TAG_ROTARY, "Position reset to %ld, display updated", rotary_data.position);
        }
    }
    
    return ret;
}

esp_err_t rotary_encoder_register_callback(rotary_event_callback_t callback)
{
    event_callback = callback;
    return ESP_OK;
}

esp_err_t rotary_encoder_set_neopixel(uint8_t red, uint8_t green, uint8_t blue)
{
    // Set NeoPixel buffer data (GRB format for WS2812)
    uint8_t color_data[3] = {green, red, blue};
    esp_err_t ret = seesaw_write_register(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF, color_data, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    // Trigger NeoPixel update
    uint8_t show_cmd = 0x01;
    return seesaw_write_register(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SHOW, &show_cmd, 1);
}

esp_err_t rotary_encoder_task_stop(void)
{
    if (!rotary_task_running) {
        return ESP_OK;
    }

    rotary_task_running = false;

    // Wait for task to finish
    if (rotary_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
        rotary_task_handle = NULL;
    }

    // Clean up mutex
    if (rotary_data_mutex != NULL) {
        vSemaphoreDelete(rotary_data_mutex);
        rotary_data_mutex = NULL;
    }

    ESP_LOGI(TAG_ROTARY, "Rotary encoder task stopped");
    return ESP_OK;
}

static esp_err_t seesaw_probe_device(void)
{
    // Use retry logic similar to Arduino implementation
    const int max_retries = 10;
    
    for (int retry = 0; retry < max_retries; retry++) {
        // Use the shared I2C bus handle
        esp_err_t ret = i2c_master_probe(shared_i2c_get_bus_handle(), ROTARY_ENCODER_ADDR, 500);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG_ROTARY, "Seesaw device detected on retry %d/%d", retry + 1, max_retries);
            return ESP_OK;
        }
        
        ESP_LOGW(TAG_ROTARY, "Device probe failed (retry %d/%d): %s", retry + 1, max_retries, esp_err_to_name(ret));
        
        // Delay between retries (10ms like Arduino)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGE(TAG_ROTARY, "Seesaw device not found after %d retries", max_retries);
    return ESP_ERR_NOT_FOUND;
}