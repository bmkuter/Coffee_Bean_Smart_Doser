/*
 * LED Task - NeoPixel Status Indicator
 * 
 * Handles the NeoPixel LED on the Adafruit ESP32-C6 Feather
 * for system status indication.
 */

#include "led_task.h"
#include "coffee_doser_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "esp_log.h"

static led_strip_handle_t led_strip = NULL;
static uint8_t led_state = 0;

static void configure_neopixel(void)
{
    ESP_LOGI(TAG_LED, "Configuring NeoPixel LED on GPIO %d", NEOPIXEL_GPIO);
    
    // Configure the NeoPixel power pin
    gpio_reset_pin(NEOPIXEL_POWER_PIN);
    gpio_set_direction(NEOPIXEL_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(NEOPIXEL_POWER_PIN, 1);  // Pull high to enable NeoPixel power
    
    // LED strip initialization
    led_strip_config_t strip_config = {
        .strip_gpio_num = NEOPIXEL_GPIO,
        .max_leds = 1, // Single NeoPixel on board
    };
    
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    // Initialize with LED off
    led_strip_clear(led_strip);
}

static void update_neopixel(void)
{
    if (led_state) {
        // Set LED to a soft white when on
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        led_strip_refresh(led_strip);
    } else {
        // Turn off LED
        led_strip_clear(led_strip);
    }
}

void led_task(void *pvParameters)
{
    ESP_LOGI(TAG_LED, "LED task starting...");
    
    // Configure the NeoPixel
    configure_neopixel();
    
    while (1) {
        // Toggle LED state
        led_state = !led_state;
        update_neopixel();
        
        ESP_LOGD(TAG_LED, "NeoPixel %s", led_state ? "ON" : "OFF");
        
        // Wait for next blink cycle
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_MS));
    }
}

void set_led_color(uint8_t red, uint8_t green, uint8_t blue)
{
    if (led_strip != NULL) {
        led_strip_set_pixel(led_strip, 0, red, green, blue);
        led_strip_refresh(led_strip);
    }
}

void clear_led(void)
{
    if (led_strip != NULL) {
        led_strip_clear(led_strip);
    }
}