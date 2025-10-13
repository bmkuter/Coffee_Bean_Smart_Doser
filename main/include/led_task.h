/*
 * LED Task Header
 */

#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdint.h>

/**
 * LED task function for FreeRTOS
 * @param pvParameters Task parameters (unused)
 */
void led_task(void *pvParameters);

/**
 * Set NeoPixel to specific RGB color
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 */
void set_led_color(uint8_t red, uint8_t green, uint8_t blue);

/**
 * Turn off the NeoPixel
 */
void clear_led(void);

#endif // LED_TASK_H