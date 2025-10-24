/*
 * Console Task - USB UART Command Interface
 * 
 * Provides interactive serial console for development and testing
 */

#include "console_task.h"
#include "motor_control_task.h"
#include "nau7802_task.h"
#include "rotary_encoder_task.h"
#include "display_task.h"
#include "servo_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>

// UART Configuration
#define CONSOLE_UART_PORT           UART_NUM_1  // Use UART1 (separate from logging)
#define CONSOLE_UART_BAUD_RATE      115200
#define CONSOLE_UART_TX_PIN         16          // GPIO 16 for TX
#define CONSOLE_UART_RX_PIN         17          // GPIO 17 for RX

// Task variables
static TaskHandle_t console_task_handle = NULL;
static bool console_task_running = false;

// Command buffer
static char rx_buffer[CONSOLE_RX_BUFFER_SIZE];
static int rx_index = 0;

// Forward declarations
static void console_task_function(void* pvParameters);
static void console_process_command(char *cmd_line);
static void console_parse_and_execute(char *cmd_line);
static void console_printf(const char *format, ...);  // Printf to console UART

// Built-in command handlers
static void cmd_help(int argc, char **argv);
static void cmd_motor_air(int argc, char **argv);
static void cmd_motor_auger(int argc, char **argv);
static void cmd_motor_stop(int argc, char **argv);
static void cmd_tare(int argc, char **argv);
static void cmd_calibrate(int argc, char **argv);
static void cmd_weight(int argc, char **argv);
static void cmd_status(int argc, char **argv);
static void cmd_encoder(int argc, char **argv);
static void cmd_servo(int argc, char **argv);
static void cmd_servo_us(int argc, char **argv);
static void cmd_servo_off(int argc, char **argv);
static void cmd_servo_sweep(int argc, char **argv);
static void cmd_servo_open(int argc, char **argv);
static void cmd_servo_close(int argc, char **argv);

// Built-in commands
static const console_command_t builtin_commands[] = {
    {"help", "Show this help message", cmd_help},
    {"?", "Show this help message", cmd_help},
    {"air", "air <speed>  - Set air pump speed (0-100%)", cmd_motor_air},
    {"auger", "auger <speed>  - Set auger motor speed (0-100%)", cmd_motor_auger},
    {"stop", "Stop all motors", cmd_motor_stop},
    {"tare", "Tare both weight channels", cmd_tare},
    {"cal", "cal <weight>  - Calibrate with known weight (grams)", cmd_calibrate},
    {"weight", "Show current weight readings", cmd_weight},
    {"status", "Show system status", cmd_status},
    {"encoder", "encoder <position>  - Set encoder position", cmd_encoder},
    {"servo", "servo <angle>  - Set servo angle (0-180°)", cmd_servo},
    {"servo_us", "servo_us <us>  - Set servo pulse width (500-2500µs)", cmd_servo_us},
    {"servo_off", "Disable servo PWM output", cmd_servo_off},
    {"servo_sweep", "servo_sweep <start> <end> <time_ms> [repeats]  - Sweep servo", cmd_servo_sweep},
    {"open", "Open servo to 32° (calibrated open position)", cmd_servo_open},
    {"close", "Close servo to 87° (calibrated closed position)", cmd_servo_close},
};

#define NUM_BUILTIN_COMMANDS (sizeof(builtin_commands) / sizeof(console_command_t))

// Helper function to print to console UART
static void console_printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0) {
        uart_write_bytes(CONSOLE_UART_PORT, buffer, len);
    }
}

// Public API Functions
esp_err_t console_task_init(void)
{
    if (console_task_running) {
        ESP_LOGW(TAG_CONSOLE, "Console task already running");
        return ESP_ERR_INVALID_STATE;
    }

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = CONSOLE_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver using an event queue
    esp_err_t ret = uart_driver_install(CONSOLE_UART_PORT, 
                                        CONSOLE_RX_BUFFER_SIZE * 2, 
                                        0,  // No TX buffer
                                        0,  // No event queue
                                        NULL, 
                                        0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_CONSOLE, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(CONSOLE_UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_CONSOLE, "Failed to configure UART: %s", esp_err_to_name(ret));
        uart_driver_delete(CONSOLE_UART_PORT);
        return ret;
    }

    ret = uart_set_pin(CONSOLE_UART_PORT, CONSOLE_UART_TX_PIN, CONSOLE_UART_RX_PIN, 
                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_CONSOLE, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(CONSOLE_UART_PORT);
        return ret;
    }

    // Create console task
    BaseType_t task_created = xTaskCreatePinnedToCore(
        console_task_function,
        "console_task",
        CONSOLE_TASK_STACK_SIZE,
        NULL,
        CONSOLE_TASK_PRIORITY,
        &console_task_handle,
        CONSOLE_TASK_CORE
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG_CONSOLE, "Failed to create console task");
        uart_driver_delete(CONSOLE_UART_PORT);
        return ESP_FAIL;
    }

    console_task_running = true;
    ESP_LOGI(TAG_CONSOLE, "Console task initialized successfully on UART%d @ %d baud", 
             CONSOLE_UART_PORT, CONSOLE_UART_BAUD_RATE);
    ESP_LOGI(TAG_CONSOLE, "Console TX: GPIO%d, RX: GPIO%d", CONSOLE_UART_TX_PIN, CONSOLE_UART_RX_PIN);
    
    // Print welcome message to UART1 console
    console_printf("\r\n\r\n");
    console_printf("========================================\r\n");
    console_printf("  Coffee Bean Smart Doser Console\r\n");
    console_printf("  UART1: GPIO16(TX) / GPIO17(RX)\r\n");
    console_printf("========================================\r\n");
    console_printf("Type 'help' for available commands\r\n\r\n");
    console_printf("> ");
    
    return ESP_OK;
}

esp_err_t console_task_deinit(void)
{
    if (!console_task_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG_CONSOLE, "Deinitializing console task");

    // Delete task
    if (console_task_handle != NULL) {
        vTaskDelete(console_task_handle);
        console_task_handle = NULL;
    }

    // Delete UART driver
    uart_driver_delete(CONSOLE_UART_PORT);

    console_task_running = false;
    return ESP_OK;
}

// Task function
static void console_task_function(void* pvParameters)
{
    uint8_t data;
    
    while (1) {
        // Read one byte at a time
        int len = uart_read_bytes(CONSOLE_UART_PORT, &data, 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Echo character
            uart_write_bytes(CONSOLE_UART_PORT, (const char*)&data, 1);
            
            // Handle backspace
            if (data == '\b' || data == 127) {
                if (rx_index > 0) {
                    rx_index--;
                    console_printf("\b \b");  // Erase character on screen
                }
                continue;
            }
            
            // Handle newline/carriage return
            if (data == '\n' || data == '\r') {
                console_printf("\r\n");
                
                if (rx_index > 0) {
                    rx_buffer[rx_index] = '\0';
                    console_process_command(rx_buffer);
                    rx_index = 0;
                }
                
                console_printf("> ");
                continue;
            }
            
            // Add to buffer if printable and space available
            if (isprint(data) && rx_index < CONSOLE_RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = data;
            }
        }
        
        // Small delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Process received command
static void console_process_command(char *cmd_line)
{
    // Trim leading whitespace
    while (*cmd_line && isspace((unsigned char)*cmd_line)) {
        cmd_line++;
    }
    
    // Ignore empty commands
    if (*cmd_line == '\0') {
        return;
    }
    
    // Trim trailing whitespace
    char *end = cmd_line + strlen(cmd_line) - 1;
    while (end > cmd_line && isspace((unsigned char)*end)) {
        *end-- = '\0';
    }
    
    console_parse_and_execute(cmd_line);
}

// Parse and execute command
static void console_parse_and_execute(char *cmd_line)
{
    char *argv[CONSOLE_CMD_MAX_ARGS];
    int argc = 0;
    
    // Tokenize command line
    char *token = strtok(cmd_line, " \t");
    while (token != NULL && argc < CONSOLE_CMD_MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " \t");
    }
    
    if (argc == 0) {
        return;
    }
    
    // Find and execute command
    for (int i = 0; i < NUM_BUILTIN_COMMANDS; i++) {
        if (strcasecmp(argv[0], builtin_commands[i].command) == 0) {
            builtin_commands[i].handler(argc, argv);
            return;
        }
    }
    
    console_printf("Unknown command: %s\r\n", argv[0]);
    console_printf("Type 'help' for available commands\r\n");
}

// Built-in command handlers
static void cmd_help(int argc, char **argv)
{
    console_printf("\r\nAvailable commands:\r\n");
    console_printf("------------------\r\n");
    for (int i = 0; i < NUM_BUILTIN_COMMANDS; i++) {
        console_printf("  %-12s - %s\r\n", builtin_commands[i].command, builtin_commands[i].help);
    }
    console_printf("\r\n");
}

static void cmd_motor_air(int argc, char **argv)
{
    if (argc < 2) {
        console_printf("Usage: air <speed>\r\n");
        console_printf("  speed: 0-100 (percent, 0=off, 100=full speed)\r\n");
        return;
    }
    
    int speed = atoi(argv[1]);
    if (speed < 0 || speed > 100) {
        console_printf("Error: Speed must be 0-100%%\r\n");
        return;
    }
    
    esp_err_t ret = motor_air_pump_set_speed((uint8_t)speed);
    if (ret == ESP_OK) {
        console_printf("Air pump set to %d%%\r\n", speed);
    } else {
        console_printf("Error setting air pump speed: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_motor_auger(int argc, char **argv)
{
    if (argc < 2) {
        console_printf("Usage: auger <speed>\r\n");
        console_printf("  speed: 0-100 (percent, 0=off, 100=full speed)\r\n");
        return;
    }
    
    int speed = atoi(argv[1]);
    if (speed < 0 || speed > 100) {
        console_printf("Error: Speed must be 0-100%%\r\n");
        return;
    }
    
    esp_err_t ret = motor_auger_set_speed((uint8_t)speed);
    if (ret == ESP_OK) {
        console_printf("Auger motor set to %d%%\r\n", speed);
    } else {
        console_printf("Error setting auger speed: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_motor_stop(int argc, char **argv)
{
    motor_air_pump_set_speed(0);
    motor_auger_set_speed(0);
    console_printf("All motors stopped\r\n");
}

static void cmd_tare(int argc, char **argv)
{
    console_printf("Taring both channels...\r\n");
    esp_err_t ret = nau7802_tare_channel(NAU7802_CHANNEL_1);
    if (ret == ESP_OK) {
        console_printf("Channel A tared successfully\r\n");
    } else {
        console_printf("Channel A tare failed: %s\r\n", esp_err_to_name(ret));
    }
    
    ret = nau7802_tare_channel(NAU7802_CHANNEL_2);
    if (ret == ESP_OK) {
        console_printf("Channel B tared successfully\r\n");
    } else {
        console_printf("Channel B tare failed: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_calibrate(int argc, char **argv)
{
    if (argc < 2) {
        console_printf("Usage: cal <weight>\r\n");
        console_printf("  weight: Known calibration weight in grams\r\n");
        return;
    }
    
    float weight = atof(argv[1]);
    if (weight <= 0.0f) {
        console_printf("Error: Weight must be greater than 0\r\n");
        return;
    }
    ESP_LOGI(TAG_NAU7802, "Long press detected - starting calibration with 358.2g weight");
    display_send_system_status("358.2g -> Ch A", false, 2000);
    vTaskDelay(pdMS_TO_TICKS(2000));

// Calibrate Channel A (Container)
    // Show instruction to move weight and press button when ready
    display_send_system_status("Move to Ch A - 5", false, 1000); 
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_send_system_status("Move to Ch A - 4", false, 1000); 
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_send_system_status("Move to Ch A - 3", false, 1000); 
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_send_system_status("Move to Ch A - 2", false, 1000);
    vTaskDelay(pdMS_TO_TICKS(1000));    
    display_send_system_status("Move to Ch A - 1", false, 1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_send_system_status("Cal Ch A...", false, 0);  // Indefinite
    
    esp_err_t ret = nau7802_calibrate_channel(NAU7802_CHANNEL_1, 358.2f);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_NAU7802, "Channel A calibrated with 358.2g weight");
    } else {
        ESP_LOGW(TAG_NAU7802, "Failed to calibrate Channel A: %s", esp_err_to_name(ret));
    }
    
// Calibrate Channel B if connected - wait for user confirmation
    // Show instruction to move weight and press button when ready
    display_send_system_status("Move to Ch B - 5", false, 1000); 
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_send_system_status("Move to Ch B - 4", false, 1000); 
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_send_system_status("Move to Ch B - 3", false, 1000); 
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_send_system_status("Move to Ch B - 2", false, 1000);
    vTaskDelay(pdMS_TO_TICKS(1000));    
    display_send_system_status("Move to Ch B - 1", false, 1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_send_system_status("Cal Ch B...", false, 0);  // Indefinite

    // Now calibrate Channel B
    ret = nau7802_calibrate_channel(NAU7802_CHANNEL_2, 358.2f);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_NAU7802, "Channel B calibrated with 358.2g weight");
    } else {
        ESP_LOGW(TAG_NAU7802, "Failed to calibrate Channel B: %s", esp_err_to_name(ret));
    }

    // Show completion message
    display_send_system_status("Cal Complete!", false, 2000);
    ESP_LOGI(TAG_NAU7802, "Calibration sequence complete");
        
}

static void cmd_weight(int argc, char **argv)
{
    nau7802_data_t data;
    esp_err_t ret = nau7802_get_data(&data);
    
    if (ret == ESP_OK) {
        console_printf("\r\nWeight Readings:\r\n");
        console_printf("  Channel A: %7.1f g (raw: %8ld)\r\n", 
               data.channel_a.filtered_weight, data.channel_a.raw_value);
        console_printf("  Channel B: %7.1f g (raw: %8ld)\r\n", 
               data.channel_b.filtered_weight, data.channel_b.raw_value);
        console_printf("\r\n");
    } else {
        console_printf("Error reading weight data: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_status(int argc, char **argv)
{
    motor_state_t motor_state;
    nau7802_data_t weight_data;
    
    console_printf("\r\n========== System Status ==========\r\n");
    
    // Motor status
    if (motor_get_state(&motor_state) == ESP_OK) {
        console_printf("Motors:\r\n");
        console_printf("  Air pump:  %4u (%.1f%%)\r\n", 
               motor_state.air_pump_speed, 
               (motor_state.air_pump_speed * 100.0f) / 4095.0f);
        console_printf("  Auger:     %4u (%.1f%%)\r\n", 
               motor_state.auger_speed, 
               (motor_state.auger_speed * 100.0f) / 4095.0f);
    }
    
    // Weight status
    if (nau7802_get_data(&weight_data) == ESP_OK) {
        console_printf("\r\nWeight:\r\n");
        console_printf("  Channel A: %7.1f g\r\n", weight_data.channel_a.filtered_weight);
        console_printf("  Channel B: %7.1f g\r\n", weight_data.channel_b.filtered_weight);
    }
    
    console_printf("===================================\r\n\r\n");
}

static void cmd_encoder(int argc, char **argv)
{
    if (argc < 2) {
        console_printf("Usage: encoder <position>\r\n");
        console_printf("  position: New encoder position value\r\n");
        return;
    }
    
    int32_t position = atoi(argv[1]);
    
    esp_err_t ret = rotary_encoder_set_position(position);
    if (ret == ESP_OK) {
        console_printf("Encoder position set to %ld\r\n", position);
    } else {
        console_printf("Error setting encoder position: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_servo(int argc, char **argv)
{
    if (argc < 2) {
        console_printf("Usage: servo <angle>\r\n");
        console_printf("  angle: Servo angle in degrees (0-180)\r\n");
        console_printf("Current: %d° (%d µs)\r\n", servo_get_angle(), servo_get_pulse_width());
        return;
    }
    
    int angle = atoi(argv[1]);
    
    if (angle < 0 || angle > 180) {
        console_printf("Error: Angle must be 0-180°\r\n");
        return;
    }
    
    esp_err_t ret = servo_set_angle(angle);
    if (ret == ESP_OK) {
        console_printf("Servo set to %d° (%d µs)\r\n", angle, servo_get_pulse_width());
    } else {
        console_printf("Error setting servo angle: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_servo_us(int argc, char **argv)
{
    if (argc < 2) {
        console_printf("Usage: servo_us <microseconds>\r\n");
        console_printf("  microseconds: Pulse width (1000-2000µs)\r\n");
        console_printf("Current: %d µs (%d°)\r\n", servo_get_pulse_width(), servo_get_angle());
        return;
    }
    
    int pulse_us = atoi(argv[1]);
    
    if (pulse_us < 1000 || pulse_us > 2000) {
        console_printf("Error: Pulse width must be 1000-2000µs\r\n");
        return;
    }
    
    esp_err_t ret = servo_set_pulse_width(pulse_us);
    if (ret == ESP_OK) {
        console_printf("Servo set to %d µs (%d°)\r\n", pulse_us, servo_get_angle());
    } else {
        console_printf("Error setting servo pulse width: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_servo_off(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    esp_err_t ret = servo_disable();
    if (ret == ESP_OK) {
        console_printf("Servo PWM disabled\r\n");
    } else {
        console_printf("Error disabling servo: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_servo_sweep(int argc, char **argv)
{
    if (argc < 4) {
        console_printf("Usage: servo_sweep <start_angle> <end_angle> <time_ms> [repeats]\r\n");
        console_printf("  start_angle: Starting angle (0-180°)\r\n");
        console_printf("  end_angle:   Ending angle (0-180°)\r\n");
        console_printf("  time_ms:     Total sweep time in milliseconds\r\n");
        console_printf("  repeats:     Number of times to repeat (optional, default=1)\r\n");
        console_printf("Example: servo_sweep 0 180 2000     (sweep once)\r\n");
        console_printf("Example: servo_sweep 0 180 2000 5   (sweep 5 times)\r\n");
        return;
    }
    
    int start_angle = atoi(argv[1]);
    int end_angle = atoi(argv[2]);
    int total_time_ms = atoi(argv[3]);
    int repeats = 1;  // Default to 1 repeat
    
    // Optional repeat count
    if (argc >= 5) {
        repeats = atoi(argv[4]);
        if (repeats < 1) {
            console_printf("Error: repeats must be at least 1\r\n");
            return;
        }
        if (repeats > 100) {
            console_printf("Error: repeats limited to 100 (safety)\r\n");
            return;
        }
    }
    
    // Validate parameters
    if (start_angle < 0 || start_angle > 180) {
        console_printf("Error: start_angle must be 0-180°\r\n");
        return;
    }
    if (end_angle < 0 || end_angle > 180) {
        console_printf("Error: end_angle must be 0-180°\r\n");
        return;
    }
    if (total_time_ms < 100) {
        console_printf("Error: time_ms must be at least 100ms\r\n");
        return;
    }
    
    if (repeats == 1) {
        console_printf("Sweeping servo: %d° → %d° → %d° over %dms\r\n", 
                       start_angle, end_angle, start_angle, total_time_ms);
    } else {
        console_printf("Sweeping servo: %d° → %d° → %d° over %dms, repeating %d times\r\n", 
                       start_angle, end_angle, start_angle, total_time_ms, repeats);
    }
    
    // Calculate step parameters
    int angle_range = abs(end_angle - start_angle);
    int step_delay_ms = 20;  // Update every 20ms (50 Hz servo refresh rate)
    int total_steps = total_time_ms / (2 * step_delay_ms);  // Divide by 2 for forward + backward
    
    if (total_steps < 1) {
        total_steps = 1;
    }
    
    float step_size = (float)angle_range / (float)total_steps;
    
    // Repeat the sweep operation
    for (int repeat = 1; repeat <= repeats; repeat++) {
        if (repeats > 1) {
            console_printf("Sweep %d of %d\r\n", repeat, repeats);
        }
        
        // Sweep forward: start → end
        console_printf("  Sweeping forward...\r\n");
        for (int step = 0; step <= total_steps; step++) {
            float progress = (float)step / (float)total_steps;
            int current_angle;
            
            if (end_angle > start_angle) {
                current_angle = start_angle + (int)(progress * angle_range);
            } else {
                current_angle = start_angle - (int)(progress * angle_range);
            }
            
            esp_err_t ret = servo_set_angle(current_angle);
            if (ret != ESP_OK) {
                console_printf("Error during sweep: %s\r\n", esp_err_to_name(ret));
                return;
            }
            
            vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
        }
        
        // Sweep backward: end → start
        console_printf("  Sweeping backward...\r\n");
        for (int step = 0; step <= total_steps; step++) {
            float progress = (float)step / (float)total_steps;
            int current_angle;
            
            if (end_angle > start_angle) {
                current_angle = end_angle - (int)(progress * angle_range);
            } else {
                current_angle = end_angle + (int)(progress * angle_range);
            }
            
            esp_err_t ret = servo_set_angle(current_angle);
            if (ret != ESP_OK) {
                console_printf("Error during sweep: %s\r\n", esp_err_to_name(ret));
                return;
            }
            
            vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
        }
    }
    
    console_printf("Sweep complete (%d cycles). Servo at %d°\r\n", repeats, servo_get_angle());
}

static void cmd_servo_open(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    const uint8_t open_angle = SERVO_OPEN_ANGLE;  // Calibrated open position

    esp_err_t ret = servo_set_angle(open_angle);
    if (ret == ESP_OK) {
        console_printf("Servo OPEN - moved to %d° (%d µs)\r\n", open_angle, servo_get_pulse_width());
    } else {
        console_printf("Error opening servo: %s\r\n", esp_err_to_name(ret));
    }
}

static void cmd_servo_close(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    const uint8_t close_angle = SERVO_CLOSE_ANGLE;  // Calibrated closed position

    esp_err_t ret = servo_set_angle(close_angle);
    if (ret == ESP_OK) {
        console_printf("Servo CLOSED - moved to %d° (%d µs)\r\n", close_angle, servo_get_pulse_width());
    } else {
        console_printf("Error closing servo: %s\r\n", esp_err_to_name(ret));
    }
}
