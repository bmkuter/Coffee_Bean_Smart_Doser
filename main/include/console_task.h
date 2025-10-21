/*
 * Console Task Header - USB UART Command Interface
 * 
 * Provides interactive serial console for development and testing:
 * - Motor control commands (PWM speed setting)
 * - System status queries
 * - Calibration triggers
 * - Debug commands
 */

#ifndef CONSOLE_TASK_H
#define CONSOLE_TASK_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Task Configuration
#define CONSOLE_TASK_STACK_SIZE     4096
#define CONSOLE_TASK_PRIORITY       1        // Low priority
#define CONSOLE_TASK_CORE           0        // Run on core 0

// Console buffer sizes
#define CONSOLE_RX_BUFFER_SIZE      256
#define CONSOLE_CMD_MAX_ARGS        8

// Command handler function type
typedef void (*console_cmd_handler_t)(int argc, char **argv);

// Command structure
typedef struct {
    const char *command;
    const char *help;
    console_cmd_handler_t handler;
} console_command_t;

/**
 * @brief Initialize console task
 * 
 * Sets up UART console and command processing
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t console_task_init(void);

/**
 * @brief Deinitialize console task
 * 
 * Stops the console task and releases resources
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t console_task_deinit(void);

/**
 * @brief Register a custom command
 * 
 * @param cmd Command name
 * @param help Help text
 * @param handler Command handler function
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t console_register_command(const char *cmd, const char *help, console_cmd_handler_t handler);

// Logging tag
#define TAG_CONSOLE "console"

#ifdef __cplusplus
}
#endif

#endif // CONSOLE_TASK_H
