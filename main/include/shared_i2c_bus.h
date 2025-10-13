/*
 * Shared I2C Bus Manager
 * 
 * Provides a single I2C master bus that multiple components can share
 * by adding their devices to the same bus.
 */

#ifndef SHARED_I2C_BUS_H
#define SHARED_I2C_BUS_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the shared I2C bus
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t shared_i2c_bus_init(void);

/**
 * @brief Get the shared I2C bus handle
 * @return I2C bus handle or NULL if not initialized
 */
i2c_master_bus_handle_t shared_i2c_get_bus_handle(void);

/**
 * @brief Add a device to the shared I2C bus
 * @param device_address I2C device address
 * @param scl_speed_hz SCL speed in Hz
 * @param dev_handle Pointer to store the device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t shared_i2c_add_device(uint16_t device_address, uint32_t scl_speed_hz, 
                                i2c_master_dev_handle_t* dev_handle);

#ifdef __cplusplus
}
#endif

#endif // SHARED_I2C_BUS_H