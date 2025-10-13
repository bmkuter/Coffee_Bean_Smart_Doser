/*
 * Shared I2C Bus Manager Implementation
 * 
 * Provides a single I2C master bus that multiple components can share.
 */

#include "shared_i2c_bus.h"
#include "coffee_doser_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "shared_i2c";
static i2c_master_bus_handle_t shared_bus_handle = NULL;
static bool bus_initialized = false;

esp_err_t shared_i2c_bus_init(void)
{
    if (bus_initialized) {
        ESP_LOGW(TAG, "Shared I2C bus already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing shared I2C bus on SDA=%d, SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);

    // Enable I2C power first (critical for ESP32-C6 Feather)
    ESP_LOGI(TAG, "Enabling I2C power on GPIO%d", I2C_POWER_GPIO);
    gpio_reset_pin(I2C_POWER_GPIO);
    gpio_set_direction(I2C_POWER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_POWER_GPIO, 1);  // Pull high to enable I2C power
    
    // Allow power to stabilize - increased delay for SSD1306 compatibility
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure I2C master bus
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &shared_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize shared I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    bus_initialized = true;
    ESP_LOGI(TAG, "Shared I2C bus initialized successfully");
    return ESP_OK;
}

i2c_master_bus_handle_t shared_i2c_get_bus_handle(void)
{
    return shared_bus_handle;
}

esp_err_t shared_i2c_add_device(uint16_t device_address, uint32_t scl_speed_hz, 
                                i2c_master_dev_handle_t* dev_handle)
{
    if (!bus_initialized || shared_bus_handle == NULL) {
        ESP_LOGE(TAG, "Shared I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (dev_handle == NULL) {
        ESP_LOGE(TAG, "Device handle pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Adding I2C device at address 0x%02X with speed %lu Hz", device_address, scl_speed_hz);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = scl_speed_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(shared_bus_handle, &dev_cfg, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device 0x%02X: %s", device_address, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C device 0x%02X added successfully", device_address);
    return ESP_OK;
}