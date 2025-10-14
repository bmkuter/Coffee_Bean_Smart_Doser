/*
 * NAU7802 ADC Task Header - 24-bit Strain Gauge ADC
 * 
 * Handles dual-channel 24-bit ADC for weight measurements:
 * - Channel A: Container weight monitoring
 * - Channel B: Dosage cup weight monitoring
 * 
 * Uses FreeRTOS message queues for thread-safe data sharing with display task
 */

#ifndef NAU7802_TASK_H
#define NAU7802_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Task Configuration
#define NAU7802_TASK_STACK_SIZE     4096
#define NAU7802_TASK_PRIORITY       3        // Higher than display, lower than encoder for responsiveness
#define NAU7802_POLL_RATE_MS        50       // 20 Hz for stable strain gauge readings (matches 50 SPS)
#define NAU7802_SAMPLE_AVERAGE      4        // Number of samples to average for stability

// NAU7802 Register Definitions (from datasheet)
#define NAU7802_REG_PU_CTRL         0x00    // Power control
#define NAU7802_REG_CTRL1           0x01    // Control register 1
#define NAU7802_REG_CTRL2           0x02    // Control register 2
#define NAU7802_REG_OCAL1_B2        0x03    // Offset calibration byte 2
#define NAU7802_REG_OCAL1_B1        0x04    // Offset calibration byte 1
#define NAU7802_REG_OCAL1_B0        0x05    // Offset calibration byte 0
#define NAU7802_REG_GCAL1_B3        0x06    // Gain calibration byte 3
#define NAU7802_REG_GCAL1_B2        0x07    // Gain calibration byte 2
#define NAU7802_REG_GCAL1_B1        0x08    // Gain calibration byte 1
#define NAU7802_REG_GCAL1_B0        0x09    // Gain calibration byte 0
#define NAU7802_REG_I2C_CONTROL     0x11    // I2C control
#define NAU7802_REG_ADCO_B2         0x12    // ADC output byte 2 (MSB)
#define NAU7802_REG_ADCO_B1         0x13    // ADC output byte 1
#define NAU7802_REG_ADCO_B0         0x14    // ADC output byte 0 (LSB)
#define NAU7802_REG_ADC             0x15    // ADC register (for direct 24-bit read)
#define NAU7802_REG_OTP_B1          0x16    // OTP byte 1
#define NAU7802_REG_OTP_B0          0x17    // OTP byte 0
#define NAU7802_REG_PGA             0x1B    // PGA control
#define NAU7802_REG_POWER_CONTROL   0x1C    // Power control
#define NAU7802_REG_REVISION_ID     0x1F    // Chip revision ID

// NAU7802 Control Bits
#define NAU7802_PU_CTRL_RR          0x01    // Register reset
#define NAU7802_PU_CTRL_PUD         0x02    // Power up digital
#define NAU7802_PU_CTRL_PUA         0x04    // Power up analog
#define NAU7802_PU_CTRL_PUR         0x08    // Power up ready
#define NAU7802_PU_CTRL_CS          0x10    // Cycle start
#define NAU7802_PU_CTRL_CR          0x20    // Cycle ready
#define NAU7802_PU_CTRL_OSCS        0x40    // System clock source
#define NAU7802_PU_CTRL_AVDDS       0x80    // AVDD source

#define NAU7802_CTRL1_GAINS         0x07    // Gain select bits (0-7)
#define NAU7802_CTRL1_VLDO          0x38    // LDO voltage select
#define NAU7802_CTRL1_DRDY_SEL      0x40    // Data ready pin function
#define NAU7802_CTRL1_CRP           0x80    // Conversion ready polarity

#define NAU7802_CTRL2_CALMOD        0x03    // Calibration mode
#define NAU7802_CTRL2_CALS          0x04    // Calibration start
#define NAU7802_CTRL2_CAL_ERROR     0x08    // Calibration error
#define NAU7802_CTRL2_CRS           0x30    // Conversion rate select
#define NAU7802_CTRL2_CHS           0x80    // Channel select (0=Ch1, 1=Ch2)

// Gain Settings
typedef enum {
    NAU7802_GAIN_1X = 0,
    NAU7802_GAIN_2X = 1,
    NAU7802_GAIN_4X = 2,
    NAU7802_GAIN_8X = 3,
    NAU7802_GAIN_16X = 4,
    NAU7802_GAIN_32X = 5,
    NAU7802_GAIN_64X = 6,
    NAU7802_GAIN_128X = 7
} nau7802_gain_t;

// Conversion Rate Settings
typedef enum {
    NAU7802_RATE_10SPS = 0,     // 10 samples per second
    NAU7802_RATE_20SPS = 1,     // 20 samples per second
    NAU7802_RATE_40SPS = 2,     // 40 samples per second
    NAU7802_RATE_80SPS = 3      // 80 samples per second
} nau7802_rate_t;

// Channel Selection
typedef enum {
    NAU7802_CHANNEL_1 = 0,      // Channel 1 (A)
    NAU7802_CHANNEL_2 = 1       // Channel 2 (B)
} nau7802_channel_t;

// Weight Data Structure
typedef struct {
    int32_t raw_value;          // Raw ADC value (24-bit signed)
    float weight_grams;         // Converted weight in grams
    bool data_ready;            // New data available flag
    uint32_t timestamp;         // Timestamp of measurement
    uint32_t sample_count;      // Number of samples taken
} nau7802_channel_data_t;

// Complete NAU7802 Data Structure
typedef struct {
    nau7802_channel_data_t channel_a;   // Container weight
    nau7802_channel_data_t channel_b;   // Dosage cup weight
    bool device_ready;                  // Device initialization status
    uint32_t total_conversions;         // Total number of conversions
    float temperature;                  // Temperature reading (if available)
} nau7802_data_t;

// Calibration Data Structure
typedef struct {
    int32_t zero_offset;        // Zero/tare offset
    float scale_factor;         // Scale factor (counts per gram)
    bool is_calibrated;         // Calibration status
} nau7802_calibration_t;

// Function Prototypes
esp_err_t nau7802_task_init(void);
esp_err_t nau7802_task_start(void);
esp_err_t nau7802_task_stop(void);

// Data Access Functions
esp_err_t nau7802_get_data(nau7802_data_t* data);
esp_err_t nau7802_get_channel_data(nau7802_channel_t channel, nau7802_channel_data_t* data);

// Calibration Functions
esp_err_t nau7802_tare_channel(nau7802_channel_t channel);
esp_err_t nau7802_calibrate_channel(nau7802_channel_t channel, float known_weight_grams);
esp_err_t nau7802_get_calibration(nau7802_channel_t channel, nau7802_calibration_t* cal_data);

// Configuration Functions
esp_err_t nau7802_set_gain(nau7802_gain_t gain);
esp_err_t nau7802_set_sample_rate(nau7802_rate_t rate);

// Timing Calibration Function
esp_err_t nau7802_run_timing_calibration(int num_samples);

// Dynamic timeout testing function
esp_err_t nau7802_test_timeout_values(uint32_t min_timeout, uint32_t max_timeout, uint32_t step_timeout, uint32_t* optimal_timeout);

// Get current optimal timeout
uint32_t nau7802_get_optimal_timeout(void);

// Recalibrate timeout during runtime
esp_err_t nau7802_recalibrate_timeout(void);

// Logging tag
#define TAG_NAU7802 "nau7802"

#ifdef __cplusplus
}
#endif

#endif // NAU7802_TASK_H