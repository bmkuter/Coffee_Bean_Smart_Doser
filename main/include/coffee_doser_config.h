/*
 * Coffee Bean Smart Doser - Configuration Header
 * 
 * This file contains all the hardware configuration and I2C addresses
 * for the automated coffee bean dispensing system.
 */

#ifndef COFFEE_DOSER_CONFIG_H
#define COFFEE_DOSER_CONFIG_H

// Hardware Pin Definitions - System
#define NEOPIXEL_GPIO           9       // NeoPixel on Adafruit ESP32-C6 Feather
#define I2C_SDA_GPIO            19      // I2C SDA pin (official ESP32-C6 Feather pinout)
#define I2C_SCL_GPIO            18      // I2C SCL pin (official ESP32-C6 Feather pinout)
#define I2C_POWER_GPIO          20      // I2C power pin (shared with NeoPixel power)
#define NEOPIXEL_POWER_PIN      I2C_POWER_GPIO      // Power pin for NeoPixel

// I2C Device Addresses - Core Devices Only
#define NAU7802_ADDR            0x2A    // Dual-channel strain gauge ADC (both container & dosage)
#define ROTARY_ENCODER_ADDR     0x37    // Seesaw rotary encoder for dosage adjustment (configured via solder pads)
#define OLED_DISPLAY_ADDR       0x3D    // SSD1306 OLED display (128x64)
#define MAX17048_ADDR           0x36    // Built-in battery monitor (ESP32-C6 Feather built-in)

// Motor Driver (to be implemented later)
// #define MOTOR_DRIVER_ADDR       0x60    // PCA9685 Motor driver for servo and stepper

// Motor Controller Selection
// Comment/uncomment to switch between motor controller implementations
// PCA9685: I2C-based, 16 channels, 1526 Hz max (audible whine), shared I2C bus
// TB6612FNG: Direct GPIO, 2 motors, 25 kHz (silent), no I2C overhead
// #define USE_PCA9685_MOTOR_DRIVER    1   // Use PCA9685 I2C PWM controller (Adafruit Motor FeatherWing)
#define USE_TB6612_MOTOR_DRIVER  1   // Use TB6612FNG dual H-bridge (Pololu #713) with 25 kHz PWM

// NAU7802 Channel Definitions
#define NAU7802_CHANNEL_A       1       // Container weight (strain gauge on container)
#define NAU7802_CHANNEL_B       2       // Dosage cup weight (strain gauge on dosage cup)

// Strain Gauge Configuration
#define STRAIN_GAUGE_EXCITATION_V   3.3 // Excitation voltage (E+ to E-)
#define CONTAINER_MAX_WEIGHT_G      1000 // Maximum container weight in grams
#define DOSAGE_MAX_WEIGHT_G         50   // Maximum dosage weight in grams

// System Configuration
#define BLINK_PERIOD_MS         1000    // NeoPixel blink period
#define I2C_FREQ_HZ             100000  // I2C clock frequency (100kHz)

// Task Priorities
#define LED_TASK_PRIORITY       1
#define I2C_TASK_PRIORITY       5
#define MAIN_TASK_PRIORITY      10

// Task Stack Sizes
#define LED_TASK_STACK_SIZE     2048
#define I2C_TASK_STACK_SIZE     4096
#define MAIN_TASK_STACK_SIZE    8192

// Hardware Component Names (for logging)
#define TAG_MAIN                "main"
#define TAG_LED                 "led_task"
#define TAG_I2C                 "i2c_mgr"
#define TAG_NAU7802             "nau7802"
#define TAG_NAU7802_CONTAINER   "nau7802_container"
#define TAG_NAU7802_DOSAGE      "nau7802_dosage"
#define TAG_ENCODER             "encoder"
#define TAG_ROTARY              "rotary"
#define TAG_DISPLAY             "display"

// Motor driver (to be implemented later)
// #define TAG_MOTOR               "motor"

// Calibration and Measurement Settings
#define WEIGHT_SAMPLES_AVG      10      // Number of samples to average for weight readings
#define CALIBRATION_WEIGHT_G    100     // Known calibration weight in grams
#define WEIGHT_UPDATE_RATE_MS   100     // Weight measurement update rate

#endif // COFFEE_DOSER_CONFIG_H