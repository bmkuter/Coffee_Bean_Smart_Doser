| Supported Targets | ESP32-C6 |
| ----------------- | -------- |

# Coffee Bean Smart Doser

An ESP32-C6 based precision coffee bean dosing system with strain gauge scale integration, rotary encoder interface, and OLED display.

## Features

### Hardware Components
- **ESP32-C6 Feather** - Main microcontroller with WiFi/Bluetooth capabilities
- **NAU7802 24-bit ADC** - Dual-channel precision strain gauge interface
- **SSD1306 OLED Display** - 128x64 pixel status and weight display
- **Adafruit Seesaw Rotary Encoder** - User interface with button and RGB LED
- **Strain Gauge Load Cells** - Precision weight measurement for container and dosing cup

### Software Features
- **Real-time Weight Monitoring** - Displays container weight in integer grams
- **Scale Taring** - Long press rotary encoder button to zero the scale
- **Status Overlay** - Temporary status messages with bordered display
- **Encoder Interface** - Position tracking with delta calculations
- **I2C Bus Management** - Shared bus for multiple devices with proper device handling
- **FreeRTOS Tasks** - Multi-threaded architecture for responsive operation

## Hardware Setup

### Required Hardware
- ESP32-C6 Feather development board
- NAU7802 strain gauge amplifier
- SSD1306 OLED display (128x64, I2C)
- Adafruit Seesaw Rotary Encoder with RGB LED
- Strain gauge load cells (HX711 compatible)
- Connecting wires and breadboard/PCB

### I2C Device Addresses
- NAU7802 ADC: `0x2A`
- SSD1306 Display: `0x3D`
- Seesaw Rotary Encoder: `0x36`

### Pin Connections
All devices connect via I2C:
- **SDA**: GPIO 19
- **SCL**: GPIO 18
- **3.3V** and **GND** as required

## Software Architecture

### Task Structure
- **NAU7802 Task** - Handles weight readings at 20Hz, tare functionality
- **Display Task** - Manages OLED screen updates with message queue
- **Rotary Encoder Task** - Processes encoder input with debouncing and long press detection
- **LED Task** - Controls status LED patterns

### Key Features Implementation

#### Weight Display
- Container weight displayed in integer grams
- Real-time updates every 50ms
- Automatic connection detection
- "NO CONN" displayed when strain gauge disconnected

#### Scale Taring
- Long press (500ms) rotary encoder button to tare
- Zeros all connected channels
- Visual confirmation via status overlay
- Immediate weight display update after taring

#### Status Messages
- 3-second timeout overlays
- Bordered white text on black background
- Appears above header lines without disrupting main display
- Examples: "SCALE TARED", "NO SCALE", "TARE FAILED"

#### Display Architecture
- Decoupled message processing from screen rendering
- Centralized `display_refresh_main_screen()` function
- Minimal flickering with targeted updates
- Persistent weight data across encoder movements

## Project Configuration

### Build Requirements
- ESP-IDF v5.2+
- CMake build system
- Component dependencies managed via `idf_component.yml`

### External Components
- [esp-idf-ssd1306](https://github.com/nopnop2002/esp-idf-ssd1306) - OLED display driver

## Building and Flashing

### Set Target
```bash
idf.py set-target esp32c6
```

### Configure Project
```bash
idf.py menuconfig
```

### Build and Flash
```bash
idf.py build
idf.py -p COM_PORT flash monitor
```

### Monitor Output
```bash
idf.py monitor
```

## Usage

### Basic Operation
1. Power on the device
2. OLED display shows encoder position, delta, button state
3. Container weight appears on bottom lines when strain gauge connected
4. Rotate encoder to see position changes
5. Long press encoder button to tare the scale

### Display Layout
```
Rotary Encoder    <- Header line 0
Long press=tare   <- Header line 1
Pos: 0           <- Encoder position
                 <- (unused line 3)
Delta: 0         <- Encoder movement delta
Btn: ---         <- Button state
Cont: 125g       <- Container weight (Channel A)
Dose: ------     <- Dosage weight (Channel B - disabled)
```

### Status Messages
Temporary overlays appear above header lines:
- `SCALE TARED` - Successful tare operation
- `NO SCALE` - No strain gauges connected
- `TARE FAILED` - Tare operation failed

## Example Output

```text
I (1234) nau7802: Channel A: raw=-104837, weight=125g
I (1234) display: Coffee Info - Current: 125.0g, Target: -999.0g
I (1234) display: Screen refreshed - Pos: 5, Delta: +1, Cont: 125g
```

## Development Notes

### Calibration
- NAU7802 calibration values stored in `channel_a_cal` structure
- Timeout optimization for reliable strain gauge readings
- Connection detection based on weight thresholds

### Performance
- 20Hz weight reading rate (50ms intervals)
- 10Hz display update rate maximum
- Shared I2C bus with proper mutex handling
- Minimal memory footprint with static allocations

## Troubleshooting

### Weight Reading Issues
- Check strain gauge connections
- Verify NAU7802 I2C address (0x2A)
- Monitor serial output for connection status
- Ensure proper power supply (3.3V stable)

### Display Problems
- Verify SSD1306 I2C address (0x3D)
- Check display initialization in logs
- Ensure adequate power for all I2C devices

### Encoder Issues
- Confirm Seesaw I2C address (0x36)
- Check for proper debouncing in logs
- Verify long press detection (500ms threshold)

## License

This project is open source. See individual component licenses for external dependencies.

## Contributing

Please ensure all changes maintain the existing architecture patterns:
- Task-based design with proper synchronization
- Centralized display refresh system
- Shared I2C bus management
- Consistent error handling and logging
