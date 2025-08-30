# Arduino: MicroI2C - Lightweight bit-bang I2C Master library

A minimal I2C master implementation using bit-banging technique, optimized for ATtiny and ATmega microcontrollers with cross-platform compatibility.

## Features

- **Lightweight**: Minimal C/C++ library implementation
- **Bit-bang I2C**: No hardware I2C peripheral required
- **Multiple I2C buses**: Support for multiple independent I2C buses using different pin pairs
- **Same address device control**: Control multiple devices with identical I2C addresses on a single microcontroller by using multiple I2C buses
- **AVR-optimized**: Direct register manipulation for ATtiny and ATmega microcontrollers
- **Cross-platform compatible**: Automatic fallback to digitalWrite/digitalRead for non-AVR platforms
- **Minimal memory footprint**: Optimized for resource-constrained microcontrollers
- **Simple API**: Easy to use functions for I2C communication with Wire-like interface
- **Portable design**: Three-level API (high/middle/low) for maximum flexibility

## Code Size

The following table shows the compiled size of MicroWire library functions for different AVR targets:

| Target | FQBN | MicroWire Size (bytes) |
|--------|------|----------------------|
| Arduino UNO | `arduino:avr:uno` | 180 |
| Arduino Leonardo | `arduino:avr:leonardo` | 180 |
| ATtiny85 (Digispark) | `ATTinyCore:avr:attinyx5micr` | 158 |
| ATtiny85 | `ATTinyCore:avr:attinyx5:chip=85` | 158 |
| ATtiny13 | `MicroCore:avr:13` | 142 |

*Note: Measurements based on SCAN example sketch without DEBUG mode. Actual size may vary depending on optimization level and used functions.*

## Supported Platforms

### AVR Microcontrollers (Optimized)

- **ATtiny13** (MicroCore) - Direct register manipulation
- **ATtiny85** (ATTinyCore, DigiSpark) - Direct register manipulation
- **ATmega328P** (Arduino UNO/Nano) - Direct register manipulation
- **ATmega32U4** (Arduino Pro Micro, Leonardo) - Direct register manipulation
- Other AVR microcontrollers with PORTB pins - Direct register manipulation

### Other Platforms (Compatible)

- **ESP32, ESP8266** - Uses digitalWrite/digitalRead
- **RP2040** (Raspberry Pi Pico) - Uses digitalWrite/digitalRead
- **STM32** - Uses digitalWrite/digitalRead
- Any Arduino-compatible platform with pinMode/digitalWrite/digitalRead support

**Note**: The library automatically detects AVR platforms and uses optimized register operations (DDRB, PORTB, PINB) for maximum efficiency. On non-AVR platforms, it falls back to standard Arduino functions for broader compatibility.

**Important**: On AVR platforms, the library currently uses PORTB-specific register operations. This means pin numbers must correspond to PORTB pins (PB0-PB7). AVR microcontrollers with different port layouts may require code modifications.

**Pin Number Difference**: AVR platforms use PORTB bit numbers (0-7), while non-AVR platforms use standard Arduino digital pin numbers. The same `MICRO_WIRE_HANDLE(PB2, PB3)` call means PB2/PB3 on AVR but GPIO2/GPIO3 on ESP32.

## Installation

### Arduino IDE Library Manager

1. Open Arduino IDE
2. Go to Sketch → Include Library → Manage Libraries
3. Search for "MicroWire"
4. Click Install

### Manual Installation

1. Download this repository as ZIP
2. In Arduino IDE: Sketch → Include Library → Add .ZIP Library
3. Select the downloaded ZIP file

## Usage

```cpp
#include "MicroWire.h"

// Define I2C using PORTB bit numbers (PB2=SDA, PB3=SCL) on AVR
// Or digital pin numbers on non-AVR platforms
#define I2C_HANDLE MICRO_WIRE_HANDLE(PB2, PB3)

void setup() {
  // Initialize I2C
  MicroWire_begin(I2C_HANDLE);
}

void loop() {
  // Write to device at address 0x38
  MicroWire_beginTransmission(I2C_HANDLE, 0x38);
  MicroWire_write(I2C_HANDLE, 0xAC);  // Command
  MicroWire_write(I2C_HANDLE, 0x33);  // Parameter 1
  MicroWire_write(I2C_HANDLE, 0x00);  // Parameter 2
  MicroWire_endTransmission(I2C_HANDLE);

  delay(100);  // Wait for measurement

  // Read from device
  uint8_t data[6];
  MicroWire_requestFrom(I2C_HANDLE, 0x38, 6);
  for (uint8_t i = 0; i < 6; i++) {
    MicroWire_read(I2C_HANDLE, (i < 5), &data[i]);  // ACK for all except last byte
  }

  delay(1000);  // Wait before next measurement
}
```

## Pin Configuration

The library uses different pin numbering schemes depending on the platform:

### AVR Platforms (ATtiny, ATmega)

Uses PORTB bit numbers (0-7) directly:

```cpp
// Define I2C bus with PORTB bit numbers (SDA=PB2, SCL=PB3)
#define I2C_HANDLE MICRO_WIRE_HANDLE(PB2, PB3)  // Uses bit numbers 2, 3
```

### Non-AVR Platforms (ESP32, RP2040, STM32, etc.)

Uses Arduino digital pin numbers:

```cpp
// Define I2C bus with digital pin numbers
#define I2C_HANDLE MICRO_WIRE_HANDLE(21, 22)  // GPIO21, GPIO22 on ESP32
// or
#define I2C_HANDLE MICRO_WIRE_HANDLE(4, 5)    // GPIO4, GPIO5 on RP2040
```

**Pin Mapping for Arduino UNO/Nano (ATmega328P)**:

When using `MICRO_WIRE_HANDLE(PB2, PB3)` on AVR:

- PB2 = Digital Pin 10
- PB3 = Digital Pin 11

Full PORTB mapping:

- PB0 = Digital Pin 8
- PB1 = Digital Pin 9
- PB2 = Digital Pin 10
- PB3 = Digital Pin 11
- PB4 = Digital Pin 12
- PB5 = Digital Pin 13

**Pin Mapping for Arduino Leonardo/Pro Micro (ATmega32U4)**:

When using `MICRO_WIRE_HANDLE(PB2, PB3)` on AVR:

- PB2 = Digital Pin 16 (MOSI)
- PB3 = Digital Pin 14 (MISO)

Full PORTB mapping:

- PB0 = Digital Pin 17 (RX LED)
- PB1 = Digital Pin 15 (SCK)
- PB2 = Digital Pin 16 (MOSI)
- PB3 = Digital Pin 14 (MISO)
- PB4 = Digital Pin 8
- PB5 = Digital Pin 9
- PB6 = Digital Pin 10
- PB7 = Digital Pin 11

## Multiple I2C Buses

MicroWire supports multiple independent I2C buses by using different pin pairs. This allows you to control multiple devices with the same I2C address on a single microcontroller.

### Example: Two independent I2C buses

```cpp
#include "MicroWire.h"

// Define two different I2C buses
#define I2C_BUS1 MICRO_WIRE_HANDLE(PB0, PB1)  // SDA=PB0, SCL=PB1
#define I2C_BUS2 MICRO_WIRE_HANDLE(PB2, PB3)  // SDA=PB2, SCL=PB3

void setup() {
  // Initialize both I2C buses
  MicroWire_begin(I2C_BUS1);
  MicroWire_begin(I2C_BUS2);
}

void loop() {
  // Communicate with device at address 0x48 on first bus
  MicroWire_beginTransmission(I2C_BUS1, 0x48);
  MicroWire_write(I2C_BUS1, 0x00);  // Command
  MicroWire_endTransmission(I2C_BUS1);

  // Communicate with device at address 0x48 on second bus
  MicroWire_beginTransmission(I2C_BUS2, 0x48);
  MicroWire_write(I2C_BUS2, 0x01);  // Different command
  MicroWire_endTransmission(I2C_BUS2);

  delay(1000);
}
```

### Benefits

- **Same address devices**: Control multiple devices with identical I2C addresses
- **Bus isolation**: Electrical isolation between different I2C buses
- **Parallel communication**: Independent communication without bus conflicts
- **Flexible pin assignment**: Use any available PORTB pins (on AVR) or GPIO pins (on other platforms)

## API Reference

### High-level APIs (Recommended)

- `MicroWire_begin(uint8_t handle)` - Initialize I2C pins
- `MicroWire_beginTransmission(uint8_t handle, uint8_t address)` - Begin transmission to device, returns true if ACK received
- `MicroWire_endTransmission(uint8_t handle)` - End transmission
- `MicroWire_requestFrom(uint8_t handle, uint8_t address, uint8_t quantity)` - Request data from device, returns true if ACK received
- `MicroWire_read(uint8_t handle, bool ack, uint8_t* data)` - Read data byte, ack=true sends ACK, ack=false sends NACK
- `MicroWire_write(uint8_t handle, uint8_t data)` - Write data byte, returns true if ACK received

### Middle-level APIs

- `MicroWire_start(uint8_t handle)` - Generate I2C start condition
- `MicroWire_stop(uint8_t handle)` - Generate I2C stop condition

### Low-level APIs (Platform abstraction)

- `MicroWire_sda_high(uint8_t sda)` - Set SDA pin high (input with pull-up)
- `MicroWire_sda_low(uint8_t sda)` - Set SDA pin low (output low)
- `MicroWire_sda_read(uint8_t sda)` - Read SDA pin state
- `MicroWire_scl_high(uint8_t scl)` - Set SCL pin high (input with pull-up)
- `MicroWire_scl_low(uint8_t scl)` - Set SCL pin low (output low)
- `MicroWire_scl_read(uint8_t scl)` - Read SCL pin state
- `MicroWire_scl_high_wait(uint8_t scl)` - Set SCL pin high and wait for clock stretching

### Macros

- `MICRO_WIRE_HANDLE(sda, scl)` - Create I2C bus configuration
- `MICRO_WIRE_SDA(bus)` - Extract SDA pin from bus configuration
- `MICRO_WIRE_SCL(bus)` - Extract SCL pin from bus configuration
- `MICRO_WIRE_DELAY()` - Timing delay (default: `_delay_us(5)`)

## Examples

See the `examples/` directory for complete usage examples:

- **AHTX0**: Temperature and humidity sensor example
- **SCAN**: I2C bus scanner example
- **SSD1306**: OLED display example showing "HELLO!" message

## License

MIT License - see LICENSE file for details.

## Contributing

Pull requests and issues are welcome on GitHub.
