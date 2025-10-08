# Multi-Platform Board Support Package (BSP)

Hardware abstraction layer (HAL) providing unified API for ESP32 and PIC18F4550 microcontrollers. Enables portable embedded firmware development across different hardware platforms.

## Description

This BSP (Board Support Package) project provides a unified interface for developing embedded applications that can run on multiple microcontroller platforms without code changes. Currently supports ESP32 and PIC18F4550, with easy extensibility for additional platforms.

**Key Benefits:**
- Write once, deploy on multiple platforms
- Hardware abstraction for GPIO, timers, UART, etc.
- Consistent API across different MCUs
- Simplified peripheral management
- Reduced development time for multi-platform projects

## Supported Platforms

| Platform | Architecture | Main Features |
|----------|--------------|---------------|
| **ESP32** | Xtensa LX6 | WiFi, Bluetooth, dual-core |
| **PIC18F4550** | 8-bit RISC | USB, low power, legacy support |

## Project Structure

```
dsi/
├── bsp/                  # Common BSP interface
│   ├── bsp.h            # Main BSP header
│   ├── config.h         # Configuration options
│   └── types.h          # Common type definitions
├── esp32/               # ESP32-specific implementation
│   ├── main.cpp         # ESP32 main application
│   ├── gpio_hal.c       # GPIO hardware abstraction
│   └── uart_hal.c       # UART hardware abstraction
├── pic18f4550/          # PIC18F4550-specific implementation
│   ├── main.c           # PIC18F main application
│   ├── gpio_hal.c       # GPIO hardware abstraction
│   └── uart_hal.c       # UART hardware abstraction
└── README.md
```

## Features

### Abstracted Peripherals

- **GPIO** - Digital I/O control
- **UART** - Serial communication
- **Timers** - Time-based operations
- **ADC** - Analog-to-digital conversion
- **PWM** - Pulse-width modulation


## Getting Started

### Prerequisites

**For ESP32:**
- ESP-IDF or Arduino IDE
- ESP32 development board

**For PIC18F4550:**
- MPLAB X IDE
- XC8 Compiler
- PIC18F4550 board or dev kit

### Building for ESP32

```
cd esp32
# Using ESP-IDF
idf.py build
idf.py flash

# Or using PlatformIO
pio run --target upload
```

### Building for PIC18F4550

```
cd pic18f4550
# Open project in MPLAB X
# Build and program using MPLABx IPE
```

## Configuration

Edit `bsp/config.h` to customize:

```
// Select target platform
#define BSP_PLATFORM_ESP32     1
#define BSP_PLATFORM_PIC18F    0

// Configure peripherals
#define BSP_UART_BAUDRATE     115200
#define BSP_USE_GPIO          1
#define BSP_USE_ADC           1
```

## Adding New Platforms

To add support for a new microcontroller:

1. Create new folder: `platformname/`
2. Implement HAL functions in `platformname/xxx_hal.c`
3. Add platform-specific main file
4. Update `bsp/config.h` with new platform define

## Use Cases

- **IoT Prototyping**: Test on ESP32, deploy on PIC for production
- **Educational Projects**: Learn embedded development with platform flexibility
- **Legacy System Upgrades**: Migrate PIC-based systems to ESP32
- **Cross-Platform Validation**: Verify firmware logic across architectures

## Authors

**Rafael Gonzalez**
- GitHub: [@surbalo1](https://github.com/surbalo1)
- LinkedIn: [Rafael Gonzalez](https://www.linkedin.com/in/rafael-glez-chong/)
