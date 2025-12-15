<div align="center">

# 🔧 Multi-Platform BSP (ESP32 & PIC18F)

[![C](https://img.shields.io/badge/C-A8B9CC?style=for-the-badge&logo=c&logoColor=black)](https://en.wikipedia.org/wiki/C_(programming_language))
[![ESP32](https://img.shields.io/badge/ESP32-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](https://espressif.com)
[![Microchip](https://img.shields.io/badge/PIC18F-EE3233?style=for-the-badge&logo=microchip&logoColor=white)](https://microchip.com)
[![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](LICENSE)

**Hardware Abstraction Layer (HAL) for portable embedded development across ESP32 and PIC18F platforms.**

*Write once, deploy anywhere • Unified API • Cross-platform firmware*

</div>

---

## 📋 Overview

A Board Support Package (BSP) providing a unified interface for developing embedded applications that run on multiple microcontroller platforms without code changes. Currently supports ESP32 and PIC18F4550.

---

## ✨ Key Benefits

| Benefit | Description |
|---------|-------------|
| **📝 Write Once** | Same code runs on both platforms |
| **🔌 Unified API** | Consistent peripheral interface |
| **⚡ Easy Extension** | Add new MCUs with minimal effort |
| **⏱️ Faster Development** | Reduced multi-platform effort |

---

## 🎯 Supported Platforms

| Platform | Architecture | Features |
|----------|--------------|----------|
| **ESP32** | Xtensa LX6 | WiFi, Bluetooth, dual-core |
| **PIC18F4550** | 8-bit RISC | USB, low power, legacy |

---

## 🔌 Abstracted Peripherals

| Peripheral | Status |
|:----------:|:------:|
| **GPIO** | ✅ |
| **UART** | ✅ |
| **Timers** | ✅ |
| **ADC** | ✅ |
| **PWM** | ✅ |

---

## 📁 Project Structure

```
dsi/
├── 📁 bsp/              # Common BSP interface
│   ├── bsp.h            # Main header
│   ├── config.h         # Platform config
│   └── types.h          # Common types
│
├── 📁 esp32/            # ESP32 implementation
│   ├── main.cpp
│   ├── gpio_hal.c
│   └── uart_hal.c
│
└── 📁 pic18f4550/       # PIC18F implementation
    ├── main.c
    ├── gpio_hal.c
    └── uart_hal.c
```

---

## 🚀 Quick Start

### ESP32

```bash
cd esp32
idf.py build && idf.py flash
# Or: pio run --target upload
```

### PIC18F4550

```bash
cd pic18f4550
# Open in MPLAB X → Build → Flash
```

---

## ⚙️ Configuration

Edit `bsp/config.h`:

```c
// Select target platform
#define BSP_PLATFORM_ESP32     1
#define BSP_PLATFORM_PIC18F    0

// Configure peripherals
#define BSP_UART_BAUDRATE     115200
#define BSP_USE_GPIO          1
#define BSP_USE_ADC           1
```

---

## 🏭 Use Cases

- **IoT Prototyping** - Test on ESP32, deploy on PIC
- **Education** - Learn embedded with platform flexibility
- **Legacy Upgrades** - Migrate PIC systems to ESP32
- **Cross-Platform Validation** - Verify firmware across architectures

---

## 📄 License

MIT License

---

<div align="center">

[![GitHub](https://img.shields.io/badge/Star_on_GitHub-181717?style=for-the-badge&logo=github&logoColor=white)](https://github.com/surbalo1/embedded-bsp-esp32-pic18f)

</div>
