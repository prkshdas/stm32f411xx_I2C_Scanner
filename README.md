# STM32F411xx I²C Scanner

This project demonstrates how to implement an **I²C device scanner** on the **STM32 Blackpill (STM32F411CEU6)** board.  
The project uses custom peripheral drivers (GPIO, I²C, USART, RCC) written from scratch for the STM32F411xx series.  

It scans the I²C bus and prints detected device addresses through UART.

---

## 📂 Repository Structure

- **drivers/**  
  Custom peripheral drivers for STM32F411xx:
  - `stm32f411xx_gpio_driver.*` – GPIO driver
  - `stm32f411xx_i2c_driver.*` – I²C driver
  - `stm32f411xx_usart_driver.*` – USART driver
  - `stm32f411xx_rcc_driver.*` – RCC driver
  - `stm32f411xx.h` – Device header

- **Src/main.c**  
  Application example that initializes I²C + UART and runs the scanner.

- **Inc/**  
  Header files for project-wide inclusion.

- **LICENSE**  
  MIT License.

---

## Features

- Developed and tested on **STM32 Blackpill (STM32F411CEU6)**
- Scans I²C addresses from **0x08 to 0x77**
- Detects active I²C devices using ACK/NACK response
- Prints results via **USART2 (115200 baud)**
- Reports:
  - Found device:  
    ```
    I2C device address : 0x3C
    ```
  - No device found:  
    ```
    No Device Found
    ```
- Runs continuously with a small delay between scans

---

## 🔌 Hardware Setup

- **Board**: STM32 Blackpill (STM32F411CEU6)

- **I²C Bus**  
  - SCL → **PB6**  
  - SDA → **PB7**  
  - Pull-up resistors (~4.7 kΩ) recommended if not already present.

- **USART2 for Output**  
  - TX → **PA2**  
  - RX → **PA3** (optional, for debugging)

- **Baud Rate**: 115200, 8 data bits, 1 stop bit, no parity (8-N-1).

---

## ⚙️ Build & Flash

1. Clone the repository:
   ```bash
   git clone https://github.com/prkshdas/stm32f411xx_I2C_Scanner.git
   cd stm32f411xx_I2C_Scanner
