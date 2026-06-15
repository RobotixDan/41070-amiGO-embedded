# 41070-amiGO-embedded
Embedded firmware for the amiGO step-tracking wearable device. STM32F303K8T6 (STM32duino), featuring self-test subroutine, GUI state machine, RFID collectibles system, and accelerometer-based step counting with pace detection

# amiGO Firmware

Embedded firmware for the amiGO step-tracking wearable device — a gamified 
handheld for encouraging physical activity in children.

## Hardware
- MCU: STM32F303K8T6 (32-bit ARM Cortex-M4, 72 MHz, LQFP32)
- Display: Adafruit 1.44" TFT ST7735R (128×128, hardware SPI1)
- Accelerometer: ADXL335 (3-axis analog, filtered via 4th-order Butterworth LPF)
- RFID: RC522 (bit-bang SPI, collectibles system)
- Buttons: 3× tactile (PA12, PB0, PB7)
- Framework: STM32duino (Arduino IDE)

## File Structure
| File | Description |
|---|---|
| `main.ino` | GUI state machine, button handling, RFID login and collectibles, alien animation |
| `StepCounter.h` | Class definition, `SelfTestResult` and `CalibrationResult` structs, enums |
| `StepCounter.cpp` | Self-test subroutine, calibration, step counting, pace detection |

## Key Features
- **Self-test subroutine** — electrically verifies the ADXL335 via PMOS ST driver, reports per-axis measured vs expected ADC deltas
- **GUI state machine** — login, menu, self-test, calibration, step tracking pages with pixel-art alien mascot animation
- **RFID collectibles** — four token types (shell, leaf, ladybug, rock) scanned during step tracking
- **Step counting** — 50 Hz TIM2 ISR, IIR filter, dynamic dominant axis, 250 ms refractory period
- **Pace detection** — cadence buffer with hysteresis (stationary / walking / running)

## Authors
- Daniya Syed — self-test subroutine, GUI design and implementation, firmware integration
- Chris El-Hachem — StepCounter library (calibration, step counting, pace detection)

## Subject
41070 Embedded Mechatronics Studio — Autumn 2026 | UTS | Lab 03 Group 05
