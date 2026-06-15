/*
 * amiGO — Main Firmware Integration
 * Author: Daniya Syed
 * Subject: 41070 Embedded Mechatronics Studio, Autumn 2026
 * Team: Lab 03 Group 05
 *
 * Integrates:
 * - GUI state machine (login, menu, self-test, calibration, step tracking)
 * - Self-test subroutine (ADXL335 ST pin via PMOS driver, per-axis ADC delta)
 * - TFT display (ST7735R, 128x128, hardware SPI1) — screen layouts, 
 *   pixel-art alien animation, collectible icons
 * - RFID reader (RC522, bit-bang SPI) — login and collectibles system
 * - StepCounter library (accelerometer processing, calibration, pace detection)
 *   Original author: Chris El-Hachem | Integration & refinement: Daniya Syed
 *
 * Hardware:
 * - MCU: STM32F303K8T6 (STM32duino framework)
 * - TFT: Adafruit ST7735R 1.44" 128x128 (hardware SPI1: PB3/PB5/PA5)
 * - RFID: RC522 (bit-bang SPI: PA4/PA6/PA5, SS: PA7, RST: PA8)
 * - Buttons: PA12 (A/heart), PB0 (B/star), PB7 (C/bolt)
 * - Accelerometer: ADXL335 via analog filter -> PA0/PA1/PA2 (ADC)
 * - ST driver: PA3 -> AO3401A PMOS (Q1) -> ADXL335 ST pin (logic inverted)
 *
 * Note on PMOS logic: PA3 HIGH = Q1 off = ST LOW (normal)
 *                     PA3 LOW  = Q1 on  = ST HIGH (self-test active)
 *
 * CHANGED (Daniya 02/06/2026):
 * - Calibration now two-stage: prompt screen first, then arrow/capture
 * - calibrationReady flag tracks which stage we're on
 * - Additional visuals: moon, UFO, stars on menu screen
 * - Pace inactivity timeout reduced: 3000ms -> 1500ms
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <MFRC522.h>
#include <HardwareTimer.h>
#include "StepCounter.h"

// TFT pins
#define TFT_CS   PA11
#define TFT_DC   PB1
#define TFT_RST  -1

// RFID pins
#define SS_PIN   PA7
#define RST_PIN  PA8

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
MFRC522 rfid(SS_PIN, RST_PIN);

// --- StepCounter ---
StepCounter *Counter;
HardwareTimer *ADCPollTimer;
const int POLL_FREQ = 50;

void adcRead() { Counter->readADC(); }

bool selfTestDone    = false;
bool calibrationDone = false; //flip to true to test step tracking
bool calibrationReady = false; // false = show prompt, true = show arrow/capture



// --- Idle frame (new alien) ---
const uint16_t idle_frame[] PROGMEM = {
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0xffa7, 0x0000, 0x9f2a, 0x9f2a, 0xffa7, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x667d, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x667d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};

// --- Walk frame 1 (left foot up, new alien) ---
const uint16_t walk_frame1[] PROGMEM = {
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0xffa7, 0x0000, 0x9f2a, 0x9f2a, 0xffa7, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x667d, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x667d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0x0000, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};

// --- Walk frame 2 (right foot up, new alien) ---
const uint16_t walk_frame2[] PROGMEM = {
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0xffa7, 0x0000, 0x9f2a, 0x9f2a, 0xffa7, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x667d, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x667d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0x0000, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};
// --- Run frame 1 (left foot up, new alien) --
const uint16_t run_frame1[] PROGMEM = {
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x64ff, 0xffa7, 0x0000, 0x9f2a, 0x9f2a, 0xffa7, 0x0000, 0x64ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};
// --- Run frame 2 (right foot up, new alien) --
const uint16_t run_frame2[] PROGMEM = {
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0xb1a6, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0xb1a6, 0x0000, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x9f2a, 0x0000, 0xffff, 0x9f2a, 0x0000, 0xb1a6, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x64ff, 0xffa7, 0x0000, 0x9f2a, 0x9f2a, 0xffa7, 0x0000, 0x64ff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb1a6, 0xb1a6, 0xb1a6, 0x0000, 0x9f2a, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9f2a, 0x9f2a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};

// --- RFID UIDs ---
byte loginUID[]   = {0x19, 0xA7, 0xAE, 0x02};
byte shellUID[]   = {0xA6, 0x7C, 0x03, 0x91};
byte leafUID[]    = {0x66, 0x00, 0xFA, 0x90};
byte ladybugUID[] = {0x63, 0x86, 0x2D, 0x03};
byte rockUID[]    = {0x26, 0xE2, 0x03, 0x91};

// --- Collectible counters ---
int shellCount   = 0;
int leafCount    = 0;
int ladybugCount = 0;
int rockCount    = 0;

// --- GUI state ---
const char* menuItems[] = {"Self-Test", "Calibration", "Step Tracking"};
const int menuCount = 3;
int selectedItem = 0;
int currentPage = -1;

int measured[] = {0, 0, 0};
int expected[] = {0, 0, 0};
bool axisPass[] = {true, true, true};
bool overallPass = true;

const char* directions[] = {"+X Up", "-X Up", "+Y Up", "-Y Up", "+Z Up", "-Z Up"};
const char* hints[] = {
  "Tilt: connector down",
  "Tilt: connector up",
  "Tilt: left side up",
  "Tilt: right side up",
  "Lay flat: screen up",
  "Lay flat: screen down"
};
const int totalDirections = 6;
int currentDirection = 0;
bool directionDone[] = {false, false, false, false, false, false};

int stepCount = 0;
int pace = 0;
const char* paceLabels[] = {"Stationary", "Walking", "Running"};
uint16_t paceColors[] = {ST77XX_WHITE, ST77XX_CYAN, ST77XX_GREEN};

// --- Animation state ---
bool showingCharacter = false;
int animFrame = 0;
unsigned long lastFrameTime = 0;
const int WALK_DELAY = 300;
const int RUN_DELAY  = 150;

// ========================
// HELPERS
// ========================
bool uidMatches(byte *uid, byte uidSize, byte *knownUID, byte knownSize) {
  if (uidSize != knownSize) return false;
  for (byte i = 0; i < uidSize; i++) {
    if (uid[i] != knownUID[i]) return false;
  }
  return true;
}

void drawFrameInCentre(const uint16_t* frame) {
  int scale = 3;
  int imgW = 18, imgH = 18;
  int xOff = 128 - (imgW * scale) - 2;
  int yOff = 17 + (78 - imgH * scale) / 2;
  tft.fillRect(64, 17, 64, 78, ST77XX_BLACK);
  drawMoon2(75, 85);  // draw moon after clearing, before alien pixels
  for (int y = 0; y < imgH; y++) {
    for (int x = 0; x < imgW; x++) {
      uint16_t color = pgm_read_word(&frame[y * imgW + x]);
      if (color != 0x0000)
        tft.fillRect(xOff + x * scale, yOff + y * scale, scale, scale, color);
    }
  }
}

void drawCollectibles() {
  tft.fillRect(0, 17, 64, 78, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(2, 22); tft.print("Shell:"); tft.print(shellCount);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(2, 36); tft.print("Leaf: "); tft.print(leafCount);
  tft.setTextColor(ST77XX_RED);
  tft.setCursor(2, 50); tft.print("Bug:  "); tft.print(ladybugCount);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(2, 64); tft.print("Rock: "); tft.print(rockCount);
  if (shellCount   > 0) drawShellIcon(0,  76);
  if (leafCount    > 0) drawLeafIcon(16,  76);
  if (ladybugCount > 0) drawLadybugIcon(32, 76);
  if (rockCount    > 0) drawRockIcon(48,  76);
}
// ========================
// ICONS
// ========================
void drawLeafIcon(int x, int y) {
  uint16_t g = ST77XX_GREEN;
  uint16_t d = 0x0300;
  int s = 2;
  tft.fillRect(x+3*s, y+0*s, s, s, g);
  tft.fillRect(x+2*s, y+1*s, 3*s, s, g);
  tft.fillRect(x+1*s, y+2*s, 5*s, s, g);
  tft.fillRect(x+1*s, y+3*s, 5*s, s, g);
  tft.fillRect(x+2*s, y+4*s, 3*s, s, g);
  tft.fillRect(x+3*s, y+5*s, s, s, g);
  tft.fillRect(x+3*s, y+6*s, s, s, d);
  tft.fillRect(x+3*s, y+7*s, s, s, d);
}

void drawLadybugIcon(int x, int y) {
  uint16_t r = ST77XX_RED;
  uint16_t b = 0x8410;
  int s = 2;
  tft.fillRect(x+1*s, y+1*s, 5*s, s, b);
  tft.fillRect(x+0*s, y+2*s, 7*s, s, r);
  tft.fillRect(x+0*s, y+3*s, 7*s, s, r);
  tft.fillRect(x+0*s, y+4*s, 7*s, s, r);
  tft.fillRect(x+0*s, y+5*s, 7*s, s, r);
  tft.fillRect(x+1*s, y+6*s, 5*s, s, r);
  tft.fillRect(x+3*s, y+2*s, s, 5*s, b);
  tft.fillRect(x+1*s, y+3*s, s, s, b);
  tft.fillRect(x+5*s, y+3*s, s, s, b);
  tft.fillRect(x+1*s, y+5*s, s, s, b);
  tft.fillRect(x+5*s, y+5*s, s, s, b);
  tft.fillRect(x+1*s, y+0*s, s, s, b);
  tft.fillRect(x+5*s, y+0*s, s, s, b);
}

void drawRockIcon(int x, int y) {
  uint16_t gr = 0x8410;
  uint16_t dg = 0x4208;
  int s = 2;
  tft.fillRect(x+2*s, y+0*s, 3*s, s, gr);
  tft.fillRect(x+1*s, y+1*s, 5*s, s, gr);
  tft.fillRect(x+0*s, y+2*s, 7*s, s, gr);
  tft.fillRect(x+0*s, y+3*s, 7*s, s, dg);
  tft.fillRect(x+0*s, y+4*s, 7*s, s, dg);
  tft.fillRect(x+1*s, y+5*s, 5*s, s, dg);
  tft.fillRect(x+2*s, y+6*s, 3*s, s, dg);
  tft.fillRect(x+2*s, y+1*s, 2*s, s, 0xFFFF);
}

void drawShellIcon(int x, int y) {
  uint16_t sh = ST77XX_YELLOW;
  uint16_t d = 0xD600;
  int s = 2;
  tft.fillRect(x+2*s, y+0*s, 3*s, s, sh);
  tft.fillRect(x+1*s, y+1*s, 5*s, s, sh);
  tft.fillRect(x+0*s, y+2*s, 7*s, s, sh);
  tft.fillRect(x+0*s, y+3*s, 7*s, s, sh);
  tft.fillRect(x+1*s, y+4*s, 5*s, s, sh);
  tft.fillRect(x+2*s, y+5*s, 3*s, s, sh);
  tft.fillRect(x+1*s, y+2*s, s, 4*s, d);
  tft.fillRect(x+3*s, y+1*s, s, 5*s, d);
  tft.fillRect(x+5*s, y+2*s, s, 4*s, d);
}


// ========================
// DRAW FUNCTIONS
// ========================
void drawLogin() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRoundRect(10, 8, 108, 85, 20, ST77XX_RED);
  tft.fillTriangle(20, 88, 10, 105, 35, 88, ST77XX_RED);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(22, 18);
  tft.print("ami");
  tft.fillCircle(88, 62, 22, ST77XX_WHITE);
  tft.fillCircle(88, 62, 18, ST77XX_RED);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(22, 48);
  tft.print("GO");
  tft.fillCircle(88, 62, 14, ST77XX_WHITE);
  tft.fillCircle(84, 58, 2, ST77XX_RED);
  tft.fillCircle(92, 58, 2, ST77XX_RED);
  tft.fillRect(83, 65, 2, 2, ST77XX_RED);
  tft.fillRect(86, 67, 4, 2, ST77XX_RED);
  tft.fillRect(91, 65, 2, 2, ST77XX_RED);
  tft.fillCircle(8,   8,   3, ST77XX_RED);
  tft.fillCircle(4,   8,   2, ST77XX_RED);
  tft.fillCircle(12,  8,   2, ST77XX_RED);
  tft.fillCircle(8,   4,   2, ST77XX_RED);
  tft.fillCircle(8,   12,  2, ST77XX_RED);
  tft.fillCircle(120, 8,   3, ST77XX_RED);
  tft.fillCircle(116, 8,   2, ST77XX_RED);
  tft.fillCircle(124, 8,   2, ST77XX_RED);
  tft.fillCircle(120, 4,   2, ST77XX_RED);
  tft.fillCircle(120, 12,  2, ST77XX_RED);
  tft.fillCircle(8,   118, 3, ST77XX_RED);
  tft.fillCircle(4,   118, 2, ST77XX_RED);
  tft.fillCircle(12,  118, 2, ST77XX_RED);
  tft.fillCircle(8,   114, 2, ST77XX_RED);
  tft.fillCircle(8,   122, 2, ST77XX_RED);
  tft.fillCircle(120, 118, 3, ST77XX_RED);
  tft.fillCircle(116, 118, 2, ST77XX_RED);
  tft.fillCircle(124, 118, 2, ST77XX_RED);
  tft.fillCircle(120, 114, 2, ST77XX_RED);
  tft.fillCircle(120, 122, 2, ST77XX_RED);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(15, 108);
  tft.print("Scan card to login");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(45, 120);
  tft.print("[ RFID ]");
}

void drawLoginDenied() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);
  tft.setCursor(20, 50);
  tft.print("Access");
  tft.setCursor(25, 70);
  tft.print("Denied!");
  delay(1500);
  drawLogin();
}

void drawMenu() {
  tft.fillScreen(ST77XX_BLACK);
  drawStars(); 
  drawUFO(85, 109);
  drawMoon(10, 115);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(35, 10);
  tft.println("amiGO");
  tft.drawFastHLine(0, 22, 128, ST77XX_RED);
  for (int i = 0; i < menuCount; i++) {
    int yPos = 35 + (i * 25);
    bool locked = (i == 2 && (!selfTestDone || !calibrationDone));
    if (i == selectedItem) {
      //tft.fillRoundRect(8, yPos - 4, 112, 18, 3, locked ? 0x8410 : ST77XX_CYAN);
      tft.fillRoundRect(8, yPos - 4, 112, locked ? 28 : 18, 3, locked ? 0x8410 : ST77XX_CYAN);
      tft.setTextColor(ST77XX_BLACK);
    } else {
      tft.setTextColor(locked ? 0x8410 : ST77XX_WHITE);
    }
    tft.setTextSize(1);
    tft.setCursor(15, yPos);
    tft.print(menuItems[i]);
    if (locked) {
      tft.setTextColor(i == selectedItem ? ST77XX_BLACK : 0x8410);
      tft.setCursor(35, yPos + 10);
      tft.print("[locked]");
    } else if (i == 0 && selfTestDone) {
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(100, yPos);
      tft.print("[OK]");
    } else if (i == 1 && calibrationDone) {
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(100, yPos);
      tft.print("[OK]");
    }
  }
}

void drawSelfTest() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(35, 5); tft.println("Self-Test");
  tft.drawFastHLine(0, 16, 128, ST77XX_RED);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, 22);  tft.print("Axis");
  tft.setCursor(35, 22); tft.print("Meas.");
  tft.setCursor(78, 22); tft.print("Exp.");
  tft.setCursor(112,22); tft.print("OK");
  tft.drawFastHLine(0, 31, 128, ST77XX_WHITE);
  const char* axisLabels[] = {"X", "Y", "Z"};
  for (int i = 0; i < 3; i++) {
    int yPos = 38 + (i * 20);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5, yPos);   tft.print(axisLabels[i]);
    tft.setCursor(28, yPos);  tft.print(measured[i]);
    tft.setCursor(72, yPos);  tft.print(expected[i]);
    if (axisPass[i]) {
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(112, yPos); tft.print("P");
    } else {
      tft.setTextColor(ST77XX_RED);
      tft.setCursor(112, yPos); tft.print("F");
    }
  }
  tft.drawFastHLine(0, 100, 128, ST77XX_WHITE);
  if (overallPass) {
    tft.fillRoundRect(5, 103, 118, 16, 3, ST77XX_GREEN);
    tft.setTextColor(ST77XX_BLACK);
    tft.setCursor(35, 107); tft.print("OVERALL: PASS");
  } else {
    tft.fillRoundRect(5, 103, 118, 16, 3, ST77XX_RED);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(35, 107); tft.print("OVERALL: FAIL");
  }
}
  void drawHeartIcon(int x, int y, uint16_t color) {
      tft.fillRect(x+1, y+0, 2, 2, color);
      tft.fillRect(x+4, y+0, 2, 2, color);
      tft.fillRect(x,   y+2, 7, 2, color);
      tft.fillRect(x+1, y+4, 5, 2, color);
      tft.fillRect(x+2, y+6, 3, 1, color);
      tft.fillRect(x+3, y+7, 1, 1, color);
  }

  void drawBoltIcon(int x, int y, uint16_t color) {
    tft.fillRect(x+4, y+0,  4, 1, color);
    tft.fillRect(x+3, y+1,  4, 1, color);
    tft.fillRect(x+2, y+2,  4, 1, color);
    tft.fillRect(x+1, y+3,  4, 1, color);
    tft.fillRect(x+0, y+4,  8, 1, color);
    tft.fillRect(x+0, y+5,  7, 1, color);
    tft.fillRect(x+3, y+6,  5, 1, color);
    tft.fillRect(x+3, y+7,  4, 1, color);
    tft.fillRect(x+2, y+8,  4, 1, color);
    tft.fillRect(x+1, y+9,  4, 1, color);
    tft.fillRect(x+0, y+10, 4, 1, color);
  }

  void drawStarIcon(int x, int y, uint16_t color) {
    // Pixel art 5-point star (11x11)
    tft.fillRect(x+4, y+0,  3, 1, color);  // top point
    tft.fillRect(x+3, y+1,  5, 1, color);
    tft.fillRect(x+0, y+2, 11, 1, color);  // wide middle
    tft.fillRect(x+1, y+3,  9, 1, color);
    tft.fillRect(x+2, y+4,  7, 1, color);
    tft.fillRect(x+2, y+5,  7, 1, color);
    tft.fillRect(x+2, y+6,  3, 1, color);  // bottom left leg - shorter
    tft.fillRect(x+6, y+6,  3, 1, color);  // bottom right leg - shorter
    tft.fillRect(x+2, y+7,  2, 1, color);
    tft.fillRect(x+7, y+7,  2, 1, color);
  }



void drawCalibrationArrow(int direction) {
  int cx = 64, cy = 68;
  uint16_t ac = ST77XX_YELLOW;
  switch (direction) {
    case 0:
      tft.fillTriangle(cx, cy+12, cx-8, cy-4, cx+8, cy-4, ac);
      tft.fillRect(cx-3, cy-12, 6, 8, ac);
      break;
    case 1:
      tft.fillTriangle(cx, cy-12, cx-8, cy+4, cx+8, cy+4, ac);
      tft.fillRect(cx-3, cy+4, 6, 8, ac);
      break;
    case 2:
      tft.fillTriangle(cx-12, cy, cx+4, cy-8, cx+4, cy+8, ac);
      tft.fillRect(cx+4, cy-3, 8, 6, ac);
      break;
    case 3:
      tft.fillTriangle(cx+12, cy, cx-4, cy-8, cx-4, cy+8, ac);
      tft.fillRect(cx-12, cy-3, 8, 6, ac);
      break;
    case 4:
      tft.drawRoundRect(cx-18, cy-10, 36, 20, 3, ac);
      tft.fillCircle(cx, cy, 3, ac);
      tft.drawFastHLine(cx-5, cy, 10, ac);
      tft.drawFastVLine(cx, cy-5, 10, ac);
      break;
    case 5:
      tft.drawRoundRect(cx-18, cy-10, 36, 20, 3, ac);
      tft.drawLine(cx-10, cy-7, cx+10, cy+7, ac);
      tft.drawLine(cx+10, cy-7, cx-10, cy+7, ac);
      break;
  }
}

void drawStars() {
  uint16_t starColor = 0xFFFF;
  tft.drawPixel(10,  30, starColor);
  tft.drawPixel(25,  45, starColor);
  tft.drawPixel(118, 35, starColor);
  tft.drawPixel(105, 55, starColor);
  tft.drawPixel(15,  70, starColor);
  tft.drawPixel(120, 75, starColor);
  tft.drawPixel(8,   90, starColor);
  tft.drawPixel(122, 95, starColor);
  tft.drawPixel(20, 115, starColor);
  tft.drawPixel(110, 118, starColor);
  tft.fillRect(95,  28, 2, 2, starColor);
  tft.fillRect(5,   55, 2, 2, starColor);
  tft.fillRect(115, 88, 2, 2, starColor);
  tft.fillRect(12, 100, 2, 2, starColor);
}
void drawUFO(int x, int y) {
  uint16_t body  = 0x04F0;
  uint16_t disk  = 0x8410;
  uint16_t light = ST77XX_YELLOW;
  uint16_t dark  = 0x2104;
  int s = 2; 

  // Dome
  tft.fillRect(x+4*s, y+0*s, 4*s, s, body);
  tft.fillRect(x+3*s, y+1*s, 6*s, s, body);
  tft.fillRect(x+2*s, y+2*s, 8*s, 2*s, body);

  // Saucer rim
  tft.fillRect(x+0*s, y+4*s, 12*s, 2*s, disk);
  tft.fillRect(x+1*s, y+6*s, 10*s, s, dark);

  // Lights
  tft.fillRect(x+2*s, y+5*s, s, s, light);
  tft.fillRect(x+5*s, y+5*s, s, s, light);
  tft.fillRect(x+8*s, y+5*s, s, s, light);
}
void drawMoon(int x, int y) {
  uint16_t moon = 0xDEFB;  // light grey/white
  uint16_t dark = 0x8410;  // darker grey for craters
  int s = 4;

  // Half circle rising from bottom - flat edge at bottom
  tft.fillRect(x+4*s,  y+0*s,  8*s,  s,   moon);
  tft.fillRect(x+2*s,  y+1*s,  12*s, s,   moon);
  tft.fillRect(x+1*s,  y+2*s,  14*s, s,   moon);
  tft.fillRect(x+0*s,  y+3*s,  16*s, 5*s, moon);

  // Craters
  tft.fillRect(x+2*s,  y+4*s,  2*s, 2*s, dark);
  tft.fillRect(x+7*s,  y+2*s,  2*s, 2*s, dark);
  tft.fillRect(x+11*s, y+4*s,  2*s, 2*s, dark);
}
void drawMoon2(int x, int y) {
  uint16_t moon = 0xDEFB;
  uint16_t dark = 0x8410;
  int s = 2;

  tft.fillRect(x+6*s, y+0*s, 12*s, s, moon);
  tft.fillRect(x+3*s, y+1*s, 18*s, s, moon);
  tft.fillRect(x+1*s, y+2*s, 22*s, s, moon);
  tft.fillRect(x+0*s, y+3*s, 24*s, s, moon);
  tft.fillRect(x+0*s, y+4*s, 24*s, s, moon);

  // Craters
  tft.fillRect(x+2*s,  y+3*s, 2*s, 2*s, dark);
  tft.fillRect(x+10*s, y+1*s, 2*s, 2*s, dark);
  tft.fillRect(x+17*s, y+3*s, 2*s, 2*s, dark);
}
// Stage 1: instruction prompt before orienting device
void drawCalibrationPrompt() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(30, 5); tft.print("Calibration");
  tft.drawFastHLine(0, 16, 128, ST77XX_RED);


  // Yellow instruction banner
  tft.fillRoundRect(5, 45, 118, 50, 4, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 52);
  tft.print("Get ready to orient");
  tft.setCursor(10, 62);
  tft.print("your device to");
  tft.setCursor(10, 72);
  tft.print("match the arrows.");

  tft.drawFastHLine(0, 104, 128, ST77XX_WHITE);
  //tft.setTextColor(ST77XX_YELLOW);
  //tft.setCursor(5, 109); tft.print("B=continue  C=menu");
  drawStarIcon(5, 107, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(16, 109); tft.print("=continue");
  drawBoltIcon(80, 107, ST77XX_YELLOW);
  tft.setCursor(91, 109); tft.print("=menu");
}

// Stage 2: arrow + capture
bool calibrationAllPassed = true;
bool calibrationFailed = false;
bool axisResult[] = {false, false, false, false, false, false};
void drawCalibration() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(30, 5); tft.print("Calibration");
  tft.drawFastHLine(0, 16, 128, ST77XX_RED);

  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(5, 22);
  tft.print(directions[currentDirection]);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, 42);
  tft.print(hints[currentDirection]);

  drawCalibrationArrow(currentDirection);

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 78); tft.print("Progress:");
  for (int i = 0; i < totalDirections; i++) {
    int xPos = 10 + (i * 18);
    if (directionDone[i]) {
      if (axisResult[i]) {
        tft.fillCircle(xPos, 92, 6, ST77XX_GREEN);
      } else {
        tft.fillCircle(xPos, 92, 6, ST77XX_RED);
      }
    } else if (i == currentDirection) {
      tft.drawCircle(xPos, 92, 6, ST77XX_CYAN);
      tft.fillCircle(xPos, 92, 3, ST77XX_CYAN);
    } else {
      tft.drawCircle(xPos, 92, 6, ST77XX_WHITE);
    }
  }
  tft.drawFastHLine(0, 104, 128, ST77XX_WHITE);
  //tft.setTextColor(ST77XX_YELLOW);
  //tft.setCursor(5, 109); tft.print("B=capture  C=menu");
  drawStarIcon(5, 107, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(16, 109); tft.print("=capture");
  drawBoltIcon(75, 107, ST77XX_YELLOW);
  tft.setCursor(86, 109); tft.print("=menu");

}

void drawCalibrationComplete() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(30, 5); tft.print("Calibration");
  tft.drawFastHLine(0, 16, 128, ST77XX_RED);

  tft.fillRoundRect(5, 22, 118, 70, 6, 0x1447);

  tft.drawLine(30, 58, 48, 76, ST77XX_GREEN);
  tft.drawLine(31, 58, 49, 76, ST77XX_GREEN);
  tft.drawLine(32, 58, 50, 76, ST77XX_GREEN);
  tft.drawLine(48, 76, 90, 34, ST77XX_GREEN);
  tft.drawLine(49, 76, 91, 34, ST77XX_GREEN);
  tft.drawLine(50, 76, 92, 34, ST77XX_GREEN);

  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(1);
  tft.setCursor(55, 45); tft.print("All 6");
  tft.setCursor(55, 57); tft.print("directions");
  tft.setCursor(55, 69); tft.print("captured!");

  tft.fillRoundRect(5, 98, 118, 16, 3, ST77XX_GREEN);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(18, 102); tft.print("CALIBRATION PASSED");

  //tft.setTextColor(ST77XX_WHITE);
  //tft.setCursor(40, 120); tft.print("C = menu");
  drawBoltIcon(40, 118, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(51, 120); tft.print("= menu");
}
void drawCalibrationFailed() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(30, 5); tft.print("Calibration");
  tft.drawFastHLine(0, 16, 128, ST77XX_RED);

  tft.fillRoundRect(5, 22, 118, 70, 6, 0x2000);

  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);
  tft.setCursor(30, 35); tft.print("FAIL!");
  tft.setTextSize(1);
  tft.setCursor(10, 58); tft.print("One or more axes");
  tft.setCursor(10, 70); tft.print("did not pass.");
  tft.setCursor(10, 82); tft.print("Please retry.");

  tft.fillRoundRect(5, 98, 118, 16, 3, ST77XX_RED);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(15, 102); tft.print("CALIBRATION FAILED");

  //tft.setTextColor(ST77XX_WHITE);
  //tft.setCursor(40, 119); tft.print("C = menu");
  drawBoltIcon(40, 117, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(51, 119); tft.print("= menu");
}

void drawStepTrackingBottom() {
  tft.fillRect(0, 95, 128, 33, ST77XX_BLACK);
  tft.drawFastHLine(0, 95, 128, ST77XX_WHITE);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, 100); tft.print("Steps:");
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(5, 110); tft.print(stepCount);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(55, 100); tft.print("Pace:");
  tft.fillRoundRect(55, 108, 72, 14, 3, paceColors[pace]);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(60, 111); tft.print(paceLabels[pace]);
}

void drawStepTracking() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(20, 5); tft.println("Step Tracking");
  tft.drawFastHLine(0, 16, 128, ST77XX_RED);
  drawStepTrackingBottom();
  drawCollectibles();
  //drawMoon2(66, 80);          // moon first
  drawFrameInCentre(idle_frame);  // alien on top
}

void drawStepTrackingLocked() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(20, 5); tft.println("Step Tracking");
  tft.drawFastHLine(0, 16, 128, ST77XX_RED);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(15, 45); tft.print("Complete first:");
  if (!selfTestDone) {
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(15, 62); tft.print("[ ] Self-Test");
  } else {
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(15, 62); tft.print("[x] Self-Test");
  }
  if (!calibrationDone) {
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(15, 78); tft.print("[ ] Calibration");
  } else {
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(15, 78); tft.print("[x] Calibration");
  }
  //tft.setTextColor(ST77XX_WHITE);
  //tft.setCursor(15, 105); tft.print("C=menu");
  drawBoltIcon(15, 103, ST77XX_YELLOW);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(26, 105); tft.print("=menu");
}

// ========================
// ANIMATION
// ========================
void updateAnimation() {
  if (currentPage != 3) return;
  if (pace == 0) return;
  int frameDelay = (pace == 2) ? RUN_DELAY : WALK_DELAY;
  unsigned long now = millis();
  if (now - lastFrameTime >= frameDelay) {
    lastFrameTime = now;
    animFrame = (animFrame + 1) % 4;
    const uint16_t* frame;
    if (pace == 1) {
      if      (animFrame == 0) frame = walk_frame1;
      else if (animFrame == 2) frame = walk_frame2;
      else                     frame = idle_frame;
    } else {
      frame = (animFrame % 2 == 0) ? run_frame1 : run_frame2;
    }
    drawFrameInCentre(frame);
  }
}

// ========================
// STEP TRACKING UPDATE
// ========================
void updateStepDisplay() {
  if (currentPage != 3) return;
  int newSteps = Counter->getSteps();
  int newPace  = (int)Counter->getPace();
  if (newSteps != stepCount || newPace != pace) {
    stepCount = newSteps;
    pace = newPace;
    drawStepTrackingBottom();
    if (pace == 0) {
      animFrame = 0;
      drawFrameInCentre(idle_frame);
    } else {
      animFrame = (animFrame + 1) % 2;
      const uint16_t* frame;
      if (pace == 2) {
        frame = (animFrame == 0) ? run_frame1 : run_frame2;
      } else {
        frame = (animFrame == 0) ? walk_frame1 : walk_frame2;
      }
      drawFrameInCentre(frame);
    }
  }
}
// ========================
// SELF TEST
// ========================
void runSelfTest() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(35, 5); tft.println("Self-Test");
  tft.drawFastHLine(0, 16, 128, ST77XX_RED);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(20, 60); tft.print("Running test...");

  ADCPollTimer->pause();
  SelfTestResult result = Counter->selfTest();
  ADCPollTimer->resume();

  expected[0] = (int)((-338)*(4095/3300.));
  expected[1] = (int)((338)*(4095/3300.));
  expected[2] = (int)((661)*(4095/3300.));

  measured[0] = result.finalReadings[0] - result.initialReadings[0];
  measured[1] = result.finalReadings[1] - result.initialReadings[1];
  measured[2] = result.finalReadings[2] - result.initialReadings[2];

  axisPass[0] = abs(measured[0] - expected[0]) < 0.2*abs(expected[0]);
  axisPass[1] = abs(measured[1] - expected[1]) < 0.2*abs(expected[1]);
  axisPass[2] = abs(measured[2] - expected[2]) < 0.2*abs(expected[2]);

  overallPass = result.passed;
  if (result.passed) {
    Counter->setNewFactors();
    Counter->setNewOffsets();
    selfTestDone = true;
  }
  drawSelfTest();
}

// ========================
// RFID
// ========================
void handleRFID() {
  static unsigned long lastRFIDCheck = 0;
  if (millis() - lastRFIDCheck < 500) return;
  lastRFIDCheck = millis();

  if (currentPage == 3) ADCPollTimer->pause();

  if (!rfid.PICC_IsNewCardPresent()) { if (currentPage == 3) ADCPollTimer->resume(); return; }
  if (!rfid.PICC_ReadCardSerial())   { if (currentPage == 3) ADCPollTimer->resume(); return; }

  if (currentPage == -1) {
    if (uidMatches(rfid.uid.uidByte, rfid.uid.size, loginUID, 4)) {
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextColor(ST77XX_GREEN);
      tft.setTextSize(2);
      tft.setCursor(20, 55);
      tft.print("Welcome!");
      delay(1500);
      currentPage = 0;
      selectedItem = 0;
      drawMenu();
    } else {
      drawLoginDenied();
    }
  } else if (currentPage == 3) {
    if (uidMatches(rfid.uid.uidByte, rfid.uid.size, shellUID, 4)) {
      shellCount++; drawCollectibles();
    } else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, leafUID, 4)) {
      leafCount++; drawCollectibles();
    } else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, ladybugUID, 4)) {
      ladybugCount++; drawCollectibles();
    } else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, rockUID, 4)) {
      rockCount++; drawCollectibles();
    }
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  if (currentPage == 3) ADCPollTimer->resume();
}


// ========================
// BUTTONS
// ========================
bool btnALast = false, btnBLast = false, btnCLast = false;

void handleButtons() {
  bool btnA = digitalRead(PA12);
  bool btnB = digitalRead(PB0);
  bool btnC = digitalRead(PB7);

  if (btnA && !btnALast) {
    if (currentPage == 0) {
      selectedItem = (selectedItem + 1) % menuCount;
      drawMenu();
    } else if (currentPage == 2 && calibrationFailed) {
      calibrationFailed = false;
      calibrationReady = true;
      drawCalibration();
    }
  }

  if (btnB && !btnBLast) {
    if (currentPage == 0) {
      if (selectedItem == 0) {
        currentPage = 1;
        runSelfTest();
      } else if (selectedItem == 1) {
        currentDirection = 0;
        calibrationReady = false;
        calibrationFailed = false;
        calibrationAllPassed = true;
        for (int i = 0; i < totalDirections; i++) {
          directionDone[i] = false;
          axisResult[i] = false;
        }
        currentPage = 2;
        drawCalibrationPrompt();
      } else if (selectedItem == 2) {
        if (!selfTestDone || !calibrationDone) {
          drawStepTrackingLocked();  // stay on page 0, don't set currentPage = 3
        } else {
          Counter->setSteps(0);
          stepCount = 0; pace = 0;
          animFrame = 0; showingCharacter = false;
          currentPage = 3;
          drawStepTracking();
        }
      }
    } else if (currentPage == 2) {
      if (currentDirection >= totalDirections) {
        // finished — ignore B
      } else if (calibrationFailed) {
        calibrationFailed = false;
        calibrationAllPassed = false;
        axisResult[currentDirection] = false;
        directionDone[currentDirection] = true;
        currentDirection++;
        calibrationReady = false;
        if (currentDirection >= totalDirections) {
          calibrationDone = false;
          drawCalibrationFailed();
        } else {
          drawCalibration();
        }
      } else {
        const Orientation orientMap[] = {
          Orientation::RIGHT, Orientation::LEFT,
          Orientation::FRONT, Orientation::BACK,
          Orientation::TOP,   Orientation::BOTTOM
        };

        if (!calibrationReady) {
          calibrationReady = true;
          drawCalibration();
        } else {
          calibrationReady = false;

          tft.fillRect(0, 105, 128, 23, ST77XX_BLACK);
          tft.setTextColor(ST77XX_CYAN);
          tft.setCursor(5, 114);
          tft.print("Measuring...");

          ADCPollTimer->pause();
          CalibrationResult result = Counter->calibrate(orientMap[currentDirection]);
          ADCPollTimer->resume();

          tft.fillRect(0, 105, 128, 23, ST77XX_BLACK);
          if (result.passedMeanTest && result.passedVarianceTest) {
            Counter->setNewOffsets();
            Counter->setNewFactors();
            axisResult[currentDirection] = true;
            tft.setTextColor(ST77XX_GREEN);
            tft.setCursor(5, 114); tft.print("PASS! Moving on...");
            delay(1500);
            directionDone[currentDirection] = true;
            currentDirection++;
            if (currentDirection >= totalDirections) {
              calibrationDone = calibrationAllPassed;
              if (calibrationAllPassed) drawCalibrationComplete();
              else drawCalibrationFailed();
            } else {
              drawCalibration();
            }
          } else {
            calibrationFailed = true;
            calibrationReady = true;
            if (!result.passedVarianceTest) {
              tft.setTextColor(ST77XX_RED);
              tft.setCursor(5, 109); tft.print("FAIL: hold still!");
            } else {
              tft.setTextColor(ST77XX_RED);
              tft.setCursor(5, 109); tft.print("FAIL: wrong axis?");
            }
            tft.setTextColor(ST77XX_YELLOW);
            drawHeartIcon(5, 118, ST77XX_YELLOW);
            tft.setCursor(16, 119); tft.print("=retry");
            drawStarIcon(68, 118, ST77XX_YELLOW);
            tft.setCursor(80, 119); tft.print("=skip");
          }
        }
      }
    } else if (currentPage == 3) {
      if (selfTestDone && calibrationDone) {
        Counter->setSteps(0);
        stepCount = 0;
        drawStepTrackingBottom();
      }
    }
  }

  if (btnC && !btnCLast) {
    if (currentPage != -1) {
      calibrationFailed = false;
      currentPage = 0;
      selectedItem = 0;
      drawMenu();
    }
  }

  btnALast = btnA;
  btnBLast = btnB;
  btnCLast = btnC;
}
// ========================
// SETUP & LOOP
// ========================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PA11, OUTPUT);
  digitalWrite(PA11, LOW);

  pinMode(PA12, INPUT);
  pinMode(PB0,  INPUT);
  pinMode(PB7,  INPUT);

  SPI.setMOSI(PB_5);
  SPI.setSCLK(PB_3);
  SPI.begin();

  tft.initR(INITR_144GREENTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  delay(100);
  rfid.PCD_Init();
  delay(50);
  rfid.PCD_Init();
  delay(50);

  analogReadResolution(12);
  Counter = new StepCounter(A0, A1, A2, A3);
  ADCPollTimer = new HardwareTimer(TIM2);
  ADCPollTimer->setOverflow(POLL_FREQ, HERTZ_FORMAT);
  ADCPollTimer->attachInterrupt(adcRead);
  ADCPollTimer->resume();

  drawLogin();
}

void loop() {
  handleButtons();
  handleRFID();
  //updateAnimation();
  updateStepDisplay();
}
