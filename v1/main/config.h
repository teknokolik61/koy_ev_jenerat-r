// =====================
// config.h (v10.003)
// =====================
#pragma once
#include <stdint.h>
#include "sifre.h"

// =====================
// VERSION
// =====================
#define PROJECT_VERSION "v10.003"
inline constexpr const char* DEVICE_NAME = "Jeneratör köy";

// =====================
// Serial
// =====================
inline constexpr uint32_t SERIAL_BAUD = 115200;

// =====================
// Telegram
// =====================
inline constexpr uint32_t TG_POLL_MS = 1200;
inline constexpr bool ENABLE_TG_STATE_ALERTS = true;

// =====================
// Preferences (NVS)
// =====================
inline constexpr const char* NVS_NAMESPACE = "koygen";

// =====================
// ADC pins (ADC1 ONLY, input-only pins)
// =====================
inline constexpr uint8_t PIN_ADC_MAINS     = 34;
inline constexpr uint8_t PIN_ADC_GEN       = 35;
inline constexpr uint8_t PIN_ADC_GEN_BATT  = 36;
inline constexpr uint8_t PIN_ADC_CAM_BATT  = 39;

// =====================
// Relay Control (LOW active)
// =====================
#define ENABLE_RELAY_CONTROL 0

inline constexpr uint8_t PIN_RELAY_FUEL  = 18;
inline constexpr uint8_t PIN_RELAY_START = 19;
inline constexpr uint8_t PIN_RELAY_STOP  = 23;

inline constexpr uint8_t  RELAY_ACTIVE_LEVEL = LOW;
inline constexpr uint8_t  RELAY_IDLE_LEVEL   = HIGH;
inline constexpr uint16_t RELAY_SAFE_GAP_MS  = 80;

// =====================
// TFT ILI9341 240x320 SPI (custom SPI pins)
// =====================
#define USE_TFT 1

inline constexpr uint8_t TFT_SCK  = 25;
inline constexpr uint8_t TFT_MOSI = 26;
inline constexpr uint8_t TFT_MISO = 255; // not used
inline constexpr uint8_t TFT_CS   = 27;
inline constexpr uint8_t TFT_DC   = 14;
inline constexpr uint8_t TFT_RST  = 13;

// Backlight (255 = yok)
inline constexpr uint8_t TFT_BACKLIGHT_PIN = 255;
inline constexpr bool    TFT_BACKLIGHT_ON  = true;

// 0..3
inline constexpr uint8_t TFT_ROTATION = 1;

// UI colors (RGB565)
inline constexpr uint16_t UI_COLOR_BG        = 0x0000;
inline constexpr uint16_t UI_COLOR_TEXT      = 0xFFFF;
inline constexpr uint16_t UI_COLOR_TEXT_DIM  = 0x7BEF;
inline constexpr uint16_t UI_COLOR_HEADER_BG = 0x001F;
inline constexpr uint16_t UI_COLOR_HEADER_FG = 0xFFFF;
inline constexpr uint16_t UI_COLOR_SEL_BG    = 0x07E0;
inline constexpr uint16_t UI_COLOR_SEL_FG    = 0x0000;
inline constexpr uint16_t UI_COLOR_WARN      = 0xF800;

// UI timing
inline constexpr uint32_t UI_TICK_MS = 20;
inline constexpr uint16_t UI_BTN_DEBOUNCE_MS     = 28;
inline constexpr uint16_t UI_BTN_REPEAT_DELAY_MS = 420;
inline constexpr uint16_t UI_BTN_REPEAT_RATE_MS  = 120;

// Hold-to-accelerate
inline constexpr bool UI_ENABLE_ACCEL = true;
inline constexpr uint16_t UI_ACCEL_1_MS = 800;
inline constexpr uint16_t UI_ACCEL_2_MS = 1600;
inline constexpr uint16_t UI_ACCEL_3_MS = 2400;
inline constexpr float UI_ACCEL_1_MULT  = 5.0f;
inline constexpr float UI_ACCEL_2_MULT  = 10.0f;
inline constexpr float UI_ACCEL_3_MULT  = 25.0f;

// Button 방향 ters ise buradan düzelt (istersen)
inline constexpr bool UI_SWAP_LEFT_RIGHT = false;
inline constexpr bool UI_SWAP_UP_DOWN    = false;

// =====================
// 6 Buttons (INPUT_PULLUP) - strap pin yok
// =====================
inline constexpr uint8_t PIN_BTN_LEFT  = 32;
inline constexpr uint8_t PIN_BTN_RIGHT = 33;
inline constexpr uint8_t PIN_BTN_UP    = 16;
inline constexpr uint8_t PIN_BTN_DOWN  = 17;
inline constexpr uint8_t PIN_BTN_BACK  = 21;
inline constexpr uint8_t PIN_BTN_OK    = 22;

// =====================
// ADC & Measure
// =====================
inline constexpr uint16_t ADC_MAX  = 4095;
inline constexpr float    ADC_VREF = 3.3f;

// Divider ratios (120k/33k)
inline constexpr float GEN_BATT_DIV_RATIO = 4.63636f;
inline constexpr float CAM_BATT_DIV_RATIO = 4.63636f;

// AC calibration
inline constexpr float CAL_MAINS = 240.0f;
inline constexpr float CAL_GEN   = 240.0f;

inline constexpr uint32_t MEASURE_MS       = 1000;
inline constexpr uint32_t SERIAL_REPORT_MS = 2000;

// Filters
inline constexpr float LPF_ALPHA_AC   = 0.15f;
inline constexpr float LPF_ALPHA_BATT = 0.20f;

// AC sampling
inline constexpr uint16_t AC_SAMPLES  = 800;
inline constexpr uint16_t AC_US_DELAY = 200;

// =====================
// Thresholds - MAINS
// =====================
inline constexpr float MAINS_HIGH_V     = 245.0f;
inline constexpr float MAINS_NORMAL_MIN = 210.0f;
inline constexpr float MAINS_NORMAL_MAX = 240.0f;
inline constexpr float MAINS_LOW_V      = 200.0f;
inline constexpr float MAINS_CRIT_V     = 150.0f;

// =====================
// Thresholds - GEN
// =====================
inline constexpr float GEN_OFF_V        = 50.0f;
inline constexpr float GEN_LOW_V        = 190.0f;
inline constexpr float GEN_NORMAL_MIN   = 210.0f;
inline constexpr float GEN_NORMAL_MAX   = 240.0f;

// =====================
// Thresholds - BATTERY
// =====================
inline constexpr float BATT_HIGH_V     = 13.2f;
inline constexpr float BATT_NORMAL_MIN = 12.2f;
inline constexpr float BATT_LOW_V      = 11.8f;
inline constexpr float BATT_CRIT_V     = 11.2f;

// =====================
// Hysteresis
// =====================
inline constexpr float HYST_V_AC   = 5.0f;
inline constexpr float HYST_V_BATT = 0.15f;

// =====================
// Stage 5 - Hours counter
// =====================
inline constexpr float    GEN_RUNNING_V          = 160.0f;
inline constexpr uint16_t GEN_RUNNING_CONFIRM_S = 10;
inline constexpr uint32_t HOURS_SAVE_PERIOD_S   = 60;

// =====================
// Stage 9 - AUTO defaults
// =====================
inline constexpr float    AUTO_START_MAINS_V      = 160.0f;
inline constexpr uint16_t MAINS_FAIL_CONFIRM_S    = 15;
inline constexpr uint16_t MAINS_RETURN_CONFIRM_S  = 20;
inline constexpr uint16_t COOLDOWN_S              = 120;

// =====================
// Stage 9 - Start/Stop timing defaults
// =====================
inline constexpr uint16_t FUEL_PRIME_MS        = 1500;
inline constexpr uint16_t START_PULSE_MS       = 1200;
inline constexpr uint32_t START_RETRY_GAP_MS   = 3500;
inline constexpr uint16_t START_SENSE_GRACE_MS = 2500;
inline constexpr uint8_t  START_MAX_ATTEMPTS   = 3;

inline constexpr uint16_t STOP_PULSE_MS        = 900;
inline constexpr uint16_t STOP_VERIFY_S        = 8;
inline constexpr uint8_t  STOP_MAX_ATTEMPTS    = 2;
inline constexpr uint16_t FUEL_OFF_DELAY_MS    = 600;

// Auto start safety
inline constexpr bool AUTO_BLOCK_ON_BATT_CRIT = true;

// FAULT backoff
inline constexpr uint8_t  FAULT_MAX_RETRIES   = 3;
inline constexpr uint16_t FAULT_RETRY_BASE_S  = 180;
inline constexpr uint16_t FAULT_RETRY_MAX_S   = 1800;
