// =====================
// config.h (v10.006)
// ESP32-WROOM-32D
// =====================
#pragma once
#include <stdint.h>

#include "sifre.h"

// =====================
// SÜRÜM
// =====================
#define PROJECT_VERSION "v10.006"

// =====================
// Cihaz Adı
// =====================
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

// Boot'ta telegram sessiz: sadece WiFi bağlanınca "Hazır"
inline constexpr bool TG_SEND_BOOT_REPORT = false;

// =====================
// Preferences (NVS)
// =====================
inline constexpr const char* NVS_NAMESPACE = "koygen";

// =====================
// Pins (ADC)  (ADC1 only)
// =====================
inline constexpr uint8_t PIN_ADC_MAINS     = 34;
inline constexpr uint8_t PIN_ADC_GEN       = 35;
inline constexpr uint8_t PIN_ADC_GEN_BATT  = 32;
inline constexpr uint8_t PIN_ADC_CAM_BATT  = 33;

// =====================
// TFT (ILI9341) - VSPI
// =====================
// VSPI default: SCK=18, MISO=19, MOSI=23
inline constexpr uint8_t PIN_TFT_SCK  = 18;
inline constexpr uint8_t PIN_TFT_MISO = 19;
inline constexpr uint8_t PIN_TFT_MOSI = 23;

// ⚠️ Strap pins used (keep HIGH at boot)
inline constexpr uint8_t PIN_TFT_CS   = 5;   // strap -> keep HIGH
inline constexpr uint8_t PIN_TFT_DC   = 2;   // strap -> keep HIGH
inline constexpr uint8_t PIN_TFT_RST  = 17;  // safe
inline constexpr uint8_t PIN_TFT_BL   = 16;  // backlight (optional), HIGH=on

// 0..3 (rotation)
inline constexpr uint8_t TFT_ROTATION = 1;   // 1: landscape (genelde 320x240)

// =====================
// UI Buttons (6) - INPUT_PULLUP (GND ile bas)
// =====================
// SOL/SAĞ/YUK/AŞA/OK/GERİ
inline constexpr uint8_t PIN_BTN_LEFT  = 27;
inline constexpr uint8_t PIN_BTN_RIGHT = 26;
inline constexpr uint8_t PIN_BTN_UP    = 13;
inline constexpr uint8_t PIN_BTN_DOWN  = 14;
inline constexpr uint8_t PIN_BTN_OK    = 25;

// ⚠️ Strap pin (keep HIGH at boot); boot sırasında basılı tutma
inline constexpr uint8_t PIN_BTN_BACK  = 4;

// Debounce & repeat
inline constexpr uint16_t BTN_DEBOUNCE_MS      = 25;
inline constexpr uint16_t BTN_REPEAT_START_MS  = 450;
inline constexpr uint16_t BTN_REPEAT_EVERY_MS  = 120;

// =====================
// Stage 9 - Relay Control (LOW aktif)
// Not: ENABLE_RELAY_CONTROL=0 iken röleler fiziksel çıkış vermez.
// =====================
#define ENABLE_RELAY_CONTROL 0

inline constexpr uint8_t PIN_RELAY_FUEL  = 21;
inline constexpr uint8_t PIN_RELAY_START = 22;
// ⚠️ Strap pin used (keep HIGH at boot / idle HIGH)
inline constexpr uint8_t PIN_RELAY_STOP  = 15;

inline constexpr uint8_t RELAY_ACTIVE_LEVEL = LOW;
inline constexpr uint8_t RELAY_IDLE_LEVEL   = HIGH;

inline constexpr uint16_t RELAY_SAFE_GAP_MS = 80;

// =====================
// ADC & Ölçüm
// =====================
inline constexpr uint16_t ADC_MAX  = 4095;
inline constexpr float    ADC_VREF = 3.3f;

// Divider oranları (örnek 120k / 33k)
inline constexpr float GEN_BATT_DIV_RATIO = 4.63636f;
inline constexpr float CAM_BATT_DIV_RATIO = 4.63636f;

// AC sensör kalibrasyonları (ZMPT101B vb.)
inline constexpr float CAL_MAINS = 240.0f;
inline constexpr float CAL_GEN   = 240.0f;

inline constexpr uint32_t MEASURE_MS       = 1000;
inline constexpr uint32_t SERIAL_REPORT_MS = 2000;

// =====================
// Filters
// =====================
inline constexpr float LPF_ALPHA_AC   = 0.15f;
inline constexpr float LPF_ALPHA_BATT = 0.20f;

inline constexpr uint16_t AC_SAMPLES  = 800;
inline constexpr uint16_t AC_US_DELAY = 200;

// =====================
// Sensor validity thresholds
// =====================
inline constexpr float AC_VALID_MIN_V   = 20.0f; // mains/gen <20V => sensör yok/boşta
inline constexpr float BATT_VALID_MIN_V = 5.0f;  // batt <5V => sensör yok/boşta

// Boot sonrası alert arm gecikmesi
inline constexpr uint16_t BOOT_MUTE_S = 20;

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
// Stage 5 - Çalışma Saat Sayacı
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

// =====================
// Auto start güvenliği
// =====================
inline constexpr bool AUTO_BLOCK_ON_BATT_CRIT = true;

// =====================
// FAULT exponential backoff retry
// =====================
inline constexpr uint8_t  FAULT_MAX_RETRIES   = 3;     // 3 deneme: 3dk -> 6dk -> 12dk
inline constexpr uint16_t FAULT_RETRY_BASE_S  = 180;   // 3 dk
inline constexpr uint16_t FAULT_RETRY_MAX_S   = 1800;  // 30 dk cap
