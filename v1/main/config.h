#pragma once
#include <stdint.h>

// =====================
// SÜRÜM
// =====================
#define PROJECT_VERSION "v5.001"

// =====================
// Serial
// =====================
inline constexpr uint32_t SERIAL_BAUD = 115200;

// =====================
// Features
// =====================
#define USE_LCD 0
#define USE_DATABASE 0

// =====================
// WiFi / Telegram
// =====================
inline constexpr const char* WIFI_SSID = "Ozturk__Ailesi";
inline constexpr const char* WIFI_PASS = "13052023Sm";
inline constexpr const char* BOT_TOKEN = "7482070875:AAH1wntK1gK99Ami9VFyX79yAT_B6JG0f6g";
inline constexpr const char* CHAT_ID   = "-1003432924253"; // grup chat id

// Admin (riskli komutlar sadece buna izinli)
inline constexpr long MASTER_ADMIN_ID = 1253195249; // kendi telegram user id

inline constexpr uint32_t TG_POLL_MS = 1200;

// =====================
// Preferences (NVS)
// =====================
inline constexpr const char* NVS_NAMESPACE = "koygen";

// =====================
// Pins
// =====================
inline constexpr uint8_t PIN_ADC_MAINS     = 34;
inline constexpr uint8_t PIN_ADC_GEN       = 35;
inline constexpr uint8_t PIN_ADC_GEN_BATT  = 32;
inline constexpr uint8_t PIN_ADC_CAM_BATT  = 33;

inline constexpr uint8_t PIN_BTN_SAVE = 27;

// =====================
// ADC & Ölçüm
// =====================
inline constexpr uint16_t ADC_MAX  = 4095;
inline constexpr float    ADC_VREF = 3.3f;

// Divider oranları (örnek 120k / 33k)
inline constexpr float GEN_BATT_DIV_RATIO = 4.63636f;
inline constexpr float CAM_BATT_DIV_RATIO = 4.63636f;

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
// Thresholds - MAINS/GEN (Stage 3)
// =====================
inline constexpr float MAINS_HIGH_V     = 245.0f;
inline constexpr float MAINS_NORMAL_MIN = 210.0f;
inline constexpr float MAINS_NORMAL_MAX = 240.0f;
inline constexpr float MAINS_LOW_V      = 200.0f;
inline constexpr float MAINS_CRIT_V     = 150.0f;

inline constexpr float GEN_OFF_V        = 50.0f;
inline constexpr float GEN_LOW_V        = 190.0f;
inline constexpr float GEN_NORMAL_MIN   = 210.0f;
inline constexpr float GEN_NORMAL_MAX   = 240.0f;

// =====================
// Thresholds - BATTERY (Stage 4)
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
// Jeneratör "çalışıyor" sayılması için voltaj eşiği
inline constexpr float GEN_RUNNING_V = 160.0f;

// False trigger engeli: en az kaç saniye üst üste çalışıyor olmalı
inline constexpr uint16_t GEN_RUNNING_CONFIRM_S = 10;

// NVS'ye çalışma süresi kaç saniyede bir yazılsın
inline constexpr uint32_t HOURS_SAVE_PERIOD_S = 60;
