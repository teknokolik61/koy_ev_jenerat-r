#pragma once
#include <stdint.h>

// =====================
// SÜRÜM
// =====================
#define PROJECT_VERSION "v2.001"

// =====================
// Serial
// =====================
inline constexpr uint32_t SERIAL_BAUD = 115200;

// =====================
// Feature Flags
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
// Pins (ESP32-WROOM-32D)
// =====================
inline constexpr uint8_t PIN_ADC_MAINS     = 34; // ZMPT101B (şebeke)
inline constexpr uint8_t PIN_ADC_GEN       = 35; // ZMPT101B (jeneratör)
inline constexpr uint8_t PIN_ADC_GEN_BATT  = 32; // divider (gen akü)
inline constexpr uint8_t PIN_ADC_CAM_BATT  = 33; // divider (cam akü)

inline constexpr uint8_t PIN_BTN_SAVE = 27; // INPUT_PULLUP

// =====================
// ADC & Ölçüm Ayarları
// =====================
inline constexpr uint16_t ADC_MAX  = 4095;
inline constexpr float    ADC_VREF = 3.3f;

// Divider oranları
inline constexpr float GEN_BATT_DIV_RATIO = 4.63636f; // 120k/33k örnek
inline constexpr float CAM_BATT_DIV_RATIO = 4.63636f;

// ZMPT kalibrasyon başlangıç
inline constexpr float CAL_MAINS = 240.0f;
inline constexpr float CAL_GEN   = 240.0f;

// Ölçüm periyotları
inline constexpr uint32_t MEASURE_MS       = 1000;  // Aşama 2: daha sık ölç
inline constexpr uint32_t SERIAL_REPORT_MS = 2000;

// =====================
// Filtre ayarları
// =====================
// 0..1 arası: küçük -> daha yumuşak ama geç tepki
inline constexpr float LPF_ALPHA_AC   = 0.15f;
inline constexpr float LPF_ALPHA_BATT = 0.20f;

// AC RMS örnek sayısı (50Hz için daha iyi stabilite)
inline constexpr uint16_t AC_SAMPLES = 800; // ~160ms (delayMicroseconds 200 ile)
inline constexpr uint16_t AC_US_DELAY = 200; // mikro saniye
