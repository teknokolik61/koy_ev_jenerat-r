#pragma once
#include <stdint.h>

// =====================
// SÜRÜM
// =====================
#define PROJECT_VERSION "v1.001"

// =====================
// Serial
// =====================
inline constexpr uint32_t SERIAL_BAUD = 115200;

// =====================
// Feature Flags (Aşama 1: altyapı)
// =====================
#define USE_LCD 0
#define USE_DATABASE 0

// =====================
// WiFi / Telegram
// =====================
inline constexpr const char* WIFI_SSID = "SSID_YAZ";
inline constexpr const char* WIFI_PASS = "PASS_YAZ";

inline constexpr const char* BOT_TOKEN = "123456:ABC..."; // Telegram bot token
inline constexpr const char* CHAT_ID   = "123456789";     // Bildirim gönderilecek chat/grup id

// Telegram user id (sayısal). Sadece bu kişi komut çalıştırır (Aşama 1)
inline constexpr long MASTER_ADMIN_ID = 123456789;

// Telegram polling
inline constexpr uint32_t TG_POLL_MS = 1200;

// =====================
// Preferences (NVS)
// =====================
inline constexpr const char* NVS_NAMESPACE = "koygen";

// =====================
// Pins (ESP32-WROOM-32D)
// =====================
// ADC1 pinleri: 32-39 (WiFi ile uyumlu)
inline constexpr uint8_t PIN_ADC_MAINS     = 34; // ZMPT101B (şebeke)
inline constexpr uint8_t PIN_ADC_GEN       = 35; // ZMPT101B (jeneratör)
inline constexpr uint8_t PIN_ADC_GEN_BATT  = 32; // bölücü (jeneratör akü)
inline constexpr uint8_t PIN_ADC_CAM_BATT  = 33; // bölücü (kamera akü)

// Buton (ayar kaydet)
inline constexpr uint8_t PIN_BTN_SAVE = 27; // INPUT_PULLUP

// =====================
// ADC & Ölçüm Ayarları
// =====================
inline constexpr uint16_t ADC_MAX  = 4095;
inline constexpr float    ADC_VREF = 3.3f;

// Batarya bölücü oranları (Vbat = Vadc * (R1+R2)/R2)
// Örn: R1=120k R2=33k -> 4.63636
inline constexpr float GEN_BATT_DIV_RATIO = 4.63636f;
inline constexpr float CAM_BATT_DIV_RATIO = 4.63636f;

// ZMPT101B kalibrasyon (Aşama 1: kaba başlangıç)
inline constexpr float CAL_MAINS = 240.0f;
inline constexpr float CAL_GEN   = 240.0f;

// Ölçüm periyotları
inline constexpr uint32_t MEASURE_MS       = 2000;
inline constexpr uint32_t SERIAL_REPORT_MS = 2000;
