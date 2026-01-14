// =====================
// config.h (v9.001)
// =====================
#pragma once
#include <stdint.h>

#include "sifre.h"   // WiFi/Telegram gizli bilgiler burada

// =====================
// SÜRÜM
// =====================
#define PROJECT_VERSION "v9.001"

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

// =====================
// Preferences (NVS)
// =====================
inline constexpr const char* NVS_NAMESPACE = "koygen";

// =====================
// Pins (ADC)
// =====================
inline constexpr uint8_t PIN_ADC_MAINS     = 34;
inline constexpr uint8_t PIN_ADC_GEN       = 35;
inline constexpr uint8_t PIN_ADC_GEN_BATT  = 32;
inline constexpr uint8_t PIN_ADC_CAM_BATT  = 33;

// Save butonu (uzun bas: kaydet)
inline constexpr uint8_t PIN_BTN_SAVE = 27;

// =====================
// Stage 9 - Relay Control
// =====================
// Donanım bağlayana kadar 0 bırak (yanlışlıkla pulse atmasın).
#define ENABLE_RELAY_CONTROL 0

// Röle çıkış pinleri (örnek)
inline constexpr uint8_t PIN_RELAY_START = 25; // marş
inline constexpr uint8_t PIN_RELAY_STOP  = 26; // stop

// Röle aktif seviyesi (kartına göre değişebilir)
inline constexpr uint8_t RELAY_ACTIVE_LEVEL = HIGH;
inline constexpr uint8_t RELAY_IDLE_LEVEL   = LOW;

// START pulse denemeleri arası bekleme
inline constexpr uint32_t START_RETRY_GAP_MS = 3500;

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
// Şebeke bu değerin altına inerse (mainsFailConfirmS boyunca) oto start
inline constexpr float    AUTO_START_MAINS_V      = 160.0f;
inline constexpr uint16_t MAINS_FAIL_CONFIRM_S    = 15;   // sn
inline constexpr uint16_t MAINS_RETURN_CONFIRM_S  = 20;   // sn
inline constexpr uint16_t COOLDOWN_S              = 120;  // sn

inline constexpr uint16_t START_PULSE_MS          = 1200;
inline constexpr uint16_t STOP_PULSE_MS           = 1000;
inline constexpr uint8_t  START_MAX_ATTEMPTS      = 3;
