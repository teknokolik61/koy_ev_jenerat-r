// =====================
// config.h (v9.007) — Aşama 9: Şebeke voltaj izleme (RMS) + raw debug
// LCD pinleri kilitli
// =====================
#pragma once
#include <stdint.h>

// Proje Kimliği
inline constexpr const char* PROJECT_NAME    = "Koy Evi Jenerator";
inline constexpr const char* PROJECT_VERSION = "v9.007";
inline constexpr const char* PROJECT_STAGE   = "Aşama 9";

// Serial
inline constexpr uint32_t SERIAL_BAUD = 115200;

// Feature flags
#define USE_LCD 1

// Heartbeat (Serial + LCD status güncelleme)
inline constexpr uint32_t HEARTBEAT_MS = 2000;

// =====================
// WiFi Ayarları
// =====================
inline constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS   = 15000;
inline constexpr uint32_t WIFI_RECONNECT_EVERY_MS   = 10000;

// =====================
// Telegram Ayarları
// =====================
inline constexpr uint32_t TG_POLL_MS                = 1500;
inline constexpr bool     TG_SEND_BOOT_MESSAGE      = true;

// Yetki
inline constexpr bool     TG_REQUIRE_AUTH           = true;  // true: komutlar admin ister
inline constexpr bool     TG_ALLOW_HELP_FOR_ALL     = true;  // /yardim /help herkese cevap
inline constexpr bool     TG_ALLOW_WHOAMI_FOR_ALL   = true;  // /whoami herkese cevap
inline constexpr bool     TG_STRICT_CHAT_ID         = true;  // true: sadece CHAT_ID sohbetinden komut kabul

// Runtime admin ekleme/silme (kalıcı değil; resetle gider)
inline constexpr bool     TG_RUNTIME_ADMIN_ENABLE   = true;
inline constexpr uint8_t  TG_RUNTIME_ADMIN_MAX      = 10;

// Ek admin listesi (MASTER_ADMIN_ID her zaman admin; buraya ekstra ID yazabilirsin)
namespace TgAuth {
  inline constexpr int64_t admin_list[] = {
    // ÖRNEK: 111111111, 222222222
  };
}

// =====================
// BUTONLAR (Aşama 7)
// Varsayılan: INPUT_PULLUP + ACTIVE_LOW
// =====================
namespace Buttons {
  inline constexpr int BTN_UP    = 4;
  inline constexpr int BTN_DOWN  = 5;
  inline constexpr int BTN_LEFT  = 6;
  inline constexpr int BTN_RIGHT = 7;
  inline constexpr int BTN_OK    = 15;
  inline constexpr int BTN_BACK  = 16;

  inline constexpr bool ACTIVE_LOW   = true;
  inline constexpr bool USE_PULLUP   = true;

  inline constexpr uint32_t DEBOUNCE_MS      = 35;
  inline constexpr uint32_t LONGPRESS_MS     = 800;
  inline constexpr uint32_t REPEAT_START_MS  = 500;
  inline constexpr uint32_t REPEAT_EVERY_MS  = 150;
}

// =====================
// UI Button tipleri (Arduino prototip üretimi için burada olmalı)
// =====================
enum class UiBtnId : uint8_t { UP, DOWN, LEFT, RIGHT, OK, BACK, COUNT };
enum class UiBtnEvt : uint8_t { PRESS, RELEASE, LONG, REPEAT };

struct UiBtnEvent { UiBtnId id; UiBtnEvt evt; uint32_t ms; };

struct UiBtnState {
  int pin = -1;
  bool enabled = false;
  bool raw = false;
  bool stable = false;
  uint32_t lastRawChangeMs = 0;
  bool pressed = false;
  uint32_t pressStartMs = 0;
  uint32_t lastRepeatMs = 0;
  bool longFired = false;
};

// =====================
// NTP / TIME (Aşama 8)
// =====================
#define ENABLE_NTP 1

// Türkiye sabit UTC+3 (DST yok). POSIX TZ formatında +3 => "TRT-3"
inline constexpr const char* NTP_TZ = "TRT-3";

// NTP sunucuları
inline constexpr const char* NTP_SERVER1 = "pool.ntp.org";
inline constexpr const char* NTP_SERVER2 = "time.google.com";
inline constexpr const char* NTP_SERVER3 = "time.cloudflare.com";

// Boot'ta ilk NTP bekleme (ms)
inline constexpr uint32_t NTP_BOOT_WAIT_MS = 8000;

// Günlük otomatik resync
inline constexpr bool     NTP_DAILY_RESYNC_ENABLE = true;
inline constexpr uint8_t  NTP_DAILY_RESYNC_HOUR   = 3;   // 03:00
inline constexpr uint8_t  NTP_DAILY_RESYNC_MIN    = 0;   // 03:00

// Ek güvenlik: uzun süre senkron olmadıysa zorla resync (ms)
inline constexpr uint32_t NTP_FORCE_RESYNC_EVERY_MS = 6UL * 60UL * 60UL * 1000UL; // 6 saat

// =====================
// ŞEBEKE VOLTAJ (Aşama 9)
// =====================
#define ENABLE_MAINS_MONITOR 1

namespace Mains {
  // ⚠️ BAĞLADIĞIN ADC pinini buraya yaz.
  inline constexpr int ADC_PIN = 1;

  inline constexpr uint32_t MEASURE_EVERY_MS = 1000;

  // 100ms pencere hedefi: 50Hz'de ~5 periyot
  inline constexpr uint16_t SAMPLES   = 500;
  inline constexpr uint16_t SAMPLE_US = 200;

  inline constexpr uint8_t  ADC_BITS  = 12;
  inline constexpr float    ADC_REF_V = 3.30f;

  // SENSOR? kriterleri (güçlendirilmiş)
  inline constexpr int      MIN_P2P_COUNTS     = 20;     // dalga genliği çok küçükse
  inline constexpr float    MIN_RMS_COUNTS     = 2.5f;   // RMS çok küçükse (gürültü)
  inline constexpr int      SAT_MARGIN_COUNTS  = 20;     // 0 veya max'a yapışma eşiği
  inline constexpr float    MEAN_MIN_FRAC      = 0.08f;  // mean çok uca kaymışsa
  inline constexpr float    MEAN_MAX_FRAC      = 0.92f;

  // Vrms = (rmsAdcV) * CAL_V_PER_ADC_RMS
  inline constexpr float    CAL_V_PER_ADC_RMS = 700.0f;

  inline constexpr float    LOW_V  = 200.0f;
  inline constexpr float    HIGH_V = 250.0f;
  inline constexpr float    HYST_V = 5.0f;

  inline constexpr bool     TG_ALERT_ENABLE = true;
  inline constexpr uint32_t TG_ALERT_MIN_INTERVAL_MS = 30000;
}

// Arduino prototip üretimi için tipler burada olmalı
enum class MainsState : uint8_t { SENSOR_INVALID, UNDERVOLT, NOMINAL, OVERVOLT };

struct MainsReading {
  bool  valid     = false;
  float vrms      = 0.0f;

  int   minRaw    = 4095;
  int   maxRaw    = 0;

  float meanCounts = 0.0f;
  float rmsCounts  = 0.0f;   // counts RMS (AC bileşen)
  float rmsAdcV    = 0.0f;   // RMS volt (ADC tarafı)

  MainsState state = MainsState::SENSOR_INVALID;
};

// =====================
// LCD / ILI9341 PİN AYARLARI (KESİNLİKLE DEĞİŞTİRİLMEYECEK)
// =====================
namespace Pins {
  inline constexpr int TFT_MOSI = 45;
  inline constexpr int TFT_MISO = 46;
  inline constexpr int TFT_SCK  = 3;

  inline constexpr int TFT_CS   = 14;
  inline constexpr int TFT_DC   = 47;
  inline constexpr int TFT_RST  = 21;

  inline constexpr int TFT_BL   = 9;
  inline constexpr bool TFT_BL_ACTIVE_HIGH = true;
}

// LCD ayarları
inline constexpr uint8_t  TFT_ROTATION = 1;        // 0-3
inline constexpr uint32_t TFT_SPI_HZ   = 40000000; // sorun olursa 27000000 yap
