// =====================
// main.ino (v10.003)
// =====================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include "config.h"

// =====================
// Globals
// =====================
Preferences prefs;

WiFiClientSecure tgClient;
UniversalTelegramBot bot(BOT_TOKEN, tgClient);

enum RunMode : uint8_t { MODE_MANUAL = 0, MODE_AUTO = 1 };
RunMode g_mode = MODE_MANUAL;

struct Settings {
  float calMains, calGen, genBattDiv, camBattDiv;

  float mainsHigh, mainsNormMin, mainsNormMax, mainsLow, mainsCrit;
  float genOff, genLow, genNormMin, genNormMax;

  float battHigh, battNormMin, battLow, battCrit;

  float hystAc;
  float hystBatt;

  // Hours
  float    genRunningV;
  uint16_t genRunConfirmS;
  uint32_t hoursSavePeriodS;

  // Auto
  float    autoStartMainsV;
  uint16_t mainsFailConfirmS;
  uint16_t mainsReturnConfirmS;
  uint16_t cooldownS;

  // Timing
  uint16_t fuelPrimeMs;
  uint16_t startPulseMs;
  uint32_t startRetryGapMs;
  uint16_t startSenseGraceMs;
  uint8_t  startMaxAttempts;

  uint16_t stopPulseMs;
  uint16_t stopVerifyS;
  uint8_t  stopMaxAttempts;
  uint16_t fuelOffDelayMs;

  // Fault
  uint8_t  faultMaxRetries;
  uint16_t faultRetryBaseS;
  uint16_t faultRetryMaxS;
} g_set;

struct Measurements {
  float mainsV_raw, genV_raw, genBattV_raw, camBattV_raw;
  float mainsV, genV, genBattV, camBattV;
  int wifiRssi;
  uint32_t uptimeS;
} g_meas;

static uint32_t tMeasure = 0, tSerial = 0, tTgPoll = 0, tUi = 0;

// ---------------------
// Battery State
// ---------------------
enum class BattState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
static BattState g_genBattState = BattState::UNKNOWN;
static BattState g_camBattState = BattState::UNKNOWN;

// ---------------------
// Mains / Gen state
// ---------------------
enum class MainsState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
enum class GenState   : uint8_t { UNKNOWN, OFF,      LOW_V, NORMAL, HIGH_V };

static MainsState g_mainsState = MainsState::UNKNOWN;
static GenState   g_genState   = GenState::UNKNOWN;

static bool g_stateAlertsArmed = false;

// ---------------------
// Hours Counter
// ---------------------
static bool     g_genRunning = false;
static uint16_t g_genRunStreakS = 0;
static uint64_t g_genRunTotalS  = 0;
static uint32_t g_lastHoursSaveS = 0;

// ---------------------
// AUTO State
// ---------------------
enum class AutoState : uint8_t {
  IDLE,
  WAIT_FAIL_CONFIRM,
  STARTING,
  RUNNING,
  WAIT_RETURN_CONFIRM,
  COOLDOWN,
  STOPPING,
  FAULT
};
static AutoState g_autoState = AutoState::IDLE;

static uint16_t g_failStreakS = 0;
static uint16_t g_returnStreakS = 0;
static uint16_t g_coolCounterS = 0;

// FAULT runtime
static uint16_t g_faultClearStreakS = 0;
static uint8_t  g_faultRetryCount = 0;
static uint32_t g_faultNextRetryS = 0;

// ---------------------
// Relay Control
// ---------------------
static bool g_fuelOn = false;
static bool g_startOn = false;
static bool g_stopOn  = false;

// Pulse engine
static bool g_pulseActive = false;
static uint8_t g_pulsePin = 255;
static uint32_t g_pulseUntilMs = 0;

// =====================
// TFT
// =====================
#if USE_TFT
static SPIClass tftSPI(VSPI);
static Adafruit_ILI9341 tft(&tftSPI, TFT_CS, TFT_DC, TFT_RST);
static bool tftReady = false;
#endif

// =====================
// UI Types (IMPORTANT: must be BEFORE uiBtnInit etc.)
// =====================
enum class UiScreen : uint8_t { STATUS, MENU, SETTINGS, EDIT, CONFIRM };

struct UiBtn {
  uint8_t pin = 255;
  bool lastRaw = true;
  bool stable = true;
  uint32_t lastChangeMs = 0;

  bool pressedEvt = false;
  bool repeatEvt  = false;

  uint32_t downSinceMs = 0;
  uint32_t nextRepeatMs = 0;
};

enum class UiCategory : uint8_t { MAINS, GEN, BATT, AUTO, TIMING, FAULT, CAL, COUNT };

enum class UiParamType : uint8_t { F32, U16, U32, U8 };

struct UiParamRef {
  UiCategory cat;
  const char* name;
  UiParamType type;

  void* ptr;           // points to g_set member
  float minF, maxF;    // for float/u16/u32/u8 (min/max in float)
  float stepF;         // base step (float)

  const char* nvsKey;  // save key
};

struct UiCatList { UiParamRef* arr; uint8_t count; };

// =====================
// Helpers
// =====================
static String fmt2(float v) {
  if (isnan(v) || isinf(v)) return "nan";
  char b[20];
  dtostrf(v, 0, 2, b);
  return String(b);
}

static String fmtDurTR(uint32_t sec) {
  uint32_t m = sec / 60U;
  uint32_t s = sec % 60U;
  if (m == 0) return String(s) + "sn";
  if (s == 0) return String(m) + "dk";
  return String(m) + "dk " + String(s) + "sn";
}

static String fmtHMS(uint64_t totalS) {
  uint64_t h = totalS / 3600ULL;
  uint64_t m = (totalS % 3600ULL) / 60ULL;
  uint64_t s = totalS % 60ULL;
  return String((unsigned long)h) + "h " + String((unsigned long)m) + "m " + String((unsigned long)s) + "s";
}

static float lpf(float prev, float x, float a) {
  if (isnan(prev) || isinf(prev)) return x;
  return prev + a * (x - prev);
}

static void notifyTo(const String& chatId, const String& msg) {
  if (WiFi.status() == WL_CONNECTED) bot.sendMessage(chatId, msg, "");
}
static void notify(const String& msg) { notifyTo(String(CHAT_ID), msg); }

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(250);
    yield();
  }
}

static const char* modeText(RunMode m) { return (m == MODE_AUTO) ? "AUTO" : "MANUAL"; }

static const char* autoStateText(AutoState st) {
  switch (st) {
    case AutoState::IDLE:                return "IDLE";
    case AutoState::WAIT_FAIL_CONFIRM:   return "WAIT_FAIL_CONFIRM";
    case AutoState::STARTING:            return "STARTING";
    case AutoState::RUNNING:             return "RUNNING";
    case AutoState::WAIT_RETURN_CONFIRM: return "WAIT_RETURN_CONFIRM";
    case AutoState::COOLDOWN:            return "COOLDOWN";
    case AutoState::STOPPING:            return "STOPPING";
    case AutoState::FAULT:               return "FAULT";
    default:                             return "UNKNOWN";
  }
}

static uint32_t faultBackoffDelayS(uint8_t retryIndexFrom0) {
  uint32_t base = (uint32_t)g_set.faultRetryBaseS;
  uint8_t sh = retryIndexFrom0;
  if (sh > 15) sh = 15;
  uint32_t d = base << sh;
  if (d > (uint32_t)g_set.faultRetryMaxS) d = (uint32_t)g_set.faultRetryMaxS;
  return d;
}

static String buildBackoffPlanText(uint8_t alreadyTriedCount) {
  if (alreadyTriedCount >= g_set.faultMaxRetries) return "Plan yok (limit doldu)";
  String s;
  uint8_t startIdx = alreadyTriedCount;
  uint8_t endIdx = g_set.faultMaxRetries - 1;
  for (uint8_t i = startIdx; i <= endIdx; i++) {
    s += fmtDurTR(faultBackoffDelayS(i));
    if (i != endIdx) s += " → ";
  }
  if (g_set.faultRetryMaxS > 0) s += " (cap: " + fmtDurTR(g_set.faultRetryMaxS) + ")";
  return s;
}

// =====================
// Relay
// =====================
static void relayWrite(uint8_t pin, bool on) {
#if ENABLE_RELAY_CONTROL
  digitalWrite(pin, on ? RELAY_ACTIVE_LEVEL : RELAY_IDLE_LEVEL);
#else
  (void)pin; (void)on;
#endif
}

static void relayAllOffNow() {
#if ENABLE_RELAY_CONTROL
  relayWrite(PIN_RELAY_START, false);
  relayWrite(PIN_RELAY_STOP,  false);
  relayWrite(PIN_RELAY_FUEL,  false);
#endif
  g_fuelOn = g_startOn = g_stopOn = false;
  g_pulseActive = false;
  g_pulsePin = 255;
  g_pulseUntilMs = 0;
}

static void relayInit() {
#if ENABLE_RELAY_CONTROL
  pinMode(PIN_RELAY_FUEL, OUTPUT);
  pinMode(PIN_RELAY_START, OUTPUT);
  pinMode(PIN_RELAY_STOP,  OUTPUT);
#endif
  relayAllOffNow();
}

static void safeSetStart(bool on) {
#if ENABLE_RELAY_CONTROL
  if (on && g_stopOn) {
    relayWrite(PIN_RELAY_STOP, false);
    g_stopOn = false;
    delay(RELAY_SAFE_GAP_MS);
  }
  relayWrite(PIN_RELAY_START, on);
#endif
  g_startOn = on;
}

static void safeSetStop(bool on) {
#if ENABLE_RELAY_CONTROL
  if (on && g_startOn) {
    relayWrite(PIN_RELAY_START, false);
    g_startOn = false;
    delay(RELAY_SAFE_GAP_MS);
  }
  relayWrite(PIN_RELAY_STOP, on);
#endif
  g_stopOn = on;
}

static void safeSetFuel(bool on) {
#if ENABLE_RELAY_CONTROL
  relayWrite(PIN_RELAY_FUEL, on);
#endif
  g_fuelOn = on;
}

static void relayPulse(uint8_t pin, uint16_t ms) {
#if ENABLE_RELAY_CONTROL
  if (pin == PIN_RELAY_START) { safeSetStop(false); safeSetStart(true); }
  if (pin == PIN_RELAY_STOP)  { safeSetStart(false); safeSetStop(true); }
  g_pulseActive = true;
  g_pulsePin = pin;
  g_pulseUntilMs = millis() + (uint32_t)ms;
#else
  (void)pin; (void)ms;
#endif
}

static void relayPulseService() {
#if ENABLE_RELAY_CONTROL
  if (!g_pulseActive) return;
  if ((int32_t)(millis() - g_pulseUntilMs) >= 0) {
    if (g_pulsePin == PIN_RELAY_START) safeSetStart(false);
    if (g_pulsePin == PIN_RELAY_STOP)  safeSetStop(false);
    g_pulseActive = false;
    g_pulsePin = 255;
  }
#endif
}

// =====================
// NVS
// =====================
static void loadSettings() {
  prefs.begin(NVS_NAMESPACE, false);

  g_set.calMains   = prefs.getFloat("calMains", CAL_MAINS);
  g_set.calGen     = prefs.getFloat("calGen",   CAL_GEN);
  g_set.genBattDiv = prefs.getFloat("genDiv",   GEN_BATT_DIV_RATIO);
  g_set.camBattDiv = prefs.getFloat("camDiv",   CAM_BATT_DIV_RATIO);

  g_set.mainsHigh    = prefs.getFloat("mHi",  MAINS_HIGH_V);
  g_set.mainsNormMin = prefs.getFloat("mNmn", MAINS_NORMAL_MIN);
  g_set.mainsNormMax = prefs.getFloat("mNmx", MAINS_NORMAL_MAX);
  g_set.mainsLow     = prefs.getFloat("mLo",  MAINS_LOW_V);
  g_set.mainsCrit    = prefs.getFloat("mCr",  MAINS_CRIT_V);

  g_set.genOff       = prefs.getFloat("gOff", GEN_OFF_V);
  g_set.genLow       = prefs.getFloat("gLo",  GEN_LOW_V);
  g_set.genNormMin   = prefs.getFloat("gNmn", GEN_NORMAL_MIN);
  g_set.genNormMax   = prefs.getFloat("gNmx", GEN_NORMAL_MAX);

  g_set.battHigh     = prefs.getFloat("bHi",  BATT_HIGH_V);
  g_set.battNormMin  = prefs.getFloat("bNmn", BATT_NORMAL_MIN);
  g_set.battLow      = prefs.getFloat("bLo",  BATT_LOW_V);
  g_set.battCrit     = prefs.getFloat("bCr",  BATT_CRIT_V);

  g_set.hystAc       = prefs.getFloat("hAc",  HYST_V_AC);
  g_set.hystBatt     = prefs.getFloat("hBt",  HYST_V_BATT);

  g_set.genRunningV      = prefs.getFloat("gRunV", GEN_RUNNING_V);
  g_set.genRunConfirmS   = prefs.getUShort("gRunC", GEN_RUNNING_CONFIRM_S);
  g_set.hoursSavePeriodS = prefs.getUInt("hSaveP", HOURS_SAVE_PERIOD_S);

  g_set.autoStartMainsV      = prefs.getFloat("aMnsV", AUTO_START_MAINS_V);
  g_set.mainsFailConfirmS    = prefs.getUShort("aFail", MAINS_FAIL_CONFIRM_S);
  g_set.mainsReturnConfirmS  = prefs.getUShort("aRet",  MAINS_RETURN_CONFIRM_S);
  g_set.cooldownS            = prefs.getUShort("aCool", COOLDOWN_S);

  g_set.fuelPrimeMs          = prefs.getUShort("pF",    FUEL_PRIME_MS);
  g_set.startPulseMs         = prefs.getUShort("pSt",   START_PULSE_MS);
  g_set.startRetryGapMs      = prefs.getUInt  ("gSt",   START_RETRY_GAP_MS);
  g_set.startSenseGraceMs    = prefs.getUShort("sSt",   START_SENSE_GRACE_MS);
  g_set.startMaxAttempts     = prefs.getUChar ("aAtt",  START_MAX_ATTEMPTS);

  g_set.stopPulseMs          = prefs.getUShort("pSp",   STOP_PULSE_MS);
  g_set.stopVerifyS          = prefs.getUShort("vSp",   STOP_VERIFY_S);
  g_set.stopMaxAttempts      = prefs.getUChar ("mSp",   STOP_MAX_ATTEMPTS);
  g_set.fuelOffDelayMs       = prefs.getUShort("dF",    FUEL_OFF_DELAY_MS);

  g_set.faultMaxRetries      = prefs.getUChar ("fMax",  FAULT_MAX_RETRIES);
  g_set.faultRetryBaseS      = prefs.getUShort("fBase", FAULT_RETRY_BASE_S);
  g_set.faultRetryMaxS       = prefs.getUShort("fCap",  FAULT_RETRY_MAX_S);

  g_mode = (RunMode)prefs.getUChar("mode", (uint8_t)MODE_MANUAL);

  uint32_t lo = prefs.getUInt("hrsLo", 0);
  uint32_t hi = prefs.getUInt("hrsHi", 0);
  g_genRunTotalS = ((uint64_t)hi << 32) | (uint64_t)lo;
}

static void saveSettings() {
  prefs.putFloat("calMains", g_set.calMains);
  prefs.putFloat("calGen",   g_set.calGen);
  prefs.putFloat("genDiv",   g_set.genBattDiv);
  prefs.putFloat("camDiv",   g_set.camBattDiv);

  prefs.putFloat("mHi",  g_set.mainsHigh);
  prefs.putFloat("mNmn", g_set.mainsNormMin);
  prefs.putFloat("mNmx", g_set.mainsNormMax);
  prefs.putFloat("mLo",  g_set.mainsLow);
  prefs.putFloat("mCr",  g_set.mainsCrit);

  prefs.putFloat("gOff", g_set.genOff);
  prefs.putFloat("gLo",  g_set.genLow);
  prefs.putFloat("gNmn", g_set.genNormMin);
  prefs.putFloat("gNmx", g_set.genNormMax);

  prefs.putFloat("bHi",  g_set.battHigh);
  prefs.putFloat("bNmn", g_set.battNormMin);
  prefs.putFloat("bLo",  g_set.battLow);
  prefs.putFloat("bCr",  g_set.battCrit);

  prefs.putFloat("hAc",  g_set.hystAc);
  prefs.putFloat("hBt",  g_set.hystBatt);

  prefs.putFloat("gRunV", g_set.genRunningV);
  prefs.putUShort("gRunC", g_set.genRunConfirmS);
  prefs.putUInt("hSaveP", g_set.hoursSavePeriodS);

  prefs.putFloat("aMnsV", g_set.autoStartMainsV);
  prefs.putUShort("aFail", g_set.mainsFailConfirmS);
  prefs.putUShort("aRet",  g_set.mainsReturnConfirmS);
  prefs.putUShort("aCool", g_set.cooldownS);

  prefs.putUShort("pF",  g_set.fuelPrimeMs);
  prefs.putUShort("pSt", g_set.startPulseMs);
  prefs.putUInt  ("gSt", g_set.startRetryGapMs);
  prefs.putUShort("sSt", g_set.startSenseGraceMs);
  prefs.putUChar ("aAtt", g_set.startMaxAttempts);

  prefs.putUShort("pSp", g_set.stopPulseMs);
  prefs.putUShort("vSp", g_set.stopVerifyS);
  prefs.putUChar ("mSp", g_set.stopMaxAttempts);
  prefs.putUShort("dF",  g_set.fuelOffDelayMs);

  prefs.putUChar ("fMax",  g_set.faultMaxRetries);
  prefs.putUShort("fBase", g_set.faultRetryBaseS);
  prefs.putUShort("fCap",  g_set.faultRetryMaxS);

  prefs.putUChar("mode", (uint8_t)g_mode);
}

static void saveHoursTotal() {
  uint32_t lo = (uint32_t)(g_genRunTotalS & 0xFFFFFFFFULL);
  uint32_t hi = (uint32_t)(g_genRunTotalS >> 32);
  prefs.putUInt("hrsLo", lo);
  prefs.putUInt("hrsHi", hi);
}

// =====================
// ADC Reads
// =====================
static float readAdcVoltage(uint8_t pin, uint16_t samples = 64) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
    yield();
  }
  float adc = (float)sum / (float)samples;
  return (adc / (float)ADC_MAX) * ADC_VREF;
}

static float readAcRmsApprox(uint8_t pin, float calScale) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < AC_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(AC_US_DELAY);
    yield();
  }
  float mean = (float)sum / (float)AC_SAMPLES;

  double sq = 0.0;
  for (uint16_t i = 0; i < AC_SAMPLES; i++) {
    float x = (float)analogRead(pin) - mean;
    sq += (double)(x * x);
    delayMicroseconds(AC_US_DELAY);
    yield();
  }
  float rmsCounts = sqrt((float)(sq / (double)AC_SAMPLES));
  float rmsVadc   = (rmsCounts / (float)ADC_MAX) * ADC_VREF;

  float vrms = rmsVadc * calScale;
  if (vrms < 0) vrms = 0;
  return vrms;
}

// =====================
// Battery State
// =====================
static const char* battStateToText(BattState st) {
  switch (st) {
    case BattState::CRITICAL: return "CRITICAL";
    case BattState::LOW_V:    return "LOW";
    case BattState::NORMAL:   return "NORMAL";
    case BattState::HIGH_V:   return "HIGH";
    default:                  return "UNKNOWN";
  }
}

static BattState evalBatt(float v, BattState prev) {
  float h = g_set.hystBatt;
  switch (prev) {
    case BattState::HIGH_V:
      if (v <= g_set.battHigh - h) return BattState::NORMAL;
      return BattState::HIGH_V;

    case BattState::NORMAL:
      if (v >= g_set.battHigh) return BattState::HIGH_V;
      if (v <  g_set.battCrit) return BattState::CRITICAL;
      if (v <  g_set.battLow)  return BattState::LOW_V;
      return BattState::NORMAL;

    case BattState::LOW_V:
      if (v >= g_set.battNormMin + h) return BattState::NORMAL;
      if (v <  g_set.battCrit)        return BattState::CRITICAL;
      return BattState::LOW_V;

    case BattState::CRITICAL:
      if (v >= g_set.battLow + h) return BattState::LOW_V;
      return BattState::CRITICAL;

    default:
      if (v >= g_set.battHigh) return BattState::HIGH_V;
      if (v <  g_set.battCrit) return BattState::CRITICAL;
      if (v <  g_set.battLow)  return BattState::LOW_V;
      return BattState::NORMAL;
  }
}

static void updateBatteryStatesOnly() {
  g_genBattState = evalBatt(g_meas.genBattV, g_genBattState);
  g_camBattState = evalBatt(g_meas.camBattV, g_camBattState);
}

// =====================
// Mains/Gen state eval + Telegram
// =====================
static String mainsStateLine(MainsState st, float v) {
  switch (st) {
    case MainsState::CRITICAL: return "🚨 Şebeke KRİTİK: " + fmt2(v) + "V";
    case MainsState::LOW_V:    return "⚠️ Şebeke DÜŞÜK: " + fmt2(v) + "V";
    case MainsState::HIGH_V:   return "⚠️ Şebeke YÜKSEK: " + fmt2(v) + "V";
    case MainsState::NORMAL:   return "✅ Şebeke NORMAL: " + fmt2(v) + "V";
    default:                   return "ℹ️ Şebeke: " + fmt2(v) + "V";
  }
}

static String genStateLine(GenState st, float v) {
  switch (st) {
    case GenState::OFF:     return "⛔ Jeneratör OFF: " + fmt2(v) + "V";
    case GenState::LOW_V:   return "⚠️ Jeneratör DÜŞÜK: " + fmt2(v) + "V";
    case GenState::HIGH_V:  return "⚠️ Jeneratör YÜKSEK: " + fmt2(v) + "V";
    case GenState::NORMAL:  return "✅ Jeneratör NORMAL: " + fmt2(v) + "V";
    default:                return "ℹ️ Jeneratör: " + fmt2(v) + "V";
  }
}

static MainsState evalMains(float v, MainsState prev) {
  float h = g_set.hystAc;
  switch (prev) {
    case MainsState::CRITICAL:
      if (v >= g_set.mainsCrit + h) return MainsState::LOW_V;
      return MainsState::CRITICAL;

    case MainsState::LOW_V:
      if (v < g_set.mainsCrit) return MainsState::CRITICAL;
      if (v >= g_set.mainsLow + h) return MainsState::NORMAL;
      return MainsState::LOW_V;

    case MainsState::NORMAL:
      if (v >= g_set.mainsHigh) return MainsState::HIGH_V;
      if (v <  g_set.mainsLow)  return MainsState::LOW_V;
      return MainsState::NORMAL;

    case MainsState::HIGH_V:
      if (v <= g_set.mainsHigh - h) return MainsState::NORMAL;
      return MainsState::HIGH_V;

    default:
      if (v < g_set.mainsCrit) return MainsState::CRITICAL;
      if (v < g_set.mainsLow)  return MainsState::LOW_V;
      if (v >= g_set.mainsHigh) return MainsState::HIGH_V;
      return MainsState::NORMAL;
  }
}

static GenState evalGen(float v, GenState prev) {
  float h = g_set.hystAc;
  switch (prev) {
    case GenState::OFF:
      if (v >= g_set.genOff + h) return GenState::LOW_V;
      return GenState::OFF;

    case GenState::LOW_V:
      if (v < g_set.genOff) return GenState::OFF;
      if (v >= g_set.genNormMin + h) return GenState::NORMAL;
      return GenState::LOW_V;

    case GenState::NORMAL:
      if (v > g_set.genNormMax) return GenState::HIGH_V;
      if (v < g_set.genLow)     return GenState::LOW_V;
      return GenState::NORMAL;

    case GenState::HIGH_V:
      if (v <= g_set.genNormMax - h) return GenState::NORMAL;
      return GenState::HIGH_V;

    default:
      if (v < g_set.genOff) return GenState::OFF;
      if (v < g_set.genLow) return GenState::LOW_V;
      if (v > g_set.genNormMax) return GenState::HIGH_V;
      return GenState::NORMAL;
  }
}

static void handleStateAlerts() {
  if (!ENABLE_TG_STATE_ALERTS) return;
  if (!g_stateAlertsArmed) return;

  MainsState nm = evalMains(g_meas.mainsV, g_mainsState);
  if (nm != g_mainsState) {
    g_mainsState = nm;
    notify(mainsStateLine(g_mainsState, g_meas.mainsV));
  }

  GenState ng = evalGen(g_meas.genV, g_genState);
  if (ng != g_genState) {
    g_genState = ng;
    notify(genStateLine(g_genState, g_meas.genV));
  }
}

// =====================
// Hours counter (1Hz)
// =====================
static bool isGenRunningNow() { return (g_meas.genV >= g_set.genRunningV); }

static void updateGenHoursCounter_1s() {
  bool above = (g_meas.genV >= g_set.genRunningV);

  if (above) {
    if (g_genRunStreakS < 65000) g_genRunStreakS++;
  } else {
    g_genRunStreakS = 0;
    g_genRunning = false;
  }

  if (!g_genRunning && g_genRunStreakS >= g_set.genRunConfirmS) {
    g_genRunning = true;
    notify("▶️ Jeneratör ÇALIŞIYOR (sayaç başladı)");
  }

  if (g_genRunning) {
    g_genRunTotalS += 1;
    if (!above) {
      g_genRunning = false;
      notify("⏹️ Jeneratör DURDU (sayaç durdu)");
    }
  }

  if (g_meas.uptimeS - g_lastHoursSaveS >= g_set.hoursSavePeriodS) {
    g_lastHoursSaveS = g_meas.uptimeS;
    saveHoursTotal();
  }
}

// =====================
// Start/Stop sequences
// =====================
enum class StartSub : uint8_t { PRIME, CRANK, SENSE, REST };
enum class StopSub  : uint8_t { PULSE, VERIFY };

struct StartSeq {
  bool active = false;
  bool manual = false;
  StartSub sub = StartSub::PRIME;
  uint32_t untilMs = 0;
  uint8_t attempt = 0;
} g_startSeq;

struct StopSeq {
  bool active = false;
  bool manual = false;
  StopSub sub = StopSub::PULSE;
  uint32_t untilMs = 0;
  uint8_t attempt = 0;
  uint32_t fuelOffAtMs = 0;
  uint32_t verifyDeadlineMs = 0;
} g_stopSeq;

static bool autoStartBlockedByBatt() {
  if (!AUTO_BLOCK_ON_BATT_CRIT) return false;
  return (g_genBattState == BattState::CRITICAL);
}

static void startSeqBegin(bool manual) {
  g_startSeq.active = true;
  g_startSeq.manual = manual;
  g_startSeq.sub = StartSub::PRIME;
  g_startSeq.untilMs = millis();
  g_startSeq.attempt = 0;
  safeSetFuel(true);
}

static void stopSeqBegin(bool manual) {
  g_stopSeq.active = true;
  g_stopSeq.manual = manual;
  g_stopSeq.sub = StopSub::PULSE;
  g_stopSeq.untilMs = millis();
  g_stopSeq.attempt = 0;
  g_stopSeq.fuelOffAtMs = 0;
  g_stopSeq.verifyDeadlineMs = 0;
}

static void startSeqService() {
  if (!g_startSeq.active) return;
  const uint32_t now = millis();

  if (isGenRunningNow()) { g_startSeq.active = false; return; }
  if (!g_startSeq.manual && autoStartBlockedByBatt()) { g_startSeq.active = false; return; }
  if ((int32_t)(now - g_startSeq.untilMs) < 0) return;

  switch (g_startSeq.sub) {
    case StartSub::PRIME:
      safeSetFuel(true);
      g_startSeq.untilMs = now + (uint32_t)g_set.fuelPrimeMs;
      g_startSeq.sub = StartSub::CRANK;
      break;

    case StartSub::CRANK:
      if (g_pulseActive) return;
      if (g_startSeq.attempt >= g_set.startMaxAttempts) { g_startSeq.active = false; return; }
      g_startSeq.attempt++;
      notify(String("🟡 Start denemesi ") + g_startSeq.attempt + "/" + g_set.startMaxAttempts);
      relayPulse(PIN_RELAY_START, g_set.startPulseMs);
      g_startSeq.untilMs = now + (uint32_t)g_set.startPulseMs + (uint32_t)g_set.startSenseGraceMs;
      g_startSeq.sub = StartSub::SENSE;
      break;

    case StartSub::SENSE:
      g_startSeq.untilMs = now + (uint32_t)g_set.startRetryGapMs;
      g_startSeq.sub = StartSub::REST;
      break;

    case StartSub::REST:
      g_startSeq.sub = StartSub::CRANK;
      g_startSeq.untilMs = now;
      break;
  }
}

static void stopSeqService() {
  if (!g_stopSeq.active) return;
  const uint32_t now = millis();

  if (!isGenRunningNow()) {
    g_stopSeq.active = false;
    safeSetFuel(false);
    return;
  }

  if ((int32_t)(now - g_stopSeq.untilMs) < 0) return;

  switch (g_stopSeq.sub) {
    case StopSub::PULSE:
      if (g_pulseActive) return;
      if (g_stopSeq.attempt >= g_set.stopMaxAttempts) { g_stopSeq.active = false; safeSetFuel(false); return; }
      g_stopSeq.attempt++;
      notify(String("🟥 Stop pulse ") + g_stopSeq.attempt + "/" + g_set.stopMaxAttempts);
      relayPulse(PIN_RELAY_STOP, g_set.stopPulseMs);
      g_stopSeq.fuelOffAtMs = now + (uint32_t)g_set.fuelOffDelayMs;
      g_stopSeq.verifyDeadlineMs = now + (uint32_t)g_set.stopVerifyS * 1000UL;
      g_stopSeq.sub = StopSub::VERIFY;
      g_stopSeq.untilMs = now + 50;
      break;

    case StopSub::VERIFY:
      if (g_stopSeq.fuelOffAtMs && (int32_t)(now - g_stopSeq.fuelOffAtMs) >= 0) {
        safeSetFuel(false);
        g_stopSeq.fuelOffAtMs = 0;
      }
      if (!isGenRunningNow()) { g_stopSeq.active = false; safeSetFuel(false); return; }
      if ((int32_t)(now - g_stopSeq.verifyDeadlineMs) >= 0) {
        g_stopSeq.sub = StopSub::PULSE;
        g_stopSeq.untilMs = now + 100;
      } else {
        g_stopSeq.untilMs = now + 120;
      }
      break;
  }
}

// =====================
// AUTO logic (1Hz)
// =====================
static bool mainsIsBadForAuto() { return (g_meas.mainsV < g_set.autoStartMainsV); }
static bool mainsIsGoodToStop() { return (g_meas.mainsV >= g_set.mainsNormMin && g_meas.mainsV <= g_set.mainsNormMax); }

static void autoSetState(AutoState st, const String& reasonMsg = "") {
  if (st == g_autoState) return;

  if (st == AutoState::FAULT) {
    g_faultClearStreakS = 0;
    g_faultNextRetryS = 0;
  }
  if (g_autoState == AutoState::FAULT && st != AutoState::FAULT) {
    g_faultClearStreakS = 0;
    g_faultRetryCount = 0;
    g_faultNextRetryS = 0;
  }

  g_autoState = st;

  if (reasonMsg.length())
    notify("🤖 AUTO: " + String(autoStateText(g_autoState)) + " — " + reasonMsg);
  else
    notify("🤖 AUTO: " + String(autoStateText(g_autoState)));
}

static void autoFuelPolicy() {
  if (g_mode != MODE_AUTO) return;

  switch (g_autoState) {
    case AutoState::STARTING:
    case AutoState::RUNNING:
    case AutoState::WAIT_RETURN_CONFIRM:
    case AutoState::COOLDOWN:
    case AutoState::STOPPING:
      safeSetFuel(true);
      break;
    default:
      safeSetFuel(false);
      break;
  }
}

static void enterFaultWithSchedule(const String& why) {
  autoSetState(AutoState::FAULT, why);
  safeSetFuel(false);

  if (g_faultRetryCount < g_set.faultMaxRetries) {
    uint32_t d = faultBackoffDelayS(g_faultRetryCount);
    g_faultNextRetryS = g_meas.uptimeS + d;

    notify(
      "🧯 FAULT: Auto-retry " + fmtDurTR(d) + " sonra. ("
      + String(g_faultRetryCount + 1) + "/" + String(g_set.faultMaxRetries) + ")\n"
      + "📌 Backoff Plan: " + buildBackoffPlanText(g_faultRetryCount) + "\n"
      + "✅ Şebeke normale dönünce auto-reset."
    );
  } else {
    g_faultNextRetryS = 0;
    notify("🧯 FAULT: Auto-retry limiti doldu.\n✅ Şebeke normale dönünce auto-reset.");
  }
}

static void autoTick_1s() {
  if (g_mode != MODE_AUTO) {
    if (g_autoState != AutoState::IDLE) {
      g_autoState = AutoState::IDLE;
      g_failStreakS = 0;
      g_returnStreakS = 0;
      g_coolCounterS = 0;
      g_faultClearStreakS = 0;
      g_faultRetryCount = 0;
      g_faultNextRetryS = 0;
    }
    return;
  }

  autoFuelPolicy();

  if (g_autoState == AutoState::FAULT) {
    if (mainsIsGoodToStop()) {
      if (g_faultClearStreakS < 65000) g_faultClearStreakS++;
    } else {
      g_faultClearStreakS = 0;
    }

    if (g_faultClearStreakS >= g_set.mainsReturnConfirmS) {
      autoSetState(AutoState::IDLE, "FAULT auto-reset (şebeke normal)");
      safeSetFuel(false);
      return;
    }

    if (!isGenRunningNow() && mainsIsBadForAuto() && !autoStartBlockedByBatt()) {
      if (g_faultRetryCount < g_set.faultMaxRetries && g_faultNextRetryS > 0) {
        if (g_meas.uptimeS >= g_faultNextRetryS) {
          g_faultRetryCount++;
          g_faultNextRetryS = 0;
          notify("🧯 FAULT: Auto-retry #" + String(g_faultRetryCount) + " başlıyor");
          autoSetState(AutoState::STARTING, "FAULT retry");
          startSeqBegin(false);
          return;
        }
      }
    }
    return;
  }

  switch (g_autoState) {
    case AutoState::IDLE: {
      g_failStreakS = 0;
      g_returnStreakS = 0;
      g_coolCounterS = 0;

      if (!isGenRunningNow() && mainsIsBadForAuto()) {
        g_failStreakS = 1;
        autoSetState(AutoState::WAIT_FAIL_CONFIRM, "Şebeke düşük, doğrulama başladı");
      }
      break;
    }

    case AutoState::WAIT_FAIL_CONFIRM: {
      if (isGenRunningNow()) { autoSetState(AutoState::RUNNING, "Jeneratör zaten çalışıyor"); break; }
      if (autoStartBlockedByBatt()) { enterFaultWithSchedule("Akü KRİTİK, auto-start bloklandı"); break; }

      if (mainsIsBadForAuto()) {
        if (g_failStreakS < 65000) g_failStreakS++;
      } else {
        autoSetState(AutoState::IDLE, "Şebeke normale döndü (iptal)");
        break;
      }

      if (g_failStreakS >= g_set.mainsFailConfirmS) {
        autoSetState(AutoState::STARTING, "Oto start");
        startSeqBegin(false);
      }
      break;
    }

    case AutoState::STARTING: {
      if (!g_startSeq.active) {
        if (isGenRunningNow()) autoSetState(AutoState::RUNNING, "Start başarılı");
        else enterFaultWithSchedule("Start başarısız");
      }
      break;
    }

    case AutoState::RUNNING: {
      if (mainsIsGoodToStop()) {
        g_returnStreakS = 1;
        autoSetState(AutoState::WAIT_RETURN_CONFIRM, "Şebeke normal, stop doğrulama");
      }
      break;
    }

    case AutoState::WAIT_RETURN_CONFIRM: {
      if (!isGenRunningNow()) { autoSetState(AutoState::IDLE, "Jeneratör durmuş"); break; }

      if (mainsIsGoodToStop()) {
        if (g_returnStreakS < 65000) g_returnStreakS++;
      } else {
        autoSetState(AutoState::RUNNING, "Şebeke tekrar bozuldu (iptal)");
        break;
      }

      if (g_returnStreakS >= g_set.mainsReturnConfirmS) {
        g_coolCounterS = 0;
        autoSetState(AutoState::COOLDOWN, "Cooldown başlıyor");
      }
      break;
    }

    case AutoState::COOLDOWN: {
      if (!isGenRunningNow()) { autoSetState(AutoState::IDLE, "Cooldown bitmeden durmuş"); break; }

      if (mainsIsBadForAuto()) {
        g_coolCounterS = 0;
        autoSetState(AutoState::RUNNING, "Şebeke tekrar bozuldu (cooldown iptal)");
        break;
      }

      if (g_coolCounterS < 65000) g_coolCounterS++;
      if (g_coolCounterS >= g_set.cooldownS) {
        g_coolCounterS = 0;
        autoSetState(AutoState::STOPPING, "Stop veriliyor");
        stopSeqBegin(false);
      }
      break;
    }

    case AutoState::STOPPING: {
      if (!g_stopSeq.active) {
        if (!isGenRunningNow()) autoSetState(AutoState::IDLE, "Stop başarılı");
        else enterFaultWithSchedule("Stop başarısız (lockout)");
      }
      break;
    }

    default:
      break;
  }
}

// =====================
// Measure
// =====================
static void readAllMeasurements() {
  g_meas.wifiRssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -999;
  g_meas.uptimeS  = millis() / 1000;

  g_meas.mainsV_raw = readAcRmsApprox(PIN_ADC_MAINS, g_set.calMains);
  g_meas.mainsV     = lpf(g_meas.mainsV, g_meas.mainsV_raw, LPF_ALPHA_AC);

  g_meas.genV_raw = readAcRmsApprox(PIN_ADC_GEN, g_set.calGen);
  g_meas.genV     = lpf(g_meas.genV, g_meas.genV_raw, LPF_ALPHA_AC);

  float vGenAdc = readAdcVoltage(PIN_ADC_GEN_BATT);
  float vCamAdc = readAdcVoltage(PIN_ADC_CAM_BATT);

  g_meas.genBattV_raw = vGenAdc * g_set.genBattDiv;
  g_meas.camBattV_raw = vCamAdc * g_set.camBattDiv;

  g_meas.genBattV = lpf(g_meas.genBattV, g_meas.genBattV_raw, LPF_ALPHA_BATT);
  g_meas.camBattV = lpf(g_meas.camBattV, g_meas.camBattV_raw, LPF_ALPHA_BATT);
}

// =====================
// UI Button engine
// =====================
static UiBtn ui_bLeft, ui_bRight, ui_bUp, ui_bDown, ui_bBack, ui_bOk;

static void uiBtnInit(UiBtn& b, uint8_t pin) {
  b.pin = pin;
  pinMode(pin, INPUT_PULLUP);
  b.lastRaw = digitalRead(pin);
  b.stable  = b.lastRaw;
  b.lastChangeMs = millis();
  b.pressedEvt = b.repeatEvt = false;
  b.downSinceMs = 0;
  b.nextRepeatMs = 0;
}

static void uiBtnScanOne(UiBtn& b) {
  b.pressedEvt = false;
  b.repeatEvt  = false;

  bool raw = digitalRead(b.pin);
  uint32_t now = millis();

  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.lastChangeMs = now;
  }

  if ((now - b.lastChangeMs) >= UI_BTN_DEBOUNCE_MS) {
    if (b.stable != raw) {
      b.stable = raw;

      // pressed (active low)
      if (b.stable == false) {
        b.pressedEvt = true;
        b.downSinceMs = now;
        b.nextRepeatMs = now + UI_BTN_REPEAT_DELAY_MS;
      } else {
        b.downSinceMs = 0;
        b.nextRepeatMs = 0;
      }
    }
  }

  // repeat
  if (b.stable == false && b.nextRepeatMs && (int32_t)(now - b.nextRepeatMs) >= 0) {
    b.repeatEvt = true;
    b.nextRepeatMs = now + UI_BTN_REPEAT_RATE_MS;
  }
}

static inline bool uiBtnPressed(const UiBtn& b) { return b.pressedEvt; }
static inline bool uiBtnRepeat(const UiBtn& b)  { return b.repeatEvt; }
static inline uint32_t uiBtnHeldMs(const UiBtn& b) {
  if (b.stable == false && b.downSinceMs) return millis() - b.downSinceMs;
  return 0;
}

static void uiScanButtons() {
  uiBtnScanOne(ui_bLeft);
  uiBtnScanOne(ui_bRight);
  uiBtnScanOne(ui_bUp);
  uiBtnScanOne(ui_bDown);
  uiBtnScanOne(ui_bBack);
  uiBtnScanOne(ui_bOk);

  if (UI_SWAP_LEFT_RIGHT) {
    UiBtn tmp = ui_bLeft; ui_bLeft = ui_bRight; ui_bRight = tmp;
  }
  if (UI_SWAP_UP_DOWN) {
    UiBtn tmp = ui_bUp; ui_bUp = ui_bDown; ui_bDown = tmp;
  }
}

// =====================
// UI Params
// =====================
static UiParamRef uiParams[] = {
  // MAINS
  {UiCategory::MAINS, "MAINS_HIGH_V",     UiParamType::F32, &g_set.mainsHigh,    150, 270, 1.0f, "mHi"},
  {UiCategory::MAINS, "MAINS_NORMAL_MIN", UiParamType::F32, &g_set.mainsNormMin,  80, 260, 1.0f, "mNmn"},
  {UiCategory::MAINS, "MAINS_NORMAL_MAX", UiParamType::F32, &g_set.mainsNormMax,  80, 270, 1.0f, "mNmx"},
  {UiCategory::MAINS, "MAINS_LOW_V",      UiParamType::F32, &g_set.mainsLow,      50, 250, 1.0f, "mLo"},
  {UiCategory::MAINS, "MAINS_CRIT_V",     UiParamType::F32, &g_set.mainsCrit,     20, 240, 1.0f, "mCr"},

  // GEN
  {UiCategory::GEN, "GEN_OFF_V",        UiParamType::F32, &g_set.genOff,     0, 120, 1.0f, "gOff"},
  {UiCategory::GEN, "GEN_LOW_V",        UiParamType::F32, &g_set.genLow,    50, 230, 1.0f, "gLo"},
  {UiCategory::GEN, "GEN_NORMAL_MIN",   UiParamType::F32, &g_set.genNormMin,80, 260, 1.0f, "gNmn"},
  {UiCategory::GEN, "GEN_NORMAL_MAX",   UiParamType::F32, &g_set.genNormMax,80, 270, 1.0f, "gNmx"},
  {UiCategory::GEN, "GEN_RUNNING_V",    UiParamType::F32, &g_set.genRunningV,50, 260, 1.0f, "gRunV"},
  {UiCategory::GEN, "GEN_RUN_CONFIRM_S",UiParamType::U16, &g_set.genRunConfirmS,1, 120, 1.0f, "gRunC"},

  // BATT
  {UiCategory::BATT, "BATT_HIGH_V",     UiParamType::F32, &g_set.battHigh,    11.5f, 15.0f, 0.05f, "bHi"},
  {UiCategory::BATT, "BATT_NORMAL_MIN", UiParamType::F32, &g_set.battNormMin, 11.0f, 14.0f, 0.05f, "bNmn"},
  {UiCategory::BATT, "BATT_LOW_V",      UiParamType::F32, &g_set.battLow,     10.0f, 14.0f, 0.05f, "bLo"},
  {UiCategory::BATT, "BATT_CRIT_V",     UiParamType::F32, &g_set.battCrit,     9.0f, 13.5f, 0.05f, "bCr"},
  {UiCategory::BATT, "HYST_V_BATT",     UiParamType::F32, &g_set.hystBatt,     0.0f,  1.0f,  0.01f, "hBt"},

  // AUTO
  {UiCategory::AUTO, "AUTO_START_MAINS_V",     UiParamType::F32, &g_set.autoStartMainsV, 50, 240, 1.0f, "aMnsV"},
  {UiCategory::AUTO, "MAINS_FAIL_CONFIRM_S",   UiParamType::U16, &g_set.mainsFailConfirmS, 1, 600, 1.0f, "aFail"},
  {UiCategory::AUTO, "MAINS_RETURN_CONFIRM_S", UiParamType::U16, &g_set.mainsReturnConfirmS, 1, 600, 1.0f, "aRet"},
  {UiCategory::AUTO, "COOLDOWN_S",             UiParamType::U16, &g_set.cooldownS, 5, 3600, 1.0f, "aCool"},

  // TIMING
  {UiCategory::TIMING, "FUEL_PRIME_MS",        UiParamType::U16, &g_set.fuelPrimeMs, 0, 10000, 50.0f, "pF"},
  {UiCategory::TIMING, "START_PULSE_MS",       UiParamType::U16, &g_set.startPulseMs, 100, 8000, 50.0f, "pSt"},
  {UiCategory::TIMING, "START_RETRY_GAP_MS",   UiParamType::U32, &g_set.startRetryGapMs, 0, 30000, 50.0f, "gSt"},
  {UiCategory::TIMING, "START_SENSE_GRACE_MS", UiParamType::U16, &g_set.startSenseGraceMs, 0, 30000, 50.0f, "sSt"},
  {UiCategory::TIMING, "START_MAX_ATTEMPTS",   UiParamType::U8,  &g_set.startMaxAttempts, 1, 10, 1.0f, "aAtt"},
  {UiCategory::TIMING, "STOP_PULSE_MS",        UiParamType::U16, &g_set.stopPulseMs, 100, 8000, 50.0f, "pSp"},
  {UiCategory::TIMING, "STOP_VERIFY_S",        UiParamType::U16, &g_set.stopVerifyS, 1, 120, 1.0f, "vSp"},
  {UiCategory::TIMING, "STOP_MAX_ATTEMPTS",    UiParamType::U8,  &g_set.stopMaxAttempts, 1, 10, 1.0f, "mSp"},
  {UiCategory::TIMING, "FUEL_OFF_DELAY_MS",    UiParamType::U16, &g_set.fuelOffDelayMs, 0, 10000, 50.0f, "dF"},
  {UiCategory::TIMING, "HOURS_SAVE_PERIOD_S",  UiParamType::U32, &g_set.hoursSavePeriodS, 10, 3600, 10.0f, "hSaveP"},

  // FAULT
  {UiCategory::FAULT, "FAULT_MAX_RETRIES",   UiParamType::U8,  &g_set.faultMaxRetries, 0, 10, 1.0f, "fMax"},
  {UiCategory::FAULT, "FAULT_RETRY_BASE_S",  UiParamType::U16, &g_set.faultRetryBaseS, 10, 3600, 10.0f, "fBase"},
  {UiCategory::FAULT, "FAULT_RETRY_MAX_S",   UiParamType::U16, &g_set.faultRetryMaxS,  10, 7200, 10.0f, "fCap"},

  // CAL
  {UiCategory::CAL, "CAL_MAINS",    UiParamType::F32, &g_set.calMains,   10, 600, 1.0f, "calMains"},
  {UiCategory::CAL, "CAL_GEN",      UiParamType::F32, &g_set.calGen,     10, 600, 1.0f, "calGen"},
  {UiCategory::CAL, "GEN_DIV",      UiParamType::F32, &g_set.genBattDiv, 1,  10,  0.01f, "genDiv"},
  {UiCategory::CAL, "CAM_DIV",      UiParamType::F32, &g_set.camBattDiv, 1,  10,  0.01f, "camDiv"},
};

static const char* uiCatName(UiCategory c) {
  switch (c) {
    case UiCategory::MAINS:  return "Mains";
    case UiCategory::GEN:    return "Gen";
    case UiCategory::BATT:   return "Batt";
    case UiCategory::AUTO:   return "Auto";
    case UiCategory::TIMING: return "Timing";
    case UiCategory::FAULT:  return "Fault";
    case UiCategory::CAL:    return "Cal";
    default:                 return "?";
  }
}

static UiCatList uiGetCatList(UiCategory cat) {
  UiParamRef* first = nullptr;
  uint8_t count = 0;

  for (uint16_t i = 0; i < (sizeof(uiParams) / sizeof(uiParams[0])); i++) {
    if (uiParams[i].cat == cat) {
      if (!first) first = &uiParams[i];
      count++;
    }
  }
  UiCatList cl; cl.arr = first; cl.count = count;
  return cl;
}

static UiParamRef* uiCurrentParamBy(UiCategory cat, uint8_t idxInCat) {
  uint8_t seen = 0;
  for (uint16_t i = 0; i < (sizeof(uiParams) / sizeof(uiParams[0])); i++) {
    if (uiParams[i].cat == cat) {
      if (seen == idxInCat) return &uiParams[i];
      seen++;
    }
  }
  return nullptr;
}

static float uiParamGetF(const UiParamRef& p) {
  switch (p.type) {
    case UiParamType::F32: return *(float*)p.ptr;
    case UiParamType::U16: return (float)(*(uint16_t*)p.ptr);
    case UiParamType::U32: return (float)(*(uint32_t*)p.ptr);
    case UiParamType::U8:  return (float)(*(uint8_t*)p.ptr);
    default: return 0;
  }
}

static void uiParamSetF(const UiParamRef& p, float v) {
  v = constrain(v, p.minF, p.maxF);
  switch (p.type) {
    case UiParamType::F32: *(float*)p.ptr = v; break;
    case UiParamType::U16: *(uint16_t*)p.ptr = (uint16_t)lroundf(v); break;
    case UiParamType::U32: *(uint32_t*)p.ptr = (uint32_t)lroundf(v); break;
    case UiParamType::U8:  *(uint8_t*)p.ptr  = (uint8_t)lroundf(v); break;
    default: break;
  }
}

static String uiParamValueText(const UiParamRef& p) {
  float v = uiParamGetF(p);
  if (p.type == UiParamType::F32) return fmt2(v);
  return String((uint32_t)lroundf(v));
}

static float uiStepAccel(const UiParamRef& p, uint32_t heldMs) {
  float s = p.stepF;
  if (!UI_ENABLE_ACCEL) return s;

  float mult = 1.0f;
  if (heldMs >= UI_ACCEL_3_MS) mult = UI_ACCEL_3_MULT;
  else if (heldMs >= UI_ACCEL_2_MS) mult = UI_ACCEL_2_MULT;
  else if (heldMs >= UI_ACCEL_1_MS) mult = UI_ACCEL_1_MULT;

  return s * mult;
}

// =====================
// UI State
// =====================
static UiScreen ui_screen = UiScreen::STATUS;
static uint8_t ui_statusPage = 0;

static uint8_t ui_menuIndex = 0;
static const uint8_t UI_MENU_COUNT = 5;

static UiCategory ui_cat = UiCategory::MAINS;
static uint8_t ui_paramIndex = 0;

static UiParamRef* ui_editParam = nullptr;
static float ui_editOrig = 0;
static float ui_editDraft = 0;
static bool  ui_editDirty = false;

static String ui_confirmText;

// =====================
// TFT drawing helpers
// =====================
#if USE_TFT
static void tftHeader(const String& title, const String& right="") {
  tft.fillRect(0, 0, 240, 24, UI_COLOR_HEADER_BG);
  tft.setTextColor(UI_COLOR_HEADER_FG);
  tft.setTextSize(2);
  tft.setCursor(4, 4);
  tft.print(title);
  if (right.length()) {
    int16_t x = 240 - (right.length() * 12) - 4;
    if (x < 120) x = 120;
    tft.setCursor(x, 4);
    tft.print(right);
  }
}

static void tftLine(int y, const String& s, uint16_t col=UI_COLOR_TEXT, uint8_t size=2) {
  tft.setTextSize(size);
  tft.setTextColor(col);
  tft.setCursor(4, y);
  tft.print(s);
}

static void tftClearBody() {
  tft.fillRect(0, 24, 240, 296, UI_COLOR_BG);
}
#endif

// =====================
// UI screens
// =====================
static void uiDrawStatus(bool force) {
#if USE_TFT
  (void)force;
  tft.fillScreen(UI_COLOR_BG);
  tftHeader(String(DEVICE_NAME), String(PROJECT_VERSION));

  tftLine(30, "Mod: " + String(modeText(g_mode)));
  tftLine(50, "Auto: " + String(autoStateText(g_autoState)));
  tftLine(70, "Fuel: " + String(g_fuelOn ? "ON" : "OFF"));

  if (ui_statusPage == 0) {
    tftLine(100, "Sayfa: MAINS");
    tftLine(120, "V=" + fmt2(g_meas.mainsV) + "  (" + String((int)g_mainsState) + ")");
    tftLine(140, mainsStateLine(g_mainsState, g_meas.mainsV));
  } else if (ui_statusPage == 1) {
    tftLine(100, "Sayfa: GEN");
    tftLine(120, "V=" + fmt2(g_meas.genV) + "  (" + String((int)g_genState) + ")");
    tftLine(140, genStateLine(g_genState, g_meas.genV));
    tftLine(170, "RunNow=" + String(isGenRunningNow() ? "YES" : "NO"));
    tftLine(190, "Total=" + fmtHMS(g_genRunTotalS), UI_COLOR_TEXT_DIM, 2);
  } else if (ui_statusPage == 2) {
    tftLine(100, "Sayfa: BATT");
    tftLine(120, "Gen=" + fmt2(g_meas.genBattV) + "V " + battStateToText(g_genBattState));
    tftLine(140, "Cam=" + fmt2(g_meas.camBattV) + "V " + battStateToText(g_camBattState));
  } else {
    tftLine(100, "Sayfa: NET");
    tftLine(120, "WiFi RSSI: " + String(g_meas.wifiRssi));
    tftLine(140, "Uptime: " + String(g_meas.uptimeS) + "s");
  }

  tftLine(280, "L/R: Sayfa  OK: Menu", UI_COLOR_TEXT_DIM, 1);
#endif
}

static void uiDrawMenu(bool force) {
#if USE_TFT
  (void)force;
  tft.fillScreen(UI_COLOR_BG);
  tftHeader("MENU", "");

  const char* items[UI_MENU_COUNT] = {
    "Ayarlar",
    "AUTO <-> MANUAL",
    "MANUAL START",
    "MANUAL STOP",
    "Reset Hours"
  };

  for (uint8_t i = 0; i < UI_MENU_COUNT; i++) {
    int y = 40 + (i * 34);
    bool sel = (i == ui_menuIndex);
    if (sel) tft.fillRect(0, y-2, 240, 28, UI_COLOR_SEL_BG);
    tft.setTextSize(2);
    tft.setCursor(8, y);
    tft.setTextColor(sel ? UI_COLOR_SEL_FG : UI_COLOR_TEXT);
    tft.print(items[i]);
  }

  tftLine(280, "UP/DN: Sec  OK: Gir  BACK: Cik", UI_COLOR_TEXT_DIM, 1);
#endif
}

static void uiDrawSettings(bool force) {
#if USE_TFT
  (void)force;
  UiCatList cl = uiGetCatList(ui_cat);

  tft.fillScreen(UI_COLOR_BG);
  String right = String(uiCatName(ui_cat)) + " " + String(ui_paramIndex + 1) + "/" + String(cl.count);
  tftHeader("AYARLAR", right);

  UiParamRef* p = uiCurrentParamBy(ui_cat, ui_paramIndex);
  if (!p) { tftLine(60, "Param yok"); return; }

  tftLine(40, "Kategori: " + String(uiCatName(ui_cat)));
  tftLine(70, String("Param: ") + p->name, UI_COLOR_TEXT, 2);
  tftLine(100, String("Deger: ") + uiParamValueText(*p));
  tftLine(130, "OK: Duzenle");

  tftLine(280, "L/R: Kategori  U/D: Param  BACK:Cik", UI_COLOR_TEXT_DIM, 1);
#endif
}

static void uiDrawEdit(bool force) {
#if USE_TFT
  (void)force;
  tft.fillScreen(UI_COLOR_BG);
  tftHeader("DUZENLE", "");

  if (!ui_editParam) { tftLine(60, "Param yok"); return; }

  tftLine(40, String("Param: ") + ui_editParam->name);
  tftLine(70, String("Deger: ") + fmt2(ui_editDraft));
  tftLine(100, ui_editDirty ? "Durum: Kaydedilmedi" : "Durum: Degismedi", ui_editDirty ? UI_COLOR_WARN : UI_COLOR_TEXT_DIM, 2);

  tftLine(140, "UP: +   DOWN: -");
  tftLine(170, "OK: Kaydet?  BACK: Iptal");
  tftLine(280, "Hold: hizli artir/azalt", UI_COLOR_TEXT_DIM, 1);
#endif
}

static void uiDrawConfirm(bool force) {
#if USE_TFT
  (void)force;
  tft.fillScreen(UI_COLOR_BG);
  tftHeader("KAYDET?", "");

  tftLine(60, ui_confirmText);
  tftLine(120, "OK: Evet (NVS)");
  tftLine(150, "BACK: Hayir");
#endif
}

static void uiEnter(UiScreen s) {
  ui_screen = s;
  switch (ui_screen) {
    case UiScreen::STATUS:   uiDrawStatus(true); break;
    case UiScreen::MENU:     uiDrawMenu(true); break;
    case UiScreen::SETTINGS: uiDrawSettings(true); break;
    case UiScreen::EDIT:     uiDrawEdit(true); break;
    case UiScreen::CONFIRM:  uiDrawConfirm(true); break;
  }
}

// =====================
// UI Actions
// =====================
static void uiEnterEdit() {
  UiParamRef* p = uiCurrentParamBy(ui_cat, ui_paramIndex);
  if (!p) return;

  ui_editParam = p;
  ui_editOrig = uiParamGetF(*p);
  ui_editDraft = ui_editOrig;
  ui_editDirty = false;
  uiEnter(UiScreen::EDIT);
}

static void uiApplyDraftToLive() {
  if (!ui_editParam) return;
  uiParamSetF(*ui_editParam, ui_editDraft);
}

static void uiCancelEdit() {
  if (!ui_editParam) return;
  uiParamSetF(*ui_editParam, ui_editOrig);
  ui_editParam = nullptr;
  ui_editDirty = false;
  uiEnter(UiScreen::SETTINGS);
}

static void uiRequestSaveConfirm() {
  if (!ui_editParam) return;

  // draft'ı live'a yaz
  uiApplyDraftToLive();

  ui_confirmText = String("⚙️ ") + ui_editParam->name + " = " + uiParamValueText(*ui_editParam);
  uiEnter(UiScreen::CONFIRM);
}

static void uiDoSave() {
  // NVS save + telegram notify
  saveSettings();
  saveHoursTotal();

  if (ui_editParam) {
    notify(String("⚙️ TFT Ayar değişti: ") + ui_editParam->name + " = " + uiParamValueText(*ui_editParam));
  } else {
    notify("⚙️ TFT Ayar kaydedildi.");
  }

  ui_editParam = nullptr;
  ui_editDirty = false;
  uiEnter(UiScreen::SETTINGS);
}

// =====================
// Telegram - minimal (senin v9.010 komutların aynı mantık)
// =====================
static String normalizeCommand(const String& textRaw) {
  String t = textRaw;
  t.trim();
  int sp = t.indexOf(' ');
  if (sp >= 0) t = t.substring(0, sp);
  int at = t.indexOf('@');
  if (at >= 0) t = t.substring(0, at);
  t.toLowerCase();
  return t;
}

static String getArg1(const String& textRaw) {
  String t = textRaw;
  t.trim();
  int sp = t.indexOf(' ');
  if (sp < 0) return "";
  String a = t.substring(sp + 1);
  a.trim();
  return a;
}

static bool isAuthorized(const telegramMessage& msg) {
  if (msg.chat_id == String(CHAT_ID)) return true;
  long fromId = msg.from_id.toInt();
  return (fromId == MASTER_ADMIN_ID);
}

static String buildStatusText() {
  String s;
  s += String(DEVICE_NAME) + "\n";
  s += "🔖 Sürüm: " + String(PROJECT_VERSION) + "\n";
  s += "🎛 Mod: " + String(modeText(g_mode)) + "\n";
  s += "🤖 AutoState: " + String(autoStateText(g_autoState)) + "\n";
  s += mainsStateLine(g_mainsState, g_meas.mainsV) + "\n";
  s += genStateLine(g_genState, g_meas.genV) + "\n";
  s += "🔋 Gen Akü: " + fmt2(g_meas.genBattV) + "V (" + battStateToText(g_genBattState) + ")\n";
  s += "🔋 Cam Akü: " + fmt2(g_meas.camBattV) + "V (" + battStateToText(g_camBattState) + ")\n";
  s += "⏱ Çalışma Süresi: " + fmtHMS(g_genRunTotalS) + "\n";
  s += "⛽ Fuel=" + String(g_fuelOn ? "ON" : "OFF") + "\n";
  return s;
}

static void handleManualStart(const String& chatId) {
  if (g_mode == MODE_AUTO) { bot.sendMessage(chatId, "⚠️ Şu an AUTO modda. Önce /manual yap.", ""); return; }
  if (isGenRunningNow())   { bot.sendMessage(chatId, "✅ Jeneratör zaten çalışıyor. Start yapılmadı.", ""); return; }
  if (g_startSeq.active || g_stopSeq.active) { bot.sendMessage(chatId, "⏳ Başka bir işlem aktif (start/stop).", ""); return; }

  startSeqBegin(true);
  bot.sendMessage(chatId, "🟡 MANUAL: Start sekansı başladı.", "");
}

static void handleManualStop(const String& chatId) {
  if (g_mode == MODE_AUTO) { bot.sendMessage(chatId, "⚠️ Şu an AUTO modda. Önce /manual yap.", ""); return; }
  if (g_startSeq.active || g_stopSeq.active) { bot.sendMessage(chatId, "⏳ Başka bir işlem aktif (start/stop).", ""); return; }

  if (!isGenRunningNow()) {
    if (g_fuelOn) {
      safeSetFuel(false);
      bot.sendMessage(chatId, "✅ Jeneratör zaten durmuş. ⛽ Fuel OFF yapıldı. Stop yapılmadı.", "");
    } else {
      bot.sendMessage(chatId, "✅ Jeneratör zaten durmuş. Stop yapılmadı.", "");
    }
    return;
  }

  stopSeqBegin(true);
  bot.sendMessage(chatId, "🟥 MANUAL: Stop sekansı başladı.", "");
}

static void handleTelegram() {
  int numNew = bot.getUpdates(bot.last_message_received + 1);
  if (numNew <= 0) return;

  while (numNew > 0) {
    for (int i = 0; i < numNew; i++) {
      telegramMessage& msg = bot.messages[i];
      if (!isAuthorized(msg)) continue;

      String cmd = normalizeCommand(msg.text);
      String arg1 = getArg1(msg.text);

      if (cmd == "/durum" || cmd == "/status") {
        bot.sendMessage(msg.chat_id, buildStatusText(), "");

      } else if (cmd == "/save") {
        saveSettings();
        saveHoursTotal();
        bot.sendMessage(msg.chat_id, "✅ Ayarlar + sayaç NVS'ye kaydedildi.", "");

      } else if (cmd == "/reset_hours") {
        g_genRunTotalS = 0;
        saveHoursTotal();
        bot.sendMessage(msg.chat_id, "✅ Çalışma saati sıfırlandı.", "");

      } else if (cmd == "/ping") {
        bot.sendMessage(msg.chat_id, "pong ✅", "");

      } else if (cmd == "/auto") {
        g_mode = MODE_AUTO;
        saveSettings();
        bot.sendMessage(msg.chat_id, "✅ Mod: AUTO", "");

      } else if (cmd == "/manual") {
        g_mode = MODE_MANUAL;
        saveSettings();
        bot.sendMessage(msg.chat_id, "✅ Mod: MANUAL", "");

      } else if (cmd == "/start") {
        handleManualStart(msg.chat_id);

      } else if (cmd == "/stop") {
        handleManualStop(msg.chat_id);

      } else if (cmd == "/cooldown") {
        int v = arg1.toInt();
        if (v <= 0) bot.sendMessage(msg.chat_id, "Kullanım: /cooldown 120 (sn)", "");
        else {
          g_set.cooldownS = (uint16_t)constrain(v, 5, 3600);
          bot.sendMessage(msg.chat_id, "✅ Cooldown=" + String(g_set.cooldownS) + "s (Kaydetmek için /save)", "");
        }

      } else if (cmd == "/autostart") {
        float v = arg1.toFloat();
        if (v <= 0) bot.sendMessage(msg.chat_id, "Kullanım: /autostart 160 (V)", "");
        else {
          g_set.autoStartMainsV = v;
          bot.sendMessage(msg.chat_id, "✅ AutoStart=" + fmt2(g_set.autoStartMainsV) + "V (Kaydetmek için /save)", "");
        }

      } else if (cmd == "/help" || cmd == "/yardim" || cmd == "/yardım") {
        String h;
        h += "Komutlar:\n";
        h += "/durum\n";
        h += "/auto /manual\n";
        h += "/start /stop (sadece MANUAL)\n";
        h += "/cooldown <sn>\n";
        h += "/autostart <V>\n";
        h += "/save /reset_hours /ping\n";
        bot.sendMessage(msg.chat_id, h, "");

      } else {
        bot.sendMessage(msg.chat_id,
          "Komut: /durum /auto /manual /start /stop /cooldown /autostart /save /reset_hours /ping",
          "");
      }
    }

    numNew = bot.getUpdates(bot.last_message_received + 1);
    if (numNew <= 0) break;
  }
}

// =====================
// UI service
// =====================
static void uiService() {
#if USE_TFT
  if (!tftReady) return;

  if (millis() - tUi < UI_TICK_MS) return;
  tUi = millis();

  uiScanButtons();

  if (ui_screen == UiScreen::STATUS) {
    if (uiBtnPressed(ui_bLeft))  { ui_statusPage = (ui_statusPage + 3) % 4; uiDrawStatus(true); }
    if (uiBtnPressed(ui_bRight)) { ui_statusPage = (ui_statusPage + 1) % 4; uiDrawStatus(true); }
    if (uiBtnPressed(ui_bOk))    { uiEnter(UiScreen::MENU); return; }
  }
  else if (ui_screen == UiScreen::MENU) {
    if (uiBtnPressed(ui_bUp))   { if (ui_menuIndex > 0) ui_menuIndex--; uiDrawMenu(true); }
    if (uiBtnPressed(ui_bDown)) { if (ui_menuIndex + 1 < UI_MENU_COUNT) ui_menuIndex++; uiDrawMenu(true); }
    if (uiBtnPressed(ui_bBack)) { uiEnter(UiScreen::STATUS); return; }

    if (uiBtnPressed(ui_bOk)) {
      switch (ui_menuIndex) {
        case 0: uiEnter(UiScreen::SETTINGS); break;
        case 1:
          g_mode = (g_mode == MODE_AUTO) ? MODE_MANUAL : MODE_AUTO;
          saveSettings();
          notify(String("🎛 Mod değişti: ") + modeText(g_mode));
          uiDrawMenu(true);
          break;
        case 2:
          if (g_mode == MODE_MANUAL) { startSeqBegin(true); notify("🟡 TFT: MANUAL Start başladı"); }
          else notify("⚠️ TFT: AUTO modda, start yok");
          uiDrawMenu(true);
          break;
        case 3:
          if (g_mode == MODE_MANUAL) { stopSeqBegin(true); notify("🟥 TFT: MANUAL Stop başladı"); }
          else notify("⚠️ TFT: AUTO modda, stop yok");
          uiDrawMenu(true);
          break;
        case 4:
          g_genRunTotalS = 0;
          saveHoursTotal();
          notify("⏱ TFT: Saat sıfırlandı");
          uiDrawMenu(true);
          break;
      }
    }
  }
  else if (ui_screen == UiScreen::SETTINGS) {
    UiCatList cl = uiGetCatList(ui_cat);

    if (uiBtnPressed(ui_bLeft))  { ui_cat = (UiCategory)((((uint8_t)ui_cat) + (uint8_t)UiCategory::COUNT - 1) % (uint8_t)UiCategory::COUNT); ui_paramIndex = 0; uiDrawSettings(true); }
    if (uiBtnPressed(ui_bRight)) { ui_cat = (UiCategory)((((uint8_t)ui_cat) + 1) % (uint8_t)UiCategory::COUNT); ui_paramIndex = 0; uiDrawSettings(true); }

    if (uiBtnPressed(ui_bUp))    { if (ui_paramIndex > 0) ui_paramIndex--; uiDrawSettings(true); }
    if (uiBtnPressed(ui_bDown))  { if (ui_paramIndex + 1 < cl.count) ui_paramIndex++; uiDrawSettings(true); }

    if (uiBtnPressed(ui_bBack))  { uiEnter(UiScreen::MENU); return; }
    if (uiBtnPressed(ui_bOk))    { uiEnterEdit(); return; }
  }
  else if (ui_screen == UiScreen::EDIT) {
    if (!ui_editParam) { uiEnter(UiScreen::SETTINGS); return; }

    bool inc = uiBtnPressed(ui_bUp)   || uiBtnRepeat(ui_bUp);
    bool dec = uiBtnPressed(ui_bDown) || uiBtnRepeat(ui_bDown);

    if (inc || dec) {
      uint32_t held = inc ? uiBtnHeldMs(ui_bUp) : uiBtnHeldMs(ui_bDown);
      float step = uiStepAccel(*ui_editParam, held);
      if (dec) step = -step;

      ui_editDraft = constrain(ui_editDraft + step, ui_editParam->minF, ui_editParam->maxF);
      ui_editDirty = (fabsf(ui_editDraft - ui_editOrig) > 1e-6f);
      uiDrawEdit(true);
    }

    if (uiBtnPressed(ui_bBack)) {
      uiCancelEdit();
      return;
    }

    if (uiBtnPressed(ui_bOk)) {
      if (!ui_editDirty) { // değişmemişse geri
        uiCancelEdit();
      } else {
        uiRequestSaveConfirm();
      }
      return;
    }
  }
  else if (ui_screen == UiScreen::CONFIRM) {
    if (uiBtnPressed(ui_bBack)) {
      // revert
      uiParamSetF(*ui_editParam, ui_editOrig);
      ui_editParam = nullptr;
      ui_editDirty = false;
      uiEnter(UiScreen::SETTINGS);
      return;
    }
    if (uiBtnPressed(ui_bOk)) {
      uiDoSave();
      return;
    }
  }
#endif
}

// =====================
// Setup / Loop
// =====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  analogReadResolution(12);

  loadSettings();
  relayInit();

#if USE_TFT
  tftSPI.begin(TFT_SCK, (TFT_MISO == 255 ? -1 : TFT_MISO), TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(UI_COLOR_BG);
  tftReady = true;
#endif

  // Buttons
  uiBtnInit(ui_bLeft,  PIN_BTN_LEFT);
  uiBtnInit(ui_bRight, PIN_BTN_RIGHT);
  uiBtnInit(ui_bUp,    PIN_BTN_UP);
  uiBtnInit(ui_bDown,  PIN_BTN_DOWN);
  uiBtnInit(ui_bBack,  PIN_BTN_BACK);
  uiBtnInit(ui_bOk,    PIN_BTN_OK);

  connectWiFi();
  tgClient.setInsecure();

  readAllMeasurements();
  delay(250);
  readAllMeasurements();

  updateBatteryStatesOnly();
  g_mainsState = evalMains(g_meas.mainsV, MainsState::UNKNOWN);
  g_genState   = evalGen(g_meas.genV,   GenState::UNKNOWN);

  if (WiFi.status() == WL_CONNECTED) {
    notify(String(DEVICE_NAME) + "\n🔖 " + PROJECT_VERSION + "\nℹ️ Bot hazır. /help");
  }

  g_stateAlertsArmed = true;

  tMeasure = millis();
  tSerial  = millis();
  tTgPoll  = millis();
  tUi      = millis();
  g_lastHoursSaveS = 0;

  uiEnter(UiScreen::STATUS);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t tRetry = 0;
    if (millis() - tRetry > 5000) {
      tRetry = millis();
      connectWiFi();
    }
  }

  relayPulseService();
  startSeqService();
  stopSeqService();

  uint32_t now = millis();

  if (now - tMeasure >= MEASURE_MS) {
    tMeasure = now;

    readAllMeasurements();
    updateBatteryStatesOnly();
    handleStateAlerts();
    updateGenHoursCounter_1s();
    autoTick_1s();

    if (ui_screen == UiScreen::STATUS) uiDrawStatus(false);
  }

  if (now - tSerial >= SERIAL_REPORT_MS) {
    tSerial = now;
    Serial.print("["); Serial.print(PROJECT_VERSION); Serial.print("] ");
    Serial.print("Mode="); Serial.print(modeText(g_mode));
    Serial.print(" Auto="); Serial.print(autoStateText(g_autoState));
    Serial.print(" Fuel="); Serial.print(g_fuelOn ? "ON" : "OFF");
    Serial.print(" Mains="); Serial.print(fmt2(g_meas.mainsV));
    Serial.print(" Gen="); Serial.print(fmt2(g_meas.genV));
    Serial.print(" RunNow="); Serial.print(isGenRunningNow() ? "YES" : "NO");
    Serial.print(" Total="); Serial.print(fmtHMS(g_genRunTotalS));
    Serial.println();
  }

  if (now - tTgPoll >= TG_POLL_MS) {
    tTgPoll = now;
    if (WiFi.status() == WL_CONNECTED) handleTelegram();
  }

  uiService();

  delay(5);
  yield();
}
