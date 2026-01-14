// =====================
// SÜRÜM v9.002  (Aşama 9: Manuel/Oto + Fuel + Start/Stop + Cooldown + Güvenlik)
// =====================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Preferences.h>

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

  // Stage 5
  float genRunningV;
  uint16_t genRunConfirmS;
  uint32_t hoursSavePeriodS;

  // Stage 9 (AUTO + COOLDOWN + RELAY timing)
  float    autoStartMainsV;      // şebeke < bu değer => oto start (confirm sonrası)
  uint16_t mainsFailConfirmS;    // şebeke düşük doğrulama
  uint16_t mainsReturnConfirmS;  // şebeke normal doğrulama (stop'a geçmek için)
  uint16_t cooldownS;            // stop öncesi boşta çalışma

  uint16_t fuelPrimeMs;          // start öncesi yakıt prime
  uint16_t startPulseMs;         // marş pulse
  uint32_t startRetryGapMs;      // denemeler arası dinlenme
  uint16_t startSenseGraceMs;    // marş sonrası yükselmeyi bekleme
  uint8_t  startMaxAttempts;     // max marş denemesi

  uint16_t stopPulseMs;          // stop pulse
  uint16_t stopVerifyS;          // stop sonrası durdu mu doğrulama
  uint8_t  stopMaxAttempts;      // max stop denemesi
  uint16_t fuelOffDelayMs;       // stop'tan sonra yakıtı kesme gecikmesi
} g_set;

struct Measurements {
  float mainsV_raw, genV_raw, genBattV_raw, camBattV_raw;
  float mainsV, genV, genBattV, camBattV;
  int wifiRssi;
  uint32_t uptimeS;
} g_meas;

static uint32_t tMeasure = 0, tSerial = 0, tTgPoll = 0;

// Save butonu
static bool lastBtn = true;
static uint32_t btnDownMs = 0;

// ---------------------
// Battery State (NO TELEGRAM for battery)
// ---------------------
enum class BattState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
static BattState g_genBattState = BattState::UNKNOWN;
static BattState g_camBattState = BattState::UNKNOWN;

// ---------------------
// Stage 8 - MAINS / GEN state + Telegram alerts
// ---------------------
enum class MainsState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
enum class GenState   : uint8_t { UNKNOWN, OFF,      LOW_V, NORMAL, HIGH_V };

static MainsState g_mainsState = MainsState::UNKNOWN;
static GenState   g_genState   = GenState::UNKNOWN;

// Boot sonrası ilk ölçümlerde ekstra mesaj atmasın diye
static bool g_stateAlertsArmed = false;

// ---------------------
// Stage 5 - Hours Counter
// ---------------------
static bool     g_genRunning = false;
static uint16_t g_genRunStreakS = 0;
static uint64_t g_genRunTotalS  = 0;
static uint32_t g_lastHoursSaveS = 0;

// ---------------------
// Stage 9 - AUTO State Machine
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

// ---------------------
// Relay Control (Fuel + Start + Stop)
// ---------------------
static bool g_fuelOn = false;
static bool g_startOn = false;
static bool g_stopOn  = false;

// Pulse engine
static bool g_pulseActive = false;
static uint8_t g_pulsePin = 255;
static uint32_t g_pulseUntilMs = 0;

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

// START/STOP interlock (aynı anda aktif olmaz) + küçük safe gap
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
// Helpers
// =====================
static String fmt2(float v) {
  if (isnan(v) || isinf(v)) return "nan";
  char b[16];
  dtostrf(v, 0, 2, b);
  return String(b);
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

// "/durum@botadi arg" -> "/durum"
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

static void notifyTo(const String& chatId, const String& msg) {
  if (WiFi.status() == WL_CONNECTED) bot.sendMessage(chatId, msg, "");
}

static void notify(const String& msg) {
  notifyTo(String(CHAT_ID), msg);
}

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

  // Stage 5
  g_set.genRunningV      = prefs.getFloat("gRunV", GEN_RUNNING_V);
  g_set.genRunConfirmS   = prefs.getUShort("gRunC", GEN_RUNNING_CONFIRM_S);
  g_set.hoursSavePeriodS = prefs.getUInt("hSaveP", HOURS_SAVE_PERIOD_S);

  // Stage 9
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

  g_mode = (RunMode)prefs.getUChar("mode", (uint8_t)MODE_MANUAL);

  // total hours load (64-bit safe: two uint32)
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
// Battery State (only compute)
// =====================
static String battStateToText(BattState st) {
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
// Stage 8 - MAINS / GEN state eval + Telegram
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
// Stage 5 - Hours Counter (1Hz)
// =====================
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
// Stage 9 - Start/Stop Sequences (non-blocking)
// =====================
enum class StartSub : uint8_t { IDLE, PRIME, CRANK, SENSE, REST, DONE, FAIL };
enum class StopSub  : uint8_t { IDLE, PULSE, VERIFY, DONE, FAIL };

struct StartSeq {
  bool active = false;
  bool manual = false;
  StartSub sub = StartSub::IDLE;
  uint32_t untilMs = 0;
  uint8_t attempt = 0;
} g_startSeq;

struct StopSeq {
  bool active = false;
  bool manual = false;
  StopSub sub = StopSub::IDLE;
  uint32_t untilMs = 0;
  uint8_t attempt = 0;
  uint32_t fuelOffAtMs = 0;
  uint32_t verifyDeadlineMs = 0;
} g_stopSeq;

static bool isGenRunningNow() {
  return (g_meas.genV >= g_set.genRunningV);
}

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

  // yakıt ON (motor çalışırken sürekli ON kalacak)
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

  if (isGenRunningNow()) {
    g_startSeq.sub = StartSub::DONE;
    g_startSeq.active = false;
    return;
  }

  if (!g_startSeq.manual && autoStartBlockedByBatt()) {
    g_startSeq.sub = StartSub::FAIL;
    g_startSeq.active = false;
    return;
  }

  if ((int32_t)(now - g_startSeq.untilMs) < 0) return;

  switch (g_startSeq.sub) {
    case StartSub::PRIME:
      safeSetFuel(true);
      g_startSeq.untilMs = now + (uint32_t)g_set.fuelPrimeMs;
      g_startSeq.sub = StartSub::CRANK;
      break;

    case StartSub::CRANK:
      if (g_pulseActive) return;
      if (g_startSeq.attempt >= g_set.startMaxAttempts) {
        g_startSeq.sub = StartSub::FAIL;
        g_startSeq.active = false;
        return;
      }
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

    default:
      break;
  }
}

static void stopSeqService() {
  if (!g_stopSeq.active) return;

  const uint32_t now = millis();

  if (!isGenRunningNow()) {
    g_stopSeq.sub = StopSub::DONE;
    g_stopSeq.active = false;
    safeSetFuel(false);
    return;
  }

  if ((int32_t)(now - g_stopSeq.untilMs) < 0) return;

  switch (g_stopSeq.sub) {
    case StopSub::PULSE:
      if (g_pulseActive) return;
      if (g_stopSeq.attempt >= g_set.stopMaxAttempts) {
        g_stopSeq.sub = StopSub::FAIL;
        g_stopSeq.active = false;
        safeSetFuel(false);
        return;
      }
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

      if (!isGenRunningNow()) {
        g_stopSeq.sub = StopSub::DONE;
        g_stopSeq.active = false;
        safeSetFuel(false);
        return;
      }

      if ((int32_t)(now - g_stopSeq.verifyDeadlineMs) >= 0) {
        g_stopSeq.sub = StopSub::PULSE;
        g_stopSeq.untilMs = now + 100;
      } else {
        g_stopSeq.untilMs = now + 120;
      }
      break;

    default:
      break;
  }
}

// =====================
// Stage 9 - AUTO logic (1Hz)
// =====================
static bool mainsIsBadForAuto() {
  return (g_meas.mainsV < g_set.autoStartMainsV);
}

static bool mainsIsGoodToStop() {
  return (g_meas.mainsV >= g_set.mainsNormMin && g_meas.mainsV <= g_set.mainsNormMax);
}

static void autoSetState(AutoState st, const String& reasonMsg = "") {
  if (st == g_autoState) return;
  g_autoState = st;

  if (reasonMsg.length())
    notify("🤖 AUTO: " + String(autoStateText(g_autoState)) + " — " + reasonMsg);
  else
    notify("🤖 AUTO: " + String(autoStateText(g_autoState)));
}

static void autoFuelPolicy() {
  // Yakıt rölesi: motor çalışırken sürekli ON
  if (g_mode != MODE_AUTO) return;

  switch (g_autoState) {
    case AutoState::STARTING:
    case AutoState::RUNNING:
    case AutoState::WAIT_RETURN_CONFIRM:
    case AutoState::COOLDOWN:
      safeSetFuel(true);
      break;
    case AutoState::STOPPING:
      // STOPPING içinde stopSeqService yakıtı gecikmeyle kesiyor
      safeSetFuel(true);
      break;
    default:
      safeSetFuel(false);
      break;
  }
}

static void autoTick_1s() {
  if (g_mode != MODE_AUTO) {
    if (g_autoState != AutoState::IDLE) {
      g_autoState = AutoState::IDLE;
      g_failStreakS = 0;
      g_returnStreakS = 0;
      g_coolCounterS = 0;
    }
    return;
  }

  autoFuelPolicy();

  if (g_autoState == AutoState::FAULT) return;

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
      if (isGenRunningNow()) {
        autoSetState(AutoState::RUNNING, "Jeneratör zaten çalışıyor");
        break;
      }

      if (autoStartBlockedByBatt()) {
        autoSetState(AutoState::FAULT, "Akü KRİTİK, auto-start bloklandı");
        safeSetFuel(false);
        break;
      }

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
        if (isGenRunningNow()) {
          autoSetState(AutoState::RUNNING, "Start başarılı");
        } else {
          autoSetState(AutoState::FAULT, "Start başarısız");
          safeSetFuel(false);
        }
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
      if (!isGenRunningNow()) {
        autoSetState(AutoState::IDLE, "Jeneratör durmuş");
        break;
      }

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
      if (!isGenRunningNow()) {
        autoSetState(AutoState::IDLE, "Cooldown bitmeden durmuş");
        break;
      }

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
        if (!isGenRunningNow()) {
          autoSetState(AutoState::IDLE, "Stop başarılı");
        } else {
          autoSetState(AutoState::FAULT, "Stop başarısız (lockout)");
          safeSetFuel(false);
        }
      }
      break;
    }

    default:
      break;
  }
}

// =====================
// Boot Report
// =====================
static String buildBootReport() {
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
  return s;
}

// =====================
// Telegram auth
// =====================
static bool isAuthorized(const telegramMessage& msg) {
  if (msg.chat_id == String(CHAT_ID)) return true;
  long fromId = msg.from_id.toInt();
  return (fromId == MASTER_ADMIN_ID);
}

static String buildStatusText() {
  return buildBootReport();
}

static void handleManualStart(const String& chatId) {
  if (g_mode == MODE_AUTO) {
    bot.sendMessage(chatId, "⚠️ Şu an AUTO modda. Önce /manual yap.", "");
    return;
  }
  if (g_startSeq.active || g_stopSeq.active) {
    bot.sendMessage(chatId, "⏳ Başka bir işlem aktif (start/stop).", "");
    return;
  }
  startSeqBegin(true);
  bot.sendMessage(chatId, "🟡 MANUAL: Start sekansı başladı (yakıt ON -> prime -> marş).", "");
}

static void handleManualStop(const String& chatId) {
  if (g_mode == MODE_AUTO) {
    bot.sendMessage(chatId, "⚠️ Şu an AUTO modda. Önce /manual yap.", "");
    return;
  }
  if (g_startSeq.active || g_stopSeq.active) {
    bot.sendMessage(chatId, "⏳ Başka bir işlem aktif (start/stop).", "");
    return;
  }
  stopSeqBegin(true);
  bot.sendMessage(chatId, "🟥 MANUAL: Stop sekansı başladı (stop pulse -> yakıt kes).", "");
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

      } else if (cmd == "/mode") {
        bot.sendMessage(msg.chat_id,
                        "🎛 Mod: " + String(modeText(g_mode)) +
                        "\n🤖 AutoState: " + String(autoStateText(g_autoState)) +
                        "\n⛽ Fuel=" + String(g_fuelOn ? "ON" : "OFF") +
                        "\n🟡 StartSeq=" + String(g_startSeq.active ? "ACTIVE" : "IDLE") +
                        "\n🟥 StopSeq=" + String(g_stopSeq.active ? "ACTIVE" : "IDLE"),
                        "");

      } else if (cmd == "/start") {
        handleManualStart(msg.chat_id);

      } else if (cmd == "/stop") {
        handleManualStop(msg.chat_id);

      } else if (cmd == "/cooldown") {
        int v = arg1.toInt();
        if (v <= 0) {
          bot.sendMessage(msg.chat_id, "Kullanım: /cooldown 120  (sn)", "");
        } else {
          g_set.cooldownS = (uint16_t)constrain(v, 5, 3600);
          bot.sendMessage(msg.chat_id, "✅ Cooldown = " + String(g_set.cooldownS) + "s (Kaydetmek için /save)", "");
        }

      } else if (cmd == "/autostart") {
        float v = arg1.toFloat();
        if (v <= 0) {
          bot.sendMessage(msg.chat_id, "Kullanım: /autostart 160  (V)", "");
        } else {
          g_set.autoStartMainsV = v;
          bot.sendMessage(msg.chat_id, "✅ AutoStart eşiği = " + fmt2(g_set.autoStartMainsV) + "V (Kaydetmek için /save)", "");
        }

      } else if (cmd == "/help" || cmd == "/yardim" || cmd == "/yardım") {
        String h;
        h += "Komutlar:\n";
        h += "/durum\n";
        h += "/mode  /auto  /manual\n";
        h += "/cooldown <sn>\n";
        h += "/autostart <V>\n";
        h += "/start /stop (sadece MANUAL)\n";
        h += "/save  /reset_hours  /ping\n";
        bot.sendMessage(msg.chat_id, h, "");

      } else {
        bot.sendMessage(msg.chat_id,
          "Komut: /durum /mode /auto /manual /cooldown /autostart /start /stop /save /reset_hours /ping",
          "");
      }
    }

    numNew = bot.getUpdates(bot.last_message_received + 1);
    if (numNew <= 0) break;
  }
}

// =====================
// Measure (MAINS + GEN + BATT)
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
// Save Button (uzun bas: kaydet)
// =====================
static void handleSaveButton() {
  bool btn = digitalRead(PIN_BTN_SAVE);
  uint32_t now = millis();

  if (lastBtn == true && btn == false) btnDownMs = now;

  if (lastBtn == false && btn == true) {
    uint32_t held = now - btnDownMs;
    if (held >= 800) {
      saveSettings();
      saveHoursTotal();
      notify("💾 Buton: Ayarlar + sayaç NVS'ye kaydedildi.");
    }
  }
  lastBtn = btn;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  analogReadResolution(12);
  pinMode(PIN_BTN_SAVE, INPUT_PULLUP);

  loadSettings();
  relayInit();

  connectWiFi();
  tgClient.setInsecure();

  // İlk ölçüm 0.00V olmasın diye 2 kez ölç
  readAllMeasurements();
  delay(250);
  readAllMeasurements();

  // İlk state'leri belirle (boot rapor doğru olsun)
  updateBatteryStatesOnly();
  g_mainsState = evalMains(g_meas.mainsV, MainsState::UNKNOWN);
  g_genState   = evalGen(g_meas.genV,   GenState::UNKNOWN);

  if (WiFi.status() == WL_CONNECTED) {
    notify(buildBootReport());
    notify("ℹ️ Bot hazır. /help");
  }

  g_stateAlertsArmed = true;

  tMeasure = millis();
  tSerial  = millis();
  tTgPoll  = millis();
  g_lastHoursSaveS = 0;
}

void loop() {
  // WiFi retry
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t tRetry = 0;
    if (millis() - tRetry > 5000) {
      tRetry = millis();
      connectWiFi();
    }
  }

  handleSaveButton();

  // Relay + sequences
  relayPulseService();
  startSeqService();
  stopSeqService();

  uint32_t now = millis();

  // 1Hz ölçüm + auto tick
  if (now - tMeasure >= MEASURE_MS) {
    tMeasure = now;

    readAllMeasurements();
    updateBatteryStatesOnly();
    handleStateAlerts();
    updateGenHoursCounter_1s();
    autoTick_1s();
  }

  // Serial
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

  // Telegram poll
  if (now - tTgPoll >= TG_POLL_MS) {
    tTgPoll = now;
    if (WiFi.status() == WL_CONNECTED) handleTelegram();
  }

  delay(5);
  yield();
}
