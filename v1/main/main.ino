// =====================
// main.ino (v10.006) - FULL
// ESP32-WROOM-32D + ILI9341 TFT + 6 Buttons + Generator Auto/Manual
// Boot TG: silent; WiFi OK -> "Hazır"
// Sensor invalid: TFT shows per-line "🧩 SENSÖR?", Telegram silent for alerts.
// =====================

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#include "config.h"

// ======================================================
// Types (MUST be above functions to avoid Arduino proto issues)
// ======================================================
enum RunMode : uint8_t { MODE_MANUAL = 0, MODE_AUTO = 1 };

struct Settings {
  float calMains, calGen, genBattDiv, camBattDiv;

  float mainsHigh, mainsNormMin, mainsNormMax, mainsLow, mainsCrit;
  float genOff, genLow, genNormMin, genNormMax;

  float battHigh, battNormMin, battLow, battCrit;

  float hystAc;
  float hystBatt;

  float genRunningV;
  uint16_t genRunConfirmS;
  uint32_t hoursSavePeriodS;

  float    autoStartMainsV;
  uint16_t mainsFailConfirmS;
  uint16_t mainsReturnConfirmS;
  uint16_t cooldownS;

  uint16_t fuelPrimeMs;
  uint16_t startPulseMs;
  uint32_t startRetryGapMs;
  uint16_t startSenseGraceMs;
  uint8_t  startMaxAttempts;

  uint16_t stopPulseMs;
  uint16_t stopVerifyS;
  uint8_t  stopMaxAttempts;
  uint16_t fuelOffDelayMs;

  uint8_t  faultMaxRetries;
  uint16_t faultRetryBaseS;
  uint16_t faultRetryMaxS;
};

struct Measurements {
  float mainsV_raw, genV_raw, genBattV_raw, camBattV_raw;
  float mainsV, genV, genBattV, camBattV;
  int wifiRssi;
  uint32_t uptimeS;
};

enum class BattState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
enum class MainsState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
enum class GenState   : uint8_t { UNKNOWN, OFF,      LOW_V, NORMAL, HIGH_V };

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

enum class StartSub : uint8_t { PRIME, CRANK, SENSE, REST };
enum class StopSub  : uint8_t { PULSE, VERIFY };

struct StartSeq {
  bool active = false;
  bool manual = false;
  StartSub sub = StartSub::PRIME;
  uint32_t untilMs = 0;
  uint8_t attempt = 0;
};

struct StopSeq {
  bool active = false;
  bool manual = false;
  StopSub sub = StopSub::PULSE;
  uint32_t untilMs = 0;
  uint8_t attempt = 0;
  uint32_t fuelOffAtMs = 0;
  uint32_t verifyDeadlineMs = 0;
};

// ---------- UI button
struct UiBtn {
  uint8_t pin = 255;
  bool stable = true;          // pullup -> true = released
  bool lastStable = true;
  bool pressedEvt = false;
  bool repeatEvt = false;
  uint32_t lastChangeMs = 0;
  uint32_t downMs = 0;
  uint32_t nextRepeatMs = 0;
  bool rawLast = true;
};

enum class UiScreen : uint8_t {
  STATUS,
  MENU,
  SETTINGS_CAT,
  SETTINGS_PARAM,
  CONFIRM_SAVE
};

enum class UiCategory : uint8_t { MAINS, GEN, BATT, AUTO, TIMING, FAULT, CALIB };

// Param type for safe get/set
enum class UiPType : uint8_t { F32, U16, U32, U8 };

struct UiParamRef {
  UiCategory cat;
  const char* name;
  UiPType type;
  void* ptr;         // points into g_work (shadow)
  float minV;
  float maxV;
  float step;
  uint8_t decimals;
  const char* key;   // NVS key (optional, for diff text)
};

struct UiCatList {
  UiCategory cat;
  uint16_t first;
  uint16_t count;
};

// ======================================================
// Globals
// ======================================================
Preferences prefs;

WiFiClientSecure tgClient;
UniversalTelegramBot bot(BOT_TOKEN, tgClient);

static RunMode g_mode = MODE_MANUAL;
static Settings g_set;      // active runtime
static Settings g_work;     // shadow (UI edits)
static Settings g_snap;     // snapshot for discard

static Measurements g_meas;

static BattState g_genBattState = BattState::UNKNOWN;
static BattState g_camBattState = BattState::UNKNOWN;

static MainsState g_mainsState = MainsState::UNKNOWN;
static GenState   g_genState   = GenState::UNKNOWN;

static bool g_stateAlertsArmed = false;
static uint32_t g_bootMs = 0;

static bool g_genRunning = false;
static uint16_t g_genRunStreakS = 0;
static uint64_t g_genRunTotalS  = 0;
static uint32_t g_lastHoursSaveS = 0;

static AutoState g_autoState = AutoState::IDLE;
static uint16_t g_failStreakS = 0;
static uint16_t g_returnStreakS = 0;
static uint16_t g_coolCounterS = 0;

static uint16_t g_faultClearStreakS = 0;
static uint8_t  g_faultRetryCount = 0;
static uint32_t g_faultNextRetryS = 0;

// Relay state
static bool g_fuelOn = false;
static bool g_startOn = false;
static bool g_stopOn  = false;

// Pulse engine
static bool g_pulseActive = false;
static uint8_t g_pulsePin = 255;
static uint32_t g_pulseUntilMs = 0;

// Sequences
static StartSeq g_startSeq;
static StopSeq  g_stopSeq;

// Timers
static uint32_t tMeasure = 0, tSerial = 0, tTgPoll = 0, tUi = 0;

// Sensor validity flags
static bool g_validMains = false;
static bool g_validGen   = false;
static bool g_validGenBatt = false;
static bool g_validCamBatt = false;

// ======================================================
// TFT + UI state
// ======================================================
Adafruit_ILI9341 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

static UiBtn bLeft, bRight, bUp, bDown, bOk, bBack;

static UiScreen ui_screen = UiScreen::STATUS;
static uint8_t ui_statusPage = 0;        // 0..3
static uint8_t ui_menuIndex = 0;         // menu selection

// Settings UI
static bool ui_dirty = false;
static bool ui_editing = false;
static UiCategory ui_cat = UiCategory::MAINS;
static uint8_t ui_catIndex = 0;
static uint16_t ui_paramIndex = 0;       // index inside category
static bool ui_confirmSaveYes = true;    // confirm screen selection

// Forward decl (all types already defined above)
static void uiInit();
static void uiService();
static void uiDraw(bool force);
static void uiEnter(UiScreen s);
static void uiEnterStatus();
static void uiEnterMenu();
static void uiEnterSettingsCat();
static void uiEnterSettingsParam(UiCategory cat);
static void uiEnterConfirmSave();

static void updateValidityFlags();

// ======================================================
// Helpers
// ======================================================
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

static String fmtDurTR(uint32_t sec) {
  uint32_t m = sec / 60U;
  uint32_t s = sec % 60U;
  if (m == 0) return String(s) + "sn";
  if (s == 0) return String(m) + "dk";
  return String(m) + "dk " + String(s) + "sn";
}

static float lpf(float prev, float x, float a) {
  if (isnan(prev) || isinf(prev)) return x;
  return prev + a * (x - prev);
}

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
    uint32_t d = faultBackoffDelayS(i);
    s += fmtDurTR(d);
    if (i != endIdx) s += " → ";
  }
  if (g_set.faultRetryMaxS > 0) s += " (cap: " + fmtDurTR(g_set.faultRetryMaxS) + ")";
  return s;
}

static inline bool isValidAc(float v)   { return !isnan(v) && !isinf(v) && (v >= AC_VALID_MIN_V); }
static inline bool isValidBatt(float v) { return !isnan(v) && !isinf(v) && (v >= BATT_VALID_MIN_V); }

static inline String sensorTag(bool ok) { return ok ? "" : "  🧩 SENSÖR?"; }

// ======================================================
// Relay control
// ======================================================
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

// ======================================================
// NVS load/save
// ======================================================
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

  // init shadow + snap
  g_work = g_set;
  g_snap = g_set;
}

static void saveSettings() {
  // Persist active g_set (not g_work)
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

// ======================================================
// ADC Reads
// ======================================================
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

static void updateValidityFlags() {
  g_validMains   = isValidAc(g_meas.mainsV);
  g_validGen     = isValidAc(g_meas.genV);
  g_validGenBatt = isValidBatt(g_meas.genBattV);
  g_validCamBatt = isValidBatt(g_meas.camBattV);
}

// ======================================================
// Battery state (NO TG)
// ======================================================
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

// ======================================================
// Stage 8 - MAINS / GEN state eval + Telegram alerts
// ======================================================
static String mainsStateLine(MainsState st, float v) {
  switch (st) {
    case MainsState::CRITICAL: return "🚨 Şebeke KRITIK: " + fmt2(v) + "V";
    case MainsState::LOW_V:    return "⚠️ Şebeke DÜŞÜK: " + fmt2(v) + "V";
    case MainsState::HIGH_V:   return "⚠️ Şebeke YÜKSEK: " + fmt2(v) + "V";
    case MainsState::NORMAL:   return "✅ Şebeke NORMAL: " + fmt2(v) + "V";
    default:                   return "ℹ️ Şebeke: " + fmt2(v) + "V";
  }
}

static String genStateLine(GenState st, float v) {
  switch (st) {
    case GenState::OFF:     return "⛔️ Jen OFF: " + fmt2(v) + "V";
    case GenState::LOW_V:   return "⚠️ Jen DÜŞÜK: " + fmt2(v) + "V";
    case GenState::HIGH_V:  return "⚠️ Jen YÜKSEK: " + fmt2(v) + "V";
    case GenState::NORMAL:  return "✅ Jen NORMAL: " + fmt2(v) + "V";
    default:                return "ℹ️ Jen: " + fmt2(v) + "V";
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
  if (WiFi.status() != WL_CONNECTED) return;

  // sensör geçersizse telegram sus
  if (g_validMains) {
    MainsState nm = evalMains(g_meas.mainsV, g_mainsState);
    if (nm != g_mainsState) {
      g_mainsState = nm;
      notify(mainsStateLine(g_mainsState, g_meas.mainsV));
    }
  }

  if (g_validGen) {
    GenState ng = evalGen(g_meas.genV, g_genState);
    if (ng != g_genState) {
      g_genState = ng;
      notify(genStateLine(g_genState, g_meas.genV));
    }
  }
}

// ======================================================
// Stage 5 - Hours Counter (1Hz)
// ======================================================
static bool isGenRunningNow() { return (g_validGen && g_meas.genV >= g_set.genRunningV); }

static void updateGenHoursCounter_1s() {
  bool above = isGenRunningNow();

  if (above) {
    if (g_genRunStreakS < 65000) g_genRunStreakS++;
  } else {
    g_genRunStreakS = 0;
    g_genRunning = false;
  }

  // Telegram mesajlarını da sensör validken ve arm sonrası yapalım
  if (!g_genRunning && g_genRunStreakS >= g_set.genRunConfirmS) {
    g_genRunning = true;
    if (WiFi.status() == WL_CONNECTED && g_stateAlertsArmed) notify("▶️ Jeneratör ÇALIŞIYOR (sayaç başladı)");
  }

  if (g_genRunning) {
    g_genRunTotalS += 1;
    if (!above) {
      g_genRunning = false;
      if (WiFi.status() == WL_CONNECTED && g_stateAlertsArmed) notify("⏹️ Jeneratör DURDU (sayaç durdu)");
    }
  }

  if (g_meas.uptimeS - g_lastHoursSaveS >= g_set.hoursSavePeriodS) {
    g_lastHoursSaveS = g_meas.uptimeS;
    saveHoursTotal();
  }
}

// ======================================================
// Start/Stop Sequences
// ======================================================
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
      if (WiFi.status() == WL_CONNECTED && g_stateAlertsArmed) notify(String("🟡 Start denemesi ") + g_startSeq.attempt + "/" + g_set.startMaxAttempts);
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
      if (WiFi.status() == WL_CONNECTED && g_stateAlertsArmed) notify(String("🟥 Stop pulse ") + g_stopSeq.attempt + "/" + g_set.stopMaxAttempts);
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

// ======================================================
// AUTO logic (1Hz)
// ======================================================
static bool mainsIsBadForAuto() { return (g_validMains && g_meas.mainsV < g_set.autoStartMainsV); }

static bool mainsIsGoodToStop() {
  return (g_validMains &&
          g_meas.mainsV >= g_set.mainsNormMin &&
          g_meas.mainsV <= g_set.mainsNormMax);
}

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

  // Boot mute boyunca otomatik mesaj yok
  if (!g_stateAlertsArmed) return;

  if (WiFi.status() == WL_CONNECTED) {
    if (reasonMsg.length())
      notify("🤖 AUTO: " + String(autoStateText(g_autoState)) + " — " + reasonMsg);
    else
      notify("🤖 AUTO: " + String(autoStateText(g_autoState)));
  }
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

  if (!g_stateAlertsArmed) return;

  if (g_faultRetryCount < g_set.faultMaxRetries) {
    uint32_t d = faultBackoffDelayS(g_faultRetryCount);
    g_faultNextRetryS = g_meas.uptimeS + d;

    if (WiFi.status() == WL_CONNECTED) {
      notify(
        "🧯 FAULT: Auto-retry " + fmtDurTR(d) + " sonra. ("
        + String(g_faultRetryCount + 1) + "/" + String(g_set.faultMaxRetries) + ")\n"
        + "📌 Backoff Plan: " + buildBackoffPlanText(g_faultRetryCount) + "\n"
        + "✅ Şebeke normale dönünce auto-reset."
      );
    }
  } else {
    g_faultNextRetryS = 0;
    if (WiFi.status() == WL_CONNECTED) {
      notify("🧯 FAULT: Auto-retry limiti doldu.\n✅ Şebeke normale dönünce auto-reset.");
    }
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

          if (g_stateAlertsArmed && WiFi.status() == WL_CONNECTED) {
            notify("🧯 FAULT: Auto-retry #" + String(g_faultRetryCount) + " başlıyor\n📌 Kalan Plan: " + buildBackoffPlanText(g_faultRetryCount));
          }

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

// ======================================================
// Telegram status text with per-measure SENSÖR?
// ======================================================
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

  s += mainsStateLine(g_mainsState, g_meas.mainsV) + sensorTag(g_validMains) + "\n";
  s += genStateLine(g_genState, g_meas.genV)       + sensorTag(g_validGen)   + "\n";

  s += "🔋 Gen Akü: " + fmt2(g_meas.genBattV) + "V (" + battStateToText(g_genBattState) + ")"
    + sensorTag(g_validGenBatt) + "\n";
  s += "🔋 Cam Akü: " + fmt2(g_meas.camBattV) + "V (" + battStateToText(g_camBattState) + ")"
    + sensorTag(g_validCamBatt) + "\n";

  s += "⛽ Fuel=" + String(g_fuelOn ? "ON" : "OFF") + "\n";
  s += "🟡 StartSeq=" + String(g_startSeq.active ? "ACTIVE" : "IDLE") + "\n";
  s += "🟥 StopSeq=" + String(g_stopSeq.active ? "ACTIVE" : "IDLE") + "\n";
  s += "⏱ Çalışma Süresi: " + fmtHMS(g_genRunTotalS) + "\n";

  if (g_autoState == AutoState::FAULT) {
    s += "🧯 FaultRetry=" + String(g_faultRetryCount) + "/" + String(g_set.faultMaxRetries) + "\n";
    if (g_faultNextRetryS) {
      uint32_t left = (g_meas.uptimeS >= g_faultNextRetryS) ? 0 : (g_faultNextRetryS - g_meas.uptimeS);
      s += "⏳ NextRetry=" + fmtDurTR(left) + "\n";
    }
    s += "📌 Backoff Plan: " + buildBackoffPlanText(g_faultRetryCount) + "\n";
    s += "✅ Şebeke normale dönünce auto-reset.\n";
  }
  return s;
}

// MANUAL /start /stop rules (same as v9.010)
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
          saveSettings();
          bot.sendMessage(msg.chat_id, "✅ Cooldown=" + String(g_set.cooldownS) + "s", "");
        }

      } else if (cmd == "/autostart") {
        float v = arg1.toFloat();
        if (v <= 0) bot.sendMessage(msg.chat_id, "Kullanım: /autostart 160 (V)", "");
        else {
          g_set.autoStartMainsV = v;
          saveSettings();
          bot.sendMessage(msg.chat_id, "✅ AutoStart=" + fmt2(g_set.autoStartMainsV) + "V", "");
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

// ======================================================
// UI: Buttons
// ======================================================
static void uiBtnInit(UiBtn& b, uint8_t pin) {
  b.pin = pin;
  pinMode(pin, INPUT_PULLUP);
  bool raw = digitalRead(pin);
  b.rawLast = raw;
  b.stable = raw;
  b.lastStable = raw;
  b.lastChangeMs = millis();
  b.downMs = 0;
  b.nextRepeatMs = 0;
  b.pressedEvt = false;
  b.repeatEvt = false;
}

static inline uint32_t uiHeldMs(const UiBtn& b) {
  if (b.stable == false) return (millis() - b.downMs);
  return 0;
}

static void uiBtnScanOne(UiBtn& b) {
  b.pressedEvt = false;
  b.repeatEvt = false;

  bool raw = digitalRead(b.pin);

  if (raw != b.rawLast) {
    b.rawLast = raw;
    b.lastChangeMs = millis();
  }

  if ((millis() - b.lastChangeMs) >= BTN_DEBOUNCE_MS) {
    if (b.stable != raw) {
      b.lastStable = b.stable;
      b.stable = raw;

      if (b.lastStable == true && b.stable == false) { // pressed
        b.pressedEvt = true;
        b.downMs = millis();
        b.nextRepeatMs = millis() + BTN_REPEAT_START_MS;
      }

      if (b.lastStable == false && b.stable == true) { // released
        b.nextRepeatMs = 0;
      }
    } else {
      if (b.stable == false && b.nextRepeatMs != 0 && (int32_t)(millis() - b.nextRepeatMs) >= 0) {
        b.repeatEvt = true;
        b.nextRepeatMs = millis() + BTN_REPEAT_EVERY_MS;
      }
    }
  }
}

static void uiBtnScanAll() {
  uiBtnScanOne(bLeft);
  uiBtnScanOne(bRight);
  uiBtnScanOne(bUp);
  uiBtnScanOne(bDown);
  uiBtnScanOne(bOk);
  uiBtnScanOne(bBack);
}

static inline bool uiPressed(const UiBtn& b) { return b.pressedEvt; }
static inline bool uiRepeat(const UiBtn& b)  { return b.repeatEvt; }

// ======================================================
// UI: Params (shadow editing)
// ======================================================
static const char* uiCatName(UiCategory c) {
  switch (c) {
    case UiCategory::MAINS:  return "Mains";
    case UiCategory::GEN:    return "Gen";
    case UiCategory::BATT:   return "Batt";
    case UiCategory::AUTO:   return "Auto";
    case UiCategory::TIMING: return "Timing";
    case UiCategory::FAULT:  return "Fault";
    case UiCategory::CALIB:  return "Calib";
    default: return "?";
  }
}

static float uiGetVal(const UiParamRef& p) {
  switch (p.type) {
    case UiPType::F32: return *(float*)p.ptr;
    case UiPType::U16: return (float)(*(uint16_t*)p.ptr);
    case UiPType::U32: return (float)(*(uint32_t*)p.ptr);
    case UiPType::U8:  return (float)(*(uint8_t*)p.ptr);
    default: return 0;
  }
}

static void uiSetVal(const UiParamRef& p, float v) {
  float x = v;
  if (x < p.minV) x = p.minV;
  if (x > p.maxV) x = p.maxV;

  switch (p.type) {
    case UiPType::F32: *(float*)p.ptr = x; break;
    case UiPType::U16: *(uint16_t*)p.ptr = (uint16_t)lroundf(x); break;
    case UiPType::U32: *(uint32_t*)p.ptr = (uint32_t)lroundf(x); break;
    case UiPType::U8:  *(uint8_t*)p.ptr  = (uint8_t)lroundf(x); break;
  }
}

static float uiStepAccel(const UiParamRef& p, uint32_t heldMs) {
  float step = p.step;
  if (heldMs >= 2500) step *= 10.0f;
  else if (heldMs >= 1200) step *= 5.0f;
  else if (heldMs >= 700) step *= 2.0f;
  return step;
}

static String uiValueText(const UiParamRef& p) {
  float v = uiGetVal(p);
  char b[24];
  dtostrf(v, 0, p.decimals, b);
  return String(b);
}

// ===== Param list (ALL)
static UiParamRef ui_params[] = {
  // CALIB
  {UiCategory::CALIB, "calMains",   UiPType::F32, &g_work.calMains,   50, 400, 1,   1, "calMains"},
  {UiCategory::CALIB, "calGen",     UiPType::F32, &g_work.calGen,     50, 400, 1,   1, "calGen"},
  {UiCategory::CALIB, "genDiv",     UiPType::F32, &g_work.genBattDiv, 1,  10,  0.01f, 2, "genDiv"},
  {UiCategory::CALIB, "camDiv",     UiPType::F32, &g_work.camBattDiv, 1,  10,  0.01f, 2, "camDiv"},

  // MAINS
  {UiCategory::MAINS, "mainsHigh",  UiPType::F32, &g_work.mainsHigh,    150, 270, 1, 0, "mHi"},
  {UiCategory::MAINS, "normMin",    UiPType::F32, &g_work.mainsNormMin, 150, 260, 1, 0, "mNmn"},
  {UiCategory::MAINS, "normMax",    UiPType::F32, &g_work.mainsNormMax, 150, 270, 1, 0, "mNmx"},
  {UiCategory::MAINS, "mainsLow",   UiPType::F32, &g_work.mainsLow,     50,  260, 1, 0, "mLo"},
  {UiCategory::MAINS, "mainsCrit",  UiPType::F32, &g_work.mainsCrit,    0,   240, 1, 0, "mCr"},
  {UiCategory::MAINS, "hystAc",     UiPType::F32, &g_work.hystAc,       0,   30,  0.5f, 1, "hAc"},

  // GEN
  {UiCategory::GEN,   "genOff",     UiPType::F32, &g_work.genOff,     0,  120, 1, 0, "gOff"},
  {UiCategory::GEN,   "genLow",     UiPType::F32, &g_work.genLow,     0,  240, 1, 0, "gLo"},
  {UiCategory::GEN,   "genNormMin", UiPType::F32, &g_work.genNormMin, 0,  260, 1, 0, "gNmn"},
  {UiCategory::GEN,   "genNormMax", UiPType::F32, &g_work.genNormMax, 0,  270, 1, 0, "gNmx"},
  {UiCategory::GEN,   "genRunV",    UiPType::F32, &g_work.genRunningV, 50, 230, 1, 0, "gRunV"},
  {UiCategory::GEN,   "genRunConf", UiPType::U16, &g_work.genRunConfirmS, 1, 120, 1, 0, "gRunC"},

  // BATT
  {UiCategory::BATT,  "battHigh",   UiPType::F32, &g_work.battHigh,    10.0f, 15.0f, 0.05f, 2, "bHi"},
  {UiCategory::BATT,  "battNormMin",UiPType::F32, &g_work.battNormMin, 10.0f, 15.0f, 0.05f, 2, "bNmn"},
  {UiCategory::BATT,  "battLow",    UiPType::F32, &g_work.battLow,     9.0f,  15.0f, 0.05f, 2, "bLo"},
  {UiCategory::BATT,  "battCrit",   UiPType::F32, &g_work.battCrit,    8.0f,  15.0f, 0.05f, 2, "bCr"},
  {UiCategory::BATT,  "hystBatt",   UiPType::F32, &g_work.hystBatt,    0.0f,  1.0f,  0.01f, 2, "hBt"},

  // AUTO
  {UiCategory::AUTO,  "autoStartV", UiPType::F32, &g_work.autoStartMainsV, 50, 230, 1, 0, "aMnsV"},
  {UiCategory::AUTO,  "failConfS",  UiPType::U16, &g_work.mainsFailConfirmS, 1, 300, 1, 0, "aFail"},
  {UiCategory::AUTO,  "retConfS",   UiPType::U16, &g_work.mainsReturnConfirmS, 1, 300, 1, 0, "aRet"},
  {UiCategory::AUTO,  "cooldownS",  UiPType::U16, &g_work.cooldownS, 5, 3600, 5, 0, "aCool"},

  // TIMING
  {UiCategory::TIMING,"fuelPrimeMs",UiPType::U16, &g_work.fuelPrimeMs, 0, 10000, 100, 0, "pF"},
  {UiCategory::TIMING,"startPulse", UiPType::U16, &g_work.startPulseMs, 100, 4000, 50, 0, "pSt"},
  {UiCategory::TIMING,"startGap",   UiPType::U32, &g_work.startRetryGapMs, 500, 20000, 250, 0, "gSt"},
  {UiCategory::TIMING,"startGrace", UiPType::U16, &g_work.startSenseGraceMs, 0, 10000, 100, 0, "sSt"},
  {UiCategory::TIMING,"startAtt",   UiPType::U8,  &g_work.startMaxAttempts, 1, 10, 1, 0, "aAtt"},
  {UiCategory::TIMING,"stopPulse",  UiPType::U16, &g_work.stopPulseMs, 100, 4000, 50, 0, "pSp"},
  {UiCategory::TIMING,"stopVerify", UiPType::U16, &g_work.stopVerifyS, 1, 60, 1, 0, "vSp"},
  {UiCategory::TIMING,"stopAtt",    UiPType::U8,  &g_work.stopMaxAttempts, 1, 5, 1, 0, "mSp"},
  {UiCategory::TIMING,"fuelOffDly", UiPType::U16, &g_work.fuelOffDelayMs, 0, 5000, 50, 0, "dF"},
  {UiCategory::TIMING,"hoursSaveS", UiPType::U32, &g_work.hoursSavePeriodS, 10, 3600, 10, 0, "hSaveP"},

  // FAULT
  {UiCategory::FAULT, "fMax",       UiPType::U8,  &g_work.faultMaxRetries, 0, 10, 1, 0, "fMax"},
  {UiCategory::FAULT, "fBaseS",     UiPType::U16, &g_work.faultRetryBaseS, 10, 3600, 10, 0, "fBase"},
  {UiCategory::FAULT, "fCapS",      UiPType::U16, &g_work.faultRetryMaxS,  10, 7200, 10, 0, "fCap"},
};

static const uint16_t UI_PARAM_COUNT = sizeof(ui_params) / sizeof(ui_params[0]);
static UiCatList ui_catLists[7];
static uint8_t ui_catCount = 0;

static void uiBuildCatIndex() {
  ui_catCount = 0;
  for (uint8_t c = 0; c < 7; c++) {
    UiCategory cat = (UiCategory)c;
    uint16_t first = 0xFFFF;
    uint16_t cnt = 0;
    for (uint16_t i = 0; i < UI_PARAM_COUNT; i++) {
      if (ui_params[i].cat == cat) {
        if (first == 0xFFFF) first = i;
        cnt++;
      }
    }
    if (first == 0xFFFF) first = 0;
    ui_catLists[ui_catCount++] = {cat, first, cnt};
  }
}

static UiCatList uiGetCatList(UiCategory cat) {
  for (uint8_t i = 0; i < ui_catCount; i++) {
    if (ui_catLists[i].cat == cat) return ui_catLists[i];
  }
  return ui_catLists[0];
}

static UiParamRef* uiCurrentParam() {
  UiCatList cl = uiGetCatList(ui_cat);
  if (cl.count == 0) return &ui_params[0];
  uint16_t idx = cl.first + ui_paramIndex;
  if (idx >= UI_PARAM_COUNT) idx = cl.first;
  return &ui_params[idx];
}

// diff builder (for Telegram save summary)
static bool uiWorkDiffersFromSet(const UiParamRef& p) {
  // Compare by key name; easiest: compare work value vs set value via mirror pointer math
  // Since ptr points into g_work, we reconstruct address in g_set by offset.
  uintptr_t off = (uintptr_t)p.ptr - (uintptr_t)&g_work;
  void* setPtr = (void*)((uintptr_t)&g_set + off);

  switch (p.type) {
    case UiPType::F32: return fabsf(*(float*)p.ptr - *(float*)setPtr) > 0.0001f;
    case UiPType::U16: return *(uint16_t*)p.ptr != *(uint16_t*)setPtr;
    case UiPType::U32: return *(uint32_t*)p.ptr != *(uint32_t*)setPtr;
    case UiPType::U8:  return *(uint8_t*)p.ptr  != *(uint8_t*)setPtr;
    default: return false;
  }
}

static String uiBuildSaveSummary() {
  String s;
  uint8_t cnt = 0;
  for (uint16_t i = 0; i < UI_PARAM_COUNT; i++) {
    if (uiWorkDiffersFromSet(ui_params[i])) cnt++;
  }
  if (cnt == 0) return "Değişiklik yok.";

  s = "💾 TFT: Ayarlar kaydedildi (" + String(cnt) + " değişiklik):\n";
  uint8_t printed = 0;
  for (uint16_t i = 0; i < UI_PARAM_COUNT; i++) {
    if (!uiWorkDiffersFromSet(ui_params[i])) continue;
    s += "• ";
    s += ui_params[i].name;
    s += "=";
    s += uiValueText(ui_params[i]);
    s += "\n";
    printed++;
    if (printed >= 12) { // çok uzamasın
      if (cnt > printed) s += "… +" + String(cnt - printed) + " daha\n";
      break;
    }
  }
  return s;
}

// ======================================================
// UI drawing helpers
// ======================================================
static void tftHeader(const String& title, const String& right = "") {
  tft.fillRect(0, 0, 320, 28, ILI9341_NAVY);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_NAVY);
  tft.setCursor(6, 6);
  tft.print(title);
  if (right.length()) {
    int16_t x = 310 - (int16_t)right.length() * 12;
    if (x < 150) x = 150;
    tft.setCursor(x, 6);
    tft.print(right);
  }
}

static void tftLine(int y, const String& text, uint16_t color = ILI9341_WHITE, uint8_t size = 2) {
  tft.fillRect(0, y, 320, 20 + (size-1)*8, ILI9341_BLACK);
  tft.setTextSize(size);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.setCursor(6, y);
  tft.print(text);
}

static String uiWifiText() {
  if (WiFi.status() == WL_CONNECTED) return "WiFi";
  return "WiFi?";
}

static String uiSensorTagInline(bool ok) {
  return ok ? "" : "  🧩 SENSÖR?";
}

// ======================================================
// UI screens
// ======================================================
static void uiDrawStatusPage(bool force) {
  (void)force;
  tft.fillScreen(ILI9341_BLACK);

  String right = String(ui_statusPage+1) + "/4  " + String(modeText(g_mode));
  tftHeader("Durum", right);

  // line y start
  int y = 34;

  if (ui_statusPage == 0) {
    String m = "Mains: " + fmt2(g_meas.mainsV) + "V" + uiSensorTagInline(g_validMains);
    String g = "Gen:   " + fmt2(g_meas.genV)   + "V" + uiSensorTagInline(g_validGen);
    tftLine(y,   m); y += 24;
    tftLine(y,   g); y += 24;
    tftLine(y, "Auto: " + String(autoStateText(g_autoState))); y += 24;
    tftLine(y, "Fuel: " + String(g_fuelOn ? "ON" : "OFF") + "  RunNow: " + String(isGenRunningNow() ? "YES":"NO")); y += 24;
    tftLine(y, uiWifiText() + " RSSI:" + String(g_meas.wifiRssi)); y += 24;

  } else if (ui_statusPage == 1) {
    tftLine(y, "GenBatt: " + fmt2(g_meas.genBattV) + "V" + uiSensorTagInline(g_validGenBatt)); y += 24;
    tftLine(y, "CamBatt: " + fmt2(g_meas.camBattV) + "V" + uiSensorTagInline(g_validCamBatt)); y += 24;
    tftLine(y, "GenBattSt: " + battStateToText(g_genBattState)); y += 24;
    tftLine(y, "CamBattSt: " + battStateToText(g_camBattState)); y += 24;

  } else if (ui_statusPage == 2) {
    tftLine(y, "StartSeq: " + String(g_startSeq.active ? "ACTIVE":"IDLE")); y += 24;
    tftLine(y, "StopSeq:  " + String(g_stopSeq.active  ? "ACTIVE":"IDLE")); y += 24;
    tftLine(y, "Pulse:    " + String(g_pulseActive ? "YES":"NO")); y += 24;
    tftLine(y, "SafeGap:  " + String(RELAY_SAFE_GAP_MS) + "ms"); y += 24;

  } else {
    tftLine(y, "Uptime: " + fmtDurTR(g_meas.uptimeS)); y += 24;
    tftLine(y, "Total:  " + fmtHMS(g_genRunTotalS)); y += 24;
    tftLine(y, "Ver: " + String(PROJECT_VERSION)); y += 24;
    tftLine(y, "OK:Menu  L/R:Sayfa"); y += 24;
  }
}

static void uiDrawMenu(bool force) {
  (void)force;
  tft.fillScreen(ILI9341_BLACK);
  tftHeader("Menu");

  const char* items[] = { "Durum", "Ayarlar", "Mod Degistir", "NVS Kaydet" };
  const uint8_t itemCount = 4;

  int y = 44;
  for (uint8_t i = 0; i < itemCount; i++) {
    uint16_t c = (i == ui_menuIndex) ? ILI9341_YELLOW : ILI9341_WHITE;
    String txt = String(i==ui_menuIndex ? "> " : "  ") + items[i];
    if (i == 2) txt += String(" (") + modeText(g_mode) + ")";
    tftLine(y, txt, c, 2);
    y += 26;
  }
  tftLine(200, "UP/DN Sec  OK Gir  BACK Cik", ILI9341_LIGHTGREY, 1);
}

static void uiDrawSettingsCat(bool force) {
  (void)force;
  tft.fillScreen(ILI9341_BLACK);
  tftHeader("Ayarlar", ui_dirty ? "*" : "");

  int y = 44;
  for (uint8_t i = 0; i < 7; i++) {
    uint16_t c = (i == ui_catIndex) ? ILI9341_YELLOW : ILI9341_WHITE;
    String txt = String(i==ui_catIndex ? "> " : "  ") + uiCatName((UiCategory)i);
    tftLine(y, txt, c, 2);
    y += 26;
  }
  tftLine(200, "OK:Kategori  BACK:Cik", ILI9341_LIGHTGREY, 1);
}

static void uiDrawSettingsParam(bool force) {
  (void)force;
  tft.fillScreen(ILI9341_BLACK);
  UiCatList cl = uiGetCatList(ui_cat);
  String right = String(uiCatName(ui_cat)) + " " + String(ui_paramIndex + 1) + "/" + String(cl.count);
  tftHeader("Param", right + (ui_dirty ? " *" : ""));

  UiParamRef* p = uiCurrentParam();

  tftLine(44, String("Kategori: ") + uiCatName(ui_cat), ILI9341_LIGHTGREY, 1);
  tftLine(70, String("Param: ") + p->name, ILI9341_WHITE, 2);

  String v = uiValueText(*p);
  String line = String("Deger: ") + v;
  if (ui_editing) line += "  [EDIT]";
  tftLine(98, line, ui_editing ? ILI9341_YELLOW : ILI9341_WHITE, 2);

  tftLine(130, String("Min: ") + String(p->minV) + "  Max: " + String(p->maxV), ILI9341_LIGHTGREY, 1);
  tftLine(146, String("Step: ") + String(p->step), ILI9341_LIGHTGREY, 1);

  tftLine(200, "L/R Param  OK Edit  BACK Geri", ILI9341_LIGHTGREY, 1);
  if (ui_editing) tftLine(214, "UP/DN Deger (Basili tut hizlanir)", ILI9341_LIGHTGREY, 1);
  else tftLine(214, "UP/DN: kapali (tek gorev)", ILI9341_DARKGREY, 1);
}

static void uiDrawConfirmSave(bool force) {
  (void)force;
  tft.fillScreen(ILI9341_BLACK);
  tftHeader("Kaydet?", ui_dirty ? "*" : "");

  tftLine(60, "Degisiklikler var.", ILI9341_WHITE, 2);
  tftLine(86, "Kaydedilsin mi?", ILI9341_WHITE, 2);

  String a = String(ui_confirmSaveYes ? "> " : "  ") + "Evet (Save)";
  String b = String(!ui_confirmSaveYes ? "> " : "  ") + "Hayir (Discard)";
  tftLine(130, a, ui_confirmSaveYes ? ILI9341_YELLOW : ILI9341_WHITE, 2);
  tftLine(156, b, !ui_confirmSaveYes ? ILI9341_YELLOW : ILI9341_WHITE, 2);

  tftLine(210, "L/R Sec  OK Uygula  BACK Geri", ILI9341_LIGHTGREY, 1);
}

// Master draw
static void uiDraw(bool force) {
  switch (ui_screen) {
    case UiScreen::STATUS:        uiDrawStatusPage(force); break;
    case UiScreen::MENU:          uiDrawMenu(force); break;
    case UiScreen::SETTINGS_CAT:  uiDrawSettingsCat(force); break;
    case UiScreen::SETTINGS_PARAM:uiDrawSettingsParam(force); break;
    case UiScreen::CONFIRM_SAVE:  uiDrawConfirmSave(force); break;
  }
}

// ======================================================
// UI navigation
// ======================================================
static void uiEnter(UiScreen s) {
  ui_screen = s;
  uiDraw(true);
}

static void uiEnterStatus() {
  ui_editing = false;
  uiEnter(UiScreen::STATUS);
}

static void uiEnterMenu() {
  ui_menuIndex = 0;
  uiEnter(UiScreen::MENU);
}

static void uiEnterSettingsCat() {
  // snapshot at entry
  g_snap = g_set;
  g_work = g_set;
  ui_dirty = false;
  ui_editing = false;
  ui_catIndex = 0;
  ui_cat = (UiCategory)ui_catIndex;
  ui_paramIndex = 0;
  uiEnter(UiScreen::SETTINGS_CAT);
}

static void uiEnterSettingsParam(UiCategory cat) {
  ui_cat = cat;
  ui_paramIndex = 0;
  ui_editing = false;
  uiEnter(UiScreen::SETTINGS_PARAM);
}

static void uiEnterConfirmSave() {
  ui_confirmSaveYes = true;
  uiEnter(UiScreen::CONFIRM_SAVE);
}

// ======================================================
// UI init/service
// ======================================================
static void uiInit() {
  // Backlight
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  // TFT begin
  tft.begin();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(ILI9341_BLACK);
  tftHeader(String("Boot ") + PROJECT_VERSION);

  // Buttons
  uiBtnInit(bLeft,  PIN_BTN_LEFT);
  uiBtnInit(bRight, PIN_BTN_RIGHT);
  uiBtnInit(bUp,    PIN_BTN_UP);
  uiBtnInit(bDown,  PIN_BTN_DOWN);
  uiBtnInit(bOk,    PIN_BTN_OK);
  uiBtnInit(bBack,  PIN_BTN_BACK);

  uiBuildCatIndex();
  uiEnterStatus();
}

static void uiService() {
  // limit UI scan rate
  if (millis() - tUi < 20) return;
  tUi = millis();

  uiBtnScanAll();

  // STATUS screen
  if (ui_screen == UiScreen::STATUS) {
    if (uiPressed(bLeft))  { ui_statusPage = (ui_statusPage + 3) % 4; uiDraw(true); }
    if (uiPressed(bRight)) { ui_statusPage = (ui_statusPage + 1) % 4; uiDraw(true); }
    if (uiPressed(bOk))    { uiEnterMenu(); return; }
    if (uiPressed(bBack))  { /* no-op */ }
    return;
  }

  // MENU
  if (ui_screen == UiScreen::MENU) {
    if (uiPressed(bUp))   { if (ui_menuIndex > 0) ui_menuIndex--; uiDraw(true); }
    if (uiPressed(bDown)) { if (ui_menuIndex < 3) ui_menuIndex++; uiDraw(true); }
    if (uiPressed(bBack)) { uiEnterStatus(); return; }

    if (uiPressed(bOk)) {
      switch (ui_menuIndex) {
        case 0: uiEnterStatus(); break;
        case 1: uiEnterSettingsCat(); break;
        case 2:
          g_mode = (g_mode == MODE_AUTO) ? MODE_MANUAL : MODE_AUTO;
          saveSettings();
          uiDraw(true);
          break;
        case 3:
          saveSettings();
          saveHoursTotal();
          if (WiFi.status() == WL_CONNECTED) notify("💾 NVS: Ayarlar + sayaç kaydedildi.");
          uiDraw(true);
          break;
      }
    }
    return;
  }

  // SETTINGS - CATEGORY
  if (ui_screen == UiScreen::SETTINGS_CAT) {
    if (uiPressed(bUp))   { if (ui_catIndex > 0) ui_catIndex--; uiDraw(true); }
    if (uiPressed(bDown)) { if (ui_catIndex < 6) ui_catIndex++; uiDraw(true); }
    if (uiPressed(bBack)) {
      if (ui_dirty) uiEnterConfirmSave();
      else uiEnterStatus();
      return;
    }
    if (uiPressed(bOk)) {
      uiEnterSettingsParam((UiCategory)ui_catIndex);
      return;
    }
    return;
  }

  // SETTINGS - PARAM
  if (ui_screen == UiScreen::SETTINGS_PARAM) {
    UiCatList cl = uiGetCatList(ui_cat);
    if (cl.count == 0) return;

    if (!ui_editing) {
      if (uiPressed(bLeft))  { if (ui_paramIndex > 0) ui_paramIndex--; uiDraw(true); }
      if (uiPressed(bRight)) { if (ui_paramIndex + 1 < cl.count) ui_paramIndex++; uiDraw(true); }

      if (uiPressed(bOk)) { ui_editing = true; uiDraw(true); }
      if (uiPressed(bBack)) {
        ui_editing = false;
        uiEnterSettingsCat();
      }
      return;
    }

    // Editing mode
    UiParamRef* p = uiCurrentParam();
    bool inc = uiPressed(bUp) || uiRepeat(bUp);
    bool dec = uiPressed(bDown) || uiRepeat(bDown);

    if (inc || dec) {
      float cur = uiGetVal(*p);
      float step = uiStepAccel(*p, uiHeldMs(inc ? bUp : bDown));
      float next = cur + (inc ? step : -step);
      uiSetVal(*p, next);
      ui_dirty = true;
      uiDraw(true);
    }

    if (uiPressed(bOk)) { // confirm param edit
      ui_editing = false;
      uiDraw(true);
    }

    if (uiPressed(bBack)) { // cancel param edit -> revert this param to snapshot
      // revert by copying snap->work for this field offset
      uintptr_t off = (uintptr_t)p->ptr - (uintptr_t)&g_work;
      memcpy((void*)((uintptr_t)&g_work + off), (void*)((uintptr_t)&g_snap + off),
             (p->type == UiPType::F32 ? sizeof(float) :
              p->type == UiPType::U16 ? sizeof(uint16_t) :
              p->type == UiPType::U32 ? sizeof(uint32_t) : sizeof(uint8_t)));
      ui_editing = false;
      // dirty recalculation (cheap)
      ui_dirty = false;
      for (uint16_t i = 0; i < UI_PARAM_COUNT; i++) if (uiWorkDiffersFromSet(ui_params[i])) { ui_dirty = true; break; }
      uiDraw(true);
    }

    return;
  }

  // CONFIRM SAVE
  if (ui_screen == UiScreen::CONFIRM_SAVE) {
    if (uiPressed(bLeft) || uiPressed(bRight)) {
      ui_confirmSaveYes = !ui_confirmSaveYes;
      uiDraw(true);
    }
    if (uiPressed(bBack)) { uiEnterSettingsCat(); return; }

    if (uiPressed(bOk)) {
      if (ui_confirmSaveYes) {
        // Apply work -> set, save NVS, TG summary (single message)
        g_set = g_work;
        saveSettings();
        saveHoursTotal();

        if (WiFi.status() == WL_CONNECTED) {
          notify(uiBuildSaveSummary());
        }
      } else {
        // Discard -> restore work from set (or snap)
        g_work = g_set;
      }
      ui_dirty = false;
      uiEnterStatus();
      return;
    }
  }
}

// ======================================================
// Setup/Loop
// ======================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  g_bootMs = millis();

  analogReadResolution(12);

  loadSettings();
  relayInit();

  uiInit();

  connectWiFi();
  tgClient.setInsecure();

  // İlk ölçümler (ama telegram'a boot report atma)
  readAllMeasurements();
  updateValidityFlags();
  updateBatteryStatesOnly();

  g_mainsState = evalMains(g_meas.mainsV, MainsState::UNKNOWN);
  g_genState   = evalGen(g_meas.genV,   GenState::UNKNOWN);

  // WiFi geldiyse sadece "Hazır"
  if (WiFi.status() == WL_CONNECTED) {
    notify("✅ Hazır. /help");
  }

  // Arm: BOOT_MUTE_S sonra, ayrıca en az bir valid ölçüm görürsek
  g_stateAlertsArmed = false;

  tMeasure = millis();
  tSerial  = millis();
  tTgPoll  = millis();
  g_lastHoursSaveS = 0;
}

void loop() {
  // WiFi reconnect
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t tRetry = 0;
    if (millis() - tRetry > 5000) {
      tRetry = millis();
      connectWiFi();
      if (WiFi.status() == WL_CONNECTED) notify("✅ Hazır. /help");
    }
  }

  relayPulseService();
  startSeqService();
  stopSeqService();

  uint32_t now = millis();

  // Measurement tick
  if (now - tMeasure >= MEASURE_MS) {
    tMeasure = now;

    readAllMeasurements();
    updateValidityFlags();
    updateBatteryStatesOnly();

    // arm logic
    if (!g_stateAlertsArmed) {
      if (g_meas.uptimeS >= BOOT_MUTE_S) {
        // en az bir valid AC ölçümü görmeden state alert açma
        if (g_validMains || g_validGen) {
          // state'i "baseline" olarak set et (ilk değişimde spam olmasın)
          if (g_validMains) g_mainsState = evalMains(g_meas.mainsV, g_mainsState);
          if (g_validGen)   g_genState   = evalGen(g_meas.genV, g_genState);
          g_stateAlertsArmed = true;
        }
      }
    }

    handleStateAlerts();
    updateGenHoursCounter_1s();
    autoTick_1s();

    // UI status screen auto refresh (light)
    if (ui_screen == UiScreen::STATUS) uiDraw(false);
  }

  // Serial report
  if (now - tSerial >= SERIAL_REPORT_MS) {
    tSerial = now;
    Serial.print("["); Serial.print(PROJECT_VERSION); Serial.print("] ");
    Serial.print("Mode="); Serial.print(modeText(g_mode));
    Serial.print(" Auto="); Serial.print(autoStateText(g_autoState));
    Serial.print(" Fuel="); Serial.print(g_fuelOn ? "ON" : "OFF");
    Serial.print(" Mains="); Serial.print(fmt2(g_meas.mainsV)); Serial.print(g_validMains ? "" : " ?");
    Serial.print(" Gen=");   Serial.print(fmt2(g_meas.genV));   Serial.print(g_validGen ? "" : " ?");
    Serial.print(" GenBatt="); Serial.print(fmt2(g_meas.genBattV)); Serial.print(g_validGenBatt ? "" : " ?");
    Serial.print(" CamBatt="); Serial.print(fmt2(g_meas.camBattV)); Serial.print(g_validCamBatt ? "" : " ?");
    Serial.print(" RunNow="); Serial.print(isGenRunningNow() ? "YES" : "NO");
    Serial.print(" Total="); Serial.print(fmtHMS(g_genRunTotalS));
    Serial.println();
  }

  // Telegram polling
  if (now - tTgPoll >= TG_POLL_MS) {
    tTgPoll = now;
    if (WiFi.status() == WL_CONNECTED) handleTelegram();
  }

  // UI
  uiService();

  delay(5);
  yield();
}
