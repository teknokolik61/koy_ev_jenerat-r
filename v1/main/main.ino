// =====================
// main.ino (v10.003)
// FIX: UiBtn/UiParamRef/UiCategory tipleri EN ÜSTE alındı.
// Böylece "UiBtn was not declared" derleme hatası biter.
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
// -------- TYPE DEFINITIONS (MUST BE ABOVE ANY FUNCTION USE) --------
// =====================

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

enum class BattState  : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
enum class MainsState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
enum class GenState   : uint8_t { UNKNOWN, OFF,      LOW_V, NORMAL, HIGH_V };

enum class AutoState : uint8_t {
  IDLE, WAIT_FAIL_CONFIRM, STARTING, RUNNING, WAIT_RETURN_CONFIRM, COOLDOWN, STOPPING, FAULT
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

// ---------- UI TYPES ----------
enum class UiState : uint8_t { STATUS, MENU, SETTINGS, CONFIRM_EXIT };

struct UiBtn {
  uint8_t pin;
  bool    stable;
  bool    lastRead;
  uint32_t lastChangeMs;

  bool    pressedEvt;
  bool    repeatEvt;
  uint32_t downMs;
  uint32_t lastRepeatMs;
};

enum class UiParamType : uint8_t { F32, U16, U32, U8 };

struct UiParamRef {
  const char* name;
  UiParamType type;
  void* ptr;
  float step;
  float stepFast;
  float vMin;
  float vMax;
  const char* unit;
};

enum class UiCategory : uint8_t { MAINS, GEN, BATT, AUTO, TIMING, FAULT, CAL, COUNT };

struct UiCatList {
  UiCategory cat;
  UiParamRef* list;
  uint16_t count;
};

// =====================
// -------- GLOBALS --------
// =====================
Preferences prefs;

WiFiClientSecure tgClient;
UniversalTelegramBot bot(BOT_TOKEN, tgClient);

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

static RunMode g_mode = MODE_MANUAL;
static Settings g_set;
static Measurements g_meas;

static uint32_t tMeasure = 0, tSerial = 0, tTgPoll = 0;

// states
static BattState  g_genBattState = BattState::UNKNOWN;
static BattState  g_camBattState = BattState::UNKNOWN;

static MainsState g_mainsState = MainsState::UNKNOWN;
static GenState   g_genState   = GenState::UNKNOWN;
static bool g_stateAlertsArmed = false;

// hours counter
static bool     g_genRunning = false;
static uint16_t g_genRunStreakS = 0;
static uint64_t g_genRunTotalS  = 0;
static uint32_t g_lastHoursSaveS = 0;

// auto state machine
static AutoState g_autoState = AutoState::IDLE;
static uint16_t g_failStreakS = 0;
static uint16_t g_returnStreakS = 0;
static uint16_t g_coolCounterS = 0;

static uint16_t g_faultClearStreakS = 0;
static uint8_t  g_faultRetryCount = 0;
static uint32_t g_faultNextRetryS = 0;

// relay states
static bool g_fuelOn = false;
static bool g_startOn = false;
static bool g_stopOn  = false;

static bool g_pulseActive = false;
static uint8_t g_pulsePin = 255;
static uint32_t g_pulseUntilMs = 0;

// sequences
static StartSeq g_startSeq;
static StopSeq  g_stopSeq;

// ---------- UI globals ----------
static UiState ui_state = UiState::STATUS;

static uint8_t ui_statusPage = 0;   // 0..3
static uint32_t ui_tRedraw = 0;

static UiBtn ui_bLeft, ui_bRight, ui_bUp, ui_bDown, ui_bBack, ui_bOk;

static uint8_t  ui_menuIndex = 0; // 0=Durum, 1=Ayarlar
static uint8_t  ui_catIndex = 0;
static uint16_t ui_paramIndex = 0;
static bool     ui_editing = false;

static Settings ui_snapshot;
static bool     ui_dirty = false;
static String   ui_changedSummary;

static uint8_t  ui_confirmChoice = 1; // 0=Discard, 1=Save
static float    ui_editOldValue = 0;
static bool     ui_editOldValid = false;

// =====================
// -------- HELPERS --------
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

// Telegram parsing
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
static void notify(const String& msg) { notifyTo(String(CHAT_ID), msg); }

// =====================
// -------- WIFI --------
// =====================
static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(250);
    yield();
  }
}

// =====================
// -------- RELAYS --------
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
// -------- NVS --------
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
// -------- ADC --------
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
// -------- STATE EVAL --------
// =====================
static String battStateToText(BattState st) {
  switch (st) {
    case BattState::CRITICAL: return "CRIT";
    case BattState::LOW_V:    return "LOW";
    case BattState::NORMAL:   return "NORM";
    case BattState::HIGH_V:   return "HIGH";
    default:                  return "UNK";
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

static String mainsStateLine(MainsState st, float v) {
  switch (st) {
    case MainsState::CRITICAL: return "🚨 Şebeke KRITIK: " + fmt2(v) + "V";
    case MainsState::LOW_V:    return "⚠️ Şebeke DUSUK: " + fmt2(v) + "V";
    case MainsState::HIGH_V:   return "⚠️ Şebeke YUKSEK: " + fmt2(v) + "V";
    case MainsState::NORMAL:   return "✅ Şebeke NORMAL: " + fmt2(v) + "V";
    default:                   return "ℹ️ Şebeke: " + fmt2(v) + "V";
  }
}

static String genStateLine(GenState st, float v) {
  switch (st) {
    case GenState::OFF:     return "⛔ Jen OFF: " + fmt2(v) + "V";
    case GenState::LOW_V:   return "⚠️ Jen DUSUK: " + fmt2(v) + "V";
    case GenState::HIGH_V:  return "⚠️ Jen YUKSEK: " + fmt2(v) + "V";
    case GenState::NORMAL:  return "✅ Jen NORMAL: " + fmt2(v) + "V";
    default:                return "ℹ️ Jen: " + fmt2(v) + "V";
  }
}

static void handleStateAlerts() {
  if (!ENABLE_TG_STATE_ALERTS) return;
  if (!g_stateAlertsArmed) return;

  MainsState nm = evalMains(g_meas.mainsV, g_mainsState);
  if (nm != g_mainsState) { g_mainsState = nm; notify(mainsStateLine(g_mainsState, g_meas.mainsV)); }

  GenState ng = evalGen(g_meas.genV, g_genState);
  if (ng != g_genState) { g_genState = ng; notify(genStateLine(g_genState, g_meas.genV)); }
}

// =====================
// -------- HOURS COUNTER --------
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
    notify("▶️ Jeneratör ÇALISIYOR (sayaç başladı)");
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
// -------- START/STOP SEQ --------
// =====================
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
// -------- AUTO (1Hz) --------
// =====================
static bool mainsIsBadForAuto() { return (g_meas.mainsV < g_set.autoStartMainsV); }
static bool mainsIsGoodToStop() { return (g_meas.mainsV >= g_set.mainsNormMin && g_meas.mainsV <= g_set.mainsNormMax); }

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
    if (i != endIdx) s += " -> ";
  }
  if (g_set.faultRetryMaxS > 0) s += " (cap: " + fmtDurTR(g_set.faultRetryMaxS) + ")";
  return s;
}

static void autoSetState(AutoState st, const String& reasonMsg = "") {
  if (st == g_autoState) return;
  if (st == AutoState::FAULT) { g_faultClearStreakS = 0; g_faultNextRetryS = 0; }
  if (g_autoState == AutoState::FAULT && st != AutoState::FAULT) {
    g_faultClearStreakS = 0; g_faultRetryCount = 0; g_faultNextRetryS = 0;
  }
  g_autoState = st;
  if (reasonMsg.length()) notify("🤖 AUTO: " + String(autoStateText(g_autoState)) + " — " + reasonMsg);
  else notify("🤖 AUTO: " + String(autoStateText(g_autoState)));
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
      g_failStreakS = 0; g_returnStreakS = 0; g_coolCounterS = 0;
      g_faultClearStreakS = 0; g_faultRetryCount = 0; g_faultNextRetryS = 0;
    }
    return;
  }

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
    case AutoState::IDLE:
      g_failStreakS = 0; g_returnStreakS = 0; g_coolCounterS = 0;
      if (!isGenRunningNow() && mainsIsBadForAuto()) {
        g_failStreakS = 1;
        autoSetState(AutoState::WAIT_FAIL_CONFIRM, "Şebeke düşük, doğrulama başladı");
      }
      break;

    case AutoState::WAIT_FAIL_CONFIRM:
      if (isGenRunningNow()) { autoSetState(AutoState::RUNNING, "Jeneratör zaten çalışıyor"); break; }
      if (autoStartBlockedByBatt()) { enterFaultWithSchedule("Akü KRITIK, auto-start bloklandı"); break; }

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

    case AutoState::STARTING:
      if (!g_startSeq.active) {
        if (isGenRunningNow()) autoSetState(AutoState::RUNNING, "Start başarılı");
        else enterFaultWithSchedule("Start başarısız");
      }
      break;

    case AutoState::RUNNING:
      if (mainsIsGoodToStop()) {
        g_returnStreakS = 1;
        autoSetState(AutoState::WAIT_RETURN_CONFIRM, "Şebeke normal, stop doğrulama");
      }
      break;

    case AutoState::WAIT_RETURN_CONFIRM:
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

    case AutoState::COOLDOWN:
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

    case AutoState::STOPPING:
      if (!g_stopSeq.active) {
        if (!isGenRunningNow()) autoSetState(AutoState::IDLE, "Stop başarılı");
        else enterFaultWithSchedule("Stop başarısız (lockout)");
      }
      break;

    default:
      break;
  }
}

// =====================
// -------- UI MODEL (PARAM LISTS) --------
// =====================
static float uiParamGet(const UiParamRef& p) {
  switch (p.type) {
    case UiParamType::F32: return *(float*)p.ptr;
    case UiParamType::U16: return (float)(*(uint16_t*)p.ptr);
    case UiParamType::U32: return (float)(*(uint32_t*)p.ptr);
    case UiParamType::U8:  return (float)(*(uint8_t*)p.ptr);
    default: return 0;
  }
}

static void uiParamSet(const UiParamRef& p, float v) {
  if (v < p.vMin) v = p.vMin;
  if (v > p.vMax) v = p.vMax;
  switch (p.type) {
    case UiParamType::F32: *(float*)p.ptr = v; break;
    case UiParamType::U16: *(uint16_t*)p.ptr = (uint16_t)lroundf(v); break;
    case UiParamType::U32: *(uint32_t*)p.ptr = (uint32_t)lroundf(v); break;
    case UiParamType::U8:  *(uint8_t*)p.ptr  = (uint8_t)lroundf(v); break;
    default: break;
  }
}

static String uiParamValueText(const UiParamRef& p) {
  float v = uiParamGet(p);
  if (p.type == UiParamType::F32) return fmt2(v) + String(p.unit);
  return String((uint32_t)lroundf(v)) + String(p.unit);
}

static const char* uiCatName(UiCategory c) {
  switch (c) {
    case UiCategory::MAINS:  return "MAINS";
    case UiCategory::GEN:    return "GEN";
    case UiCategory::BATT:   return "BATT";
    case UiCategory::AUTO:   return "AUTO";
    case UiCategory::TIMING: return "TIMING";
    case UiCategory::FAULT:  return "FAULT";
    case UiCategory::CAL:    return "CAL";
    default:                 return "UNK";
  }
}

// Param arrays
static UiParamRef UI_PARAM_MAINS[] = {
  {"mainsCrit",    UiParamType::F32, &g_set.mainsCrit,    1, 5,   0, 260, "V"},
  {"mainsLow",     UiParamType::F32, &g_set.mainsLow,     1, 5,   0, 260, "V"},
  {"mainsNormMin", UiParamType::F32, &g_set.mainsNormMin, 1, 5,   0, 260, "V"},
  {"mainsNormMax", UiParamType::F32, &g_set.mainsNormMax, 1, 5,   0, 260, "V"},
  {"mainsHigh",    UiParamType::F32, &g_set.mainsHigh,    1, 5,   0, 300, "V"},
  {"hystAc",       UiParamType::F32, &g_set.hystAc,       1, 2,   0,  50, "V"},
};

static UiParamRef UI_PARAM_GEN[] = {
  {"genOff",     UiParamType::F32, &g_set.genOff,     1, 5,  0, 260, "V"},
  {"genLow",     UiParamType::F32, &g_set.genLow,     1, 5,  0, 260, "V"},
  {"genNormMin", UiParamType::F32, &g_set.genNormMin, 1, 5,  0, 260, "V"},
  {"genNormMax", UiParamType::F32, &g_set.genNormMax, 1, 5,  0, 300, "V"},
  {"genRunningV",UiParamType::F32, &g_set.genRunningV,1, 5,  0, 260, "V"},
  {"genRunConfirmS", UiParamType::U16, &g_set.genRunConfirmS, 1, 5,  1, 120, "s"},
};

static UiParamRef UI_PARAM_BATT[] = {
  {"battCrit",    UiParamType::F32, &g_set.battCrit,    0.05f, 0.2f,  8.0f, 15.0f, "V"},
  {"battLow",     UiParamType::F32, &g_set.battLow,     0.05f, 0.2f,  8.0f, 15.0f, "V"},
  {"battNormMin", UiParamType::F32, &g_set.battNormMin, 0.05f, 0.2f,  8.0f, 15.0f, "V"},
  {"battHigh",    UiParamType::F32, &g_set.battHigh,    0.05f, 0.2f,  8.0f, 15.0f, "V"},
  {"hystBatt",    UiParamType::F32, &g_set.hystBatt,    0.01f, 0.05f, 0.00f, 1.0f,  "V"},
};

static UiParamRef UI_PARAM_AUTO[] = {
  {"autoStartMainsV",   UiParamType::F32, &g_set.autoStartMainsV,   1, 5,   0, 260, "V"},
  {"mainsFailConfirmS", UiParamType::U16, &g_set.mainsFailConfirmS, 1, 5,   1, 300, "s"},
  {"mainsReturnConfirmS",UiParamType::U16,&g_set.mainsReturnConfirmS,1, 5,  1, 600, "s"},
  {"cooldownS",         UiParamType::U16, &g_set.cooldownS,         5, 30,  5, 3600,"s"},
};

static UiParamRef UI_PARAM_TIMING[] = {
  {"fuelPrimeMs",       UiParamType::U16, &g_set.fuelPrimeMs,       50, 200,  0, 10000, "ms"},
  {"startPulseMs",      UiParamType::U16, &g_set.startPulseMs,      50, 200,  50, 10000, "ms"},
  {"startRetryGapMs",   UiParamType::U32, &g_set.startRetryGapMs,   100, 500,  0, 60000, "ms"},
  {"startSenseGraceMs", UiParamType::U16, &g_set.startSenseGraceMs, 100, 500,  0, 60000, "ms"},
  {"startMaxAttempts",  UiParamType::U8,  &g_set.startMaxAttempts,  1, 1,    1,  10, ""},
  {"stopPulseMs",       UiParamType::U16, &g_set.stopPulseMs,       50, 200,  50, 10000, "ms"},
  {"stopVerifyS",       UiParamType::U16, &g_set.stopVerifyS,       1, 5,    1, 120, "s"},
  {"stopMaxAttempts",   UiParamType::U8,  &g_set.stopMaxAttempts,   1, 1,    1,  10, ""},
  {"fuelOffDelayMs",    UiParamType::U16, &g_set.fuelOffDelayMs,    50, 200,  0, 10000, "ms"},
  {"hoursSavePeriodS",  UiParamType::U32, &g_set.hoursSavePeriodS,  10, 60,  10, 3600, "s"},
};

static UiParamRef UI_PARAM_FAULT[] = {
  {"faultMaxRetries",   UiParamType::U8,  &g_set.faultMaxRetries,   1, 1,    0, 10, ""},
  {"faultRetryBaseS",   UiParamType::U16, &g_set.faultRetryBaseS,   10, 60,  10, 7200, "s"},
  {"faultRetryMaxS",    UiParamType::U16, &g_set.faultRetryMaxS,    10, 60,  0,  7200, "s"},
};

static UiParamRef UI_PARAM_CAL[] = {
  {"calMains",   UiParamType::F32, &g_set.calMains,   1, 5,   10, 500, ""},
  {"calGen",     UiParamType::F32, &g_set.calGen,     1, 5,   10, 500, ""},
  {"genBattDiv", UiParamType::F32, &g_set.genBattDiv, 0.01f, 0.10f, 1.0f, 10.0f, ""},
  {"camBattDiv", UiParamType::F32, &g_set.camBattDiv, 0.01f, 0.10f, 1.0f, 10.0f, ""},
};

static UiCatList ui_cats[] = {
  {UiCategory::MAINS,  UI_PARAM_MAINS,  (uint16_t)(sizeof(UI_PARAM_MAINS)/sizeof(UI_PARAM_MAINS[0]))},
  {UiCategory::GEN,    UI_PARAM_GEN,    (uint16_t)(sizeof(UI_PARAM_GEN)/sizeof(UI_PARAM_GEN[0]))},
  {UiCategory::BATT,   UI_PARAM_BATT,   (uint16_t)(sizeof(UI_PARAM_BATT)/sizeof(UI_PARAM_BATT[0]))},
  {UiCategory::AUTO,   UI_PARAM_AUTO,   (uint16_t)(sizeof(UI_PARAM_AUTO)/sizeof(UI_PARAM_AUTO[0]))},
  {UiCategory::TIMING, UI_PARAM_TIMING, (uint16_t)(sizeof(UI_PARAM_TIMING)/sizeof(UI_PARAM_TIMING[0]))},
  {UiCategory::FAULT,  UI_PARAM_FAULT,  (uint16_t)(sizeof(UI_PARAM_FAULT)/sizeof(UI_PARAM_FAULT[0]))},
  {UiCategory::CAL,    UI_PARAM_CAL,    (uint16_t)(sizeof(UI_PARAM_CAL)/sizeof(UI_PARAM_CAL[0]))},
};

// =====================
// -------- UI BUTTONS --------
// =====================
static void uiBtnInit(UiBtn& b, uint8_t pin) {
  b.pin = pin;
  pinMode(pin, INPUT_PULLUP);
  bool r = digitalRead(pin);
  b.stable = r;
  b.lastRead = r;
  b.lastChangeMs = millis();
  b.pressedEvt = false;
  b.repeatEvt = false;
  b.downMs = 0;
  b.lastRepeatMs = 0;
}

static void uiBtnScanOne(UiBtn& b) {
  uint32_t now = millis();
  bool r = digitalRead(b.pin);

  b.pressedEvt = false;
  b.repeatEvt  = false;

  if (r != b.lastRead) {
    b.lastRead = r;
    b.lastChangeMs = now;
  }

  if (now - b.lastChangeMs >= BTN_DEBOUNCE_MS) {
    if (b.stable != b.lastRead) {
      b.stable = b.lastRead;
      if (b.stable == LOW) {
        b.pressedEvt = true;
        b.downMs = now;
        b.lastRepeatMs = now;
      }
    }
  }

  if (b.stable == LOW) {
    uint32_t held = now - b.downMs;
    if (held >= BTN_REPEAT_START_MS) {
      if (now - b.lastRepeatMs >= BTN_REPEAT_PERIOD_MS) {
        b.lastRepeatMs = now;
        b.repeatEvt = true;
      }
    }
  }
}

static void uiBtnScanAll() {
  uiBtnScanOne(ui_bLeft);
  uiBtnScanOne(ui_bRight);
  uiBtnScanOne(ui_bUp);
  uiBtnScanOne(ui_bDown);
  uiBtnScanOne(ui_bBack);
  uiBtnScanOne(ui_bOk);
}

static bool uiBtnPressedEvt(UiBtn& b) { return b.pressedEvt; }
static bool uiBtnRepeatEvt(UiBtn& b)  { return b.repeatEvt; }

static uint32_t uiBtnHeldMs(const UiBtn& b) {
  if (b.stable == LOW) return millis() - b.downMs;
  return 0;
}

// =====================
// -------- TFT DRAW --------
// =====================
static void uiTftHeader(const String& title, const String& right = "") {
  tft.fillRect(0, 0, 320, 28, ILI9341_NAVY);
  tft.setCursor(6, 6);
  tft.setTextColor(ILI9341_WHITE, ILI9341_NAVY);
  tft.setTextSize(2);
  tft.print(title);

  if (right.length()) {
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(right, 0, 0, &x1, &y1, &w, &h);
    int16_t rx = 320 - 6 - (int16_t)w;
    tft.setCursor(rx, 6);
    tft.print(right);
  }
}

static void uiTftLine(int y, const String& text, uint16_t color = ILI9341_WHITE) {
  tft.fillRect(0, y, 320, 26, ILI9341_BLACK);
  tft.setCursor(6, y + 5);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print(text);
}

static void uiTftSmall(int y, const String& text, uint16_t color = ILI9341_LIGHTGREY) {
  tft.fillRect(0, y, 320, 18, ILI9341_BLACK);
  tft.setCursor(6, y + 3);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.print(text);
}

// =====================
// -------- UI FLOW --------
// =====================
static void uiTrackChange(const char* pname, const String& vtxt) {
  ui_dirty = true;
  String line = String(pname) + "=" + vtxt;
  if (ui_changedSummary.indexOf(line) < 0) {
    if (ui_changedSummary.length() < 700) {
      if (ui_changedSummary.length()) ui_changedSummary += ", ";
      ui_changedSummary += line;
    }
  }
}

static void uiDrawStatusPage(bool force) {
  if (!force && millis() - ui_tRedraw < UI_STATUS_REFRESH_MS) return;
  ui_tRedraw = millis();

  String top = String("P") + (ui_statusPage + 1) + "/4 " + String(modeText(g_mode));
  uiTftHeader(String(DEVICE_NAME), top);

  switch (ui_statusPage) {
    case 0:
      uiTftLine(32, "MAINS: " + fmt2(g_meas.mainsV) + "V", ILI9341_CYAN);
      uiTftLine(60, "GEN:   " + fmt2(g_meas.genV)   + "V", ILI9341_GREEN);
      uiTftLine(88, String("Auto: ") + autoStateText(g_autoState), ILI9341_YELLOW);
      uiTftLine(116, String("Fuel: ") + (g_fuelOn ? "ON" : "OFF") + "  Run:" + (isGenRunningNow() ? "YES" : "NO"),
                g_fuelOn ? ILI9341_ORANGE : ILI9341_LIGHTGREY);
      uiTftSmall(146, "OK=Menu  L/R=Sayfa");
      break;

    case 1:
      uiTftLine(32, "GenBatt: " + fmt2(g_meas.genBattV) + "V " + battStateToText(g_genBattState), ILI9341_GREEN);
      uiTftLine(60, "CamBatt: " + fmt2(g_meas.camBattV) + "V " + battStateToText(g_camBattState), ILI9341_GREEN);
      uiTftLine(88, "AutoStart: " + fmt2(g_set.autoStartMainsV) + "V", ILI9341_YELLOW);
      uiTftLine(116, "Hyst AC: " + fmt2(g_set.hystAc) + "V  BT: " + fmt2(g_set.hystBatt) + "V", ILI9341_LIGHTGREY);
      uiTftSmall(146, "OK=Menu  L/R=Sayfa");
      break;

    case 2:
      uiTftLine(32, String("StartSeq: ") + (g_startSeq.active ? "ACTIVE" : "IDLE"), ILI9341_YELLOW);
      uiTftLine(60, String("StopSeq:  ") + (g_stopSeq.active  ? "ACTIVE" : "IDLE"), ILI9341_YELLOW);
      uiTftLine(88, String("Pulse:    ") + (g_pulseActive ? "ON" : "OFF"), ILI9341_LIGHTGREY);
      uiTftLine(116, String("Relays F/S/T: ") + (g_fuelOn?"1":"0") + "/" + (g_startOn?"1":"0") + "/" + (g_stopOn?"1":"0"),
                ILI9341_CYAN);
      uiTftSmall(146, "OK=Menu  L/R=Sayfa");
      break;

    default:
      uiTftLine(32, "Uptime: " + fmtDurTR(g_meas.uptimeS), ILI9341_CYAN);
      uiTftLine(60, "Hours:  " + fmtHMS(g_genRunTotalS), ILI9341_GREEN);
      uiTftLine(88, "WiFi RSSI: " + String(g_meas.wifiRssi), ILI9341_LIGHTGREY);
      uiTftLine(116, String("WiFi: ") + (WiFi.status() == WL_CONNECTED ? "OK" : "OFF"),
                WiFi.status() == WL_CONNECTED ? ILI9341_GREEN : ILI9341_RED);
      uiTftSmall(146, "OK=Menu  L/R=Sayfa");
      break;
  }
}

static void uiDrawMenu(bool force) {
  (void)force;
  uiTftHeader("MENU", String(modeText(g_mode)));
  uiTftLine(40, (ui_menuIndex == 0 ? "> " : "  ") + String("Durum"), ui_menuIndex==0 ? ILI9341_YELLOW : ILI9341_WHITE);
  uiTftLine(68, (ui_menuIndex == 1 ? "> " : "  ") + String("Ayarlar"), ui_menuIndex==1 ? ILI9341_YELLOW : ILI9341_WHITE);
  uiTftSmall(110, "Up/Down=Sec  OK=Gir  Back=Cik");
}

static void uiDrawSettings(bool force) {
  (void)force;

  UiCatList& cl = ui_cats[ui_catIndex];
  if (ui_paramIndex >= cl.count) ui_paramIndex = 0;
  UiParamRef& p = cl.list[ui_paramIndex];

  String right = String(uiCatName(cl.cat)) + " " + String(ui_paramIndex + 1) + "/" + String(cl.count);
  if (ui_dirty) right += " *";
  uiTftHeader("AYARLAR", right);

  uiTftLine(40, String("Param: ") + p.name, ILI9341_CYAN);
  uiTftLine(68, String("Value: ") + uiParamValueText(p), ILI9341_GREEN);

  if (ui_editing) {
    uiTftLine(96, "EDIT MODE", ILI9341_YELLOW);
    uiTftSmall(128, "Up/Down=Degistir  OK=Onay  Back=Iptal");
  } else {
    uiTftLine(96, String("Kategori: ") + uiCatName(cl.cat), ILI9341_LIGHTGREY);
    uiTftSmall(128, "L/R=Param  Up/Down=Kategori  OK=Edit  Back=Cik");
    uiTftSmall(146, "Dirty varsa cikista Save/Discard");
  }
}

static void uiDrawConfirmExit(bool force) {
  (void)force;
  uiTftHeader("KAYDET?", ui_dirty ? "*" : "");
  uiTftLine(50, "Degisiklikler var.", ILI9341_WHITE);
  uiTftLine(78, "Kaydedilsin mi?", ILI9341_WHITE);

  String a = (ui_confirmChoice == 0 ? "> " : "  "); a += "Hayir (Discard)";
  String b = (ui_confirmChoice == 1 ? "> " : "  "); b += "Evet (Save)";
  uiTftLine(112, a, ui_confirmChoice==0 ? ILI9341_YELLOW : ILI9341_WHITE);
  uiTftLine(140, b, ui_confirmChoice==1 ? ILI9341_YELLOW : ILI9341_WHITE);
  uiTftSmall(172, "L/R=Sec  OK=Uygula  Back=Geri");
}

static void uiEnterMenu() { ui_state = UiState::MENU; uiDrawMenu(true); }
static void uiEnterStatus() { ui_state = UiState::STATUS; uiDrawStatusPage(true); }

static void uiEnterSettings() {
  ui_state = UiState::SETTINGS;
  ui_editing = false;
  ui_dirty = false;
  ui_changedSummary = "";
  ui_snapshot = g_set;
  ui_catIndex = 0;
  ui_paramIndex = 0;
  ui_editOldValid = false;
  uiDrawSettings(true);
}

static void uiEnterConfirmExit() { ui_state = UiState::CONFIRM_EXIT; ui_confirmChoice = 1; uiDrawConfirmExit(true); }

static void uiDiscardChanges() {
  g_set = ui_snapshot;
  ui_dirty = false;
  ui_changedSummary = "";
}

static void uiSaveChangesAndNotify() {
  saveSettings();
  saveHoursTotal();

  String msg = "💾 TFT: Ayarlar kaydedildi.";
  if (ui_changedSummary.length()) msg += "\n⚙️ Degisenler: " + ui_changedSummary;
  notify(msg);

  ui_snapshot = g_set;
  ui_dirty = false;
  ui_changedSummary = "";
}

static float uiCurrentStepFor(const UiParamRef& p) {
  uint32_t heldUp   = uiBtnHeldMs(ui_bUp);
  uint32_t heldDown = uiBtnHeldMs(ui_bDown);
  uint32_t held = max(heldUp, heldDown);
  if (held >= BTN_FAST_STEP_AFTER_MS) return p.stepFast;
  return p.step;
}

static void uiService() {
  uiBtnScanAll();

  if (ui_state == UiState::STATUS) {
    uiDrawStatusPage(false);
    if (uiBtnPressedEvt(ui_bLeft))  { ui_statusPage = (ui_statusPage + 3) % 4; uiDrawStatusPage(true); }
    if (uiBtnPressedEvt(ui_bRight)) { ui_statusPage = (ui_statusPage + 1) % 4; uiDrawStatusPage(true); }
    if (uiBtnPressedEvt(ui_bOk))    { uiEnterMenu(); }
    return;
  }

  if (ui_state == UiState::MENU) {
    if (uiBtnPressedEvt(ui_bUp))   { if (ui_menuIndex > 0) ui_menuIndex--; uiDrawMenu(true); }
    if (uiBtnPressedEvt(ui_bDown)) { if (ui_menuIndex < 1) ui_menuIndex++; uiDrawMenu(true); }
    if (uiBtnPressedEvt(ui_bBack)) { uiEnterStatus(); return; }
    if (uiBtnPressedEvt(ui_bOk)) {
      if (ui_menuIndex == 0) uiEnterStatus();
      else uiEnterSettings();
      return;
    }
    return;
  }

  if (ui_state == UiState::SETTINGS) {
    UiCatList& cl = ui_cats[ui_catIndex];
    if (ui_paramIndex >= cl.count) ui_paramIndex = 0;
    UiParamRef& p = cl.list[ui_paramIndex];

    if (!ui_editing) {
      if (uiBtnPressedEvt(ui_bUp))   { ui_catIndex = (ui_catIndex == 0) ? ((uint8_t)UiCategory::COUNT - 1) : (ui_catIndex - 1); ui_paramIndex = 0; uiDrawSettings(true); }
      if (uiBtnPressedEvt(ui_bDown)) { ui_catIndex = (ui_catIndex + 1) % (uint8_t)UiCategory::COUNT; ui_paramIndex = 0; uiDrawSettings(true); }

      if (uiBtnPressedEvt(ui_bLeft))  { ui_paramIndex = (ui_paramIndex == 0) ? (cl.count - 1) : (ui_paramIndex - 1); uiDrawSettings(true); }
      if (uiBtnPressedEvt(ui_bRight)) { ui_paramIndex = (ui_paramIndex + 1) % cl.count; uiDrawSettings(true); }

      if (uiBtnPressedEvt(ui_bOk)) {
        ui_editing = true;
        ui_editOldValue = uiParamGet(p);
        ui_editOldValid = true;
        uiDrawSettings(true);
      }

      if (uiBtnPressedEvt(ui_bBack)) {
        if (ui_dirty) uiEnterConfirmExit();
        else uiEnterMenu();
      }
      return;
    }

    // editing
    if (uiBtnPressedEvt(ui_bOk)) { ui_editing = false; ui_editOldValid = false; uiDrawSettings(true); return; }
    if (uiBtnPressedEvt(ui_bBack)) { if (ui_editOldValid) uiParamSet(p, ui_editOldValue); ui_editing = false; ui_editOldValid = false; uiDrawSettings(true); return; }

    bool inc = uiBtnPressedEvt(ui_bUp)   || uiBtnRepeatEvt(ui_bUp);
    bool dec = uiBtnPressedEvt(ui_bDown) || uiBtnRepeatEvt(ui_bDown);

    if (inc || dec) {
      float step = uiCurrentStepFor(p);
      float oldV = uiParamGet(p);
      float v = oldV + (inc ? step : 0) - (dec ? step : 0);
      uiParamSet(p, v);

      float newV = uiParamGet(p);
      if (fabsf(newV - oldV) > 0.00001f) {
        uiTrackChange(p.name, uiParamValueText(p));
        uiDrawSettings(true);
      }
    }
    return;
  }

  if (ui_state == UiState::CONFIRM_EXIT) {
    if (uiBtnPressedEvt(ui_bLeft) || uiBtnPressedEvt(ui_bRight)) { ui_confirmChoice = (ui_confirmChoice == 0) ? 1 : 0; uiDrawConfirmExit(true); }
    if (uiBtnPressedEvt(ui_bBack)) { ui_state = UiState::SETTINGS; uiDrawSettings(true); return; }
    if (uiBtnPressedEvt(ui_bOk)) {
      if (ui_confirmChoice == 1) uiSaveChangesAndNotify();
      else uiDiscardChanges();
      uiEnterMenu();
      return;
    }
  }
}

// =====================
// TFT init
// =====================
static void tftInit() {
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);

  if (PIN_TFT_BL >= 0) {
    pinMode((uint8_t)PIN_TFT_BL, OUTPUT);
    digitalWrite((uint8_t)PIN_TFT_BL, TFT_BL_ACTIVE_LEVEL);
  }

  tft.begin();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(ILI9341_BLACK);
  uiTftHeader(String(DEVICE_NAME), String(PROJECT_VERSION));
  uiTftLine(40, "TFT UI hazir", ILI9341_GREEN);
  uiTftSmall(80, "OK=Menu  L/R=Sayfa");
}

// =====================
// Telegram Auth + Status
// =====================
static bool isAuthorized(const telegramMessage& msg) {
  if (msg.chat_id == String(CHAT_ID)) return true;
  long fromId = msg.from_id.toInt();
  return (fromId == MASTER_ADMIN_ID);
}

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

static String buildStatusText() { return buildBootReport(); }

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
    if (g_fuelOn) { safeSetFuel(false); bot.sendMessage(chatId, "✅ Jeneratör zaten durmuş. ⛽ Fuel OFF yapıldı.", ""); }
    else bot.sendMessage(chatId, "✅ Jeneratör zaten durmuş. Stop yapılmadı.", "");
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

      if (cmd == "/durum" || cmd == "/status") bot.sendMessage(msg.chat_id, buildStatusText(), "");
      else if (cmd == "/save") { saveSettings(); saveHoursTotal(); bot.sendMessage(msg.chat_id, "✅ Ayarlar + sayaç NVS'ye kaydedildi.", ""); }
      else if (cmd == "/auto") { g_mode = MODE_AUTO; saveSettings(); bot.sendMessage(msg.chat_id, "✅ Mod: AUTO", ""); }
      else if (cmd == "/manual") { g_mode = MODE_MANUAL; saveSettings(); bot.sendMessage(msg.chat_id, "✅ Mod: MANUAL", ""); }
      else if (cmd == "/start") handleManualStart(msg.chat_id);
      else if (cmd == "/stop")  handleManualStop(msg.chat_id);
      else if (cmd == "/help" || cmd == "/yardim" || cmd == "/yardım") {
        bot.sendMessage(msg.chat_id, "Komutlar: /durum /auto /manual /start /stop /save", "");
      } else {
        bot.sendMessage(msg.chat_id, "Komut: /durum /auto /manual /start /stop /save", "");
      }
    }

    numNew = bot.getUpdates(bot.last_message_received + 1);
    if (numNew <= 0) break;
  }
}

// =====================
// Setup / Loop
// =====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  analogReadResolution(12);

  uiBtnInit(ui_bLeft,  PIN_BTN_LEFT);
  uiBtnInit(ui_bRight, PIN_BTN_RIGHT);
  uiBtnInit(ui_bUp,    PIN_BTN_UP);
  uiBtnInit(ui_bDown,  PIN_BTN_DOWN);
  uiBtnInit(ui_bBack,  PIN_BTN_BACK);
  uiBtnInit(ui_bOk,    PIN_BTN_OK);

  loadSettings();
  relayInit();
  tftInit();

  connectWiFi();
  tgClient.setInsecure();

  readAllMeasurements();
  delay(250);
  readAllMeasurements();

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

  uiEnterStatus();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t tRetry = 0;
    if (millis() - tRetry > 5000) { tRetry = millis(); connectWiFi(); }
  }

  relayPulseService();
  startSeqService();
  stopSeqService();

  uiService();

  uint32_t now = millis();

  if (now - tMeasure >= MEASURE_MS) {
    tMeasure = now;
    readAllMeasurements();
    updateBatteryStatesOnly();
    handleStateAlerts();
    updateGenHoursCounter_1s();
    autoTick_1s();
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

  delay(3);
  yield();
}
