// =====================
// main.ino (v10.005) - ESP32-WROOM-32D
// TFT: ILI9341 240x320 SPI
// UI: 6 button (UP/DOWN/LEFT/RIGHT/OK/BACK)
// Settings: Category -> Param -> Edit -> Confirm Save/Discard
// TG: Boot'ta rapor YOK. WiFi bağlanınca sadece "✅ Hazır".
// Sensor invalid => TFT "SENSÖR?" + Telegram SUS (state alerts / auto messages).
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

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

enum RunMode : uint8_t { MODE_MANUAL = 0, MODE_AUTO = 1 };
static RunMode g_mode = MODE_MANUAL;

struct Settings {
  // calib/div
  float calMains, calGen, genBattDiv, camBattDiv;

  // mains
  float mainsHigh, mainsNormMin, mainsNormMax, mainsLow, mainsCrit;

  // gen
  float genOff, genLow, genNormMin, genNormMax;

  // battery
  float battHigh, battNormMin, battLow, battCrit;

  // hyst
  float hystAc, hystBatt;

  // running detect + hours
  float    genRunningV;
  uint16_t genRunConfirmS;
  uint32_t hoursSavePeriodS;

  // auto thresholds
  float    autoStartMainsV;
  uint16_t mainsFailConfirmS;
  uint16_t mainsReturnConfirmS;
  uint16_t cooldownS;

  // timing start/stop
  uint16_t fuelPrimeMs;
  uint16_t startPulseMs;
  uint32_t startRetryGapMs;
  uint16_t startSenseGraceMs;
  uint8_t  startMaxAttempts;

  uint16_t stopPulseMs;
  uint16_t stopVerifyS;
  uint8_t  stopMaxAttempts;
  uint16_t fuelOffDelayMs;

  // fault retry
  uint8_t  faultMaxRetries;
  uint16_t faultRetryBaseS;
  uint16_t faultRetryMaxS;
};
static Settings g_set;

struct Measurements {
  float mainsV_raw, genV_raw, genBattV_raw, camBattV_raw;
  float mainsV, genV, genBattV, camBattV;
  bool mainsValid, genValid, genBattValid, camBattValid;
  int  wifiRssi;
  uint32_t uptimeS;
};
static Measurements g_meas;

static uint32_t tMeasure = 0, tSerial = 0, tTgPoll = 0;

// =====================
// States
// =====================
enum class BattState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
enum class MainsState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
enum class GenState   : uint8_t { UNKNOWN, OFF,      LOW_V, NORMAL, HIGH_V };

static BattState  g_genBattState = BattState::UNKNOWN;
static BattState  g_camBattState = BattState::UNKNOWN;
static MainsState g_mainsState   = MainsState::UNKNOWN;
static GenState   g_genState     = GenState::UNKNOWN;

static bool g_stateAlertsArmed = false;

// hours
static bool     g_genRunning = false;
static uint16_t g_genRunStreakS = 0;
static uint64_t g_genRunTotalS  = 0;
static uint32_t g_lastHoursSaveS = 0;

// auto state
enum class AutoState : uint8_t {
  IDLE, WAIT_FAIL_CONFIRM, STARTING, RUNNING, WAIT_RETURN_CONFIRM, COOLDOWN, STOPPING, FAULT
};
static AutoState g_autoState = AutoState::IDLE;

static uint16_t g_failStreakS = 0;
static uint16_t g_returnStreakS = 0;
static uint16_t g_coolCounterS = 0;

static uint16_t g_faultClearStreakS = 0;
static uint8_t  g_faultRetryCount = 0;
static uint32_t g_faultNextRetryS = 0;

// relays
static bool g_fuelOn = false;
static bool g_startOn = false;
static bool g_stopOn  = false;

static bool g_pulseActive = false;
static uint8_t  g_pulsePin = 255;
static uint32_t g_pulseUntilMs = 0;

// start/stop sequences
enum class StartSub : uint8_t { PRIME, CRANK, SENSE, REST };
enum class StopSub  : uint8_t { PULSE, VERIFY };

struct StartSeq {
  bool active=false;
  bool manual=false;
  StartSub sub=StartSub::PRIME;
  uint32_t untilMs=0;
  uint8_t attempt=0;
};
static StartSeq g_startSeq;

struct StopSeq {
  bool active=false;
  bool manual=false;
  StopSub sub=StopSub::PULSE;
  uint32_t untilMs=0;
  uint8_t attempt=0;
  uint32_t fuelOffAtMs=0;
  uint32_t verifyDeadlineMs=0;
};
static StopSeq g_stopSeq;

// =====================
// Helpers
// =====================
static String fmt2(float v){
  if (isnan(v) || isinf(v)) return "nan";
  char b[24]; dtostrf(v, 0, 2, b);
  return String(b);
}

static String fmtDurTR(uint32_t sec){
  uint32_t m = sec/60U, s = sec%60U;
  if (m==0) return String(s) + "sn";
  if (s==0) return String(m) + "dk";
  return String(m) + "dk " + String(s) + "sn";
}

static String fmtHMS(uint64_t totalS){
  uint64_t h = totalS/3600ULL;
  uint64_t m = (totalS%3600ULL)/60ULL;
  uint64_t s = totalS%60ULL;
  return String((unsigned long)h)+"h "+String((unsigned long)m)+"m "+String((unsigned long)s)+"s";
}

static float lpf(float prev, float x, float a){
  if (isnan(prev) || isinf(prev)) return x;
  return prev + a*(x-prev);
}

static bool sensorsAnyInvalid(){
  return !(g_meas.mainsValid && g_meas.genValid && g_meas.genBattValid && g_meas.camBattValid);
}

static const char* modeText(RunMode m){ return (m==MODE_AUTO) ? "AUTO" : "MANUAL"; }
static const char* autoStateText(AutoState st){
  switch(st){
    case AutoState::IDLE: return "IDLE";
    case AutoState::WAIT_FAIL_CONFIRM: return "WAIT_FAIL_CONFIRM";
    case AutoState::STARTING: return "STARTING";
    case AutoState::RUNNING: return "RUNNING";
    case AutoState::WAIT_RETURN_CONFIRM: return "WAIT_RETURN_CONFIRM";
    case AutoState::COOLDOWN: return "COOLDOWN";
    case AutoState::STOPPING: return "STOPPING";
    case AutoState::FAULT: return "FAULT";
    default: return "?";
  }
}

static String normalizeCommand(const String& raw){
  String t=raw; t.trim();
  int sp=t.indexOf(' '); if(sp>=0) t=t.substring(0,sp);
  int at=t.indexOf('@'); if(at>=0) t=t.substring(0,at);
  t.toLowerCase();
  return t;
}
static String getArg1(const String& raw){
  String t=raw; t.trim();
  int sp=t.indexOf(' '); if(sp<0) return "";
  String a=t.substring(sp+1); a.trim();
  return a;
}

static void notifyTo(const String& chatId, const String& msg){
  if (WiFi.status()==WL_CONNECTED) bot.sendMessage(chatId,msg,"");
}
static void notify(const String& msg){ notifyTo(String(CHAT_ID), msg); }

// =====================
// WiFi
// =====================
static void connectWiFi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-start<15000){
    delay(250); yield();
  }
}

// =====================
// Relay Control
// =====================
static void relayWrite(uint8_t pin, bool on){
#if ENABLE_RELAY_CONTROL
  digitalWrite(pin, on ? RELAY_ACTIVE_LEVEL : RELAY_IDLE_LEVEL);
#else
  (void)pin; (void)on;
#endif
}

static void relayAllOffNow(){
#if ENABLE_RELAY_CONTROL
  relayWrite(PIN_RELAY_START,false);
  relayWrite(PIN_RELAY_STOP,false);
  relayWrite(PIN_RELAY_FUEL,false);
#endif
  g_fuelOn=g_startOn=g_stopOn=false;
  g_pulseActive=false; g_pulsePin=255; g_pulseUntilMs=0;
}

static void relayInit(){
#if ENABLE_RELAY_CONTROL
  pinMode(PIN_RELAY_FUEL, OUTPUT);
  pinMode(PIN_RELAY_START, OUTPUT);
  pinMode(PIN_RELAY_STOP, OUTPUT);
#endif
  relayAllOffNow();
}

static void safeSetStart(bool on){
#if ENABLE_RELAY_CONTROL
  if(on && g_stopOn){
    relayWrite(PIN_RELAY_STOP,false); g_stopOn=false;
    delay(RELAY_SAFE_GAP_MS);
  }
  relayWrite(PIN_RELAY_START,on);
#endif
  g_startOn=on;
}

static void safeSetStop(bool on){
#if ENABLE_RELAY_CONTROL
  if(on && g_startOn){
    relayWrite(PIN_RELAY_START,false); g_startOn=false;
    delay(RELAY_SAFE_GAP_MS);
  }
  relayWrite(PIN_RELAY_STOP,on);
#endif
  g_stopOn=on;
}

static void safeSetFuel(bool on){
#if ENABLE_RELAY_CONTROL
  relayWrite(PIN_RELAY_FUEL,on);
#endif
  g_fuelOn=on;
}

static void relayPulse(uint8_t pin, uint16_t ms){
#if ENABLE_RELAY_CONTROL
  if(pin==PIN_RELAY_START){ safeSetStop(false); safeSetStart(true); }
  if(pin==PIN_RELAY_STOP){  safeSetStart(false); safeSetStop(true); }
  g_pulseActive=true; g_pulsePin=pin;
  g_pulseUntilMs=millis() + (uint32_t)ms;
#else
  (void)pin; (void)ms;
#endif
}

static void relayPulseService(){
#if ENABLE_RELAY_CONTROL
  if(!g_pulseActive) return;
  if((int32_t)(millis()-g_pulseUntilMs)>=0){
    if(g_pulsePin==PIN_RELAY_START) safeSetStart(false);
    if(g_pulsePin==PIN_RELAY_STOP)  safeSetStop(false);
    g_pulseActive=false; g_pulsePin=255;
  }
#endif
}

// =====================
// NVS
// =====================
static void loadSettings(){
  prefs.begin(NVS_NAMESPACE,false);

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

  uint32_t lo = prefs.getUInt("hrsLo",0);
  uint32_t hi = prefs.getUInt("hrsHi",0);
  g_genRunTotalS = ((uint64_t)hi<<32) | (uint64_t)lo;
}

static void saveSettings(){
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

static void saveHoursTotal(){
  uint32_t lo=(uint32_t)(g_genRunTotalS & 0xFFFFFFFFULL);
  uint32_t hi=(uint32_t)(g_genRunTotalS >> 32);
  prefs.putUInt("hrsLo",lo);
  prefs.putUInt("hrsHi",hi);
}

// =====================
// ADC Reads + Validity
// =====================
static float readAdcVoltage(uint8_t pin, bool &valid, uint16_t samples=64){
  uint32_t sum=0; uint16_t mn=65535, mx=0;
  for(uint16_t i=0;i<samples;i++){
    uint16_t r=(uint16_t)analogRead(pin);
    sum += r;
    if(r<mn) mn=r;
    if(r>mx) mx=r;
    delay(2); yield();
  }
  float adc=(float)sum/(float)samples;

  if(adc<5 || adc>(ADC_MAX-5)) valid=false;
  else valid=true;

  if((mx-mn)<2) valid=false;

  if(!valid) return NAN;
  return (adc/(float)ADC_MAX)*ADC_VREF;
}

static float readAcRmsApprox(uint8_t pin, float calScale, bool &valid){
  uint32_t sum=0; uint16_t mn=65535, mx=0;

  for(uint16_t i=0;i<AC_SAMPLES;i++){
    uint16_t r=(uint16_t)analogRead(pin);
    sum += r;
    if(r<mn) mn=r;
    if(r>mx) mx=r;
    delayMicroseconds(AC_US_DELAY);
    yield();
  }
  float mean=(float)sum/(float)AC_SAMPLES;
  if(mean<AC_MEAN_MIN_COUNTS || mean>AC_MEAN_MAX_COUNTS){
    valid=false;
    return NAN;
  }

  double sq=0.0;
  for(uint16_t i=0;i<AC_SAMPLES;i++){
    float x=(float)analogRead(pin)-mean;
    sq += (double)(x*x);
    delayMicroseconds(AC_US_DELAY);
    yield();
  }
  float rmsCounts=sqrt((float)(sq/(double)AC_SAMPLES));
  float rmsVadc=(rmsCounts/(float)ADC_MAX)*ADC_VREF;

  valid=true;
  float vrms=rmsVadc*calScale;
  if(vrms<0) vrms=0;
  return vrms;
}

static void readAllMeasurements(){
  g_meas.wifiRssi = (WiFi.status()==WL_CONNECTED) ? WiFi.RSSI() : -999;
  g_meas.uptimeS  = millis()/1000U;

  g_meas.mainsV_raw = readAcRmsApprox(PIN_ADC_MAINS, g_set.calMains, g_meas.mainsValid);
  g_meas.genV_raw   = readAcRmsApprox(PIN_ADC_GEN,   g_set.calGen,   g_meas.genValid);

  g_meas.mainsV = g_meas.mainsValid ? lpf(g_meas.mainsV, g_meas.mainsV_raw, LPF_ALPHA_AC) : NAN;
  g_meas.genV   = g_meas.genValid   ? lpf(g_meas.genV,   g_meas.genV_raw,   LPF_ALPHA_AC) : NAN;

  float vGenAdc = readAdcVoltage(PIN_ADC_GEN_BATT, g_meas.genBattValid);
  float vCamAdc = readAdcVoltage(PIN_ADC_CAM_BATT, g_meas.camBattValid);

  g_meas.genBattV_raw = vGenAdc * g_set.genBattDiv;
  g_meas.camBattV_raw = vCamAdc * g_set.camBattDiv;

  g_meas.genBattV = g_meas.genBattValid ? lpf(g_meas.genBattV, g_meas.genBattV_raw, LPF_ALPHA_BATT) : NAN;
  g_meas.camBattV = g_meas.camBattValid ? lpf(g_meas.camBattV, g_meas.camBattV_raw, LPF_ALPHA_BATT) : NAN;
}

// =====================
// State Eval (Batt/Mains/Gen)
// =====================
static BattState evalBatt(float v, BattState prev){
  if(isnan(v)) return BattState::UNKNOWN;
  float h=g_set.hystBatt;

  switch(prev){
    case BattState::HIGH_V:
      if(v <= g_set.battHigh - h) return BattState::NORMAL;
      return BattState::HIGH_V;

    case BattState::NORMAL:
      if(v >= g_set.battHigh) return BattState::HIGH_V;
      if(v <  g_set.battCrit) return BattState::CRITICAL;
      if(v <  g_set.battLow)  return BattState::LOW_V;
      return BattState::NORMAL;

    case BattState::LOW_V:
      if(v >= g_set.battNormMin + h) return BattState::NORMAL;
      if(v <  g_set.battCrit)        return BattState::CRITICAL;
      return BattState::LOW_V;

    case BattState::CRITICAL:
      if(v >= g_set.battLow + h) return BattState::LOW_V;
      return BattState::CRITICAL;

    default:
      if(v >= g_set.battHigh) return BattState::HIGH_V;
      if(v <  g_set.battCrit) return BattState::CRITICAL;
      if(v <  g_set.battLow)  return BattState::LOW_V;
      return BattState::NORMAL;
  }
}

static MainsState evalMains(float v, MainsState prev){
  if(isnan(v)) return MainsState::UNKNOWN;
  float h=g_set.hystAc;

  switch(prev){
    case MainsState::CRITICAL:
      if(v >= g_set.mainsCrit + h) return MainsState::LOW_V;
      return MainsState::CRITICAL;

    case MainsState::LOW_V:
      if(v < g_set.mainsCrit) return MainsState::CRITICAL;
      if(v >= g_set.mainsLow + h) return MainsState::NORMAL;
      return MainsState::LOW_V;

    case MainsState::NORMAL:
      if(v >= g_set.mainsHigh) return MainsState::HIGH_V;
      if(v <  g_set.mainsLow)  return MainsState::LOW_V;
      return MainsState::NORMAL;

    case MainsState::HIGH_V:
      if(v <= g_set.mainsHigh - h) return MainsState::NORMAL;
      return MainsState::HIGH_V;

    default:
      if(v < g_set.mainsCrit) return MainsState::CRITICAL;
      if(v < g_set.mainsLow)  return MainsState::LOW_V;
      if(v >= g_set.mainsHigh) return MainsState::HIGH_V;
      return MainsState::NORMAL;
  }
}

static GenState evalGen(float v, GenState prev){
  if(isnan(v)) return GenState::UNKNOWN;
  float h=g_set.hystAc;

  switch(prev){
    case GenState::OFF:
      if(v >= g_set.genOff + h) return GenState::LOW_V;
      return GenState::OFF;

    case GenState::LOW_V:
      if(v < g_set.genOff) return GenState::OFF;
      if(v >= g_set.genNormMin + h) return GenState::NORMAL;
      return GenState::LOW_V;

    case GenState::NORMAL:
      if(v > g_set.genNormMax) return GenState::HIGH_V;
      if(v < g_set.genLow)     return GenState::LOW_V;
      return GenState::NORMAL;

    case GenState::HIGH_V:
      if(v <= g_set.genNormMax - h) return GenState::NORMAL;
      return GenState::HIGH_V;

    default:
      if(v < g_set.genOff) return GenState::OFF;
      if(v < g_set.genLow) return GenState::LOW_V;
      if(v > g_set.genNormMax) return GenState::HIGH_V;
      return GenState::NORMAL;
  }
}

static void updateStates(){
  g_genBattState = evalBatt(g_meas.genBattV, g_genBattState);
  g_camBattState = evalBatt(g_meas.camBattV, g_camBattState);
  g_mainsState   = evalMains(g_meas.mainsV,  g_mainsState);
  g_genState     = evalGen(g_meas.genV,      g_genState);
}

static String mainsStateLine(MainsState st, float v){
  switch(st){
    case MainsState::CRITICAL: return "🚨 Şebeke KRİTİK: " + fmt2(v) + "V";
    case MainsState::LOW_V:    return "⚠️ Şebeke DÜŞÜK: "  + fmt2(v) + "V";
    case MainsState::HIGH_V:   return "⚠️ Şebeke YÜKSEK: " + fmt2(v) + "V";
    case MainsState::NORMAL:   return "✅ Şebeke NORMAL: " + fmt2(v) + "V";
    default:                   return "ℹ️ Şebeke: " + fmt2(v) + "V";
  }
}
static String genStateLine(GenState st, float v){
  switch(st){
    case GenState::OFF:    return "⛔ Jeneratör OFF: " + fmt2(v) + "V";
    case GenState::LOW_V:  return "⚠️ Jeneratör DÜŞÜK: " + fmt2(v) + "V";
    case GenState::HIGH_V: return "⚠️ Jeneratör YÜKSEK: " + fmt2(v) + "V";
    case GenState::NORMAL: return "✅ Jeneratör NORMAL: " + fmt2(v) + "V";
    default:               return "ℹ️ Jeneratör: " + fmt2(v) + "V";
  }
}

static void handleStateAlerts(){
  if(!ENABLE_TG_STATE_ALERTS) return;
  if(!g_stateAlertsArmed) return;
  if(sensorsAnyInvalid()) return; // sensör invalid => TG SUS

  static MainsState lastM = MainsState::UNKNOWN;
  static GenState   lastG = GenState::UNKNOWN;

  if(g_mainsState != lastM){
    lastM = g_mainsState;
    notify(mainsStateLine(g_mainsState, g_meas.mainsV));
  }
  if(g_genState != lastG){
    lastG = g_genState;
    notify(genStateLine(g_genState, g_meas.genV));
  }
}

// =====================
// Running / Hours (1Hz)
// =====================
static bool isGenRunningNow(){
  if(!g_meas.genValid) return false;
  if(isnan(g_meas.genV)) return false;
  return g_meas.genV >= g_set.genRunningV;
}

static void updateGenHoursCounter_1s(){
  if(sensorsAnyInvalid()) return;
  bool above = isGenRunningNow();

  if(above){
    if(g_genRunStreakS<65000) g_genRunStreakS++;
  }else{
    g_genRunStreakS=0;
    g_genRunning=false;
  }

  if(!g_genRunning && g_genRunStreakS >= g_set.genRunConfirmS){
    g_genRunning=true;
    notify("▶️ Jeneratör ÇALIŞIYOR (sayaç başladı)");
  }

  if(g_genRunning){
    g_genRunTotalS++;
    if(!above){
      g_genRunning=false;
      notify("⏹️ Jeneratör DURDU (sayaç durdu)");
    }
  }

  if(g_meas.uptimeS - g_lastHoursSaveS >= g_set.hoursSavePeriodS){
    g_lastHoursSaveS = g_meas.uptimeS;
    saveHoursTotal();
  }
}

// =====================
// Start/Stop sequences
// =====================
static bool autoStartBlockedByBatt(){
  if(!AUTO_BLOCK_ON_BATT_CRIT) return false;
  return (g_genBattState == BattState::CRITICAL);
}

static void startSeqBegin(bool manual){
  g_startSeq.active=true; g_startSeq.manual=manual;
  g_startSeq.sub=StartSub::PRIME;
  g_startSeq.untilMs=millis();
  g_startSeq.attempt=0;
  safeSetFuel(true);
}
static void stopSeqBegin(bool manual){
  g_stopSeq.active=true; g_stopSeq.manual=manual;
  g_stopSeq.sub=StopSub::PULSE;
  g_stopSeq.untilMs=millis();
  g_stopSeq.attempt=0;
  g_stopSeq.fuelOffAtMs=0;
  g_stopSeq.verifyDeadlineMs=0;
}

static void startSeqService(){
  if(!g_startSeq.active) return;
  uint32_t now=millis();

  if(isGenRunningNow()){ g_startSeq.active=false; return; }
  if(!g_startSeq.manual && autoStartBlockedByBatt()){ g_startSeq.active=false; return; }
  if((int32_t)(now-g_startSeq.untilMs)<0) return;

  switch(g_startSeq.sub){
    case StartSub::PRIME:
      safeSetFuel(true);
      g_startSeq.untilMs = now + (uint32_t)g_set.fuelPrimeMs;
      g_startSeq.sub = StartSub::CRANK;
      break;

    case StartSub::CRANK:
      if(g_pulseActive) return;
      if(g_startSeq.attempt >= g_set.startMaxAttempts){ g_startSeq.active=false; return; }
      g_startSeq.attempt++;
      if(!sensorsAnyInvalid()) notify(String("🟡 Start denemesi ")+g_startSeq.attempt+"/"+g_set.startMaxAttempts);
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

static void stopSeqService(){
  if(!g_stopSeq.active) return;
  uint32_t now=millis();

  if(!isGenRunningNow()){
    g_stopSeq.active=false;
    safeSetFuel(false);
    return;
  }
  if((int32_t)(now-g_stopSeq.untilMs)<0) return;

  switch(g_stopSeq.sub){
    case StopSub::PULSE:
      if(g_pulseActive) return;
      if(g_stopSeq.attempt >= g_set.stopMaxAttempts){ g_stopSeq.active=false; safeSetFuel(false); return; }
      g_stopSeq.attempt++;
      if(!sensorsAnyInvalid()) notify(String("🟥 Stop pulse ")+g_stopSeq.attempt+"/"+g_set.stopMaxAttempts);
      relayPulse(PIN_RELAY_STOP, g_set.stopPulseMs);
      g_stopSeq.fuelOffAtMs = now + (uint32_t)g_set.fuelOffDelayMs;
      g_stopSeq.verifyDeadlineMs = now + (uint32_t)g_set.stopVerifyS*1000UL;
      g_stopSeq.sub = StopSub::VERIFY;
      g_stopSeq.untilMs = now + 50;
      break;

    case StopSub::VERIFY:
      if(g_stopSeq.fuelOffAtMs && (int32_t)(now-g_stopSeq.fuelOffAtMs)>=0){
        safeSetFuel(false);
        g_stopSeq.fuelOffAtMs=0;
      }
      if(!isGenRunningNow()){
        g_stopSeq.active=false;
        safeSetFuel(false);
        return;
      }
      if((int32_t)(now-g_stopSeq.verifyDeadlineMs)>=0){
        g_stopSeq.sub = StopSub::PULSE;
        g_stopSeq.untilMs = now + 100;
      }else{
        g_stopSeq.untilMs = now + 120;
      }
      break;
  }
}

// =====================
// AUTO (1Hz) - minimal (sensör invalidse SUS)
// =====================
static bool mainsIsBadForAuto(){
  if(!g_meas.mainsValid || isnan(g_meas.mainsV)) return false;
  return (g_meas.mainsV < g_set.autoStartMainsV);
}
static bool mainsIsGoodToStop(){
  if(!g_meas.mainsValid || isnan(g_meas.mainsV)) return false;
  return (g_meas.mainsV >= g_set.mainsNormMin && g_meas.mainsV <= g_set.mainsNormMax);
}
static void autoSetState(AutoState st, const String& why=""){
  if(st==g_autoState) return;
  g_autoState=st;
  if(sensorsAnyInvalid()) return; // sensör invalid => TG SUS
  if(why.length()) notify(String("🤖 AUTO: ")+autoStateText(g_autoState)+" — "+why);
  else notify(String("🤖 AUTO: ")+autoStateText(g_autoState));
}

static void autoTick_1s(){
  if(sensorsAnyInvalid()) return;

  if(g_mode!=MODE_AUTO){
    if(g_autoState!=AutoState::IDLE){
      g_autoState=AutoState::IDLE;
      g_failStreakS=g_returnStreakS=g_coolCounterS=0;
      g_faultClearStreakS=0; g_faultRetryCount=0; g_faultNextRetryS=0;
    }
    return;
  }

  switch(g_autoState){
    case AutoState::IDLE:
      if(!isGenRunningNow() && mainsIsBadForAuto()){
        g_failStreakS=1;
        autoSetState(AutoState::WAIT_FAIL_CONFIRM, "Şebeke düşük, doğrulama başladı");
      }
      break;

    case AutoState::WAIT_FAIL_CONFIRM:
      if(isGenRunningNow()){ autoSetState(AutoState::RUNNING, "Zaten çalışıyor"); break; }
      if(autoStartBlockedByBatt()){ autoSetState(AutoState::FAULT, "Akü KRİTİK, bloklandı"); break; }
      if(mainsIsBadForAuto()){
        if(g_failStreakS<65000) g_failStreakS++;
        if(g_failStreakS>=g_set.mainsFailConfirmS){
          autoSetState(AutoState::STARTING, "Oto start");
          startSeqBegin(false);
        }
      }else{
        autoSetState(AutoState::IDLE, "Şebeke düzeldi (iptal)");
      }
      break;

    case AutoState::STARTING:
      if(!g_startSeq.active){
        if(isGenRunningNow()) autoSetState(AutoState::RUNNING, "Start başarılı");
        else autoSetState(AutoState::FAULT, "Start başarısız");
      }
      break;

    case AutoState::RUNNING:
      if(mainsIsGoodToStop()){
        g_returnStreakS=1;
        autoSetState(AutoState::WAIT_RETURN_CONFIRM, "Şebeke normal, stop doğrulama");
      }
      break;

    case AutoState::WAIT_RETURN_CONFIRM:
      if(!isGenRunningNow()){ autoSetState(AutoState::IDLE, "Jeneratör durmuş"); break; }
      if(mainsIsGoodToStop()){
        if(g_returnStreakS<65000) g_returnStreakS++;
        if(g_returnStreakS>=g_set.mainsReturnConfirmS){
          g_coolCounterS=0;
          autoSetState(AutoState::COOLDOWN, "Cooldown");
        }
      }else{
        autoSetState(AutoState::RUNNING, "Şebeke bozuldu (iptal)");
      }
      break;

    case AutoState::COOLDOWN:
      if(!isGenRunningNow()){ autoSetState(AutoState::IDLE, "Durmuş"); break; }
      if(mainsIsBadForAuto()){ autoSetState(AutoState::RUNNING, "Şebeke bozuldu"); break; }
      if(g_coolCounterS<65000) g_coolCounterS++;
      if(g_coolCounterS>=g_set.cooldownS){
        autoSetState(AutoState::STOPPING, "Stop veriliyor");
        stopSeqBegin(false);
      }
      break;

    case AutoState::STOPPING:
      if(!g_stopSeq.active){
        if(!isGenRunningNow()) autoSetState(AutoState::IDLE, "Stop başarılı");
        else autoSetState(AutoState::FAULT, "Stop başarısız");
      }
      break;

    case AutoState::FAULT:
      // burada şimdilik basit: şebeke normale dönünce reset
      if(mainsIsGoodToStop()){
        if(g_faultClearStreakS<65000) g_faultClearStreakS++;
        if(g_faultClearStreakS>=g_set.mainsReturnConfirmS){
          g_faultClearStreakS=0;
          autoSetState(AutoState::IDLE, "FAULT auto-reset (şebeke normal)");
        }
      }else g_faultClearStreakS=0;
      break;
  }
}

// =====================
// Telegram
// =====================
static bool isAuthorized(const telegramMessage& msg){
  if(msg.chat_id == String(CHAT_ID)) return true;
  long fromId = msg.from_id.toInt();
  return (fromId == MASTER_ADMIN_ID);
}

static String buildStatusText(){
  String s;
  s += String(DEVICE_NAME) + "\n";
  s += "🔖 Sürüm: " + String(PROJECT_VERSION) + "\n";
  s += "🎛 Mod: " + String(modeText(g_mode)) + "\n";
  s += "🤖 AutoState: " + String(autoStateText(g_autoState)) + "\n";

  if(sensorsAnyInvalid()){
    s += "🧩 SENSÖR? (ölçüm geçersiz)\n";
  }else{
    s += mainsStateLine(g_mainsState, g_meas.mainsV) + "\n";
    s += genStateLine(g_genState, g_meas.genV) + "\n";
    s += "🔋 Gen Akü: " + fmt2(g_meas.genBattV) + "V\n";
    s += "🔋 Cam Akü: " + fmt2(g_meas.camBattV) + "V\n";
  }

  s += "⛽ Fuel=" + String(g_fuelOn ? "ON":"OFF") + "\n";
  s += "⏱ Çalışma Süresi: " + fmtHMS(g_genRunTotalS) + "\n";
  return s;
}

static void handleTelegram(){
  int numNew = bot.getUpdates(bot.last_message_received + 1);
  if(numNew<=0) return;

  while(numNew>0){
    for(int i=0;i<numNew;i++){
      telegramMessage& msg = bot.messages[i];
      if(!isAuthorized(msg)) continue;

      String cmd = normalizeCommand(msg.text);
      String arg1 = getArg1(msg.text);

      if(cmd=="/durum" || cmd=="/status"){
        bot.sendMessage(msg.chat_id, buildStatusText(), "");

      }else if(cmd=="/auto"){
        g_mode=MODE_AUTO; saveSettings();
        bot.sendMessage(msg.chat_id, "✅ Mod: AUTO", "");

      }else if(cmd=="/manual"){
        g_mode=MODE_MANUAL; saveSettings();
        bot.sendMessage(msg.chat_id, "✅ Mod: MANUAL", "");

      }else if(cmd=="/start"){
        if(g_mode==MODE_AUTO){ bot.sendMessage(msg.chat_id,"⚠️ AUTO modda. Önce /manual",""); }
        else if(isGenRunningNow()){ bot.sendMessage(msg.chat_id,"✅ Jeneratör zaten çalışıyor. Start yok.",""); }
        else if(g_startSeq.active || g_stopSeq.active){ bot.sendMessage(msg.chat_id,"⏳ İşlem aktif.",""); }
        else { startSeqBegin(true); bot.sendMessage(msg.chat_id,"🟡 MANUAL: Start başladı.",""); }

      }else if(cmd=="/stop"){
        if(g_mode==MODE_AUTO){ bot.sendMessage(msg.chat_id,"⚠️ AUTO modda. Önce /manual",""); }
        else if(g_startSeq.active || g_stopSeq.active){ bot.sendMessage(msg.chat_id,"⏳ İşlem aktif.",""); }
        else if(!isGenRunningNow()){
          if(g_fuelOn){ safeSetFuel(false); bot.sendMessage(msg.chat_id,"✅ Zaten durmuş. Fuel OFF.",""); }
          else bot.sendMessage(msg.chat_id,"✅ Zaten durmuş. Stop yok.","");
        }else { stopSeqBegin(true); bot.sendMessage(msg.chat_id,"🟥 MANUAL: Stop başladı.",""); }

      }else if(cmd=="/save"){
        saveSettings(); saveHoursTotal();
        bot.sendMessage(msg.chat_id,"✅ NVS kaydedildi.","");

      }else if(cmd=="/help" || cmd=="/yardim" || cmd=="/yardım"){
        String h;
        h += "Komutlar:\n";
        h += "/durum\n/auto /manual\n/start /stop\n/save\n";
        bot.sendMessage(msg.chat_id,h,"");

      }else{
        bot.sendMessage(msg.chat_id,"Komut: /durum /auto /manual /start /stop /save /help","");
      }
    }
    numNew = bot.getUpdates(bot.last_message_received + 1);
    if(numNew<=0) break;
  }
}

// =====================
// UI (tamamen izole)  -> namespace UI
// =====================
namespace UI {

struct Btn {
  uint8_t pin=255;
  bool stable=true;
  bool lastRead=true;
  uint32_t lastChangeMs=0;
  bool pressedEvt=false;
  bool repeatEvt=false;
  uint32_t downSinceMs=0;
  uint32_t lastRepeatMs=0;
};

static Btn bUp,bDown,bLeft,bRight,bOk,bBack;

static void btnInit(Btn& b, uint8_t pin){
  b.pin=pin;
  pinMode(pin, INPUT_PULLUP);
  b.stable = digitalRead(pin);
  b.lastRead = b.stable;
  b.lastChangeMs = millis();
  b.pressedEvt=false; b.repeatEvt=false;
  b.downSinceMs=0; b.lastRepeatMs=0;
}

static void btnScanOne(Btn& b){
  b.pressedEvt=false;
  b.repeatEvt=false;

  bool r=digitalRead(b.pin);
  if(r!=b.lastRead){ b.lastRead=r; b.lastChangeMs=millis(); }

  if(millis()-b.lastChangeMs >= UI_BTN_DEBOUNCE_MS){
    if(b.stable != b.lastRead){
      b.stable = b.lastRead;
      if(b.stable==LOW){
        b.pressedEvt=true;
        b.downSinceMs=millis();
        b.lastRepeatMs=millis();
      }else{
        b.downSinceMs=0;
      }
    }
  }

  if(b.stable==LOW && b.downSinceMs!=0){
    uint32_t held=millis()-b.downSinceMs;
    uint32_t rep=(held<UI_REPEAT_ACCEL_MS) ? UI_REPEAT_MS : UI_REPEAT_FAST_MS;
    if(millis()-b.lastRepeatMs >= rep){
      b.lastRepeatMs=millis();
      b.repeatEvt=true;
    }
  }
}

static void scanAll(){
  btnScanOne(bUp); btnScanOne(bDown); btnScanOne(bLeft);
  btnScanOne(bRight); btnScanOne(bOk); btnScanOne(bBack);
}

static inline bool pressed(const Btn& b){ return b.pressedEvt; }
static inline bool repeat(const Btn& b){ return b.repeatEvt; }
static inline uint32_t heldMs(const Btn& b){ return (b.downSinceMs==0)?0:(millis()-b.downSinceMs); }

// ---------- UI screens ----------
enum class Screen : uint8_t { STATUS=0, MENU, CAT, PARAM, CONFIRM, MSG };

static Screen screen = Screen::STATUS;
static uint8_t statusPage=0;
static uint8_t menuIndex=0;

enum class Cat : uint8_t { CALIB=0, MAINS, GEN, BATT, AUTO, TIMING, FAULT, COUNT };
static Cat cat = Cat::MAINS;
static uint8_t paramIndex=0;

static float oldValue=0;
static bool  hasPending=false;
static uint8_t confirmSel=0; // 0=save 1=discard
static String msg1,msg2;
static uint32_t msgUntil=0;

static const char* catName(Cat c){
  switch(c){
    case Cat::CALIB: return "Calib";
    case Cat::MAINS: return "Mains";
    case Cat::GEN: return "Gen";
    case Cat::BATT: return "Batt";
    case Cat::AUTO: return "Auto";
    case Cat::TIMING: return "Timing";
    case Cat::FAULT: return "Fault";
    default: return "?";
  }
}

// Param model: float pointer OR getter/setter callback
typedef float (*GetFn)();
typedef void  (*SetFn)(float);

struct Param {
  const char* key;
  const char* name;
  float* ptr;     // if not null => direct float
  GetFn  getFn;   // if ptr null => use getFn/setFn
  SetFn  setFn;
  float minV,maxV,step;
  uint8_t decimals;
};

static float get_u16_genRunC(){ return (float)g_set.genRunConfirmS; }
static void  set_u16_genRunC(float v){ g_set.genRunConfirmS=(uint16_t)constrain((int)v,1,60000); }

static float get_u16_aFail(){ return (float)g_set.mainsFailConfirmS; }
static void  set_u16_aFail(float v){ g_set.mainsFailConfirmS=(uint16_t)constrain((int)v,1,60000); }

static float get_u16_aRet(){ return (float)g_set.mainsReturnConfirmS; }
static void  set_u16_aRet(float v){ g_set.mainsReturnConfirmS=(uint16_t)constrain((int)v,1,60000); }

static float get_u16_aCool(){ return (float)g_set.cooldownS; }
static void  set_u16_aCool(float v){ g_set.cooldownS=(uint16_t)constrain((int)v,5,60000); }

static float get_u16_pF(){ return (float)g_set.fuelPrimeMs; }
static void  set_u16_pF(float v){ g_set.fuelPrimeMs=(uint16_t)constrain((int)v,0,65000); }

static float get_u16_pSt(){ return (float)g_set.startPulseMs; }
static void  set_u16_pSt(float v){ g_set.startPulseMs=(uint16_t)constrain((int)v,200,65000); }

static float get_u32_gSt(){ return (float)g_set.startRetryGapMs; }
static void  set_u32_gSt(float v){ g_set.startRetryGapMs=(uint32_t)constrain((int)v,500,600000); }

static float get_u16_sSt(){ return (float)g_set.startSenseGraceMs; }
static void  set_u16_sSt(float v){ g_set.startSenseGraceMs=(uint16_t)constrain((int)v,0,65000); }

static float get_u8_aAtt(){ return (float)g_set.startMaxAttempts; }
static void  set_u8_aAtt(float v){ g_set.startMaxAttempts=(uint8_t)constrain((int)v,1,20); }

static float get_u16_pSp(){ return (float)g_set.stopPulseMs; }
static void  set_u16_pSp(float v){ g_set.stopPulseMs=(uint16_t)constrain((int)v,200,65000); }

static float get_u16_vSp(){ return (float)g_set.stopVerifyS; }
static void  set_u16_vSp(float v){ g_set.stopVerifyS=(uint16_t)constrain((int)v,1,120); }

static float get_u8_mSp(){ return (float)g_set.stopMaxAttempts; }
static void  set_u8_mSp(float v){ g_set.stopMaxAttempts=(uint8_t)constrain((int)v,1,20); }

static float get_u16_dF(){ return (float)g_set.fuelOffDelayMs; }
static void  set_u16_dF(float v){ g_set.fuelOffDelayMs=(uint16_t)constrain((int)v,0,65000); }

static float get_u8_fMax(){ return (float)g_set.faultMaxRetries; }
static void  set_u8_fMax(float v){ g_set.faultMaxRetries=(uint8_t)constrain((int)v,0,20); }

static float get_u16_fBase(){ return (float)g_set.faultRetryBaseS; }
static void  set_u16_fBase(float v){ g_set.faultRetryBaseS=(uint16_t)constrain((int)v,10,65000); }

static float get_u16_fCap(){ return (float)g_set.faultRetryMaxS; }
static void  set_u16_fCap(float v){ g_set.faultRetryMaxS=(uint16_t)constrain((int)v,10,65000); }

// param arrays
static Param P_CALIB[] = {
  {"calMains","Cal Mains",&g_set.calMains,nullptr,nullptr, 10,600,1,1},
  {"calGen","Cal Gen",&g_set.calGen,nullptr,nullptr, 10,600,1,1},
  {"genDiv","Gen Div",&g_set.genBattDiv,nullptr,nullptr, 1.0f,20.0f,0.01f,3},
  {"camDiv","Cam Div",&g_set.camBattDiv,nullptr,nullptr, 1.0f,20.0f,0.01f,3},
};

static Param P_MAINS[] = {
  {"mHi","Mains High",&g_set.mainsHigh,nullptr,nullptr, 180,280,1,1},
  {"mNmn","Mains Norm Min",&g_set.mainsNormMin,nullptr,nullptr, 150,260,1,1},
  {"mNmx","Mains Norm Max",&g_set.mainsNormMax,nullptr,nullptr, 150,270,1,1},
  {"mLo","Mains Low",&g_set.mainsLow,nullptr,nullptr, 100,240,1,1},
  {"mCr","Mains Crit",&g_set.mainsCrit,nullptr,nullptr, 50,210,1,1},
  {"hAc","Hyst AC",&g_set.hystAc,nullptr,nullptr, 0,30,0.5f,1},
};

static Param P_GEN[] = {
  {"gOff","Gen OFF",&g_set.genOff,nullptr,nullptr, 0,120,1,1},
  {"gLo","Gen Low",&g_set.genLow,nullptr,nullptr, 100,240,1,1},
  {"gNmn","Gen Norm Min",&g_set.genNormMin,nullptr,nullptr, 150,260,1,1},
  {"gNmx","Gen Norm Max",&g_set.genNormMax,nullptr,nullptr, 180,280,1,1},
  {"gRunV","Gen RunningV",&g_set.genRunningV,nullptr,nullptr, 50,220,1,1},
  {"gRunC","Gen Run Conf",nullptr,get_u16_genRunC,set_u16_genRunC, 1,120,1,0},
};

static Param P_BATT[] = {
  {"bHi","Batt High",&g_set.battHigh,nullptr,nullptr, 11.0f,15.0f,0.05f,2},
  {"bNmn","Batt Norm Min",&g_set.battNormMin,nullptr,nullptr, 10.0f,14.0f,0.05f,2},
  {"bLo","Batt Low",&g_set.battLow,nullptr,nullptr, 9.0f,13.5f,0.05f,2},
  {"bCr","Batt Crit",&g_set.battCrit,nullptr,nullptr, 8.5f,13.0f,0.05f,2},
  {"hBt","Hyst Batt",&g_set.hystBatt,nullptr,nullptr, 0.00f,1.00f,0.01f,2},
};

static Param P_AUTO[] = {
  {"aMnsV","AutoStartV",&g_set.autoStartMainsV,nullptr,nullptr, 50,230,1,1},
  {"aFail","Fail Confirm",nullptr,get_u16_aFail,set_u16_aFail, 1,180,1,0},
  {"aRet","Return Confirm",nullptr,get_u16_aRet,set_u16_aRet, 1,240,1,0},
  {"aCool","CooldownS",nullptr,get_u16_aCool,set_u16_aCool, 5,3600,5,0},
};

static Param P_TIMING[] = {
  {"pF","Fuel Prime ms",nullptr,get_u16_pF,set_u16_pF, 0,5000,50,0},
  {"pSt","Start Pulse ms",nullptr,get_u16_pSt,set_u16_pSt, 200,4000,50,0},
  {"gSt","Retry Gap ms",nullptr,get_u32_gSt,set_u32_gSt, 500,15000,100,0},
  {"sSt","Sense Grace ms",nullptr,get_u16_sSt,set_u16_sSt, 0,8000,50,0},
  {"aAtt","Max Attempts",nullptr,get_u8_aAtt,set_u8_aAtt, 1,10,1,0},

  {"pSp","Stop Pulse ms",nullptr,get_u16_pSp,set_u16_pSp, 200,4000,50,0},
  {"vSp","Stop Verify s",nullptr,get_u16_vSp,set_u16_vSp, 1,60,1,0},
  {"mSp","Stop Attempts",nullptr,get_u8_mSp,set_u8_mSp, 1,10,1,0},
  {"dF","FuelOffDelay ms",nullptr,get_u16_dF,set_u16_dF, 0,5000,50,0},
};

static Param P_FAULT[] = {
  {"fMax","Fault MaxRetry",nullptr,get_u8_fMax,set_u8_fMax, 0,10,1,0},
  {"fBase","Fault Base s",nullptr,get_u16_fBase,set_u16_fBase, 10,3600,10,0},
  {"fCap","Fault Cap s",nullptr,get_u16_fCap,set_u16_fCap, 10,7200,10,0},
};

struct CatList { Param* list; uint8_t count; };

static CatList getCatList(Cat c){
  switch(c){
    case Cat::CALIB:  return { P_CALIB,  (uint8_t)(sizeof(P_CALIB)/sizeof(P_CALIB[0])) };
    case Cat::MAINS:  return { P_MAINS,  (uint8_t)(sizeof(P_MAINS)/sizeof(P_MAINS[0])) };
    case Cat::GEN:    return { P_GEN,    (uint8_t)(sizeof(P_GEN)/sizeof(P_GEN[0])) };
    case Cat::BATT:   return { P_BATT,   (uint8_t)(sizeof(P_BATT)/sizeof(P_BATT[0])) };
    case Cat::AUTO:   return { P_AUTO,   (uint8_t)(sizeof(P_AUTO)/sizeof(P_AUTO[0])) };
    case Cat::TIMING: return { P_TIMING, (uint8_t)(sizeof(P_TIMING)/sizeof(P_TIMING[0])) };
    case Cat::FAULT:  return { P_FAULT,  (uint8_t)(sizeof(P_FAULT)/sizeof(P_FAULT[0])) };
    default:          return { nullptr, 0 };
  }
}

static Param* currentParam(){
  CatList cl=getCatList(cat);
  if(!cl.list || cl.count==0) return nullptr;
  if(paramIndex>=cl.count) paramIndex=0;
  return &cl.list[paramIndex];
}

static float getValue(const Param& p){
  if(p.ptr) return *p.ptr;
  if(p.getFn) return p.getFn();
  return 0;
}
static void setValue(const Param& p, float v){
  if(p.ptr) *p.ptr=v;
  else if(p.setFn) p.setFn(v);
}

static String valueText(const Param& p, float v){
  char b[24];
  dtostrf(v, 0, p.decimals, b);
  return String(b);
}

static float stepAccel(const Param& p, uint32_t held){
  if(held>3000) return p.step*20.0f;
  if(held>1000) return p.step*5.0f;
  return p.step;
}

// ---------- drawing ----------
static void header(const String& title, const String& right=""){
  tft.fillRect(0,0,240,28, ILI9341_NAVY);
  tft.setTextColor(ILI9341_WHITE, ILI9341_NAVY);
  tft.setTextSize(2);
  tft.setCursor(6,6);
  tft.print(title);
  if(right.length()){
    tft.setCursor(240-6-(right.length()*12),6);
    tft.print(right);
  }
}
static void line(int y, const String& s, uint16_t c=ILI9341_WHITE, uint8_t size=2){
  tft.fillRect(0,y,240,20, ILI9341_BLACK);
  tft.setTextColor(c, ILI9341_BLACK);
  tft.setTextSize(size);
  tft.setCursor(6,y);
  tft.print(s);
}

static void drawStatus(bool full){
  if(full) tft.fillScreen(ILI9341_BLACK);
  header("STATUS", String(modeText(g_mode))+" "+String(PROJECT_VERSION));

  if(sensorsAnyInvalid()){
    line(40,"SENSÖR?", ILI9341_YELLOW,3);
    line(76,"Olcum gecersiz", ILI9341_LIGHTGREY,2);
    line(100,"Kablo/ADC?", ILI9341_LIGHTGREY,2);
    return;
  }

  if(statusPage==0){
    line(34,"Mains: "+fmt2(g_meas.mainsV)+"V");
    line(56,"Gen  : "+fmt2(g_meas.genV)+"V");
    line(78,"GenB : "+fmt2(g_meas.genBattV)+"V");
    line(100,"CamB : "+fmt2(g_meas.camBattV)+"V");
    line(122,"Auto : "+String(autoStateText(g_autoState)));
    line(144,String("Fuel : ")+(g_fuelOn?"ON":"OFF"));
  }else{
    line(34,mainsStateLine(g_mainsState,g_meas.mainsV), ILI9341_CYAN,2);
    line(56,genStateLine(g_genState,g_meas.genV), ILI9341_CYAN,2);
    line(78,"RSSI: "+String(g_meas.wifiRssi));
    line(100,"Uptime: "+fmtDurTR(g_meas.uptimeS));
    line(122,"Hours: "+fmtHMS(g_genRunTotalS));
  }

  line(200,"L/R:Sayfa  OK:Menu", ILI9341_DARKGREY,2);
}

static void drawMenu(bool full){
  if(full) tft.fillScreen(ILI9341_BLACK);
  header("MENU","");

  const char* items[]={
    "1) Status",
    "2) Mod AUTO/MAN",
    "3) Ayarlar",
    "4) NVS Kaydet",
    "5) Manual START",
    "6) Manual STOP"
  };
  const uint8_t n=sizeof(items)/sizeof(items[0]);
  for(uint8_t i=0;i<n;i++){
    uint16_t c=(i==menuIndex)?ILI9341_YELLOW:ILI9341_WHITE;
    line(40+i*22, String(items[i]), c,2);
  }
  line(200,"UP/DN:Sec OK:Onay", ILI9341_DARKGREY,2);
  line(220,"BACK:Geri", ILI9341_DARKGREY,2);
}

static void drawCat(bool full){
  if(full) tft.fillScreen(ILI9341_BLACK);
  header("AYAR KAT","");

  for(uint8_t i=0;i<(uint8_t)Cat::COUNT;i++){
    uint16_t c=(i==(uint8_t)cat)?ILI9341_YELLOW:ILI9341_WHITE;
    line(40+i*22, String(catName((Cat)i)), c,2);
  }
  line(200,"UP/DN:Sec OK:Gir", ILI9341_DARKGREY,2);
  line(220,"BACK:Geri", ILI9341_DARKGREY,2);
}

static void drawParam(bool full){
  if(full) tft.fillScreen(ILI9341_BLACK);
  CatList cl=getCatList(cat);
  header("AYAR", String(catName(cat))+" "+String(paramIndex+1)+"/"+String(cl.count));

  Param* p=currentParam();
  if(!p){ line(60,"Param yok", ILI9341_RED,2); return; }

  float v=getValue(*p);

  line(40, String(p->name), ILI9341_CYAN,2);
  line(70, "Deger: "+valueText(*p,v), ILI9341_WHITE,3);

  if(hasPending){
    line(110,"Degisti!", ILI9341_YELLOW,2);
    line(132,"Eski: "+valueText(*p,oldValue), ILI9341_LIGHTGREY,2);
  }else{
    line(110," ", ILI9341_BLACK,2);
    line(132," ", ILI9341_BLACK,2);
  }

  line(180,"UP/DN:+/-  L/R:Param", ILI9341_DARKGREY,2);
  line(200,"OK:Kaydet? BACK:Geri", ILI9341_DARKGREY,2);
}

static void drawConfirm(bool full){
  if(full) tft.fillScreen(ILI9341_BLACK);
  header("KAYIT?","");

  Param* p=currentParam();
  if(!p) return;

  float v=getValue(*p);
  line(40, String(p->name), ILI9341_CYAN,2);
  line(70, "Yeni: "+valueText(*p,v), ILI9341_WHITE,3);

  String a=(confirmSel==0)?"> Kaydet":"  Kaydet";
  String b=(confirmSel==1)?"> Kaydetme":"  Kaydetme";
  line(130,a, (confirmSel==0)?ILI9341_YELLOW:ILI9341_WHITE,2);
  line(152,b, (confirmSel==1)?ILI9341_YELLOW:ILI9341_WHITE,2);

  line(200,"UP/DN:Sec OK:Onay", ILI9341_DARKGREY,2);
  line(220,"BACK:Geri", ILI9341_DARKGREY,2);
}

static void drawMsg(){
  tft.fillRect(0,140,240,80, ILI9341_DARKGREEN);
  tft.setTextColor(ILI9341_WHITE, ILI9341_DARKGREEN);
  tft.setTextSize(2);
  tft.setCursor(6,150); tft.print(msg1);
  tft.setCursor(6,172); tft.print(msg2);
}

static void enter(Screen s){
  screen=s;
  switch(screen){
    case Screen::STATUS: drawStatus(true); break;
    case Screen::MENU: drawMenu(true); break;
    case Screen::CAT: drawCat(true); break;
    case Screen::PARAM: drawParam(true); break;
    case Screen::CONFIRM: drawConfirm(true); break;
    case Screen::MSG: drawMsg(); break;
  }
}

static void toast(const String& a, const String& b, uint32_t ms=1200){
  msg1=a; msg2=b;
  msgUntil=millis()+ms;
  enter(Screen::MSG);
}

static void beginUI(){
  btnInit(bUp, PIN_BTN_UP);
  btnInit(bDown, PIN_BTN_DOWN);
  btnInit(bLeft, PIN_BTN_LEFT);
  btnInit(bRight, PIN_BTN_RIGHT);
  btnInit(bOk, PIN_BTN_OK);
  btnInit(bBack, PIN_BTN_BACK);

  enter(Screen::STATUS);
}

static void service(){
  scanAll();

  if(screen==Screen::MSG){
    if((int32_t)(millis()-msgUntil)>=0) enter(Screen::STATUS);
    return;
  }

  if(screen==Screen::STATUS){
    if(pressed(bLeft)){ statusPage = (statusPage+1)%2; drawStatus(true); }
    if(pressed(bRight)){ statusPage = (statusPage+1)%2; drawStatus(true); }
    if(pressed(bOk)) enter(Screen::MENU);
    return;
  }

  if(screen==Screen::MENU){
    const uint8_t maxIdx=5;
    if(pressed(bUp)){ if(menuIndex>0) menuIndex--; drawMenu(true); }
    if(pressed(bDown)){ if(menuIndex<maxIdx) menuIndex++; drawMenu(true); }
    if(pressed(bBack)){ enter(Screen::STATUS); return; }

    if(pressed(bOk)){
      switch(menuIndex){
        case 0: enter(Screen::STATUS); break;

        case 1:
          g_mode = (g_mode==MODE_AUTO)?MODE_MANUAL:MODE_AUTO;
          saveSettings();
          toast("Mod degisti", String(modeText(g_mode)));
          break;

        case 2:
          enter(Screen::CAT);
          break;

        case 3:
          saveSettings(); saveHoursTotal();
          toast("NVS","Kaydedildi");
          if(WiFi.status()==WL_CONNECTED) notify("💾 TFT: Ayarlar + sayaç NVS'ye kaydedildi.");
          break;

        case 4:
          if(g_mode==MODE_AUTO){ toast("AUTO mod","/manual"); break; }
          if(isGenRunningNow()){ toast("Zaten calisiyor","Start yok"); break; }
          if(g_startSeq.active || g_stopSeq.active){ toast("Islem aktif","Bekle"); break; }
          startSeqBegin(true);
          toast("MANUAL","Start basladi");
          break;

        case 5:
          if(g_mode==MODE_AUTO){ toast("AUTO mod","/manual"); break; }
          if(g_startSeq.active || g_stopSeq.active){ toast("Islem aktif","Bekle"); break; }
          if(!isGenRunningNow()){
            if(g_fuelOn){ safeSetFuel(false); toast("Zaten durmus","Fuel OFF"); }
            else toast("Zaten durmus","Stop yok");
            break;
          }
          stopSeqBegin(true);
          toast("MANUAL","Stop basladi");
          break;
      }
    }
    return;
  }

  if(screen==Screen::CAT){
    if(pressed(bUp)){
      if((uint8_t)cat>0) cat=(Cat)((uint8_t)cat-1);
      drawCat(true);
    }
    if(pressed(bDown)){
      if((uint8_t)cat < (uint8_t)Cat::COUNT-1) cat=(Cat)((uint8_t)cat+1);
      drawCat(true);
    }
    if(pressed(bBack)){ enter(Screen::MENU); return; }
    if(pressed(bOk)){
      paramIndex=0; hasPending=false; confirmSel=0;
      enter(Screen::PARAM);
    }
    return;
  }

  if(screen==Screen::PARAM){
    Param* p=currentParam();
    if(!p){ if(pressed(bBack)) enter(Screen::CAT); return; }

    if(pressed(bBack)){ enter(Screen::CAT); return; }

    if(pressed(bLeft)){
      if(paramIndex>0) paramIndex--;
      hasPending=false;
      drawParam(true);
    }
    if(pressed(bRight)){
      CatList cl=getCatList(cat);
      if(paramIndex+1<cl.count) paramIndex++;
      hasPending=false;
      drawParam(true);
    }

    bool inc = pressed(bUp) || repeat(bUp);
    bool dec = pressed(bDown) || repeat(bDown);

    if(inc || dec){
      uint32_t held = inc ? heldMs(bUp) : heldMs(bDown);
      float step = stepAccel(*p, held);

      float v=getValue(*p);
      if(!hasPending){ oldValue=v; hasPending=true; }

      v += inc ? step : -step;
      v = constrain(v, p->minV, p->maxV);
      setValue(*p, v);

      drawParam(true);
    }

    if(pressed(bOk)){
      confirmSel=0;
      enter(Screen::CONFIRM);
    }
    return;
  }

  if(screen==Screen::CONFIRM){
    if(pressed(bBack)){ enter(Screen::PARAM); return; }

    if(pressed(bUp) || pressed(bDown)){
      confirmSel = (confirmSel==0)?1:0;
      drawConfirm(true);
    }

    if(pressed(bOk)){
      Param* p=currentParam();
      if(!p){ enter(Screen::PARAM); return; }

      if(confirmSel==0){
        // SAVE
        saveSettings();
        float v=getValue(*p);
        if(WiFi.status()==WL_CONNECTED){
          notify("⚙️ TFT Ayar değişti: " + String(p->name) + " = " + valueText(*p,v) + " (kaydedildi)");
        }
        hasPending=false;
        toast("Kaydedildi", String(p->name));
      }else{
        // DISCARD => revert
        setValue(*p, oldValue);
        float v=getValue(*p);
        if(WiFi.status()==WL_CONNECTED){
          notify("⚙️ TFT Ayar değişti: " + String(p->name) + " = " + valueText(*p,v) + " (kaydedilmedi)");
        }
        hasPending=false;
        toast("Iptal", String(p->name));
      }
    }
    return;
  }
}

} // namespace UI

// =====================
// Setup / Loop
// =====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  analogReadResolution(12);

  loadSettings();
  relayInit();

  // TFT init
  tft.begin();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(ILI9341_BLACK);

  // UI init
  UI::beginUI();

  // WiFi + TG
  connectWiFi();
  tgClient.setInsecure();

  // first measurements
  readAllMeasurements();
  delay(250);
  readAllMeasurements();
  updateStates();

  // Boot'ta rapor yok. Sadece "Hazır"
  if (WiFi.status() == WL_CONNECTED) notify("✅ Hazır");

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
      if (WiFi.status() == WL_CONNECTED) notify("✅ Hazır");
    }
  }

  relayPulseService();
  startSeqService();
  stopSeqService();

  // UI
  UI::service();

  uint32_t now = millis();

  if (now - tMeasure >= MEASURE_MS) {
    tMeasure = now;

    readAllMeasurements();
    updateStates();

    handleStateAlerts();
    updateGenHoursCounter_1s();
    autoTick_1s();

    if (UI::screen == UI::Screen::STATUS) UI::drawStatus(false);
  }

  if (now - tSerial >= SERIAL_REPORT_MS) {
    tSerial = now;
    Serial.print("["); Serial.print(PROJECT_VERSION); Serial.print("] ");
    Serial.print("Mode="); Serial.print(modeText(g_mode));
    Serial.print(" Auto="); Serial.print(autoStateText(g_autoState));
    Serial.print(" Fuel="); Serial.print(g_fuelOn ? "ON" : "OFF");
    Serial.print(" Valid="); Serial.print(sensorsAnyInvalid() ? "NO" : "YES");
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

  delay(5);
  yield();
}
