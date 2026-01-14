// =====================
// SÜRÜM v8.004
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

  float genRunningV;
  uint16_t genRunConfirmS;
  uint32_t hoursSavePeriodS;
} g_set;

struct Measurements {
  float mainsV_raw, genV_raw, genBattV_raw, camBattV_raw;
  float mainsV, genV, genBattV, camBattV;
  int wifiRssi;
  uint32_t uptimeS;
} g_meas;

static uint32_t tMeasure = 0, tSerial = 0, tTgPoll = 0;

// ---------------------
// Battery State (no telegram spam)
// ---------------------
enum class BattState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
static BattState g_genBattState = BattState::UNKNOWN;
static BattState g_camBattState = BattState::UNKNOWN;

// ---------------------
// MAINS / GEN state alerts
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

// Komut normalize: "/durum@bot arg" -> "/durum"
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

static bool tgSendTo(const String& chatId, const String& msg) {
  bool ok = false;
  if (WiFi.status() == WL_CONNECTED) {
    ok = bot.sendMessage(chatId, msg, "");
  }
  Serial.print("[TG] send to "); Serial.print(chatId);
  Serial.print(" => "); Serial.println(ok ? "OK" : "FAIL");
  return ok;
}

static void tgSendBootBoth(const String& msg) {
  // 1) Grup CHAT_ID
  tgSendTo(String(CHAT_ID), msg);

  // 2) Admin'a özel mesaj denemesi
  String adminChat = String(MASTER_ADMIN_ID);
  if (adminChat != String(CHAT_ID)) {
    tgSendTo(adminChat, msg);
  }
}

// =====================
// WiFi
// =====================
static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("[WiFi] Connecting to "); Serial.println(WIFI_SSID);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    Serial.print(".");
    yield();
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] OK IP="); Serial.println(WiFi.localIP());
    Serial.print("[WiFi] RSSI="); Serial.println(WiFi.RSSI());
  } else {
    Serial.println("[WiFi] FAIL (no connection)");
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

  g_set.genRunningV      = prefs.getFloat("gRunV", GEN_RUNNING_V);
  g_set.genRunConfirmS   = prefs.getUShort("gRunC", GEN_RUNNING_CONFIRM_S);
  g_set.hoursSavePeriodS = prefs.getUInt("hSaveP", HOURS_SAVE_PERIOD_S);

  g_mode = (RunMode)prefs.getUChar("mode", (uint8_t)MODE_MANUAL);

  uint32_t lo = prefs.getUInt("hrsLo", 0);
  uint32_t hi = prefs.getUInt("hrsHi", 0);
  g_genRunTotalS = ((uint64_t)hi << 32) | (uint64_t)lo;
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
// Battery state
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
// MAINS/GEN state + alerts (minimal for debug)
// =====================
static String buildBootReport() {
  String s;
  s += String(DEVICE_NAME) + "\n";
  s += "🔖 Sürüm: " + String(PROJECT_VERSION) + "\n";
  s += "🔌 Şebeke: " + fmt2(g_meas.mainsV) + " V\n";
  s += "🟠 Jeneratör: " + fmt2(g_meas.genV) + " V\n";
  s += "🔋 Gen Akü: " + fmt2(g_meas.genBattV) + " V (" + battStateToText(g_genBattState) + ")\n";
  s += "🔋 Cam Akü: " + fmt2(g_meas.camBattV) + " V (" + battStateToText(g_camBattState) + ")\n";
  s += "⏱ Çalışma Süresi: " + fmtHMS(g_genRunTotalS) + "\n";
  return s;
}

static void handleTelegram() {
  int numNew = bot.getUpdates(bot.last_message_received + 1);
  Serial.print("[TG] getUpdates => "); Serial.println(numNew);

  if (numNew <= 0) return;

  while (numNew > 0) {
    for (int i = 0; i < numNew; i++) {
      telegramMessage& msg = bot.messages[i];

      Serial.print("[TG] from_id="); Serial.print(msg.from_id);
      Serial.print(" chat_id="); Serial.print(msg.chat_id);
      Serial.print(" text="); Serial.println(msg.text);

      String cmd = normalizeCommand(msg.text);

      if (cmd == "/ping") {
        tgSendTo(msg.chat_id, "pong ✅");
      } else if (cmd == "/myid") {
        String r = "chat_id: " + msg.chat_id + "\nfrom_id: " + msg.from_id + "\ntext: " + msg.text;
        tgSendTo(msg.chat_id, r);
      } else if (cmd == "/durum" || cmd == "/status") {
        tgSendTo(msg.chat_id, buildBootReport());
      } else {
        tgSendTo(msg.chat_id, "Komut: /durum /ping /myid");
      }
    }

    numNew = bot.getUpdates(bot.last_message_received + 1);
    Serial.print("[TG] getUpdates (loop) => "); Serial.println(numNew);
    if (numNew <= 0) break;
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println();
  Serial.println("=== BOOT ===");
  Serial.print("Version: "); Serial.println(PROJECT_VERSION);

  analogReadResolution(12);

  loadSettings();

  // TLS
  tgClient.setInsecure();

  connectWiFi();

  // ölçüm 2 kez
  readAllMeasurements();
  delay(250);
  readAllMeasurements();

  updateBatteryStatesOnly();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[TG] Boot message sending...");
    tgSendBootBoth(buildBootReport());
    tgSendBootBoth("ℹ️ Bot hazır. Test: /ping  | /myid");
  } else {
    Serial.println("[TG] skipped (WiFi not connected)");
  }

  tMeasure = millis();
  tSerial  = millis();
  tTgPoll  = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t tRetry = 0;
    if (millis() - tRetry > 5000) {
      tRetry = millis();
      connectWiFi();
    }
  }

  uint32_t now = millis();

  if (now - tMeasure >= MEASURE_MS) {
    tMeasure = now;
    readAllMeasurements();
    updateBatteryStatesOnly();
  }

  if (now - tSerial >= SERIAL_REPORT_MS) {
    tSerial = now;
    Serial.print("["); Serial.print(PROJECT_VERSION); Serial.print("] ");
    Serial.print("WiFi="); Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "FAIL");
    Serial.print(" IP="); Serial.print(WiFi.localIP());
    Serial.print(" Mains="); Serial.print(fmt2(g_meas.mainsV));
    Serial.print(" Gen="); Serial.print(fmt2(g_meas.genV));
    Serial.print(" GenBatt="); Serial.print(fmt2(g_meas.genBattV));
    Serial.print(" CamBatt="); Serial.print(fmt2(g_meas.camBattV));
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
