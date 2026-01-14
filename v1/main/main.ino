// =====================
// SÜRÜM v4.001
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

  // MAINS thresholds
  float mainsHigh, mainsNormMin, mainsNormMax, mainsLow, mainsCrit;

  // GEN thresholds
  float genOff, genLow, genNormMin, genNormMax;

  // BATT thresholds
  float battHigh, battNormMin, battLow, battCrit;

  // hysteresis
  float hystAc;
  float hystBatt;
} g_set;

struct Measurements {
  float mainsV_raw, genV_raw, genBattV_raw, camBattV_raw;
  float mainsV, genV, genBattV, camBattV;
  int wifiRssi;
  uint32_t uptimeS;
} g_meas;

static uint32_t tMeasure = 0, tSerial = 0, tTgPoll = 0;
static bool lastBtn = true;
static uint32_t btnDownMs = 0;

// ---------------------
// State Machines (LOW/HIGH macro çakışmasın diye _V)
// ---------------------
enum class MainsState : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };
enum class GenState   : uint8_t { UNKNOWN, OFF, LOW_V, NORMAL, HIGH_V };
enum class BattState  : uint8_t { UNKNOWN, CRITICAL, LOW_V, NORMAL, HIGH_V };

static MainsState g_mainsState = MainsState::UNKNOWN;
static GenState   g_genState   = GenState::UNKNOWN;
static BattState  g_genBattState = BattState::UNKNOWN;
static BattState  g_camBattState = BattState::UNKNOWN;

static bool g_mainsLatched = false;
static bool g_genLatched = false;
static bool g_genBattLatched = false;
static bool g_camBattLatched = false;

// =====================
// Helpers
// =====================
static String fmt2(float v) {
  if (isnan(v) || isinf(v)) return "nan";
  char b[16];
  dtostrf(v, 0, 2, b);
  return String(b);
}

static float lpf(float prev, float x, float a) {
  if (isnan(prev) || isinf(prev)) return x;
  return prev + a * (x - prev);
}

static void notify(const String& msg) {
  if (WiFi.status() == WL_CONNECTED) bot.sendMessage(CHAT_ID, msg, "");
}

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

  g_mode = (RunMode)prefs.getUChar("mode", (uint8_t)MODE_MANUAL);
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

  prefs.putUChar("mode", (uint8_t)g_mode);
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

// UniversalTelegramBot struct adı telegramMessage
static bool isAuthorized(const telegramMessage& msg) {
  long fromId = msg.from_id.toInt();
  return (fromId == MASTER_ADMIN_ID);
}

// ---------------------
// State Text
// ---------------------
static String battStateToText(BattState st) {
  switch (st) {
    case BattState::CRITICAL: return "CRITICAL";
    case BattState::LOW_V:    return "LOW";
    case BattState::NORMAL:   return "NORMAL";
    case BattState::HIGH_V:   return "HIGH";
    default:                  return "UNKNOWN";
  }
}

// ---------------------
// Battery State Eval (histerezisli)
// ---------------------
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

static void handleBatteryNotifications() {
  // GEN batt
  BattState newGB = evalBatt(g_meas.genBattV, g_genBattState);
  if (newGB != g_genBattState) { g_genBattState = newGB; g_genBattLatched = false; }
  if (!g_genBattLatched) {
    g_genBattLatched = true;
    if (g_genBattState == BattState::CRITICAL) notify("🚨 Gen Akü KRİTİK: " + fmt2(g_meas.genBattV) + "V");
    else if (g_genBattState == BattState::LOW_V) notify("⚠️ Gen Akü DÜŞÜK: " + fmt2(g_meas.genBattV) + "V");
    else if (g_genBattState == BattState::HIGH_V) notify("⚠️ Gen Akü YÜKSEK: " + fmt2(g_meas.genBattV) + "V");
    else if (g_genBattState == BattState::NORMAL) notify("✅ Gen Akü NORMAL: " + fmt2(g_meas.genBattV) + "V");
  }

  // CAM batt
  BattState newCB = evalBatt(g_meas.camBattV, g_camBattState);
  if (newCB != g_camBattState) { g_camBattState = newCB; g_camBattLatched = false; }
  if (!g_camBattLatched) {
    g_camBattLatched = true;
    if (g_camBattState == BattState::CRITICAL) notify("🚨 Cam Akü KRİTİK: " + fmt2(g_meas.camBattV) + "V");
    else if (g_camBattState == BattState::LOW_V) notify("⚠️ Cam Akü DÜŞÜK: " + fmt2(g_meas.camBattV) + "V");
    else if (g_camBattState == BattState::HIGH_V) notify("⚠️ Cam Akü YÜKSEK: " + fmt2(g_meas.camBattV) + "V");
    else if (g_camBattState == BattState::NORMAL) notify("✅ Cam Akü NORMAL: " + fmt2(g_meas.camBattV) + "V");
  }
}

// ---------------------
// Telegram (safe loop)
// ---------------------
static String buildStatusText() {
  String s;
  s += "📌 Köy Jeneratör Proje-3\n";
  s += String("🔖 Sürüm: ") + PROJECT_VERSION + "\n";
  s += String("⏱ Uptime: ") + String(g_meas.uptimeS) + " sn\n";
  s += String("📶 RSSI: ") + String(g_meas.wifiRssi) + " dBm\n\n";
  s += "🔋 Gen Akü: " + fmt2(g_meas.genBattV) + " V (" + battStateToText(g_genBattState) + ")\n";
  s += "🔋 Cam Akü: " + fmt2(g_meas.camBattV) + " V (" + battStateToText(g_camBattState) + ")\n";
  return s;
}

static void handleTelegram() {
  int numNew = bot.getUpdates(bot.last_message_received + 1);
  if (numNew <= 0) return;

  while (numNew > 0) {
    for (int i = 0; i < numNew; i++) {
      telegramMessage& msg = bot.messages[i];
      if (!isAuthorized(msg)) continue;

      String text = msg.text;
      text.trim();

      if (text == "/durum" || text == "/status") {
        bot.sendMessage(msg.chat_id, buildStatusText(), "");
      } else if (text == "/save") {
        saveSettings();
        bot.sendMessage(msg.chat_id, "✅ Ayarlar NVS'ye kaydedildi.", "");
      } else {
        bot.sendMessage(msg.chat_id, "Komut: /durum /save", "");
      }
    }

    numNew = bot.getUpdates(bot.last_message_received + 1);
    if (numNew <= 0) break;
  }
}

// ---------------------
// Measure
// ---------------------
static void readAllMeasurements() {
  g_meas.wifiRssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -999;
  g_meas.uptimeS  = millis() / 1000;

  float vGenAdc = readAdcVoltage(PIN_ADC_GEN_BATT);
  float vCamAdc = readAdcVoltage(PIN_ADC_CAM_BATT);

  g_meas.genBattV_raw = vGenAdc * g_set.genBattDiv;
  g_meas.camBattV_raw = vCamAdc * g_set.camBattDiv;

  g_meas.genBattV = lpf(g_meas.genBattV, g_meas.genBattV_raw, LPF_ALPHA_BATT);
  g_meas.camBattV = lpf(g_meas.camBattV, g_meas.camBattV_raw, LPF_ALPHA_BATT);
}

// ---------------------
// Save Button
// ---------------------
static void handleSaveButton() {
  bool btn = digitalRead(PIN_BTN_SAVE); // INPUT_PULLUP
  uint32_t now = millis();

  if (lastBtn == true && btn == false) btnDownMs = now;

  if (lastBtn == false && btn == true) {
    uint32_t held = now - btnDownMs;
    if (held >= 800) {
      saveSettings();
      notify("💾 Buton: Ayarlar NVS'ye kaydedildi.");
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
  connectWiFi();
  tgClient.setInsecure();

  if (WiFi.status() == WL_CONNECTED) {
    notify(String("✅ Sistem açıldı. Sürüm: ") + PROJECT_VERSION);
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

  handleSaveButton();

  uint32_t now = millis();

  if (now - tMeasure >= MEASURE_MS) {
    tMeasure = now;
    readAllMeasurements();
    handleBatteryNotifications();
  }

  if (now - tSerial >= SERIAL_REPORT_MS) {
    tSerial = now;
    Serial.print("["); Serial.print(PROJECT_VERSION); Serial.print("] ");
    Serial.print("GenBatt="); Serial.print(fmt2(g_meas.genBattV));
    Serial.print(" ("); Serial.print(battStateToText(g_genBattState)); Serial.print(")");
    Serial.print(" CamBatt="); Serial.print(fmt2(g_meas.camBattV));
    Serial.print(" ("); Serial.print(battStateToText(g_camBattState)); Serial.print(")");
    Serial.print(" RSSI="); Serial.print(g_meas.wifiRssi);
    Serial.println("dBm");
  }

  if (now - tTgPoll >= TG_POLL_MS) {
    tTgPoll = now;
    if (WiFi.status() == WL_CONNECTED) handleTelegram();
  }

  delay(5);
  yield();
}
