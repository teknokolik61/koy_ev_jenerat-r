// =====================
// SÜRÜM v2.001
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
  float calMains;
  float calGen;
  float genBattDiv;
  float camBattDiv;
} g_set;

struct Measurements {
  float mainsV_raw;
  float genV_raw;
  float genBattV_raw;
  float camBattV_raw;

  float mainsV;     // filtreli
  float genV;       // filtreli
  float genBattV;   // filtreli
  float camBattV;   // filtreli

  int   wifiRssi;
  uint32_t uptimeS;
} g_meas;

static uint32_t tMeasure = 0;
static uint32_t tSerial  = 0;
static uint32_t tTgPoll  = 0;

static bool lastBtn = true;
static uint32_t btnDownMs = 0;

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

static void loadSettings() {
  prefs.begin(NVS_NAMESPACE, false);

  g_set.calMains   = prefs.getFloat("calMains", CAL_MAINS);
  g_set.calGen     = prefs.getFloat("calGen",   CAL_GEN);
  g_set.genBattDiv = prefs.getFloat("genDiv",   GEN_BATT_DIV_RATIO);
  g_set.camBattDiv = prefs.getFloat("camDiv",   CAM_BATT_DIV_RATIO);

  g_mode = (RunMode)prefs.getUChar("mode", (uint8_t)MODE_MANUAL);
}

static void saveSettings() {
  prefs.putFloat("calMains", g_set.calMains);
  prefs.putFloat("calGen",   g_set.calGen);
  prefs.putFloat("genDiv",   g_set.genBattDiv);
  prefs.putFloat("camDiv",   g_set.camBattDiv);
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

// AC RMS ölçüm: offset + RMS (daha uzun pencere)
static float readAcRmsApprox(uint8_t pin, float calScale) {
  // mean
  uint32_t sum = 0;
  for (uint16_t i = 0; i < AC_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(AC_US_DELAY);
    yield();
  }
  float mean = (float)sum / (float)AC_SAMPLES;

  // rms
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

// UniversalTelegramBot sürümüne göre struct adı telegramMessage
static bool isAuthorized(const telegramMessage& msg) {
  long fromId = msg.from_id.toInt();
  return (fromId == MASTER_ADMIN_ID);
}

static String buildStatusText() {
  String s;
  s += "📌 Köy Jeneratör Proje-3\n";
  s += String("🔖 Sürüm: ") + PROJECT_VERSION + "\n";
  s += String("⏱ Uptime: ") + String(g_meas.uptimeS) + " sn\n";
  s += String("📶 WiFi RSSI: ") + String(g_meas.wifiRssi) + " dBm\n";
  s += String("⚙️ Mod: ") + (g_mode == MODE_AUTO ? "AUTO" : "MANUAL") + "\n\n";

  s += "🔌 Şebeke (RMS~): " + fmt2(g_meas.mainsV) + " V\n";
  s += "🟠 Jeneratör (RMS~): " + fmt2(g_meas.genV) + " V\n";
  s += "🔋 Gen Akü: " + fmt2(g_meas.genBattV) + " V\n";
  s += "🔋 Cam Akü: " + fmt2(g_meas.camBattV) + " V\n";
  return s;
}

static String helpText() {
  String h;
  h += "Komutlar (Aşama 2):\n";
  h += "/durum  -> ölçümler\n";
  h += "/auto   -> AUTO mod\n";
  h += "/manual -> MANUAL mod\n";
  h += "/save   -> ayarları NVS kaydet\n";
  h += "/setcalmains <x>\n";
  h += "/setcalgen <x>\n";
  h += "/setgendiv <x>\n";
  h += "/setcamdiv <x>\n";
  return h;
}

static void handleTelegram() {
  int numNew = bot.getUpdates(bot.last_message_received + 1);
  while (numNew) {
    for (int i = 0; i < numNew; i++) {
      telegramMessage& msg = bot.messages[i];
      if (!isAuthorized(msg)) continue;

      String text = msg.text;
      text.trim();

      if (text == "/start" || text == "/help" || text == "/yardim" || text == "/yardım") {
        bot.sendMessage(msg.chat_id, helpText(), "");
      } else if (text == "/durum" || text == "/status") {
        bot.sendMessage(msg.chat_id, buildStatusText(), "");
      } else if (text == "/auto") {
        g_mode = MODE_AUTO;
        bot.sendMessage(msg.chat_id, "✅ Mod AUTO yapıldı.", "");
      } else if (text == "/manual") {
        g_mode = MODE_MANUAL;
        bot.sendMessage(msg.chat_id, "✅ Mod MANUAL yapıldı.", "");
      } else if (text == "/save") {
        saveSettings();
        bot.sendMessage(msg.chat_id, "✅ Ayarlar NVS'ye kaydedildi.", "");
      } else if (text.startsWith("/setcalmains")) {
        float v = text.substring(String("/setcalmains").length()).toFloat();
        if (v > 0.01f) { g_set.calMains = v; bot.sendMessage(msg.chat_id, "✅ calMains = " + fmt2(v), ""); }
        else bot.sendMessage(msg.chat_id, "❌ Geçersiz.", "");
      } else if (text.startsWith("/setcalgen")) {
        float v = text.substring(String("/setcalgen").length()).toFloat();
        if (v > 0.01f) { g_set.calGen = v; bot.sendMessage(msg.chat_id, "✅ calGen = " + fmt2(v), ""); }
        else bot.sendMessage(msg.chat_id, "❌ Geçersiz.", "");
      } else if (text.startsWith("/setgendiv")) {
        float v = text.substring(String("/setgendiv").length()).toFloat();
        if (v > 1.0f) { g_set.genBattDiv = v; bot.sendMessage(msg.chat_id, "✅ genDiv = " + fmt2(v), ""); }
        else bot.sendMessage(msg.chat_id, "❌ Geçersiz.", "");
      } else if (text.startsWith("/setcamdiv")) {
        float v = text.substring(String("/setcamdiv").length()).toFloat();
        if (v > 1.0f) { g_set.camBattDiv = v; bot.sendMessage(msg.chat_id, "✅ camDiv = " + fmt2(v), ""); }
        else bot.sendMessage(msg.chat_id, "❌ Geçersiz.", "");
      } else {
        bot.sendMessage(msg.chat_id, "Komut tanınmadı. /help yaz.", "");
      }
    }
    numNew = bot.getUpdates(bot.last_message_received + 1);
  }
}

static void readAllMeasurements() {
  g_meas.wifiRssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -999;
  g_meas.uptimeS  = millis() / 1000;

  // Raw
  g_meas.mainsV_raw = readAcRmsApprox(PIN_ADC_MAINS, g_set.calMains);
  g_meas.genV_raw   = readAcRmsApprox(PIN_ADC_GEN,   g_set.calGen);

  float vGenAdc = readAdcVoltage(PIN_ADC_GEN_BATT);
  float vCamAdc = readAdcVoltage(PIN_ADC_CAM_BATT);

  g_meas.genBattV_raw = vGenAdc * g_set.genBattDiv;
  g_meas.camBattV_raw = vCamAdc * g_set.camBattDiv;

  // Filtered
  g_meas.mainsV   = lpf(g_meas.mainsV,   g_meas.mainsV_raw,   LPF_ALPHA_AC);
  g_meas.genV     = lpf(g_meas.genV,     g_meas.genV_raw,     LPF_ALPHA_AC);
  g_meas.genBattV = lpf(g_meas.genBattV, g_meas.genBattV_raw, LPF_ALPHA_BATT);
  g_meas.camBattV = lpf(g_meas.camBattV, g_meas.camBattV_raw, LPF_ALPHA_BATT);
}

static void handleSaveButton() {
  bool btn = digitalRead(PIN_BTN_SAVE); // PULLUP: basılıyken LOW
  uint32_t now = millis();

  if (lastBtn == true && btn == false) btnDownMs = now;

  if (lastBtn == false && btn == true) {
    uint32_t held = now - btnDownMs;
    if (held >= 800) {
      saveSettings();
      if (WiFi.status() == WL_CONNECTED) {
        bot.sendMessage(CHAT_ID, "💾 Buton: Ayarlar NVS'ye kaydedildi.", "");
      }
    }
  }

  lastBtn = btn;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  analogReadResolution(12);
  pinMode(PIN_BTN_SAVE, INPUT_PULLUP);

  Serial.println();
  Serial.println("=== Koy Jenerator Proje-3 ===");
  Serial.print("Version: ");
  Serial.println(PROJECT_VERSION);

  loadSettings();
  connectWiFi();

  tgClient.setInsecure();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK. IP: ");
    Serial.println(WiFi.localIP());
    bot.sendMessage(CHAT_ID, String("✅ Sistem açıldı. Sürüm: ") + PROJECT_VERSION, "");
  } else {
    Serial.println("WiFi BAGLANAMADI (15sn timeout).");
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
  }

  if (now - tSerial >= SERIAL_REPORT_MS) {
    tSerial = now;

    Serial.print("[");
    Serial.print(PROJECT_VERSION);
    Serial.print("] Mains~=");
    Serial.print(fmt2(g_meas.mainsV));
    Serial.print("V (raw ");
    Serial.print(fmt2(g_meas.mainsV_raw));
    Serial.print(")  Gen~=");
    Serial.print(fmt2(g_meas.genV));
    Serial.print("V (raw ");
    Serial.print(fmt2(g_meas.genV_raw));
    Serial.print(")  GenBatt=");
    Serial.print(fmt2(g_meas.genBattV));
    Serial.print("V  CamBatt=");
    Serial.print(fmt2(g_meas.camBattV));
    Serial.print("V  RSSI=");
    Serial.print(g_meas.wifiRssi);
    Serial.println("dBm");
  }

  if (now - tTgPoll >= TG_POLL_MS) {
    tTgPoll = now;
    if (WiFi.status() == WL_CONNECTED) {
      handleTelegram();
    }
  }

  delay(5);
  yield();
}
