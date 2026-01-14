// =====================
// SÜRÜM v3.001
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

  // Aşama 3 thresholds (NVS kalıcı)
  float mainsHigh, mainsNormMin, mainsNormMax, mainsLow, mainsCrit;
  float genOff, genLow, genNormMin, genNormMax;
  float hystV;
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

enum class MainsState : uint8_t { UNKNOWN, CRITICAL, LOW, NORMAL, HIGH };
enum class GenState   : uint8_t { UNKNOWN, OFF, LOW, NORMAL, HIGH };

static MainsState g_mainsState = MainsState::UNKNOWN;
static GenState   g_genState   = GenState::UNKNOWN;

// latch: aynı state tekrar tekrar mesaj atmasın
static bool g_mainsLatched = false;
static bool g_genLatched   = false;

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

  g_set.mainsHigh    = prefs.getFloat("mHi",  MAINS_HIGH_V);
  g_set.mainsNormMin = prefs.getFloat("mNmn", MAINS_NORMAL_MIN);
  g_set.mainsNormMax = prefs.getFloat("mNmx", MAINS_NORMAL_MAX);
  g_set.mainsLow     = prefs.getFloat("mLo",  MAINS_LOW_V);
  g_set.mainsCrit    = prefs.getFloat("mCr",  MAINS_CRIT_V);

  g_set.genOff       = prefs.getFloat("gOff", GEN_OFF_V);
  g_set.genLow       = prefs.getFloat("gLo",  GEN_LOW_V);
  g_set.genNormMin   = prefs.getFloat("gNmn", GEN_NORMAL_MIN);
  g_set.genNormMax   = prefs.getFloat("gNmx", GEN_NORMAL_MAX);

  g_set.hystV        = prefs.getFloat("hyst", HYST_V);

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

  prefs.putFloat("hyst", g_set.hystV);

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

static bool isAuthorized(const telegramMessage& msg) {
  long fromId = msg.from_id.toInt();
  return (fromId == MASTER_ADMIN_ID);
}

static String mainsStateToText(MainsState st) {
  switch (st) {
    case MainsState::CRITICAL: return "CRITICAL";
    case MainsState::LOW:      return "LOW";
    case MainsState::NORMAL:   return "NORMAL";
    case MainsState::HIGH:     return "HIGH";
    default:                   return "UNKNOWN";
  }
}
static String genStateToText(GenState st) {
  switch (st) {
    case GenState::OFF:    return "OFF";
    case GenState::LOW:    return "LOW";
    case GenState::NORMAL: return "NORMAL";
    case GenState::HIGH:   return "HIGH";
    default:               return "UNKNOWN";
  }
}

static String buildStatusText() {
  String s;
  s += "📌 Köy Jeneratör Proje-3\n";
  s += String("🔖 Sürüm: ") + PROJECT_VERSION + "\n";
  s += String("⏱ Uptime: ") + String(g_meas.uptimeS) + " sn\n";
  s += String("📶 RSSI: ") + String(g_meas.wifiRssi) + " dBm\n";
  s += String("⚙️ Mod: ") + (g_mode == MODE_AUTO ? "AUTO" : "MANUAL") + "\n\n";

  s += "🔌 Şebeke: " + fmt2(g_meas.mainsV) + " V (" + mainsStateToText(g_mainsState) + ")\n";
  s += "🟠 Jeneratör: " + fmt2(g_meas.genV) + " V (" + genStateToText(g_genState) + ")\n";
  s += "🔋 Gen Akü: " + fmt2(g_meas.genBattV) + " V\n";
  s += "🔋 Cam Akü: " + fmt2(g_meas.camBattV) + " V\n";
  return s;
}

static String helpText() {
  String h;
  h += "Komutlar (Aşama 3):\n";
  h += "/durum  -> ölçümler\n";
  h += "/auto   -> AUTO mod\n";
  h += "/manual -> MANUAL mod\n";
  h += "/save   -> ayarları NVS kaydet\n";
  return h;
}

static void notify(const String& msg) {
  if (WiFi.status() == WL_CONNECTED) {
    bot.sendMessage(CHAT_ID, msg, "");
  }
}

static MainsState evalMains(float v) {
  // Histerezisli geçiş: state'e göre eşik kaydır
  float h = g_set.hystV;

  switch (g_mainsState) {
    case MainsState::HIGH:
      if (v <= g_set.mainsNormMax - h) return MainsState::NORMAL;
      return MainsState::HIGH;

    case MainsState::NORMAL:
      if (v >= g_set.mainsHigh) return MainsState::HIGH;
      if (v <  g_set.mainsCrit) return MainsState::CRITICAL;
      if (v <  g_set.mainsLow)  return MainsState::LOW;
      return MainsState::NORMAL;

    case MainsState::LOW:
      if (v >= g_set.mainsNormMin + h) return MainsState::NORMAL;
      if (v <  g_set.mainsCrit)        return MainsState::CRITICAL;
      return MainsState::LOW;

    case MainsState::CRITICAL:
      if (v >= g_set.mainsLow + h) return MainsState::LOW;
      return MainsState::CRITICAL;

    default:
      // ilk kez
      if (v >= g_set.mainsHigh) return MainsState::HIGH;
      if (v <  g_set.mainsCrit) return MainsState::CRITICAL;
      if (v <  g_set.mainsLow)  return MainsState::LOW;
      return MainsState::NORMAL;
  }
}

static GenState evalGen(float v) {
  float h = g_set.hystV;

  switch (g_genState) {
    case GenState::OFF:
      if (v >= g_set.genOff + h) return GenState::LOW;
      return GenState::OFF;

    case GenState::LOW:
      if (v <  g_set.genOff)           return GenState::OFF;
      if (v >= g_set.genNormMin + h)   return GenState::NORMAL;
      return GenState::LOW;

    case GenState::NORMAL:
      if (v <  g_set.genOff)         return GenState::OFF;
      if (v <  g_set.genLow)         return GenState::LOW;
      if (v >  g_set.genNormMax)     return GenState::HIGH;
      return GenState::NORMAL;

    case GenState::HIGH:
      if (v <= g_set.genNormMax - h) return GenState::NORMAL;
      return GenState::HIGH;

    default:
      if (v < g_set.genOff) return GenState::OFF;
      if (v < g_set.genLow) return GenState::LOW;
      if (v > g_set.genNormMax) return GenState::HIGH;
      return GenState::NORMAL;
  }
}

static void handleStateNotifications() {
  // mains
  MainsState newM = evalMains(g_meas.mainsV);
  if (newM != g_mainsState) {
    g_mainsState = newM;
    g_mainsLatched = false; // state değişti -> tekrar bildirim serbest
  }
  if (!g_mainsLatched) {
    g_mainsLatched = true;
    if (g_mainsState == MainsState::CRITICAL) notify("🚨 Şebeke KRİTİK: " + fmt2(g_meas.mainsV) + "V");
    else if (g_mainsState == MainsState::LOW) notify("⚠️ Şebeke DÜŞÜK: " + fmt2(g_meas.mainsV) + "V");
    else if (g_mainsState == MainsState::HIGH) notify("⚠️ Şebeke YÜKSEK: " + fmt2(g_meas.mainsV) + "V");
    else if (g_mainsState == MainsState::NORMAL) notify("✅ Şebeke NORMAL: " + fmt2(g_meas.mainsV) + "V");
  }

  // gen
  GenState newG = evalGen(g_meas.genV);
  if (newG != g_genState) {
    g_genState = newG;
    g_genLatched = false;
  }
  if (!g_genLatched) {
    g_genLatched = true;
    if (g_genState == GenState::OFF) notify("⛔ Jeneratör OFF: " + fmt2(g_meas.genV) + "V");
    else if (g_genState == GenState::LOW) notify("⚠️ Jeneratör DÜŞÜK: " + fmt2(g_meas.genV) + "V");
    else if (g_genState == GenState::HIGH) notify("⚠️ Jeneratör YÜKSEK: " + fmt2(g_meas.genV) + "V");
    else if (g_genState == GenState::NORMAL) notify("✅ Jeneratör NORMAL: " + fmt2(g_meas.genV) + "V");
  }
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

  g_meas.mainsV_raw = readAcRmsApprox(PIN_ADC_MAINS, g_set.calMains);
  g_meas.genV_raw   = readAcRmsApprox(PIN_ADC_GEN,   g_set.calGen);

  float vGenAdc = readAdcVoltage(PIN_ADC_GEN_BATT);
  float vCamAdc = readAdcVoltage(PIN_ADC_CAM_BATT);

  g_meas.genBattV_raw = vGenAdc * g_set.genBattDiv;
  g_meas.camBattV_raw = vCamAdc * g_set.camBattDiv;

  g_meas.mainsV   = lpf(g_meas.mainsV,   g_meas.mainsV_raw,   LPF_ALPHA_AC);
  g_meas.genV     = lpf(g_meas.genV,     g_meas.genV_raw,     LPF_ALPHA_AC);
  g_meas.genBattV = lpf(g_meas.genBattV, g_meas.genBattV_raw, LPF_ALPHA_BATT);
  g_meas.camBattV = lpf(g_meas.camBattV, g_meas.camBattV_raw, LPF_ALPHA_BATT);
}

static void handleSaveButton() {
  bool btn = digitalRead(PIN_BTN_SAVE);
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

  Serial.println();
  Serial.println("=== Koy Jenerator Proje-3 ===");
  Serial.print("Version: ");
  Serial.println(PROJECT_VERSION);

  loadSettings();
  connectWiFi();
  tgClient.setInsecure();

  if (WiFi.status() == WL_CONNECTED) {
    notify(String("✅ Sistem açıldı. Sürüm: ") + PROJECT_VERSION);
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
    handleStateNotifications(); // Aşama 3: bildirim motoru
  }

  if (now - tSerial >= SERIAL_REPORT_MS) {
    tSerial = now;
    Serial.print("["); Serial.print(PROJECT_VERSION); Serial.print("] ");
    Serial.print("Mains="); Serial.print(fmt2(g_meas.mainsV));
    Serial.print(" ("); Serial.print(mainsStateToText(g_mainsState)); Serial.print(")");
    Serial.print(" Gen="); Serial.print(fmt2(g_meas.genV));
    Serial.print(" ("); Serial.print(genStateToText(g_genState)); Serial.print(")");
    Serial.print(" GenBatt="); Serial.print(fmt2(g_meas.genBattV));
    Serial.print(" CamBatt="); Serial.print(fmt2(g_meas.camBattV));
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
