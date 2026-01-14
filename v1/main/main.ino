// =====================
// SÜRÜM v8.005
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

struct Settings {
  float calMains, calGen, genBattDiv, camBattDiv;
} g_set;

struct Measurements {
  float mainsV, genV, genBattV, camBattV;
  int wifiRssi;
  uint32_t uptimeS;
} g_meas;

static uint32_t tTgPoll = 0;
static uint32_t tSerial = 0;
static uint32_t tTestSend = 0;
static bool g_testSendDone = false;

// =====================
// Helpers
// =====================
static String fmt2(float v) {
  if (isnan(v) || isinf(v)) return "nan";
  char b[16];
  dtostrf(v, 0, 2, b);
  return String(b);
}

// "/durum@bot arg" -> "/durum"
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

static void printLastError(const char* tag) {
  // UniversalTelegramBot içinde last_error String olarak tutuluyor
  // (Bazı sürümlerde boş kalabilir; yine de yazdırıyoruz.)
  Serial.print(tag);
  Serial.print(" last_error: ");
  Serial.println(bot.last_error);
}

static bool tgSendTo(const String& chatId, const String& msg) {
  bool ok = false;
  if (WiFi.status() == WL_CONNECTED) {
    ok = bot.sendMessage(chatId, msg, "");
  }
  Serial.print("[TG] send to "); Serial.print(chatId);
  Serial.print(" => "); Serial.println(ok ? "OK" : "FAIL");
  printLastError("[TG]");
  return ok;
}

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

static void loadMinimalSettings() {
  prefs.begin(NVS_NAMESPACE, false);
  g_set.calMains   = prefs.getFloat("calMains", CAL_MAINS);
  g_set.calGen     = prefs.getFloat("calGen",   CAL_GEN);
  g_set.genBattDiv = prefs.getFloat("genDiv",   GEN_BATT_DIV_RATIO);
  g_set.camBattDiv = prefs.getFloat("camDiv",   CAM_BATT_DIV_RATIO);
}

static String buildStatusText() {
  String s;
  s += String(DEVICE_NAME) + "\n";
  s += "🔖 Sürüm: " + String(PROJECT_VERSION) + " (DBG)\n";
  s += "WiFi: " + String(WiFi.status() == WL_CONNECTED ? "OK" : "FAIL") + " IP=" + WiFi.localIP().toString() + "\n";
  s += "CHAT_ID=" + String(CHAT_ID) + "\n";
  s += "ADMIN_ID=" + String(MASTER_ADMIN_ID) + "\n";
  return s;
}

static void handleTelegram() {
  int numNew = bot.getUpdates(bot.last_message_received + 1);

  Serial.print("[TG] getUpdates => ");
  Serial.println(numNew);
  printLastError("[TG]");

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
        tgSendTo(msg.chat_id, buildStatusText());
      } else {
        tgSendTo(msg.chat_id, "Komut: /durum /ping /myid");
      }
    }

    numNew = bot.getUpdates(bot.last_message_received + 1);
    Serial.print("[TG] getUpdates(loop) => ");
    Serial.println(numNew);
    printLastError("[TG]");
    if (numNew <= 0) break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println();
  Serial.println("=== BOOT ===");
  Serial.print("Version: "); Serial.println(PROJECT_VERSION);

  loadMinimalSettings();

  // TLS (sertifika uğraşmasın)
  tgClient.setInsecure();

  connectWiFi();

  // Başlangıç debug mesajı: hem gruba hem admin'e dene
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[TG] Boot send test starting...");
    tgSendTo(String(CHAT_ID),   "DBG boot ✅ (grup) | yaz: /ping veya /myid");
    tgSendTo(String(MASTER_ADMIN_ID), "DBG boot ✅ (ozel) | yaz: /ping veya /myid");
  }

  tTgPoll = millis();
  tSerial = millis();
  tTestSend = millis();
}

void loop() {
  // WiFi watchdog
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t tRetry = 0;
    if (millis() - tRetry > 5000) {
      tRetry = millis();
      connectWiFi();
    }
  }

  uint32_t now = millis();

  // 20 sn'de bir CHAT_ID'ye test mesajı dene (başarılı olunca durur)
  if (!g_testSendDone && (now - tTestSend >= 20000)) {
    tTestSend = now;
    bool ok = tgSendTo(String(CHAT_ID), "DBG test -> CHAT_ID (20sn) ✅");
    if (ok) g_testSendDone = true;
  }

  if (now - tSerial >= 3000) {
    tSerial = now;
    Serial.print("["); Serial.print(PROJECT_VERSION); Serial.print("] ");
    Serial.print("WiFi="); Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "FAIL");
    Serial.print(" IP="); Serial.print(WiFi.localIP());
    Serial.print(" last_err="); Serial.println(bot.last_error);
  }

  if (now - tTgPoll >= TG_POLL_MS) {
    tTgPoll = now;
    if (WiFi.status() == WL_CONNECTED) handleTelegram();
  }

  delay(5);
  yield();
}
