// =====================
// SÜRÜM v8.006
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

static uint32_t tTgPoll = 0;
static uint32_t tSerial = 0;
static uint32_t tTestSend = 0;
static bool g_testSendDone = false;

// =====================
// Helpers
// =====================
static String normalizeCommand(const String& textRaw) {
  // "/durum@bot arg" -> "/durum"
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

  // 2) Admin'a özel mesaj denemesi (user chat_id ile aynı olabilir)
  String adminChat = String(MASTER_ADMIN_ID);
  if (adminChat != String(CHAT_ID)) {
    tgSendTo(adminChat, msg);
  }
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

static String buildStatusText() {
  String s;
  s += String(DEVICE_NAME) + "\n";
  s += "🔖 Sürüm: " + String(PROJECT_VERSION) + " (DBG)\n";
  s += "WiFi: " + String(WiFi.status() == WL_CONNECTED ? "OK" : "FAIL") + "\n";
  s += "IP: " + WiFi.localIP().toString() + "\n";
  s += "CHAT_ID: " + String(CHAT_ID) + "\n";
  s += "MASTER_ADMIN_ID: " + String(MASTER_ADMIN_ID) + "\n";
  s += "\nTest komutları: /ping  /myid  /durum\n";
  return s;
}

static void handleTelegram() {
  int numNew = bot.getUpdates(bot.last_message_received + 1);

  Serial.print("[TG] getUpdates => ");
  Serial.println(numNew);

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
    if (numNew <= 0) break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println();
  Serial.println("=== BOOT ===");
  Serial.print("Version: "); Serial.println(PROJECT_VERSION);

  // TLS (sertifika uğraştırmasın)
  tgClient.setInsecure();

  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[TG] Boot send test...");
    tgSendBootBoth("DBG boot ✅ | Özelden bot'a /ping yaz. Sonra /myid yaz.");
    tgSendBootBoth(buildStatusText());
  } else {
    Serial.println("[TG] skipped (WiFi not connected)");
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
    Serial.print(" IP="); Serial.println(WiFi.localIP());
  }

  if (now - tTgPoll >= TG_POLL_MS) {
    tTgPoll = now;
    if (WiFi.status() == WL_CONNECTED) handleTelegram();
  }

  delay(5);
  yield();
}
