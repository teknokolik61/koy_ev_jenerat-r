#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// =====================
// config.ino içinden gelecek ayarlar
// =====================
extern const char* WIFI_SSID;
extern const char* WIFI_PASS;
extern const char* BOT_TOKEN;

extern const char* CHAT_ID;
extern const long  MASTER_ADMIN_ID;

// =====================
// RTC spam guard (boot report 60sn)
// =====================
RTC_DATA_ATTR uint32_t rtc_lastBootReportMs = 0;
RTC_DATA_ATTR uint32_t rtc_bootCounter = 0;
const uint32_t BOOT_REPORT_COOLDOWN_MS = 60UL * 1000UL;

// =====================
// WiFi reconnect policy
// =====================
const int WIFI_MAX_RESTART = 3;
int wifiRestartCount = 0;
bool wifiWaiting30Min = false;
unsigned long wifiFailWaitStartMs = 0;
const unsigned long WIFI_WAIT_30MIN_MS = 30UL * 60UL * 1000UL;
unsigned long lastWifiRetryMs = 0;

// =====================
// Telegram + TLS
// =====================
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

// =====================
// NVS (Preferences)
// =====================
Preferences prefs;
const char* NVS_NS = "genctrl";

// =====================
// Telegram poll
// =====================
unsigned long lastBotPollMs = 0;
const unsigned long BOT_POLL_INTERVAL_MS = 800; // ✅

// =====================
// Pins
// =====================
const int PIN_ADC_MAINS = 34; // ZMPT101B OUT (mains)
const int PIN_ADC_GEN   = 35; // ZMPT101B OUT (generator)
const int PIN_ADC_BATT  = 32; // ✅ Akü ölçümü (ADC1)

const int PIN_STARTER_RELAY = 26; // START
const int PIN_IGN_RELAY     = 27; // IGN / KONTAK
const int PIN_FUEL_RELAY    = 25; // FUEL / SOLENOID

// Röle kartı LOW tetiklemeli:
const int RELAY_ACTIVE_LEVEL = LOW;

// =====================
// Defaults (thresholds)
// =====================
const float D_MAINS_HIGH_ALERT_V = 240.0;
const float D_MAINS_OK_MIN_V     = 210.0;
const float D_MAINS_OK_MAX_V     = 230.0;

const float D_MAINS_CRITICAL_V   = 150.0;
const float D_GEN_RUNNING_V      = 100.0;

const float D_ALERT_HYST_V       = 3.0;

// auto start + soru bandı
const float D_AUTO_START_V = 100.0; // <100V: soru sormadan otomatik start
const float D_ASK_LOW_V    = 150.0; // 150–200V: soru sor
const float D_ASK_HIGH_V   = 200.0;

// Runtime thresholds (NVS’den yüklenir)
float MAINS_HIGH_ALERT_V = D_MAINS_HIGH_ALERT_V;
float MAINS_OK_MIN_V     = D_MAINS_OK_MIN_V;
float MAINS_OK_MAX_V     = D_MAINS_OK_MAX_V;

float MAINS_CRITICAL_V   = D_MAINS_CRITICAL_V;
float GEN_RUNNING_V      = D_GEN_RUNNING_V;

float ALERT_HYST_V       = D_ALERT_HYST_V;

float AUTO_START_V = D_AUTO_START_V;
float ASK_LOW_V    = D_ASK_LOW_V;
float ASK_HIGH_V   = D_ASK_HIGH_V;

// =====================
// Battery sense (GPIO32 / ADC1)
// Divider: R1=120k, R2=33k
// =====================
const float BATT_R1 = 120000.0f;
const float BATT_R2 = 33000.0f;
float BATT_CAL = 1.000f;                 // NVS’den yüklenecek
const float BATT_START_BLOCK_V = 11.0f;  // ✅ akü < 11V ise start iptal

// Battery Vmin (start sırasında akü çökmesi)
float battVminCycle = 99.0f; // son start döngüsünde min
float battVlast = 0.0f;

// =====================
// Modes
// =====================
bool autoMode = true;

// =====================
// Debounce / Filter
// =====================
const int GEN_ON_CONFIRM_COUNT  = 3;
const int GEN_OFF_CONFIRM_COUNT = 5;
int genAboveCount = 0;
int genBelowCount = 0;

// =====================
// RMS sampling (optimize)
// =====================
const int RMS_SAMPLES = 300;         // ✅
const int RMS_SAMPLE_DELAY_US = 100; // ✅

float CAL_MAINS = 0.310;
float CAL_GEN   = 0.310;

// RMS cache (500ms)
unsigned long lastMeasureMs = 0;
float mainsV_cache = 0;
float genV_cache   = 0;

// =====================
// Timers
// =====================
const unsigned long REPLY_TIMEOUT_MS = 60UL * 1000UL;
const unsigned long MAINS_RETURN_STOP_DELAY_MS = 30UL * 1000UL;
unsigned long PERIODIC_REPORT_MS = 60UL * 1000UL;

// Cooldown settings (Telegram’dan ayarlanır)
const uint32_t D_COOLDOWN_SEC = 10;
uint32_t COOLDOWN_SEC = D_COOLDOWN_SEC;

// =====================
// State machine
// =====================
enum SystemState {
  ST_MONITOR,
  ST_WAIT_REPLY,
  ST_PRECHECK_AFTER_YES,
  ST_STARTING,
  ST_COOLDOWN
};
SystemState state = ST_MONITOR;

// latches
bool askBandLatched = false;
bool autoStartLatched = false;
bool highAlertLatched = false;

// Reply state
bool awaitingReply = false;
unsigned long waitReplyStartMs = 0;

// Start procedure timers
unsigned long lastGenCheckMs = 0;
int starterSecondCounter = 0;
unsigned long cooldownStartMs = 0;

// Generator running / stop scheduling
bool generatorRunning = false;
unsigned long mainsNormalSinceMs = 0;
bool stopScheduled = false;

unsigned long lastPeriodicReportMs = 0;

// Last alarm
String lastAlarm = "Yok";

// =====================
// Start attempt limit
// =====================
const int MAX_START_ATTEMPTS = 10;
int startAttempt = 0;

// Battery weak heuristic (ZMPT’ye ek olarak)
const float BATTERY_WEAK_GEN_V = 12.0;
const int   BATTERY_WEAK_AFTER_SEC = 2;
bool batteryWeakWarnSent = false;

// =====================
// Stats (NVS)
// =====================
uint32_t stat_totalAttempts = 0;
uint32_t stat_totalSuccess  = 0;
uint32_t stat_totalFail     = 0;

uint32_t stat_lastCycleAttempts = 0;
String   stat_lastCycleResult   = "Yok";

// =====================
// Relay state persist (NVS)
// bit0=START, bit1=IGN, bit2=FUEL
// =====================
uint8_t lastRelayMask = 0;

// =====================
// AUTH (Dynamic whitelist + names)
// =====================
const int MAX_ALLOWED = 20;
struct AllowedEntry {
  long id;
  String name;
};
AllowedEntry allowedList[MAX_ALLOWED];
int allowedCount = 0;

// =====================
// Telegram offline queue (son 5 mesaj)
// =====================
const int TG_Q_SIZE = 5;
String tgQueue[TG_Q_SIZE];
int tgQHead = 0;
int tgQCount = 0;

bool tgWasOffline = false;
uint32_t tgDroppedCount = 0;
unsigned long tgOfflineSinceMs = 0;

unsigned long lastWifiCheckMs = 0;
bool lastWifiConnected = false;

// =====================
// Forward decl
// =====================
void sendTG(const String& msg);
String normalizeTR(String s);
bool isMaster(long userId);
bool isAllowedUser(long userId);

void sendInlineMenu();
void sendInlineYesNo(const String& reason, float v);
void handleInlineCallback(const String& data, const String& fromIdStr, const String& fromName);

void handleTelegramText(const String& text, const String& fromIdStr, const String& fromName);
void checkTelegram();

float readMainsV();
float readGenV();
float readBatteryV();

void startProcedureBegin(const String& why);
void stopGeneratorNow(const String& why);
void updateGeneratorRunningDebounced(float genV);
void bootDetectGeneratorRunning();

void handleMainsReturnStopLogic(float mainsV);
void handlePeriodicReport(float mainsV, float genV);

void askGeneratorQuestion(const String& reason, float v);

void saveAllowedListToNVS();
void loadAllowedListFromNVS();
void saveThresholdsToNVS();
void loadSettingsFromNVS();
void setDefaultsAndSave();

void sendStatus();
void sendThresholds();
void sendBootReport();
void sendStats();

void saveReportSecToNVS(uint32_t sec);
void saveAutoModeToNVS(bool on);

void saveCooldownSecToNVS(uint32_t sec);
void saveStatsToNVS();
void saveBattCalToNVS(float cal);

void saveRelayStateToNVS(uint8_t mask);
uint8_t loadRelayStateFromNVS();

void tgQueuePush(const String& msg);
String tgQueueDump();
void tgQueueClear();
String buildOfflineReport(float mainsV, float genV);

void wifiManagerTick();

// =====================
// Helpers
// =====================
String normalizeTR(String s) { s.toLowerCase(); s.trim(); return s; }
bool isMaster(long userId) { return userId == MASTER_ADMIN_ID; }

String onOffFromPin(int pin) {
  return (digitalRead(pin) == RELAY_ACTIVE_LEVEL) ? "ON" : "OFF";
}

String stateToStr(SystemState st) {
  switch (st) {
    case ST_MONITOR: return "MONITOR";
    case ST_WAIT_REPLY: return "WAIT_REPLY";
    case ST_PRECHECK_AFTER_YES: return "PRECHECK";
    case ST_STARTING: return "STARTING";
    case ST_COOLDOWN: return "COOLDOWN";
    default: return "UNKNOWN";
  }
}

String rssiToQuality(int rssi) {
  if (rssi >= -55) return "Mükemmel";
  if (rssi >= -67) return "İyi";
  if (rssi >= -75) return "Orta";
  if (rssi >= -85) return "Zayıf";
  return "Çok zayıf";
}

// =====================
// Relay write (persist)
// =====================
void saveRelayStateToNVS(uint8_t mask) {
  prefs.begin(NVS_NS, false);
  prefs.putUChar("rmask", mask);
  prefs.end();
}

uint8_t loadRelayStateFromNVS() {
  prefs.begin(NVS_NS, true);
  uint8_t m = prefs.getUChar("rmask", 0);
  prefs.end();
  return m;
}

void relayWrite(int pin, bool on) {
  digitalWrite(pin, on ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);

  if (pin == PIN_STARTER_RELAY) {
    if (on) lastRelayMask |=  (1 << 0); else lastRelayMask &= ~(1 << 0);
  } else if (pin == PIN_IGN_RELAY) {
    if (on) lastRelayMask |=  (1 << 1); else lastRelayMask &= ~(1 << 1);
  } else if (pin == PIN_FUEL_RELAY) {
    if (on) lastRelayMask |=  (1 << 2); else lastRelayMask &= ~(1 << 2);
  }
  saveRelayStateToNVS(lastRelayMask);
}

void starter(bool on) { relayWrite(PIN_STARTER_RELAY, on); }
void ign(bool on)     { relayWrite(PIN_IGN_RELAY, on); }
void fuel(bool on)    { relayWrite(PIN_FUEL_RELAY, on); }

// =====================
// Telegram offline queue helpers
// =====================
void tgQueuePush(const String& msg) {
  tgQueue[tgQHead] = msg;
  tgQHead = (tgQHead + 1) % TG_Q_SIZE;
  if (tgQCount < TG_Q_SIZE) tgQCount++;
}

String tgQueueDump() {
  if (tgQCount == 0) return "(yok)";

  String out = "";
  int start = tgQHead - tgQCount;
  if (start < 0) start += TG_Q_SIZE;

  for (int i = 0; i < tgQCount; i++) {
    int idx = (start + i) % TG_Q_SIZE;
    String m = tgQueue[idx];
    if (m.length() > 180) m = m.substring(0, 180) + "...";
    out += String(i + 1) + ") " + m + "\n";
  }
  return out;
}

void tgQueueClear() {
  tgQHead = 0;
  tgQCount = 0;
  for (int i = 0; i < TG_Q_SIZE; i++) tgQueue[i] = "";
}

String buildOfflineReport(float mainsV, float genV) {
  unsigned long offlineSec = 0;
  if (tgOfflineSinceMs != 0) offlineSec = (millis() - tgOfflineSinceMs) / 1000UL;

  String rep = "📶 Telegram tekrar online.\n";
  rep += "Kaçırılan mesaj: " + String(tgDroppedCount) + "\n";
  if (offlineSec > 0) rep += "Offline süre: ~" + String(offlineSec) + " sn\n";

  rep += "Şebeke: " + String(mainsV,1) + "V | Gen: " + String(genV,1) + "V\n";
  rep += "Akü: " + String(readBatteryV(), 2) + "V\n";
  if (battVminCycle < 98.0f) rep += "Vmin: " + String(battVminCycle, 2) + "V\n";
  rep += "Son alarm: " + lastAlarm + "\n";
  rep += "Çıkışlar: START=" + onOffFromPin(PIN_STARTER_RELAY);
  rep += " IGN=" + onOffFromPin(PIN_IGN_RELAY);
  rep += " FUEL=" + onOffFromPin(PIN_FUEL_RELAY) + "\n";

  rep += "Son " + String(TG_Q_SIZE) + " mesaj:\n";
  rep += tgQueueDump();

  return rep;
}

// =====================
// Safe Telegram sender
// =====================
void sendTG(const String& msg) {
  if (WiFi.status() != WL_CONNECTED) {
    if (!tgWasOffline) {
      tgWasOffline = true;
      tgOfflineSinceMs = millis();
    }
    tgDroppedCount++;
    tgQueuePush(msg);
    return;
  }

  if (tgWasOffline) {
    tgWasOffline = false;

    String rep = buildOfflineReport(mainsV_cache, genV_cache);

    tgDroppedCount = 0;
    tgOfflineSinceMs = 0;
    tgQueueClear();

    bot.sendMessage(CHAT_ID, rep, "");
  }

  bot.sendMessage(CHAT_ID, msg, "");
}

// =====================
// InlineKeyboard
// =====================
void sendInlineMenu() {
  String kb =
    "["
      "["
        "{\"text\":\"📟 Status\",\"callback_data\":\"CMD:/status\"},"
        "{\"text\":\"🔋 Battery\",\"callback_data\":\"CMD:/battery\"}"
      "],"
      "["
        "{\"text\":\"✅ Auto ON\",\"callback_data\":\"CMD:/auto_on\"},"
        "{\"text\":\"⛔ Auto OFF\",\"callback_data\":\"CMD:/auto_off\"}"
      "],"
      "["
        "{\"text\":\"▶️ Start\",\"callback_data\":\"CMD:/start_gen\"},"
        "{\"text\":\"🛑 Stop\",\"callback_data\":\"CMD:/stop\"}"
      "],"
      "["
        "{\"text\":\"⚙️ Thresholds\",\"callback_data\":\"CMD:/get_thresholds\"},"
        "{\"text\":\"📊 Stats\",\"callback_data\":\"CMD:/stats\"}"
      "],"
      "["
        "{\"text\":\"⏱ Report\",\"callback_data\":\"CMD:/get_report\"},"
        "{\"text\":\"🧊 Cooldown\",\"callback_data\":\"CMD:/get_cooldown\"}"
      "],"
      "["
        "{\"text\":\"♻️ Defaults\",\"callback_data\":\"CMD:/defaults\"},"
        "{\"text\":\"👮 Liste\",\"callback_data\":\"CMD:/liste\"}"
      "]"
    "]";

  bot.sendMessageWithInlineKeyboard(
    CHAT_ID,
    "🧩 Menü (Inline)",
    "",
    kb
  );
}

void sendInlineYesNo(const String& reason, float v) {
  String kb =
    "["
      "["
        "{\"text\":\"✅ EVET\",\"callback_data\":\"ANS:EVET\"},"
        "{\"text\":\"❌ HAYIR\",\"callback_data\":\"ANS:HAYIR\"}"
      "],"
      "["
        "{\"text\":\"📟 Status\",\"callback_data\":\"CMD:/status\"},"
        "{\"text\":\"🧩 Menü\",\"callback_data\":\"CMD:/start\"}"
      "]"
    "]";

  String msg = reason + "\n";
  msg += "Şebeke: " + String(v, 1) + "V\n";
  msg += "Jeneratör çalıştırayım mı?";
  bot.sendMessageWithInlineKeyboard(CHAT_ID, msg, "", kb);
}

// =====================
// Allowed list ops
// =====================
int findAllowedIndex(long userId) {
  for (int i = 0; i < allowedCount; i++) if (allowedList[i].id == userId) return i;
  return -1;
}

bool isAllowedUser(long userId) {
  if (userId == MASTER_ADMIN_ID) return true;
  return findAllowedIndex(userId) >= 0;
}

bool addAllowed(long userId) {
  if (userId == MASTER_ADMIN_ID) return true;
  if (findAllowedIndex(userId) >= 0) return true;
  if (allowedCount >= MAX_ALLOWED) return false;
  allowedList[allowedCount].id = userId;
  allowedList[allowedCount].name = "";
  allowedCount++;
  return true;
}

bool removeAllowed(long userId) {
  if (userId == MASTER_ADMIN_ID) return false;
  int idx = findAllowedIndex(userId);
  if (idx < 0) return false;
  for (int i = idx; i < allowedCount - 1; i++) allowedList[i] = allowedList[i + 1];
  allowedCount--;
  return true;
}

void updateNameIfNeeded(long userId, const String& fromName) {
  if (userId == MASTER_ADMIN_ID) return;
  int idx = findAllowedIndex(userId);
  if (idx < 0) return;
  if (allowedList[idx].name.length() == 0 || allowedList[idx].name != fromName) {
    allowedList[idx].name = fromName;
    saveAllowedListToNVS();
  }
}

bool setAllowedName(long userId, const String& newName) {
  if (userId == MASTER_ADMIN_ID) return false;
  int idx = findAllowedIndex(userId);
  if (idx < 0) return false;
  String n = newName; n.trim();
  if (n.length() == 0) return false;
  allowedList[idx].name = n;
  saveAllowedListToNVS();
  return true;
}

String allowedListToString() {
  String s = "👮 Yetkili liste:\n";
  s += "MASTER: " + String(MASTER_ADMIN_ID) + "\n";
  if (allowedCount == 0) { s += "(Ek yetkili yok)"; return s; }
  for (int i = 0; i < allowedCount; i++) {
    String nm = allowedList[i].name;
    if (nm.length() == 0) nm = "Tanımsız";
    s += String(i + 1) + ") " + nm + " (" + String(allowedList[i].id) + ")\n";
  }
  return s;
}

void saveAllowedListToNVS() {
  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.createNestedArray("users");
  for (int i = 0; i < allowedCount; i++) {
    JsonObject o = arr.createNestedObject();
    o["id"] = (long long)allowedList[i].id;
    o["name"] = allowedList[i].name;
  }
  String out;
  serializeJson(doc, out);

  prefs.begin(NVS_NS, false);
  prefs.putString("allow2", out);
  prefs.end();
}

void loadAllowedListFromNVS() {
  allowedCount = 0;

  prefs.begin(NVS_NS, true);
  String s = prefs.getString("allow2", "");
  prefs.end();
  if (s.length() == 0) return;

  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, s)) return;

  JsonArray arr = doc["users"].as<JsonArray>();
  for (JsonVariant v : arr) {
    if (allowedCount >= MAX_ALLOWED) break;
    long id = (long)(v["id"] | 0);
    const char* nm = v["name"] | "";
    if (id > 0 && id != MASTER_ADMIN_ID) {
      allowedList[allowedCount].id = id;
      allowedList[allowedCount].name = String(nm);
      allowedCount++;
    }
  }
}

// =====================
// NVS settings load/save
// =====================
void saveThresholdsToNVS() {
  prefs.begin(NVS_NS, false);
  prefs.putFloat("high", MAINS_HIGH_ALERT_V);
  prefs.putFloat("okmin", MAINS_OK_MIN_V);
  prefs.putFloat("okmax", MAINS_OK_MAX_V);
  prefs.putFloat("crit", MAINS_CRITICAL_V);
  prefs.putFloat("genrun", GEN_RUNNING_V);
  prefs.putFloat("hyst", ALERT_HYST_V);

  prefs.putFloat("autost", AUTO_START_V);
  prefs.putFloat("asklo",  ASK_LOW_V);
  prefs.putFloat("askhi",  ASK_HIGH_V);
  prefs.end();
}

void saveReportSecToNVS(uint32_t sec) {
  prefs.begin(NVS_NS, false);
  prefs.putUInt("repsec", sec);
  prefs.end();
}

void saveAutoModeToNVS(bool on) {
  prefs.begin(NVS_NS, false);
  prefs.putBool("auto", on);
  prefs.end();
}

void saveCooldownSecToNVS(uint32_t sec) {
  prefs.begin(NVS_NS, false);
  prefs.putUInt("cdsec", sec);
  prefs.end();
}

void saveStatsToNVS() {
  prefs.begin(NVS_NS, false);
  prefs.putUInt("st_att", stat_totalAttempts);
  prefs.putUInt("st_suc", stat_totalSuccess);
  prefs.putUInt("st_fail", stat_totalFail);
  prefs.putUInt("st_lca", stat_lastCycleAttempts);
  prefs.putString("st_lcr", stat_lastCycleResult);
  prefs.end();
}

void saveBattCalToNVS(float cal) {
  prefs.begin(NVS_NS, false);
  prefs.putFloat("bcal", cal);
  prefs.end();
}

void setDefaultsAndSave() {
  MAINS_HIGH_ALERT_V = D_MAINS_HIGH_ALERT_V;
  MAINS_OK_MIN_V     = D_MAINS_OK_MIN_V;
  MAINS_OK_MAX_V     = D_MAINS_OK_MAX_V;

  MAINS_CRITICAL_V   = D_MAINS_CRITICAL_V;
  GEN_RUNNING_V      = D_GEN_RUNNING_V;
  ALERT_HYST_V       = D_ALERT_HYST_V;

  AUTO_START_V = D_AUTO_START_V;
  ASK_LOW_V    = D_ASK_LOW_V;
  ASK_HIGH_V   = D_ASK_HIGH_V;

  PERIODIC_REPORT_MS = 60UL * 1000UL;
  autoMode = true;

  COOLDOWN_SEC = D_COOLDOWN_SEC;
  BATT_CAL = 1.000f;

  saveThresholdsToNVS();
  saveReportSecToNVS(60);
  saveAutoModeToNVS(true);
  saveCooldownSecToNVS(COOLDOWN_SEC);
  saveBattCalToNVS(BATT_CAL);
}

void loadSettingsFromNVS() {
  prefs.begin(NVS_NS, true);

  MAINS_HIGH_ALERT_V = prefs.getFloat("high",  D_MAINS_HIGH_ALERT_V);
  MAINS_OK_MIN_V     = prefs.getFloat("okmin", D_MAINS_OK_MIN_V);
  MAINS_OK_MAX_V     = prefs.getFloat("okmax", D_MAINS_OK_MAX_V);

  MAINS_CRITICAL_V   = prefs.getFloat("crit",  D_MAINS_CRITICAL_V);
  GEN_RUNNING_V      = prefs.getFloat("genrun",D_GEN_RUNNING_V);
  ALERT_HYST_V       = prefs.getFloat("hyst",  D_ALERT_HYST_V);

  AUTO_START_V = prefs.getFloat("autost", D_AUTO_START_V);
  ASK_LOW_V    = prefs.getFloat("asklo",  D_ASK_LOW_V);
  ASK_HIGH_V   = prefs.getFloat("askhi",  D_ASK_HIGH_V);

  uint32_t repSec = prefs.getUInt("repsec", 60);
  if (repSec < 10) repSec = 10;
  if (repSec > 3600) repSec = 3600;
  PERIODIC_REPORT_MS = (unsigned long)repSec * 1000UL;

  autoMode = prefs.getBool("auto", true);

  COOLDOWN_SEC = prefs.getUInt("cdsec", D_COOLDOWN_SEC);
  if (COOLDOWN_SEC < 5) COOLDOWN_SEC = 5;
  if (COOLDOWN_SEC > 180) COOLDOWN_SEC = 180;

  BATT_CAL = prefs.getFloat("bcal", 1.000f);
  if (BATT_CAL < 0.80f) BATT_CAL = 0.80f;
  if (BATT_CAL > 1.30f) BATT_CAL = 1.30f;

  stat_totalAttempts = prefs.getUInt("st_att", 0);
  stat_totalSuccess  = prefs.getUInt("st_suc", 0);
  stat_totalFail     = prefs.getUInt("st_fail", 0);
  stat_lastCycleAttempts = prefs.getUInt("st_lca", 0);
  stat_lastCycleResult   = prefs.getString("st_lcr", "Yok");

  prefs.end();

  if (ASK_LOW_V > ASK_HIGH_V) { float tmp = ASK_LOW_V; ASK_LOW_V = ASK_HIGH_V; ASK_HIGH_V = tmp; }
  if (AUTO_START_V < 0) AUTO_START_V = 0;

  loadAllowedListFromNVS();
}

// =====================
// RMS measurement (double safe)
// =====================
float readVoltageRMS(int adcPin, float calFactor) {
  double sum = 0;
  double sumSq = 0;

  for (int i = 0; i < RMS_SAMPLES; i++) {
    int v = analogRead(adcPin);
    sum += v;
    delayMicroseconds(RMS_SAMPLE_DELAY_US);
  }
  double mean = sum / (double)RMS_SAMPLES;

  for (int i = 0; i < RMS_SAMPLES; i++) {
    int v = analogRead(adcPin);
    double ac = (double)v - mean;
    sumSq += ac * ac;
    delayMicroseconds(RMS_SAMPLE_DELAY_US);
  }

  double meanSq = sumSq / (double)RMS_SAMPLES;
  double rmsAdc = sqrt(meanSq);
  return (float)(rmsAdc * calFactor);
}

float readMainsV() { return readVoltageRMS(PIN_ADC_MAINS, CAL_MAINS); }
float readGenV()   { return readVoltageRMS(PIN_ADC_GEN,   CAL_GEN);   }

float readBatteryV() {
  const int N = 40;
  uint32_t sum = 0;
  for (int i = 0; i < N; i++) {
    sum += (uint32_t)analogRead(PIN_ADC_BATT);
    delay(2);
  }
  float adc = (float)sum / (float)N;
  float v_adc = (adc / 4095.0f) * 3.3f;
  float v_bat = v_adc * ((BATT_R1 + BATT_R2) / BATT_R2);
  v_bat *= BATT_CAL;
  return v_bat;
}

// =====================
// Generator logic
// =====================
void prepareGeneratorRun() { ign(true); fuel(true); }

void stopGeneratorNow(const String& why) {
  starter(false);
  fuel(false);
  ign(false);

  startAttempt = 0;

  generatorRunning = false;
  stopScheduled = false;
  mainsNormalSinceMs = 0;

  genAboveCount = 0;
  genBelowCount = 0;

  sendTG("🛑 Jeneratör STOP: " + why);
  sendInlineMenu();
}

void startProcedureBegin(const String& why) {
  if (state == ST_STARTING || state == ST_COOLDOWN) {
    sendTG("ℹ️ Zaten çalıştırma döngüsündeyim. (" + why + ")");
    return;
  }

  // 🔋 Start öncesi akü kontrolü
  float vb = readBatteryV();
  battVlast = vb;
  battVminCycle = vb; // döngü başında min'e set
  if (vb < BATT_START_BLOCK_V) {
    lastAlarm = "🛑 Start iptal: Akü düşük (" + String(vb,2) + "V)";
    sendTG(lastAlarm + "\nLimit: " + String(BATT_START_BLOCK_V,1) + "V\nSebep: " + why);

    stat_lastCycleAttempts = 0;
    stat_lastCycleResult = "ABORT (LOW_BATT " + String(vb,2) + "V)";
    saveStatsToNVS();

    sendInlineMenu();
    return;
  }

  sendTG("▶️ Jeneratör çalıştırma prosedürü başlıyor. (" + why + ")");

  startAttempt = 0;
  batteryWeakWarnSent = false;

  stat_lastCycleAttempts = 0;
  stat_lastCycleResult = "RUNNING";
  saveStatsToNVS();

  prepareGeneratorRun();
  starterSecondCounter = 0;
  lastGenCheckMs = 0;

  starter(true);
  state = ST_STARTING;
}

void handleMainsReturnStopLogic(float mainsV) {
  bool mainsNormal = (mainsV >= MAINS_OK_MIN_V && mainsV <= MAINS_OK_MAX_V);

  if (generatorRunning) {
    if (mainsNormal) {
      if (!stopScheduled) {
        stopScheduled = true;
        mainsNormalSinceMs = millis();
        sendTG("✅ Şebeke normale döndü (" + String(mainsV,1) + "V). 30 sn sonra jeneratörü durduracağım.");
      }
    } else {
      if (stopScheduled) {
        stopScheduled = false;
        mainsNormalSinceMs = 0;
        sendTG("⚠️ Şebeke tekrar normal dışına çıktı. Stop planı iptal.");
      }
    }

    if (stopScheduled && (millis() - mainsNormalSinceMs >= MAINS_RETURN_STOP_DELAY_MS)) {
      stopGeneratorNow("Şebeke 30 sn normal kaldı");
    }
  } else {
    stopScheduled = false;
    mainsNormalSinceMs = 0;
  }
}

void handlePeriodicReport(float mainsV, float genV) {
  if (!generatorRunning) return;

  bool mainsNormal = (mainsV >= MAINS_OK_MIN_V && mainsV <= MAINS_OK_MAX_V);
  if (mainsNormal) return;

  if (millis() - lastPeriodicReportMs >= PERIODIC_REPORT_MS) {
    lastPeriodicReportMs = millis();
    String msg = "📡 Durum bildirimi:\n";
    msg += "Şebeke: " + String(mainsV,1) + "V\n";
    msg += "Jeneratör: " + String(genV,1) + "V\n";
    msg += "Akü: " + String(readBatteryV(),2) + "V\n";
    msg += "Auto: " + String(autoMode ? "ON" : "OFF");
    sendTG(msg);
  }
}

void updateGeneratorRunningDebounced(float genV) {
  if (genV > GEN_RUNNING_V) { genAboveCount++; genBelowCount = 0; }
  else { genBelowCount++; genAboveCount = 0; }

  if (!generatorRunning && genAboveCount >= GEN_ON_CONFIRM_COUNT) {
    generatorRunning = true;
    lastPeriodicReportMs = millis();
    sendTG("✅ Jeneratör çalışıyor (3/3 onay). (" + String(genV,1) + "V)");
  }

  if (generatorRunning && state == ST_MONITOR && genBelowCount >= GEN_OFF_CONFIRM_COUNT) {
    generatorRunning = false;
    stopScheduled = false;
    mainsNormalSinceMs = 0;
    sendTG("ℹ️ Jeneratör durdu (5/5 onay). (" + String(genV,1) + "V)");
  }
}

void bootDetectGeneratorRunning() {
  int hits = 0;
  for (int i = 0; i < 3; i++) {
    float g = readGenV();
    if (g > GEN_RUNNING_V) hits++;
    delay(150);
  }
  generatorRunning = (hits >= 2);
  genAboveCount = generatorRunning ? GEN_ON_CONFIRM_COUNT : 0;
  genBelowCount = generatorRunning ? 0 : GEN_OFF_CONFIRM_COUNT;
}

// =====================
// Status / Boot report
// =====================
String resetReasonToStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT_RESET";
    case ESP_RST_SW: return "SW_RESET";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

void sendThresholds() {
  String msg;
  msg += "⚙️ Thresholds:\n";
  msg += "okmin=" + String(MAINS_OK_MIN_V,1) + "V\n";
  msg += "okmax=" + String(MAINS_OK_MAX_V,1) + "V\n";
  msg += "high=" + String(MAINS_HIGH_ALERT_V,1) + "V\n";
  msg += "critical=" + String(MAINS_CRITICAL_V,1) + "V\n";
  msg += "genrun=" + String(GEN_RUNNING_V,1) + "V\n";
  msg += "hyst=" + String(ALERT_HYST_V,1) + "V\n";
  msg += "auto_start=" + String(AUTO_START_V,1) + "V\n";
  msg += "ask_low=" + String(ASK_LOW_V,1) + "V\n";
  msg += "ask_high=" + String(ASK_HIGH_V,1) + "V\n";
  msg += "report=" + String(PERIODIC_REPORT_MS/1000UL) + " sn\n";
  msg += "cooldown=" + String(COOLDOWN_SEC) + " sn\n";
  msg += "batt_cal=" + String(BATT_CAL,3) + "\n";
  msg += "auto=" + String(autoMode ? "ON" : "OFF");
  sendTG(msg);
}

void sendStats() {
  String msg;
  msg += "📊 İstatistikler:\n";
  msg += "Toplam deneme: " + String(stat_totalAttempts) + "\n";
  msg += "Başarı: " + String(stat_totalSuccess) + "\n";
  msg += "Fail: " + String(stat_totalFail) + "\n";
  msg += "Son döngü: " + String(stat_lastCycleAttempts) + " deneme\n";
  msg += "Son sonuç: " + stat_lastCycleResult + "\n";
  msg += "Cooldown: " + String(COOLDOWN_SEC) + " sn\n";
  msg += "Akü (şu an): " + String(readBatteryV(), 2) + "V\n";
  if (battVminCycle < 98.0f) msg += "Vmin (son marş döngüsü): " + String(battVminCycle, 2) + "V";
  else msg += "Vmin (son marş döngüsü): (yok)";
  sendTG(msg);
}

void sendStatus() {
  float mainsV = mainsV_cache;
  float genV   = genV_cache;
  float vb     = readBatteryV();
  int rssi = WiFi.RSSI();

  String msg;
  msg += "📟 /status\n";
  msg += "Auto: " + String(autoMode ? "ON" : "OFF") + "\n";
  msg += "State: " + stateToStr(state) + "\n";
  msg += "GenRunning: " + String(generatorRunning ? "YES" : "NO") + "\n";
  msg += "Şebeke: " + String(mainsV, 1) + "V\n";
  msg += "Jeneratör: " + String(genV, 1) + "V\n";
  msg += "Akü: " + String(vb, 2) + "V\n";
  if (battVminCycle < 98.0f) msg += "Vmin: " + String(battVminCycle, 2) + "V\n";
  msg += "Son alarm: " + lastAlarm + "\n";
  msg += "Wi-Fi: " + String(rssi) + " dBm (" + rssiToQuality(rssi) + ")\n";
  msg += "SSID: " + String(WiFi.SSID()) + "\n";
  msg += "IP: " + WiFi.localIP().toString() + "\n";
  msg += "Rapor: " + String(PERIODIC_REPORT_MS/1000UL) + " sn\n";
  msg += "Cooldown: " + String(COOLDOWN_SEC) + " sn\n";
  msg += "Çıkışlar: START=" + onOffFromPin(PIN_STARTER_RELAY);
  msg += " IGN=" + onOffFromPin(PIN_IGN_RELAY);
  msg += " FUEL=" + onOffFromPin(PIN_FUEL_RELAY);

  if (stopScheduled) {
    unsigned long left = (MAINS_RETURN_STOP_DELAY_MS - (millis() - mainsNormalSinceMs)) / 1000UL;
    msg += "\nStop planlı: " + String(left) + " sn sonra";
  }
  sendTG(msg);
}

void sendBootReport() {
  float mainsV = mainsV_cache;
  float genV = genV_cache;
  int rssi = WiFi.RSSI();

  bool lastIgn  = (lastRelayMask & (1 << 1)) != 0;
  bool lastFuel = (lastRelayMask & (1 << 2)) != 0;

  String msg;
  msg += "🔄 ESP32 yeniden başladı\n";
  msg += "Reset: " + resetReasonToStr(esp_reset_reason()) + "\n";
  msg += "BootCount: " + String(rtc_bootCounter) + "\n";
  msg += "Auto: " + String(autoMode ? "ON" : "OFF") + "\n";
  msg += "State: " + stateToStr(state) + "\n";
  msg += "Şebeke: " + String(mainsV,1) + "V\n";
  msg += "Jeneratör: " + String(genV,1) + "V\n";
  msg += "Akü: " + String(readBatteryV(),2) + "V\n";
  msg += "GenRunning: " + String(generatorRunning ? "YES" : "NO") + "\n";
  msg += "Wi-Fi: " + String(rssi) + " dBm (" + rssiToQuality(rssi) + ")\n";
  msg += "SSID: " + String(WiFi.SSID()) + "\n";
  msg += "IP: " + WiFi.localIP().toString();

  msg += "\n🛡 Fail-safe restore:";
  msg += "\nSTART=OFF (reset)";
  msg += "\nIGN=" + String(lastIgn ? "ON" : "OFF");
  msg += " FUEL=" + String(lastFuel ? "ON" : "OFF");

  sendTG(msg);
  sendInlineMenu();
}

// =====================
// Ask flow
// =====================
void askGeneratorQuestion(const String& reason, float v) {
  lastAlarm = reason + " (" + String(v,1) + "V)";
  sendInlineYesNo(reason, v);
  awaitingReply = true;
  waitReplyStartMs = millis();
  state = ST_WAIT_REPLY;
}

// =====================
// Inline callback handler
// =====================
void handleInlineCallback(const String& data, const String& fromIdStr, const String& fromName) {
  long fromId = fromIdStr.toInt();

  if (!isAllowedUser(fromId)) {
    sendTG("⛔ Yetkisiz komut. Kullanıcı: " + fromName + " (" + String(fromId) + ")");
    return;
  }

  if (data.startsWith("CMD:")) {
    String cmd = data.substring(4);
    handleTelegramText(cmd, fromIdStr, fromName);
    return;
  }

  if (data == "ANS:EVET") {
    if (state == ST_WAIT_REPLY && awaitingReply) {
      awaitingReply = false;
      state = ST_PRECHECK_AFTER_YES;
      sendTG("✅ Onay alındı. Şebeke tekrar kontrol ediliyor...");
    }
    return;
  }

  if (data == "ANS:HAYIR") {
    if (state == ST_WAIT_REPLY && awaitingReply) {
      awaitingReply = false;
      state = ST_MONITOR;
      sendTG("👍 Tamam. İşlem yapılmadı.");
      sendInlineMenu();
    }
    return;
  }
}

// =====================
// Telegram handler
// =====================
void handleTelegramText(const String& text, const String& fromIdStr, const String& fromName) {
  long fromId = fromIdStr.toInt();
  String t = normalizeTR(text);

  // herkese açık
  if (t == "/myid") {
    sendTG("🆔 " + fromName + " user_id: " + String(fromId));
    return;
  }

  // isim güncelle (yetkiliyse)
  if (isAllowedUser(fromId)) updateNameIfNeeded(fromId, fromName);

  // yetkisiz
  if (!isAllowedUser(fromId)) {
    if (t.startsWith("/")) {
      sendTG("⛔ Yetkisiz komut. Kullanıcı: " + fromName + " (" + String(fromId) + ")");
    }
    return;
  }

  // Menü
  if (t == "/start" || t == "/help" || t == "/komutlar" || t == "/commands") {
    sendInlineMenu();
    return;
  }

  // Alarm yanıtını yazıyla da kabul et
  if (state == ST_WAIT_REPLY && awaitingReply) {
    if (t == "evet" || t == "e" || t == "yes") {
      awaitingReply = false;
      state = ST_PRECHECK_AFTER_YES;
      sendTG("✅ Onay alındı. Şebeke tekrar kontrol ediliyor...");
      return;
    }
    if (t == "hayır" || t == "hayir" || t == "h" || t == "no") {
      awaitingReply = false;
      state = ST_MONITOR;
      sendTG("👍 Tamam. İşlem yapılmadı.");
      sendInlineMenu();
      return;
    }
  }

  // MASTER + yetkili komutlar
  if (t == "/liste") { sendTG(allowedListToString()); return; }

  if (t.startsWith("/ekle")) {
    if (!isMaster(fromId)) { sendTG("⛔ Bu komut sadece MASTER için."); return; }
    int sp = text.indexOf(' ');
    if (sp < 0) { sendTG("Kullanım: /ekle 123456789"); return; }
    long id = text.substring(sp + 1).toInt();
    if (id <= 0) { sendTG("❌ Geçersiz id."); return; }
    if (!addAllowed(id)) { sendTG("❌ Liste dolu (max " + String(MAX_ALLOWED) + ")."); return; }
    saveAllowedListToNVS();
    sendTG("✅ Eklendi: " + String(id) + " (isim ilk mesajla eklenecek)");
    return;
  }

  if (t.startsWith("/sil")) {
    if (!isMaster(fromId)) { sendTG("⛔ Bu komut sadece MASTER için."); return; }
    int sp = text.indexOf(' ');
    if (sp < 0) { sendTG("Kullanım: /sil 123456789"); return; }
    long id = text.substring(sp + 1).toInt();
    if (id == MASTER_ADMIN_ID) { sendTG("❌ MASTER silinemez."); return; }
    if (!removeAllowed(id)) { sendTG("ℹ️ Listede yok: " + String(id)); return; }
    saveAllowedListToNVS();
    sendTG("✅ Silindi: " + String(id));
    return;
  }

  if (t.startsWith("/isim")) {
    if (!isMaster(fromId)) { sendTG("⛔ Bu komut sadece MASTER için."); return; }
    int firstSpace = text.indexOf(' ');
    if (firstSpace < 0) { sendTG("Kullanım: /isim 123456789 Mehmet Yılmaz"); return; }
    int secondSpace = text.indexOf(' ', firstSpace + 1);
    if (secondSpace < 0) { sendTG("Kullanım: /isim 123456789 Mehmet Yılmaz"); return; }

    long id = text.substring(firstSpace + 1, secondSpace).toInt();
    if (id <= 0) { sendTG("❌ Geçersiz id."); return; }

    String newName = text.substring(secondSpace + 1);
    newName.trim();
    if (newName.length() == 0) { sendTG("❌ İsim boş olamaz."); return; }

    if (!setAllowedName(id, newName)) {
      sendTG("❌ Bu ID listede yok veya master ID. Önce /ekle, sonra /isim.");
      return;
    }
    sendTG("✅ İsim güncellendi: " + newName + " (" + String(id) + ")");
    return;
  }

  // Normal komutlar
  if (t == "/status") { sendStatus(); return; }
  if (t == "/battery") {
    float vb = readBatteryV();
    sendTG("🔋 Akü voltajı: " + String(vb, 2) + "V\nCal: " + String(BATT_CAL, 3));
    return;
  }

  if (t.startsWith("/set_batt_cal")) {
    int sp = text.indexOf(' ');
    if (sp < 0) { sendTG("Kullanım: /set_batt_cal 1.033   (0.80..1.30)"); return; }

    float cal = text.substring(sp + 1).toFloat();
    if (cal < 0.80f) cal = 0.80f;
    if (cal > 1.30f) cal = 1.30f;

    BATT_CAL = cal;
    saveBattCalToNVS(BATT_CAL);

    float vb = readBatteryV();
    sendTG("✅ BATT_CAL kaydedildi: " + String(BATT_CAL, 3) +
           "\n🔋 Şu an akü: " + String(vb, 2) + "V");
    return;
  }

  if (t == "/get_thresholds") { sendThresholds(); return; }
  if (t == "/stats") { sendStats(); return; }

  if (t == "/get_report") {
    sendTG("⏱️ Rapor süresi: " + String(PERIODIC_REPORT_MS/1000UL) + " sn");
    return;
  }

  if (t == "/get_cooldown") {
    sendTG("🧊 Cooldown süresi: " + String(COOLDOWN_SEC) + " sn");
    return;
  }

  if (t.startsWith("/set_cooldown")) {
    int sp = text.indexOf(' ');
    if (sp < 0) { sendTG("Kullanım: /set_cooldown 10   (5..180 sn)"); return; }
    uint32_t sec = (uint32_t)text.substring(sp + 1).toInt();
    if (sec < 5) sec = 5;
    if (sec > 180) sec = 180;
    COOLDOWN_SEC = sec;
    saveCooldownSecToNVS(sec);
    sendTG("✅ Cooldown ayarlandı: " + String(sec) + " sn");
    return;
  }

  if (t == "/auto_on")  { autoMode = true;  saveAutoModeToNVS(true);  sendTG("✅ Otomatik mod AÇIK."); return; }
  if (t == "/auto_off") { autoMode = false; saveAutoModeToNVS(false); awaitingReply = false; if (state == ST_WAIT_REPLY) state = ST_MONITOR; sendTG("⛔ Otomatik mod KAPALI."); return; }

  if (t == "/start_gen") { startProcedureBegin("manuel /start_gen"); return; }
  if (t == "/stop")  { stopGeneratorNow("manuel /stop"); state = ST_MONITOR; awaitingReply = false; return; }

  if (t == "/defaults") { setDefaultsAndSave(); sendTG("♻️ Varsayılanlar yüklendi ve kaydedildi."); sendThresholds(); return; }

  if (t.startsWith("/set_report")) {
    int sp = text.indexOf(' ');
    if (sp < 0) { sendTG("Kullanım: /set_report 30  (10..3600 sn)"); return; }
    uint32_t sec = (uint32_t)text.substring(sp + 1).toInt();
    if (sec < 10) sec = 10;
    if (sec > 3600) sec = 3600;
    PERIODIC_REPORT_MS = (unsigned long)sec * 1000UL;
    saveReportSecToNVS(sec);
    sendTG("✅ Rapor süresi: " + String(sec) + " sn");
    return;
  }

  if (t.startsWith("/set_thresholds")) {
    float v; bool changed = false;
    String tn = normalizeTR(text);
    int idx = 0;

    while (idx >= 0) {
      int next = tn.indexOf(' ', idx);
      String token = (next < 0) ? tn.substring(idx) : tn.substring(idx, next);
      idx = (next < 0) ? -1 : next + 1;
      if (token == "/set_thresholds") continue;

      auto parseKV = [&](const String& key)->bool{
        String prefix = key + "=";
        if (!token.startsWith(prefix)) return false;
        v = token.substring(prefix.length()).toFloat();
        return true;
      };

      if (parseKV("high"))       { MAINS_HIGH_ALERT_V = v; changed = true; continue; }
      if (parseKV("okmin"))      { MAINS_OK_MIN_V = v; changed = true; continue; }
      if (parseKV("okmax"))      { MAINS_OK_MAX_V = v; changed = true; continue; }
      if (parseKV("critical"))   { MAINS_CRITICAL_V = v; changed = true; continue; }
      if (parseKV("genrun"))     { GEN_RUNNING_V = v; changed = true; continue; }
      if (parseKV("hyst"))       { ALERT_HYST_V = v; changed = true; continue; }

      if (parseKV("auto_start")) { AUTO_START_V = v; changed = true; continue; }
      if (parseKV("ask_low"))    { ASK_LOW_V    = v; changed = true; continue; }
      if (parseKV("ask_high"))   { ASK_HIGH_V   = v; changed = true; continue; }
    }

    if (!changed) {
      sendTG("Kullanım: /set_thresholds okmin=210 okmax=230 high=240 critical=150 genrun=100 hyst=3 auto_start=100 ask_low=150 ask_high=200");
      return;
    }

    if (ASK_LOW_V > ASK_HIGH_V) { float tmp = ASK_LOW_V; ASK_LOW_V = ASK_HIGH_V; ASK_HIGH_V = tmp; }
    if (MAINS_OK_MIN_V >= MAINS_OK_MAX_V) {
      sendTG("❌ Hata: okmin okmax'tan küçük olmalı. Değişiklik iptal.");
      loadSettingsFromNVS();
      return;
    }
    if (AUTO_START_V < 0) AUTO_START_V = 0;

    saveThresholdsToNVS();
    sendTG("✅ Thresholds kaydedildi.");
    sendThresholds();
    return;
  }
}

// =====================
// Telegram polling
// =====================
void checkTelegram() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastBotPollMs < BOT_POLL_INTERVAL_MS) return;
  lastBotPollMs = millis();

  int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  while (numNewMessages) {
    for (int i = 0; i < numNewMessages; i++) {
      if (bot.messages[i].chat_id != String(CHAT_ID)) continue;

      if (bot.messages[i].type == "callback_query") {
        String data = bot.messages[i].text;
        handleInlineCallback(data, bot.messages[i].from_id, bot.messages[i].from_name);
        continue;
      }

      handleTelegramText(bot.messages[i].text, bot.messages[i].from_id, bot.messages[i].from_name);
    }
    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  }
}

// =====================
// WiFi manager tick (30dk retry + online transition)
// =====================
void wifiManagerTick() {
  // 5sn'de bir durum kontrol
  if (millis() - lastWifiCheckMs >= 5000) {
    lastWifiCheckMs = millis();
    bool nowConnected = (WiFi.status() == WL_CONNECTED);

    if (!lastWifiConnected && nowConnected) {
      secured_client.setInsecure();
      sendTG("✅ Wi-Fi geri geldi.");
    }
    lastWifiConnected = nowConnected;
  }

  // 30dk bekleme modunda mı?
  if (wifiWaiting30Min) {
    if (millis() - wifiFailWaitStartMs >= WIFI_WAIT_30MIN_MS) {
      wifiWaiting30Min = false;
      wifiRestartCount = 0;
      WiFi.disconnect(true);
      delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastWifiRetryMs >= 60000) {
        lastWifiRetryMs = millis();
        WiFi.disconnect(true);
        delay(200);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
      }
    }
  }
}


// =====================
// setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  rtc_bootCounter++;

  pinMode(PIN_STARTER_RELAY, OUTPUT);
  pinMode(PIN_IGN_RELAY, OUTPUT);
  pinMode(PIN_FUEL_RELAY, OUTPUT);

  // güvenli inaktif
  digitalWrite(PIN_STARTER_RELAY, !RELAY_ACTIVE_LEVEL);
  digitalWrite(PIN_IGN_RELAY,     !RELAY_ACTIVE_LEVEL);
  digitalWrite(PIN_FUEL_RELAY,    !RELAY_ACTIVE_LEVEL);
  delay(10);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  loadSettingsFromNVS();

  // Röle restore: START her zaman OFF, IGN+FUEL restore
  lastRelayMask = loadRelayStateFromNVS();
  bool lastIgn  = (lastRelayMask & (1 << 1)) != 0;
  bool lastFuel = (lastRelayMask & (1 << 2)) != 0;

  starter(false);   // ✅ reset sonrası marş OFF
  ign(lastIgn);     // ✅ restore
  fuel(lastFuel);   // ✅ restore

  // WiFi connect (20 sn)
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print(".");
  }

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (!wifiConnected) {
    wifiRestartCount++;
    if (wifiRestartCount <= WIFI_MAX_RESTART) {
      Serial.println("\nWiFi yok, restart... (" + String(wifiRestartCount) + ")");
      delay(1000);
      ESP.restart();
    } else {
      Serial.println("\nWiFi yok. Offline devam. 30dk sonra tekrar denenecek.");
      wifiWaiting30Min = true;
      wifiFailWaitStartMs = millis();
    }
  } else {
    Serial.println("\nWiFi bağlı.");
    wifiRestartCount = 0;
    secured_client.setInsecure();
  }

  // İlk ölçümler
  mainsV_cache = readMainsV();
  genV_cache   = readGenV();
  battVlast    = readBatteryV();

  bootDetectGeneratorRunning();

  // Boot report sadece WiFi varsa ve 60sn cooldown’a uyuyorsa
  if (WiFi.status() == WL_CONNECTED) {
    delay(2000);
    uint32_t nowMs = millis();
    bool allowBootReport = true;
    if (rtc_lastBootReportMs != 0 && (nowMs - rtc_lastBootReportMs) < BOOT_REPORT_COOLDOWN_MS) {
      allowBootReport = false;
    }
    if (allowBootReport) {
      sendBootReport();
      rtc_lastBootReportMs = nowMs;
    }
  }
}

void loop() {
  // RMS cache update
  if (millis() - lastMeasureMs >= 500) {
    lastMeasureMs = millis();
    mainsV_cache = readMainsV();
    genV_cache   = readGenV();
  }

  wifiManagerTick();
  checkTelegram();

  float mainsV = mainsV_cache;
  float genV   = genV_cache;

  updateGeneratorRunningDebounced(genV);
  handleMainsReturnStopLogic(mainsV);
  handlePeriodicReport(mainsV, genV);

  // Reply timeout
  if (state == ST_WAIT_REPLY && awaitingReply) {
    if (millis() - waitReplyStartMs > REPLY_TIMEOUT_MS) {
      awaitingReply = false;
      state = ST_MONITOR;
      sendTG("⏱️ Cevap gelmedi. İşlem yapılmadı.");
      sendInlineMenu();
    }
  }

  // AUTO mantık
  if (state == ST_MONITOR && autoMode) {
    if (mainsV < AUTO_START_V) {
      if (!autoStartLatched && !generatorRunning) {
        autoStartLatched = true;
        lastAlarm = "🚨 Şebeke < auto_start, otomatik start (" + String(mainsV,1) + "V)";
        sendTG(lastAlarm);
        startProcedureBegin("AUTO: mains < auto_start");
        delay(15);
        return;
      }
    } else {
      if (mainsV > (AUTO_START_V + ALERT_HYST_V)) autoStartLatched = false;
    }

    if (mainsV >= ASK_LOW_V && mainsV <= ASK_HIGH_V) {
      if (!askBandLatched && !generatorRunning) {
        askBandLatched = true;
        askGeneratorQuestion("⚠️ Şebeke düşük (ASK bandı)", mainsV);
        delay(15);
        return;
      }
    } else {
      if (mainsV >= (ASK_HIGH_V + ALERT_HYST_V) || mainsV <= (ASK_LOW_V - ALERT_HYST_V)) {
        askBandLatched = false;
      }
    }

    if (mainsV > MAINS_HIGH_ALERT_V) {
      if (!highAlertLatched) {
        highAlertLatched = true;
        askGeneratorQuestion("⚠️ Şebeke gerilimi yüksek!", mainsV);
        delay(15);
        return;
      }
    } else {
      highAlertLatched = false;
    }

    delay(15);
    return;
  }

  // EVET sonrası: şebeke normal değilse çalıştır
  if (state == ST_PRECHECK_AFTER_YES) {
    float v = mainsV;

    if (v >= MAINS_OK_MIN_V && v <= MAINS_OK_MAX_V) {
      sendTG("✅ Şebeke normale dönmüş (" + String(v,1) + "V). Jeneratör çalıştırmıyorum.");
      state = ST_MONITOR;
      sendInlineMenu();
      return;
    }

    startProcedureBegin("EVET sonrası: kullanıcı onayı (Şebeke " + String(v,1) + "V)");
    return;
  }

  // Start prosedürü
  if (state == ST_STARTING) {
    if (millis() - lastGenCheckMs >= 1000) {
      lastGenCheckMs = millis();
      starterSecondCounter++;

      // Deneme başlangıç bildirimi (ilk saniye)
      if (starterSecondCounter == 1) {
        stat_totalAttempts++;
        stat_lastCycleAttempts = startAttempt + 1;
        saveStatsToNVS();

        sendTG("▶️ Marş denemesi başlıyor: " +
               String(startAttempt + 1) + "/" + String(MAX_START_ATTEMPTS));
      }

      // Start sırasında akü Vmin takip
      float vb = readBatteryV();
      battVlast = vb;
      if (vb < battVminCycle) battVminCycle = vb;

      float g = genV_cache;

      // Akü zayıf olabilir (ZMPT heuristik)
      if (!batteryWeakWarnSent && starterSecondCounter >= BATTERY_WEAK_AFTER_SEC) {
        if (g < BATTERY_WEAK_GEN_V) {
          batteryWeakWarnSent = true;
          sendTG("🔋 (ZMPT) Akü zayıf olabilir / marş zorlanıyor olabilir.\n"
                 "GEN: " + String(g,1) + "V\n"
                 "Deneme: " + String(startAttempt + 1) + "/" + String(MAX_START_ATTEMPTS));
        }
      }

      // Voltaj geldiyse marşı bırak
      if (g > GEN_RUNNING_V) {
        starter(false);
        startAttempt = 0;

        generatorRunning = true;
        genAboveCount = GEN_ON_CONFIRM_COUNT;
        genBelowCount = 0;
        lastPeriodicReportMs = millis();

        stat_totalSuccess++;
        stat_lastCycleResult = "SUCCESS (" + String(stat_lastCycleAttempts) + " deneme)";
        saveStatsToNVS();

        sendTG("✅ Jeneratör çalıştı. Marş başarıyla tamamlandı.\n"
               "GEN " + String(g,1) + "V | Vmin " + String(battVminCycle,2) + "V");
        state = ST_MONITOR;
        sendInlineMenu();
        return;
      }

      // 5sn marş bitti => fail / cooldown / limit
      if (starterSecondCounter >= 5) {
        starter(false);

        startAttempt++;
        int remaining = MAX_START_ATTEMPTS - startAttempt;

        String info = "❌ Marş başarısız. Deneme: " + String(startAttempt) + "/" + String(MAX_START_ATTEMPTS) +
                      " | Kalan: " + String(remaining) +
                      " (Cooldown: " + String(COOLDOWN_SEC) + "sn)\n";
        info += "⛽ FUEL=" + onOffFromPin(PIN_FUEL_RELAY) + " IGN=" + onOffFromPin(PIN_IGN_RELAY);
        info += " | GEN: " + String(g,1) + "V";
        info += "\n🔋 Vmin: " + String(battVminCycle,2) + "V";
        sendTG(info);

        if (startAttempt >= MAX_START_ATTEMPTS) {
          fuel(false);
          ign(false);

          stat_totalFail++;
          stat_lastCycleResult = "FAIL (" + String(MAX_START_ATTEMPTS) + " deneme)";
          saveStatsToNVS();

          sendTG("🛑 START BAŞARISIZ: " + String(MAX_START_ATTEMPTS) +
                 " deneme bitti.\n(IGN+FUEL kapatıldı) Manuel kontrol gerekebilir.");
          state = ST_MONITOR;
          sendInlineMenu();
          return;
        }

        cooldownStartMs = millis();
        state = ST_COOLDOWN;
        return;
      }
    }
    delay(10);
    return;
  }

  // cooldown (COOLDOWN_SEC)
  if (state == ST_COOLDOWN) {
    if (millis() - cooldownStartMs >= (unsigned long)COOLDOWN_SEC * 1000UL) {
      starterSecondCounter = 0;
      lastGenCheckMs = 0;
      starter(true);
      state = ST_STARTING;
      return;
    }
    delay(15);
    return;
  }

  if (state == ST_WAIT_REPLY) {
    delay(15);
    return;
  }

  delay(15);
}
