// =====================
// main.ino (v9.007) — Aşama 9: Şebeke voltaj izleme (RMS)
// + 100ms pencere (SAMPLES=500, SAMPLE_US=200)
// + SENSOR? kriterleri güçlendirme (p2p + rmsCounts + saturation + mean-range)
// + Telegram: /mainsraw (min/max/p2p/mean/rmsCounts/rmsAdcV/vrms/state)
// =====================

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <ArduinoJson.h>
#include <UniversalTelegramBot.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#if ENABLE_NTP
  #include <time.h>
#endif

#include <math.h>

#include "config.h"
#include "sifre.h"

#if !defined(ARDUINO_ARCH_ESP32)
  #error "Bu proje ESP32 Arduino core ile derlenmelidir."
#endif

#ifndef ENABLE_MAINS_MONITOR
  #define ENABLE_MAINS_MONITOR 0
#endif

// ============================================================================
// 0) BUTON SİSTEMİ (Aşama 7)
// ============================================================================
static UiBtnState g_btn[(uint8_t)UiBtnId::COUNT];

static constexpr uint8_t BTN_QN = 16;
static UiBtnEvent g_btnQ[BTN_QN];
static uint8_t g_btnQh = 0;
static uint8_t g_btnQt = 0;

static bool btnQPush(const UiBtnEvent& e) {
  uint8_t next = (uint8_t)((g_btnQh + 1) % BTN_QN);
  if (next == g_btnQt) return false;
  g_btnQ[g_btnQh] = e;
  g_btnQh = next;
  return true;
}
static bool btnQPop(UiBtnEvent& out) {
  if (g_btnQt == g_btnQh) return false;
  out = g_btnQ[g_btnQt];
  g_btnQt = (uint8_t)((g_btnQt + 1) % BTN_QN);
  return true;
}

static const char* btnName(UiBtnId id) {
  switch (id) {
    case UiBtnId::UP: return "UP"; case UiBtnId::DOWN: return "DOWN";
    case UiBtnId::LEFT: return "LEFT"; case UiBtnId::RIGHT: return "RIGHT";
    case UiBtnId::OK: return "OK"; case UiBtnId::BACK: return "BACK";
    default: return "?";
  }
}
static const char* evtName(UiBtnEvt e) {
  switch (e) {
    case UiBtnEvt::PRESS: return "PRESS"; case UiBtnEvt::RELEASE: return "RELEASE";
    case UiBtnEvt::LONG: return "LONG"; case UiBtnEvt::REPEAT: return "REPEAT";
    default: return "?";
  }
}

static bool readPressedRaw(int pin) {
  const int v = digitalRead(pin);
  const bool levelHigh = (v != 0);
  return Buttons::ACTIVE_LOW ? (!levelHigh) : (levelHigh);
}

static void btnInitOne(UiBtnState& b, int pin) {
  b.pin = pin;
  if (pin < 0) { b.enabled = false; return; }

  if (Buttons::USE_PULLUP) pinMode(pin, INPUT_PULLUP);
  else pinMode(pin, INPUT);

  b.enabled = true;
  b.raw = readPressedRaw(pin);
  b.stable = b.raw;
  b.lastRawChangeMs = millis();
  b.pressed = b.stable;
  b.pressStartMs = b.pressed ? millis() : 0;
  b.lastRepeatMs = 0;
  b.longFired = false;
}

static void buttonsInit() {
  btnInitOne(g_btn[(uint8_t)UiBtnId::UP],    Buttons::BTN_UP);
  btnInitOne(g_btn[(uint8_t)UiBtnId::DOWN],  Buttons::BTN_DOWN);
  btnInitOne(g_btn[(uint8_t)UiBtnId::LEFT],  Buttons::BTN_LEFT);
  btnInitOne(g_btn[(uint8_t)UiBtnId::RIGHT], Buttons::BTN_RIGHT);
  btnInitOne(g_btn[(uint8_t)UiBtnId::OK],    Buttons::BTN_OK);
  btnInitOne(g_btn[(uint8_t)UiBtnId::BACK],  Buttons::BTN_BACK);
}

static void buttonsUpdate() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < (uint8_t)UiBtnId::COUNT; i++) {
    UiBtnState& b = g_btn[i];
    if (!b.enabled) continue;

    const bool r = readPressedRaw(b.pin);
    if (r != b.raw) { b.raw = r; b.lastRawChangeMs = now; }

    if ((now - b.lastRawChangeMs) >= Buttons::DEBOUNCE_MS) {
      if (b.stable != b.raw) {
        b.stable = b.raw;
        if (b.stable) {
          b.pressed = true;
          b.pressStartMs = now;
          b.lastRepeatMs = now;
          b.longFired = false;
          btnQPush(UiBtnEvent{ (UiBtnId)i, UiBtnEvt::PRESS, now });
        } else {
          b.pressed = false;
          btnQPush(UiBtnEvent{ (UiBtnId)i, UiBtnEvt::RELEASE, now });
        }
      }
    }

    if (b.pressed) {
      if (!b.longFired && (now - b.pressStartMs) >= Buttons::LONGPRESS_MS) {
        b.longFired = true;
        btnQPush(UiBtnEvent{ (UiBtnId)i, UiBtnEvt::LONG, now });
      }
      if ((now - b.pressStartMs) >= Buttons::REPEAT_START_MS) {
        if ((now - b.lastRepeatMs) >= Buttons::REPEAT_EVERY_MS) {
          b.lastRepeatMs = now;
          btnQPush(UiBtnEvent{ (UiBtnId)i, UiBtnEvt::REPEAT, now });
        }
      }
    }
  }
}

// ============================================================================
// 1) ŞEBEKE (Aşama 9)
// ============================================================================
#if ENABLE_MAINS_MONITOR
static MainsReading g_mains;
static uint32_t g_lastMainsMeasureMs = 0;
static MainsState g_mainsLatchedState = MainsState::SENSOR_INVALID;
static uint32_t g_lastMainsAlertMs = 0;

static const char* mainsStateName(MainsState s) {
  switch (s) {
    case MainsState::SENSOR_INVALID: return "SENSOR?";
    case MainsState::UNDERVOLT:      return "LOW";
    case MainsState::NOMINAL:        return "OK";
    case MainsState::OVERVOLT:       return "HIGH";
    default:                         return "?";
  }
}

static MainsState classifyMains(float v) {
  if (v < Mains::LOW_V)  return MainsState::UNDERVOLT;
  if (v > Mains::HIGH_V) return MainsState::OVERVOLT;
  return MainsState::NOMINAL;
}

static MainsState applyHysteresis(MainsState current, float v) {
  if (current == MainsState::SENSOR_INVALID) return classifyMains(v);

  if (current == MainsState::UNDERVOLT) {
    if (v >= (Mains::LOW_V + Mains::HYST_V)) return MainsState::NOMINAL;
    return MainsState::UNDERVOLT;
  }
  if (current == MainsState::OVERVOLT) {
    if (v <= (Mains::HIGH_V - Mains::HYST_V)) return MainsState::NOMINAL;
    return MainsState::OVERVOLT;
  }

  // NOMINAL
  if (v <= (Mains::LOW_V - Mains::HYST_V))  return MainsState::UNDERVOLT;
  if (v >= (Mains::HIGH_V + Mains::HYST_V)) return MainsState::OVERVOLT;
  return MainsState::NOMINAL;
}

static void mainsInitAdc() {
  analogReadResolution(Mains::ADC_BITS);
  analogSetPinAttenuation(Mains::ADC_PIN, ADC_11db);
  pinMode(Mains::ADC_PIN, INPUT);
}

// Güçlendirilmiş SENSOR? kriterleri burada
static MainsReading readMainsOnce() {
  MainsReading r;
  const uint16_t N = Mains::SAMPLES;

  float sum = 0.0f;
  float sumsq = 0.0f;
  int mn = 4095, mx = 0;

  for (uint16_t i = 0; i < N; i++) {
    int raw = analogRead(Mains::ADC_PIN);
    if (raw < mn) mn = raw;
    if (raw > mx) mx = raw;

    sum   += (float)raw;
    sumsq += (float)raw * (float)raw;

    if ((i % 40) == 0) yield();

    if (Mains::SAMPLE_US > 0) {
      const uint32_t t0 = micros();
      while ((uint32_t)(micros() - t0) < (uint32_t)Mains::SAMPLE_US) { }
    }
  }

  r.minRaw = mn;
  r.maxRaw = mx;

  const int p2p = mx - mn;
  const float mean = sum / (float)N;
  r.meanCounts = mean;

  const float var0 = (sumsq / (float)N) - (mean * mean);
  float var = var0;
  if (var < 0.0f) var = 0.0f;
  r.rmsCounts = sqrtf(var);

  const float adcMax = (float)((1u << Mains::ADC_BITS) - 1u);

  // 1) p2p küçükse
  if (p2p < Mains::MIN_P2P_COUNTS) {
    r.valid = false;
    r.state = MainsState::SENSOR_INVALID;
    return r;
  }

  // 2) rmsCounts çok küçükse (gürültü/flatline)
  if (r.rmsCounts < Mains::MIN_RMS_COUNTS) {
    r.valid = false;
    r.state = MainsState::SENSOR_INVALID;
    return r;
  }

  // 3) saturation kontrolü (0 veya max'a yapışıyorsa)
  if (mn <= Mains::SAT_MARGIN_COUNTS || mx >= ((int)adcMax - Mains::SAT_MARGIN_COUNTS)) {
    r.valid = false;
    r.state = MainsState::SENSOR_INVALID;
    return r;
  }

  // 4) mean aşırı uca kaydıysa (DC offset bozuk/kablo vs.)
  const float meanFrac = mean / adcMax;
  if (meanFrac < Mains::MEAN_MIN_FRAC || meanFrac > Mains::MEAN_MAX_FRAC) {
    r.valid = false;
    r.state = MainsState::SENSOR_INVALID;
    return r;
  }

  // RMS ADC volt
  r.rmsAdcV = (r.rmsCounts / adcMax) * Mains::ADC_REF_V;

  // Şebeke Vrms
  r.vrms = r.rmsAdcV * Mains::CAL_V_PER_ADC_RMS;

  // Sanity
  if (!(r.vrms >= 1.0f && r.vrms <= 400.0f)) {
    r.valid = false;
    r.state = MainsState::SENSOR_INVALID;
    return r;
  }

  r.valid = true;
  r.state = classifyMains(r.vrms);
  return r;
}

static String mainsShortLine() {
  if (!g_mains.valid) return "MAINS: SENSOR?";
  return "MAINS: " + String(g_mains.vrms, 1) + "V " + String(mainsStateName(g_mains.state));
}

static String mainsRawText() {
  const int p2p = g_mains.maxRaw - g_mains.minRaw;
  String s;
  s += "MAINS RAW\n";
  s += "min/max: " + String(g_mains.minRaw) + "/" + String(g_mains.maxRaw) + " (p2p=" + String(p2p) + ")\n";
  s += "mean: " + String(g_mains.meanCounts, 1) + " cnt\n";
  s += "rmsCounts: " + String(g_mains.rmsCounts, 3) + " cnt\n";
  s += "rmsAdcV: " + String(g_mains.rmsAdcV, 4) + " V\n";
  s += "vrms: " + String(g_mains.vrms, 1) + " V\n";
  s += "state: " + String(mainsStateName(g_mains.valid ? g_mains.state : MainsState::SENSOR_INVALID));
  return s;
}
#endif

// ============================================================================
// 2) LCD + WiFi ikon + durum
// ============================================================================
static SPIClass* tftSPI = &SPI;
static Adafruit_ILI9341 tft(Pins::TFT_CS, Pins::TFT_DC, Pins::TFT_RST);

static uint32_t lastBeatMs    = 0;
static uint32_t lastWiFiTryMs = 0;

static String   lastCmd = "-";
static uint32_t lastCmdMs = 0;
static uint32_t tgMsgCount = 0;
static bool     lastCmdAuthorized = false;

// ✅ SADECE 1 kere
static String   g_lastBtnText = "-";
static uint32_t g_lastBtnMs   = 0;

#if ENABLE_NTP
static bool     g_timeValid = false;
static uint32_t g_lastNtpRequestMs = 0;
static int      g_lastDailySyncYday = -1;
static uint32_t g_ntpRequestCount = 0;
#endif

static uint32_t g_lastClockDrawMs = 0;

static void backlightOn() {
#if USE_LCD
  if (Pins::TFT_BL < 0) return;
  pinMode(Pins::TFT_BL, OUTPUT);
  digitalWrite(Pins::TFT_BL, Pins::TFT_BL_ACTIVE_HIGH ? HIGH : LOW);
#endif
}
static void backlightOff() {
#if USE_LCD
  if (Pins::TFT_BL < 0) return;
  pinMode(Pins::TFT_BL, OUTPUT);
  digitalWrite(Pins::TFT_BL, Pins::TFT_BL_ACTIVE_HIGH ? LOW : HIGH);
#endif
}

static void assertChipIsS3() {
  const String model = ESP.getChipModel();
  if (!model.startsWith("ESP32-S3")) {
    Serial.println(F("!!! HATA: Bu firmware ESP32-S3 icin tasarlandi !!!"));
    Serial.println(model);
    while (true) { delay(1000); }
  }
}

static void printBootInfo() {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F("Project: ")); Serial.println(PROJECT_NAME);
  Serial.print(F("Version: ")); Serial.println(PROJECT_VERSION);
  Serial.print(F("Stage:   ")); Serial.println(PROJECT_STAGE);
  Serial.print(F("Chip:    ")); Serial.println(ESP.getChipModel());
  Serial.println(F("========================================"));
}

static String ipToString(const IPAddress& ip) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

static uint8_t rssiToBars(int rssi) {
  if (rssi <= -90) return 1;
  if (rssi <= -80) return 2;
  if (rssi <= -70) return 3;
  return 4;
}

static int16_t uiWiFiIconX() {
#if USE_LCD
  return tft.width() - 34;
#else
  return 0;
#endif
}

static void drawWiFiIcon(int16_t x, int16_t y, bool connected, int rssi) {
#if USE_LCD
  tft.fillRect(x, y, 30, 30, ILI9341_BLACK);
  tft.drawRect(x, y, 30, 30, ILI9341_DARKGREY);

  uint8_t bars = connected ? rssiToBars(rssi) : 0;

  const int16_t baseY = y + 26;
  const int16_t barW = 5;
  const int16_t gap  = 2;
  const int16_t h1 = 6, h2 = 10, h3 = 14, h4 = 18;

  const uint16_t onColor  = connected ? ILI9341_GREEN : ILI9341_RED;
  const uint16_t offColor = ILI9341_DARKGREY;

  int16_t bx = x + 5;
  tft.fillRect(bx, baseY - h1, barW, h1, (bars >= 1) ? onColor : offColor); bx += barW + gap;
  tft.fillRect(bx, baseY - h2, barW, h2, (bars >= 2) ? onColor : offColor); bx += barW + gap;
  tft.fillRect(bx, baseY - h3, barW, h3, (bars >= 3) ? onColor : offColor); bx += barW + gap;
  tft.fillRect(bx, baseY - h4, barW, h4, (bars >= 4) ? onColor : offColor);

  if (!connected) {
    tft.drawLine(x + 6, y + 6, x + 24, y + 24, ILI9341_RED);
    tft.drawLine(x + 24, y + 6, x + 6, y + 24, ILI9341_RED);
  }
#else
  (void)x; (void)y; (void)connected; (void)rssi;
#endif
}

#if ENABLE_NTP
static bool isTimeValidNow() {
  time_t now = time(nullptr);
  return (now > 1600000000);
}

static String formatLocalTimeHM() {
  if (!isTimeValidNow()) return "--:--";
  struct tm tmNow;
  if (!getLocalTime(&tmNow, 50)) return "--:--";
  char buf[8];
  strftime(buf, sizeof(buf), "%H:%M", &tmNow);
  return String(buf);
}

static String formatLocalTimeShort() {
  if (!isTimeValidNow()) return "--:--:--";
  struct tm tmNow;
  if (!getLocalTime(&tmNow, 50)) return "--:--:--";
  char buf[16];
  strftime(buf, sizeof(buf), "%H:%M:%S", &tmNow);
  return String(buf);
}

static String formatLocalTimeFull() {
  if (!isTimeValidNow()) return "---- -- -- --:--:--";
  struct tm tmNow;
  if (!getLocalTime(&tmNow, 200)) return "---- -- -- --:--:--";
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmNow);
  return String(buf);
}

static void requestNtpSync(const char* reason) {
  if (WiFi.status() != WL_CONNECTED) return;
  g_ntpRequestCount++;
  g_lastNtpRequestMs = millis();
  Serial.print(F("[NTP] request sync (")); Serial.print(reason); Serial.println(F(")"));
  configTzTime(NTP_TZ, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
}
#endif

// --- SOL ÜST SAAT (HH:MM) ---
static void uiDrawClockTopLeft() {
#if USE_LCD
  tft.fillRect(0, 0, 110, 26, ILI9341_BLACK);
  tft.setCursor(8, 8);
  tft.setTextSize(2);
#if ENABLE_NTP
  const String hm = formatLocalTimeHM();
  tft.setTextColor(isTimeValidNow() ? ILI9341_WHITE : ILI9341_DARKGREY);
  tft.print(hm);
#else
  tft.setTextColor(ILI9341_DARKGREY);
  tft.print("--:--");
#endif
#endif
}

// --- "JENERATÖR" başlığı: "JENERATOR" + O üstüne iki nokta ---
static void uiDrawTitleGenerator(int16_t x, int16_t y) {
#if USE_LCD
  tft.setCursor(x, y);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("JENERATOR");

  const int16_t charAdv = 12;
  const int16_t oLeft = x + 7 * charAdv;

  const int16_t dotY = y - 2;
  const int16_t dotX1 = oLeft + 2;
  const int16_t dotX2 = oLeft + 7;
  tft.fillRect(dotX1, dotY, 2, 2, ILI9341_WHITE);
  tft.fillRect(dotX2, dotY, 2, 2, ILI9341_WHITE);
#else
  (void)x; (void)y;
#endif
}

static void uiSetLine(int16_t x, int16_t y, const String& value, uint16_t color = ILI9341_WHITE) {
#if USE_LCD
  const int16_t w = tft.width() - x - 8;
  tft.fillRect(x, y, w, 12, ILI9341_BLACK);
  tft.setCursor(x, y);
  tft.setTextSize(1);
  tft.setTextColor(color);
  tft.print(value);
#endif
}

static void uiHeader() {
#if USE_LCD
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextWrap(false);

  uiDrawClockTopLeft();
  drawWiFiIcon(uiWiFiIconX(), 4, (WiFi.status() == WL_CONNECTED), WiFi.RSSI());
  uiDrawTitleGenerator(120, 8);

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_LIGHTGREY);
  tft.setCursor(8, 30);
  tft.print(PROJECT_VERSION);
  tft.print(" / ");
  tft.print(PROJECT_STAGE);

  tft.drawFastHLine(0, 42, tft.width(), ILI9341_DARKGREY);

  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(8, 52);  tft.print("WiFi :");
  tft.setCursor(8, 70);  tft.print("IP   :");
  tft.setCursor(8, 88);  tft.print("RSSI :");
  tft.setCursor(8, 106); tft.print("TG   :");
  tft.setCursor(8, 124); tft.print("AUTH :");
  tft.setCursor(8, 142); tft.print("CMD  :");
  tft.setCursor(8, 160); tft.print("MAINS:");
  tft.setCursor(8, 178); tft.print("SYS  :");
  tft.setCursor(8, 196); tft.print("BTN  :");

  tft.drawFastHLine(0, 214, tft.width(), ILI9341_DARKGREY);
  tft.setCursor(8, 224);
  tft.setTextColor(ILI9341_LIGHTGREY);
  tft.print("Komut: /durum /mains /mainsraw");
#endif
}

static void uiUpdate() {
#if USE_LCD
  const bool wifiOk = (WiFi.status() == WL_CONNECTED);

  drawWiFiIcon(uiWiFiIconX(), 4, wifiOk, WiFi.RSSI());

  uiSetLine(60, 52, wifiOk ? "OK" : "YOK", wifiOk ? ILI9341_GREEN : ILI9341_RED);
  uiSetLine(60, 70, wifiOk ? ipToString(WiFi.localIP()) : "-");
  uiSetLine(60, 88, wifiOk ? (String(WiFi.RSSI()) + " dBm") : "-");
  uiSetLine(60, 106, wifiOk ? ("OK (" + String(tgMsgCount) + ")") : "Bekle...");

  uiSetLine(60, 124, lastCmdAuthorized ? "OK" : "NO", lastCmdAuthorized ? ILI9341_GREEN : ILI9341_RED);

  const String cmdAge = (lastCmdMs == 0) ? "-" : (String((millis() - lastCmdMs) / 1000) + "s");
  uiSetLine(60, 142, lastCmd + " (" + cmdAge + ")", ILI9341_YELLOW);

#if ENABLE_MAINS_MONITOR
  if (!g_mains.valid) {
    // kısa, ekrana sığacak debug
    const int p2p = g_mains.maxRaw - g_mains.minRaw;
    uiSetLine(60, 160, "SENSOR? p2p=" + String(p2p) + " rms=" + String(g_mains.rmsCounts, 1), ILI9341_RED);
  } else {
    uint16_t c = ILI9341_GREEN;
    if (g_mains.state == MainsState::UNDERVOLT) c = ILI9341_YELLOW;
    if (g_mains.state == MainsState::OVERVOLT)  c = ILI9341_YELLOW;
    uiSetLine(60, 160, String(g_mains.vrms, 1) + "V " + String(mainsStateName(g_mains.state)), c);
  }
#else
  uiSetLine(60, 160, "-", ILI9341_DARKGREY);
#endif

#if ENABLE_NTP
  const String sysLine = "Time " + formatLocalTimeShort() + "  Up " + String(millis() / 1000) + "s  Heap " + String(ESP.getFreeHeap() / 1024) + "KB";
  uiSetLine(60, 178, sysLine, isTimeValidNow() ? ILI9341_WHITE : ILI9341_DARKGREY);
#else
  uiSetLine(60, 178, "Up " + String(millis() / 1000) + "s  Heap " + String(ESP.getFreeHeap() / 1024) + "KB", ILI9341_WHITE);
#endif

  const String btnAge = (g_lastBtnMs == 0) ? "" : (" (" + String((millis() - g_lastBtnMs) / 1000) + "s)");
  uiSetLine(60, 196, g_lastBtnText + btnAge, ILI9341_CYAN);
#endif
}

// ============================================================================
// 3) Telegram
// ============================================================================
static WiFiClientSecure tgClient;
static UniversalTelegramBot bot(BOT_TOKEN, tgClient);
static uint32_t lastTgPollMs  = 0;

static String tgNormalizeCmd(const String& raw) {
  String t = raw;
  t.trim();
  int sp = t.indexOf(' ');
  if (sp >= 0) t = t.substring(0, sp);
  int at = t.indexOf('@');
  if (at >= 0) t = t.substring(0, at);
  t.trim();
  return t;
}

static int64_t toInt64Safe(const String& s) {
  String t = s; t.trim();
  if (!t.length()) return 0;
  return (int64_t)strtoll(t.c_str(), nullptr, 10);
}

static bool isAdmin(int64_t userId) {
  if (userId == (int64_t)MASTER_ADMIN_ID) return true;
  for (size_t i = 0; i < (sizeof(TgAuth::admin_list) / sizeof(TgAuth::admin_list[0])); i++) {
    if ((int64_t)TgAuth::admin_list[i] == userId) return true;
  }
  return false;
}

static void tgSendToConfiguredChat(const String& msg, const String& parseMode = "") {
  bot.sendMessage(String(CHAT_ID), msg, parseMode);
}

static void tgSendHelp(const String& chat_id) {
  bot.sendMessage(chat_id, "Komutlar: /durum /mains /mainsraw /time /sync /ping /whoami /yardim", "");
}

static void tgSendWhoAmI(const String& chat_id, int64_t uid) {
  String s;
  s += "UID: " + String((long long)uid) + "\n";
  s += "CHAT: " + chat_id + "\n";
  s += "Admin: " + String(isAdmin(uid) ? "YES" : "NO");
  bot.sendMessage(chat_id, s, "");
}

static void tgSendStatus(const String& chat_id) {
  String s;
  s += String(PROJECT_VERSION) + " / " + String(PROJECT_STAGE) + "\n";
  s += "WiFi: " + String((WiFi.status() == WL_CONNECTED) ? "OK" : "YOK") + "\n";
  if (WiFi.status() == WL_CONNECTED) {
    s += "IP: " + ipToString(WiFi.localIP()) + "\n";
    s += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
  }
#if ENABLE_NTP
  s += "Time: " + formatLocalTimeFull() + "\n";
#endif
#if ENABLE_MAINS_MONITOR
  s += mainsShortLine() + "\n";
#endif
  s += "Up: " + String(millis() / 1000) + "s\n";
  bot.sendMessage(chat_id, s, "");
}

static void handleTelegramMessage(int i) {
  const String chat_id = bot.messages[i].chat_id;
  String text = bot.messages[i].text; text.trim();

  const String cmd = tgNormalizeCmd(text);
  const int64_t uid = toInt64Safe(bot.messages[i].from_id);
  const bool admin = isAdmin(uid);

  tgMsgCount++;
  lastCmd = text;
  lastCmdMs = millis();

  const bool chatOk = (!TG_STRICT_CHAT_ID) || (chat_id == String(CHAT_ID));

  if ((cmd == "/help" || cmd == "/yardim" || cmd == "/yardım" || cmd == "/start") && TG_ALLOW_HELP_FOR_ALL) {
    tgSendHelp(chat_id);
    lastCmdAuthorized = true;
    return;
  }
  if ((cmd == "/whoami") && TG_ALLOW_WHOAMI_FOR_ALL) {
    tgSendWhoAmI(chat_id, uid);
    lastCmdAuthorized = true;
    return;
  }

  if (!chatOk) { lastCmdAuthorized = false; return; }

  lastCmdAuthorized = (!TG_REQUIRE_AUTH) ? true : admin;
  if (TG_REQUIRE_AUTH && !admin) { bot.sendMessage(chat_id, "⛔ Yetkisiz. /whoami", ""); return; }

  if (cmd == "/ping") { bot.sendMessage(chat_id, "pong ✅", ""); return; }
  if (cmd == "/durum") { tgSendStatus(chat_id); return; }

#if ENABLE_MAINS_MONITOR
  if (cmd == "/mains" || cmd == "/sebekev") {
    bot.sendMessage(chat_id, mainsShortLine(), "");
    return;
  }
  if (cmd == "/mainsraw") {
    bot.sendMessage(chat_id, mainsRawText(), "");
    return;
  }
#endif

  if (cmd == "/time" || cmd == "/saat") {
#if ENABLE_NTP
    bot.sendMessage(chat_id, formatLocalTimeFull(), "");
#else
    bot.sendMessage(chat_id, "NTP kapali", "");
#endif
    return;
  }

  if (cmd == "/sync" || cmd == "/ntp") {
#if ENABLE_NTP
    requestNtpSync("tg_cmd");
    bot.sendMessage(chat_id, "NTP sync istendi ✅", "");
#else
    bot.sendMessage(chat_id, "NTP kapali", "");
#endif
    return;
  }

  bot.sendMessage(chat_id, "Komut taninmadi. /yardim", "");
}

static void pollTelegram() {
  if (WiFi.status() != WL_CONNECTED) return;
  const uint32_t now = millis();
  if (now - lastTgPollMs < TG_POLL_MS) return;
  lastTgPollMs = now;

  int num = bot.getUpdates(bot.last_message_received + 1);
  while (num) {
    for (int i = 0; i < num; i++) { handleTelegramMessage(i); yield(); }
    num = bot.getUpdates(bot.last_message_received + 1);
  }
}

// ============================================================================
// 4) WiFi connect/reconnect
// ============================================================================
static bool connectWiFiOnce() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250); yield();
  }
  return (WiFi.status() == WL_CONNECTED);
}

static void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  const uint32_t now = millis();
  if (now - lastWiFiTryMs < WIFI_RECONNECT_EVERY_MS) return;
  lastWiFiTryMs = now;
  connectWiFiOnce();
}

// ============================================================================
// 5) NTP bakım (Aşama 8)
// ============================================================================
#if ENABLE_NTP
static void ntpTryMarkValid() {
  const bool v = isTimeValidNow();
  if (v && !g_timeValid) g_timeValid = true;
}

static void ntpMaintenance() {
  if (WiFi.status() != WL_CONNECTED) return;

  ntpTryMarkValid();

  if (g_lastNtpRequestMs == 0 || (millis() - g_lastNtpRequestMs) >= NTP_FORCE_RESYNC_EVERY_MS) {
    requestNtpSync("periodic");
  }

  if (!NTP_DAILY_RESYNC_ENABLE) return;
  if (!isTimeValidNow()) return;

  struct tm tmNow;
  if (!getLocalTime(&tmNow, 20)) return;

  const bool atTime =
    (tmNow.tm_hour == (int)NTP_DAILY_RESYNC_HOUR) &&
    (tmNow.tm_min  == (int)NTP_DAILY_RESYNC_MIN)  &&
    (tmNow.tm_sec  <= 3);

  if (atTime && tmNow.tm_yday != g_lastDailySyncYday) {
    g_lastDailySyncYday = tmNow.tm_yday;
    requestNtpSync("daily_03");
  }
}
#endif

// ============================================================================
// 6) Buton event handler
// ============================================================================
static void onBtnEvent(const UiBtnEvent& e) {
  g_lastBtnText = String(btnName(e.id)) + " " + String(evtName(e.evt));
  g_lastBtnMs   = e.ms;

  if (e.id == UiBtnId::OK && e.evt == UiBtnEvt::LONG) {
    static bool bl = true;
    bl = !bl;
    if (bl) backlightOn(); else backlightOff();
  }

#if ENABLE_NTP
  if (e.id == UiBtnId::BACK && e.evt == UiBtnEvt::LONG) {
    requestNtpSync("btn_back_long");
  }
#endif
}

// ============================================================================
// 7) Mains maintenance + Telegram alert
// ============================================================================
#if ENABLE_MAINS_MONITOR
static void maybeSendMainsAlert(MainsState newState) {
  if (!Mains::TG_ALERT_ENABLE) return;

  const uint32_t now = millis();
  if (now - g_lastMainsAlertMs < Mains::TG_ALERT_MIN_INTERVAL_MS) return;
  if (newState == g_mainsLatchedState) return;

  g_lastMainsAlertMs = now;
  g_mainsLatchedState = newState;

  String msg;
  if (newState == MainsState::SENSOR_INVALID) {
    msg = "🧩 Şebeke ölçümü geçersiz: SENSÖR? (kontrol et)";
  } else if (newState == MainsState::UNDERVOLT) {
    msg = "⚠️ Şebeke düşük: " + String(g_mains.vrms, 1) + "V";
  } else if (newState == MainsState::OVERVOLT) {
    msg = "⚠️ Şebeke yüksek: " + String(g_mains.vrms, 1) + "V";
  } else {
    msg = "✅ Şebeke normal: " + String(g_mains.vrms, 1) + "V";
  }
  tgSendToConfiguredChat(msg, "");
}

static void mainsMaintenance() {
  const uint32_t now = millis();
  if (now - g_lastMainsMeasureMs < Mains::MEASURE_EVERY_MS) return;
  g_lastMainsMeasureMs = now;

  const MainsReading r = readMainsOnce();

  MainsReading next = r;
  if (!r.valid) next.state = MainsState::SENSOR_INVALID;
  else next.state = g_mains.valid ? applyHysteresis(g_mains.state, r.vrms) : classifyMains(r.vrms);

  g_mains = next;

  Serial.print(F("[MAINS] "));
  if (!g_mains.valid) {
    Serial.print(F("INVALID "));
    Serial.print(F("min=")); Serial.print(g_mains.minRaw);
    Serial.print(F(" max=")); Serial.print(g_mains.maxRaw);
    Serial.print(F(" mean=")); Serial.print(g_mains.meanCounts, 1);
    Serial.print(F(" rms=")); Serial.print(g_mains.rmsCounts, 3);
  } else {
    Serial.print(g_mains.vrms, 1);
    Serial.print(F("V "));
    Serial.print(mainsStateName(g_mains.state));
    Serial.print(F(" (p2p="));
    Serial.print(g_mains.maxRaw - g_mains.minRaw);
    Serial.print(F(" rmsAdcV="));
    Serial.print(g_mains.rmsAdcV, 4);
    Serial.print(F(")"));
  }
  Serial.println();

  maybeSendMainsAlert(g_mains.valid ? g_mains.state : MainsState::SENSOR_INVALID);
}
#endif

// ============================================================================
// Arduino
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(80);

  assertChipIsS3();
  printBootInfo();

#if USE_LCD
  backlightOn();
  tftSPI->begin(Pins::TFT_SCK, Pins::TFT_MISO, Pins::TFT_MOSI, Pins::TFT_CS);
  tftSPI->setFrequency(TFT_SPI_HZ);
  tft.begin();
  tft.setRotation(TFT_ROTATION);
  uiHeader();
  uiUpdate();
#endif

  buttonsInit();

#if ENABLE_MAINS_MONITOR
  mainsInitAdc();
#endif

  connectWiFiOnce();
  tgClient.setInsecure();

#if ENABLE_NTP
  if (WiFi.status() == WL_CONNECTED) {
    requestNtpSync("boot");
    const uint32_t t0 = millis();
    while ((millis() - t0) < NTP_BOOT_WAIT_MS) {
      if (isTimeValidNow()) break;
      delay(200); yield();
    }
    ntpTryMarkValid();
  }
#endif

  if (TG_SEND_BOOT_MESSAGE) {
    tgSendToConfiguredChat("Boot " + String(PROJECT_VERSION), "");
  }
}

void loop() {
  const uint32_t now = millis();

  ensureWiFi();
  pollTelegram();

#if ENABLE_NTP
  ntpMaintenance();
#endif

#if ENABLE_MAINS_MONITOR
  mainsMaintenance();
#endif

  buttonsUpdate();
  UiBtnEvent e;
  while (btnQPop(e)) {
    onBtnEvent(e);
    uiUpdate();
    yield();
  }

#if USE_LCD
  if (now - g_lastClockDrawMs >= 1000) {
    g_lastClockDrawMs = now;
    uiDrawClockTopLeft();
  }
#endif

  if (now - lastBeatMs >= HEARTBEAT_MS) {
    lastBeatMs = now;
    uiUpdate();
  }

  delay(10);
  yield();
}
