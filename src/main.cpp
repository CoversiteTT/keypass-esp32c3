#include <Arduino.h>
#include <BleKeyboard.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <Wire.h>
#if defined(USE_NIMBLE)
#include <NimBLEDevice.h>
#endif
#if __has_include("user_secrets.h")
#include "user_secrets.h"
#endif

// ====== User settings ======
static const char* BLE_DEVICE_NAME = "keypass";
static const char* BLE_MANUFACTURER = "keypass";

static const uint8_t TOUCH_PIN = 0;               // TTP223 OUT -> GPIO0
static const unsigned long DEBOUNCE_MS = 35;
static const unsigned long LONG_PRESS_MS = 600;   // Long press = confirm
static const unsigned long DOUBLE_CLICK_MS = 320; // Double click = back
static const unsigned long BLE_COOLDOWN_MS = 800;
static const unsigned long BLE_READY_AFTER_CONNECT_MS = 1200;
static const bool BLE_RELAXED_SECURITY = true;

static const int AIR_UART_RX_PIN = 20;            // ESP RX pad <- Sensor A(TX)
static const int AIR_UART_TX_PIN = 21;            // ESP TX pad -> Sensor B(RX), optional
static const uint32_t AIR_UART_BAUD = 9600;
static const unsigned long REPORT_INTERVAL_MS = 2000;

static const int OLED_SDA_PIN = 5;                // Built-in OLED SDA
static const int OLED_SCL_PIN = 6;                // Built-in OLED SCL
static const unsigned long OLED_REFRESH_MS = 16;  // ~60Hz

static const unsigned long SAMPLE_INTERVAL_MS = 22;             // ~45Hz
static const unsigned long SECOND_WINDOW_MS = 1000;
static const unsigned long HISTORY_WINDOW_MS = 10UL * 60UL * 1000UL;  // 10 min

static const uint8_t REALTIME_SECONDS = 60;       // 1 minute
static const uint16_t HISTORY_SLOTS = 288;        // 48h / 10min

static const uint8_t CHART_X = 12;
static const uint8_t CHART_Y = 11;
static const uint8_t CHART_W = 60;
static const uint8_t CHART_H = 18;
// ===========================

struct KeyPreset {
  const char* name;
  const char* text;
};

#ifndef KEY_PRESET_1_NAME
#define KEY_PRESET_1_NAME "Qx"
#endif
#ifndef KEY_PRESET_1_TEXT
#define KEY_PRESET_1_TEXT "change_me_1"
#endif
#ifndef KEY_PRESET_2_NAME
#define KEY_PRESET_2_NAME "P2"
#endif
#ifndef KEY_PRESET_2_TEXT
#define KEY_PRESET_2_TEXT "change_me_2"
#endif
#ifndef KEY_PRESET_3_NAME
#define KEY_PRESET_3_NAME "P3"
#endif
#ifndef KEY_PRESET_3_TEXT
#define KEY_PRESET_3_TEXT "change_me_3"
#endif

static const KeyPreset KEY_PRESETS[] = {
    {KEY_PRESET_1_NAME, KEY_PRESET_1_TEXT},
    {KEY_PRESET_2_NAME, KEY_PRESET_2_TEXT},
    {KEY_PRESET_3_NAME, KEY_PRESET_3_TEXT},
};
static const uint8_t KEY_PRESET_COUNT = sizeof(KEY_PRESETS) / sizeof(KEY_PRESETS[0]);
static_assert(KEY_PRESET_COUNT > 0, "At least one key preset is required.");

enum class Screen { Root, Key, Air, AirView, Settings };
enum class RootItem : uint8_t { Key = 0, Air = 1, Settings = 2 };
enum class AirItem : uint8_t { Co2 = 0, Tvoc = 1, Hcho = 2 };
enum class PlotMode : uint8_t { Realtime = 0, History = 1 };
enum class ButtonEvent { None, SingleClick, DoubleClick, LongPress };
enum class SettingItem : uint8_t { AutoEnter = 0, AirStartMode = 1 };

static const uint8_t METRIC_COUNT = 3;

struct AirData {
  bool valid = false;
  uint16_t tvocRaw = 0;
  uint16_t hchoRaw = 0;
  uint16_t co2Raw = 0;
  unsigned long updatedMs = 0;
};

struct RealtimeSeries {
  uint16_t avg[REALTIME_SECONDS] = {0};
  uint16_t peak[REALTIME_SECONDS] = {0};
  uint8_t head = 0;  // Next write index
  uint8_t count = 0;
};

struct HistorySeries {
  uint16_t avg[HISTORY_SLOTS] = {0};
  uint16_t peak[HISTORY_SLOTS] = {0};
  uint16_t head = 0;  // Next write index
  uint16_t count = 0;
};

struct RunningAccum {
  uint32_t sum = 0;
  uint16_t peak = 0;
  uint32_t count = 0;
};

struct ChartDataPoint {
  uint16_t avg = 0;
  uint16_t peak = 0;
  bool valid = false;
};

struct GraphScale {
  bool valid = false;
  uint16_t minVal = 0;
  uint16_t maxVal = 0;
};

class UartAirParser {
 public:
  bool feed(uint8_t byteIn, AirData& out) {
    switch (state_) {
      case State::WaitHeader1:
        if (byteIn == 0x2C) {
          frame_[0] = byteIn;
          state_ = State::WaitHeader2;
        }
        break;
      case State::WaitHeader2:
        if (byteIn == 0xE4) {
          frame_[1] = byteIn;
          payloadIdx_ = 0;
          state_ = State::ReadPayload;
        } else if (byteIn == 0x2C) {
          frame_[0] = byteIn;
        } else {
          state_ = State::WaitHeader1;
        }
        break;
      case State::ReadPayload:
        frame_[2 + payloadIdx_] = byteIn;
        payloadIdx_++;
        if (payloadIdx_ >= 7) {
          state_ = State::WaitHeader1;
          return decodeFrame(out);
        }
        break;
    }
    return false;
  }

 private:
  enum class State { WaitHeader1, WaitHeader2, ReadPayload };

  bool decodeFrame(AirData& out) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      checksum = static_cast<uint8_t>(checksum + frame_[i]);
    }
    if (checksum != frame_[8]) {
      return false;
    }

    out.tvocRaw = static_cast<uint16_t>((frame_[2] << 8) | frame_[3]);
    out.hchoRaw = static_cast<uint16_t>((frame_[4] << 8) | frame_[5]);
    out.co2Raw = static_cast<uint16_t>((frame_[6] << 8) | frame_[7]);
    out.valid = true;
    out.updatedMs = millis();
    return true;
  }

  State state_ = State::WaitHeader1;
  uint8_t payloadIdx_ = 0;
  uint8_t frame_[9] = {0};
};

BleKeyboard bleKeyboard(BLE_DEVICE_NAME, BLE_MANUFACTURER, 100);
HardwareSerial AirSerial(1);
U8G2_SSD1306_72X40_ER_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
Preferences prefs;
UartAirParser airParser;

AirData latestAirData;

RealtimeSeries realtimeSeries[METRIC_COUNT];
HistorySeries historySeries[METRIC_COUNT];
RunningAccum secondAccum[METRIC_COUNT];
RunningAccum tenMinAccum[METRIC_COUNT];

Screen currentScreen = Screen::Root;
uint8_t selectedRoot = static_cast<uint8_t>(RootItem::Key);
uint8_t selectedKey = 0;
uint8_t selectedAir = static_cast<uint8_t>(AirItem::Co2);
uint8_t selectedSetting = static_cast<uint8_t>(SettingItem::AutoEnter);
AirItem activeAirSelection = AirItem::Co2;
PlotMode plotMode = PlotMode::Realtime;
bool settingAutoEnter = true;
PlotMode settingAirStartMode = PlotMode::Realtime;
bool lastBleConnectedState = false;
unsigned long bleConnectedSinceMs = 0;
uint32_t bleConnectCount = 0;
uint32_t bleDisconnectCount = 0;

char serialCmdBuf[128] = {0};
uint8_t serialCmdLen = 0;

bool lastStableTouchState = false;
bool lastRawTouchState = false;
unsigned long lastRawChangeMs = 0;
unsigned long pressStartMs = 0;
bool longPressTriggered = false;
bool pendingSingleClick = false;
unsigned long lastShortReleaseMs = 0;

unsigned long lastBleSendMs = 0;
unsigned long lastReportMs = 0;
unsigned long lastOledRefreshMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastSampleTickMs = 0;
unsigned long secondWindowStartMs = 0;
unsigned long tenMinWindowStartMs = 0;

char statusText[22] = "Ready";
unsigned long statusUntilMs = 0;

float rawToMgPerM3(uint16_t raw) { return static_cast<float>(raw) * 0.001f; }

const char* screenName(Screen s) {
  switch (s) {
    case Screen::Root:
      return "ROOT";
    case Screen::Key:
      return "KEY";
    case Screen::Air:
      return "AIR";
    case Screen::AirView:
      return "AIRV";
    case Screen::Settings:
      return "SET";
  }
  return "?";
}

const char* metricName(AirItem item) {
  switch (item) {
    case AirItem::Co2:
      return "CO2";
    case AirItem::Tvoc:
      return "TVOC";
    case AirItem::Hcho:
      return "HCHO";
  }
  return "UNK";
}

uint16_t readMetricRaw(AirItem item, const AirData& data) {
  switch (item) {
    case AirItem::Co2:
      return data.co2Raw;
    case AirItem::Tvoc:
      return data.tvocRaw;
    case AirItem::Hcho:
      return data.hchoRaw;
  }
  return 0;
}

void setStatus(const char* text, unsigned long holdMs = 1000) {
  snprintf(statusText, sizeof(statusText), "%s", text);
  statusUntilMs = millis() + holdMs;
}

void printCurrentBleStatus() {
  Serial.print("[BLE] connected=");
  Serial.print(bleKeyboard.isConnected() ? "yes" : "no");
  if (bleKeyboard.isConnected()) {
    const unsigned long age = millis() - bleConnectedSinceMs;
    Serial.print(", age_ms=");
    Serial.print(age);
    Serial.print(", ready=");
    Serial.print(age >= BLE_READY_AFTER_CONNECT_MS ? "yes" : "no");
  }
  Serial.print(", conn_cnt=");
  Serial.print(bleConnectCount);
  Serial.print(", disc_cnt=");
  Serial.print(bleDisconnectCount);
  Serial.print(", auto_enter=");
  Serial.println(settingAutoEnter ? "on" : "off");
}

bool isBleReadyToSend() {
  if (!bleKeyboard.isConnected()) {
    return false;
  }
  return (millis() - bleConnectedSinceMs) >= BLE_READY_AFTER_CONNECT_MS;
}

bool readDebouncedTouch() {
  const bool raw = (digitalRead(TOUCH_PIN) == HIGH);
  if (raw != lastRawTouchState) {
    lastRawTouchState = raw;
    lastRawChangeMs = millis();
  }
  if ((millis() - lastRawChangeMs) >= DEBOUNCE_MS) {
    lastStableTouchState = raw;
  }
  return lastStableTouchState;
}

ButtonEvent pollButtonEvent() {
  const unsigned long now = millis();
  const bool touch = readDebouncedTouch();
  static bool previousTouch = false;

  if (touch && !previousTouch) {
    pressStartMs = now;
    longPressTriggered = false;
  }

  if (touch && !longPressTriggered && (now - pressStartMs) >= LONG_PRESS_MS) {
    longPressTriggered = true;
    pendingSingleClick = false;
    previousTouch = touch;
    return ButtonEvent::LongPress;
  }

  if (!touch && previousTouch) {
    if (longPressTriggered) {
      longPressTriggered = false;
      previousTouch = touch;
      return ButtonEvent::None;
    }
    if (pendingSingleClick && (now - lastShortReleaseMs) <= DOUBLE_CLICK_MS) {
      pendingSingleClick = false;
      previousTouch = touch;
      return ButtonEvent::DoubleClick;
    }
    pendingSingleClick = true;
    lastShortReleaseMs = now;
  }

  if (pendingSingleClick && (now - lastShortReleaseMs) > DOUBLE_CLICK_MS) {
    pendingSingleClick = false;
    previousTouch = touch;
    return ButtonEvent::SingleClick;
  }

  previousTouch = touch;
  return ButtonEvent::None;
}

void pushRealtimePoint(uint8_t metric, uint16_t avg, uint16_t peak) {
  RealtimeSeries& s = realtimeSeries[metric];
  s.avg[s.head] = avg;
  s.peak[s.head] = peak;
  s.head = static_cast<uint8_t>((s.head + 1) % REALTIME_SECONDS);
  if (s.count < REALTIME_SECONDS) {
    s.count++;
  }
}

void pushHistoryPoint(uint8_t metric, uint16_t avg, uint16_t peak) {
  HistorySeries& s = historySeries[metric];
  s.avg[s.head] = avg;
  s.peak[s.head] = peak;
  s.head = static_cast<uint16_t>((s.head + 1) % HISTORY_SLOTS);
  if (s.count < HISTORY_SLOTS) {
    s.count++;
  }
}

ChartDataPoint getRealtimeByChrono(uint8_t metric, uint8_t posFromOldest) {
  ChartDataPoint out;
  const RealtimeSeries& s = realtimeSeries[metric];
  if (posFromOldest >= s.count) {
    return out;
  }
  const uint8_t oldest = static_cast<uint8_t>((s.head + REALTIME_SECONDS - s.count) % REALTIME_SECONDS);
  const uint8_t idx = static_cast<uint8_t>((oldest + posFromOldest) % REALTIME_SECONDS);
  out.avg = s.avg[idx];
  out.peak = s.peak[idx];
  out.valid = true;
  return out;
}

ChartDataPoint getHistoryByChrono(uint8_t metric, uint16_t posFromOldest) {
  ChartDataPoint out;
  const HistorySeries& s = historySeries[metric];
  if (posFromOldest >= s.count) {
    return out;
  }
  const uint16_t oldest = static_cast<uint16_t>((s.head + HISTORY_SLOTS - s.count) % HISTORY_SLOTS);
  const uint16_t idx = static_cast<uint16_t>((oldest + posFromOldest) % HISTORY_SLOTS);
  out.avg = s.avg[idx];
  out.peak = s.peak[idx];
  out.valid = true;
  return out;
}

void saveHistoryToNvs() {
  prefs.begin("airhist", false);
  prefs.putBytes("h0", &historySeries[0], sizeof(HistorySeries));
  prefs.putBytes("h1", &historySeries[1], sizeof(HistorySeries));
  prefs.putBytes("h2", &historySeries[2], sizeof(HistorySeries));
  prefs.end();
}

void loadHistoryFromNvs() {
  prefs.begin("airhist", true);
  if (prefs.getBytesLength("h0") == sizeof(HistorySeries)) {
    prefs.getBytes("h0", &historySeries[0], sizeof(HistorySeries));
  }
  if (prefs.getBytesLength("h1") == sizeof(HistorySeries)) {
    prefs.getBytes("h1", &historySeries[1], sizeof(HistorySeries));
  }
  if (prefs.getBytesLength("h2") == sizeof(HistorySeries)) {
    prefs.getBytes("h2", &historySeries[2], sizeof(HistorySeries));
  }
  prefs.end();
}

void saveSettingsToNvs() {
  prefs.begin("cfg", false);
  prefs.putBool("auto_enter", settingAutoEnter);
  prefs.putUChar("air_start_mode", static_cast<uint8_t>(settingAirStartMode));
  prefs.end();
}

void loadSettingsFromNvs() {
  prefs.begin("cfg", true);
  settingAutoEnter = prefs.getBool("auto_enter", true);
  const uint8_t mode = prefs.getUChar("air_start_mode", 0);
  settingAirStartMode = (mode == static_cast<uint8_t>(PlotMode::History))
                            ? PlotMode::History
                            : PlotMode::Realtime;
  prefs.end();
}

bool canSendBleNow() { return (millis() - lastBleSendMs) >= BLE_COOLDOWN_MS; }

void sendTextToBle(const char* text) {
  const size_t textLen = strlen(text);
  Serial.print("[BLE_SEND] req len=");
  Serial.print(static_cast<unsigned>(textLen));
  Serial.print(", auto_enter=");
  Serial.print(settingAutoEnter ? "on" : "off");
  Serial.print(", connected=");
  Serial.println(bleKeyboard.isConnected() ? "yes" : "no");

  if (!bleKeyboard.isConnected()) {
    setStatus("BLE not connected");
    Serial.println("[BLE_SEND] aborted: not connected");
    return;
  }
  if (!isBleReadyToSend()) {
    setStatus("BLE wait ready");
    Serial.println("[BLE_SEND] aborted: link not ready");
    return;
  }
  if (!canSendBleNow()) {
    setStatus("Send cooldown");
    Serial.println("[BLE_SEND] aborted: cooldown");
    return;
  }
  Serial.print("[BLE_SEND] text: ");
  Serial.println(text);

  // Type per-character for better host compatibility in HID text fields.
  for (const char* p = text; *p != '\0'; ++p) {
    bleKeyboard.write(*p);
    Serial.print("[BLE_SEND] char: ");
    Serial.print(*p);
    Serial.print(" (0x");
    Serial.print(static_cast<uint8_t>(*p), HEX);
    Serial.println(")");
    delay(3);
  }
  if (settingAutoEnter) {
    bleKeyboard.write(KEY_RETURN);
    Serial.println("[BLE_SEND] enter key sent");
  }
  bleKeyboard.releaseAll();
  lastBleSendMs = millis();
  setStatus("Sent");
  Serial.println("[BLE_SEND] done");
}

const char* metricUnit(AirItem item) {
  if (item == AirItem::Co2) {
    return "ppm";
  }
  return "mg/m3";
}

void formatCurrentMetricValue(AirItem item, char* out, size_t outLen) {
  if (!latestAirData.valid) {
    snprintf(out, outLen, "%s --", metricName(item));
    return;
  }
  const uint16_t raw = readMetricRaw(item, latestAirData);
  if (item == AirItem::Co2) {
    snprintf(out, outLen, "CO2 %u", static_cast<unsigned>(raw));
  } else {
    snprintf(out, outLen, "%s %.3f", metricName(item), rawToMgPerM3(raw));
  }
}

void formatAxisMetricValue(AirItem item, uint16_t raw, char* out, size_t outLen) {
  if (item == AirItem::Co2) {
    if (raw >= 10000) {
      snprintf(out, outLen, "%uk", static_cast<unsigned>(raw / 1000));
    } else {
      snprintf(out, outLen, "%u", static_cast<unsigned>(raw));
    }
    return;
  }
  snprintf(out, outLen, "%.2f", rawToMgPerM3(raw));
  if (out[0] == '0' && out[1] == '.') {
    memmove(out, out + 1, strlen(out));
  }
}

void formatMetricForBle(AirItem item, char* out, size_t outLen) {
  if (!latestAirData.valid) {
    snprintf(out, outLen, "NO_AIR_DATA");
    return;
  }
  const uint16_t raw = readMetricRaw(item, latestAirData);
  if (item == AirItem::Co2) {
    snprintf(out, outLen, "CO2=%uppm", static_cast<unsigned>(raw));
  } else {
    snprintf(out, outLen, "%s=%.3fmg/m3", metricName(item), rawToMgPerM3(raw));
  }
}

void pollAirSensor() {
  while (AirSerial.available() > 0) {
    const int b = AirSerial.read();
    if (b < 0) {
      break;
    }
    AirData parsed;
    if (airParser.feed(static_cast<uint8_t>(b), parsed)) {
      latestAirData = parsed;
    }
  }
}

void reportAirDataPeriodically() {
  if (!latestAirData.valid) {
    return;
  }
  const unsigned long now = millis();
  if ((now - lastReportMs) < REPORT_INTERVAL_MS) {
    return;
  }
  lastReportMs = now;

  Serial.print("TVOC=");
  Serial.print(rawToMgPerM3(latestAirData.tvocRaw), 3);
  Serial.print(" mg/m3, HCHO=");
  Serial.print(rawToMgPerM3(latestAirData.hchoRaw), 3);
  Serial.print(" mg/m3, CO2=");
  Serial.print(latestAirData.co2Raw);
  Serial.println(" ppm");
}

void reportBleConnectionChange() {
  const bool nowConnected = bleKeyboard.isConnected();
  if (nowConnected != lastBleConnectedState) {
    lastBleConnectedState = nowConnected;
    if (nowConnected) {
      bleConnectedSinceMs = millis();
      bleConnectCount++;
    } else {
      bleDisconnectCount++;
    }
    Serial.print("[BLE] state changed: ");
    Serial.println(nowConnected ? "CONNECTED" : "DISCONNECTED");
    printCurrentBleStatus();
  }
}

void reportHeartbeat() {
  const unsigned long now = millis();
  if ((now - lastHeartbeatMs) < 1000) {
    return;
  }
  lastHeartbeatMs = now;

  Serial.print("[HB] t=");
  Serial.print(now / 1000);
  Serial.print("s screen=");
  Serial.print(screenName(currentScreen));
  Serial.print(" ble=");
  Serial.print(bleKeyboard.isConnected() ? "yes" : "no");
  Serial.print(" touch=");
  Serial.print(readDebouncedTouch() ? "1" : "0");
  Serial.print(" air=");
  Serial.println(latestAirData.valid ? "ok" : "none");
}

void runSerialCommand(const char* cmdLine) {
  if (strncmp(cmdLine, "send ", 5) == 0) {
    const char* payload = cmdLine + 5;
    if (*payload == '\0') {
      Serial.println("[CMD] send requires text");
      return;
    }
    Serial.println("[CMD] manual send triggered");
    sendTextToBle(payload);
    return;
  }

  if (strcmp(cmdLine, "status") == 0) {
    printCurrentBleStatus();
    return;
  }

  if (strcmp(cmdLine, "help") == 0) {
    Serial.println("[CMD] help:");
    Serial.println("  status        -> show BLE status");
    Serial.println("  send <text>   -> send text via BLE keyboard");
#if defined(USE_NIMBLE)
    Serial.println("  clearbonds    -> delete all NimBLE bonds");
#endif
    return;
  }

#if defined(USE_NIMBLE)
  if (strcmp(cmdLine, "clearbonds") == 0) {
    NimBLEDevice::deleteAllBonds();
    Serial.println("[CMD] all bonds deleted");
    return;
  }
#endif

  Serial.print("[CMD] unknown: ");
  Serial.println(cmdLine);
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch < 0) {
      break;
    }

    if (ch == '\r' || ch == '\n') {
      if (serialCmdLen > 0) {
        serialCmdBuf[serialCmdLen] = '\0';
        runSerialCommand(serialCmdBuf);
        serialCmdLen = 0;
      }
      continue;
    }

    if (serialCmdLen < (sizeof(serialCmdBuf) - 1)) {
      serialCmdBuf[serialCmdLen++] = static_cast<char>(ch);
    }
  }
}

void finalizeSecondWindow() {
  for (uint8_t i = 0; i < METRIC_COUNT; ++i) {
    if (secondAccum[i].count == 0) {
      continue;
    }
    const uint16_t avg = static_cast<uint16_t>(secondAccum[i].sum / secondAccum[i].count);
    pushRealtimePoint(i, avg, secondAccum[i].peak);
    secondAccum[i] = RunningAccum{};
  }
}

void finalizeTenMinuteWindow() {
  bool any = false;
  for (uint8_t i = 0; i < METRIC_COUNT; ++i) {
    if (tenMinAccum[i].count == 0) {
      continue;
    }
    const uint16_t avg = static_cast<uint16_t>(tenMinAccum[i].sum / tenMinAccum[i].count);
    pushHistoryPoint(i, avg, tenMinAccum[i].peak);
    tenMinAccum[i] = RunningAccum{};
    any = true;
  }
  if (any) {
    saveHistoryToNvs();
  }
}

void accumulateSample(uint16_t co2, uint16_t tvoc, uint16_t hcho) {
  const uint16_t values[METRIC_COUNT] = {co2, tvoc, hcho};
  for (uint8_t i = 0; i < METRIC_COUNT; ++i) {
    secondAccum[i].sum += values[i];
    if (secondAccum[i].count == 0 || values[i] > secondAccum[i].peak) {
      secondAccum[i].peak = values[i];
    }
    secondAccum[i].count++;

    tenMinAccum[i].sum += values[i];
    if (tenMinAccum[i].count == 0 || values[i] > tenMinAccum[i].peak) {
      tenMinAccum[i].peak = values[i];
    }
    tenMinAccum[i].count++;
  }
}

void processSampling() {
  const unsigned long now = millis();
  if (lastSampleTickMs == 0) {
    lastSampleTickMs = now;
    secondWindowStartMs = now;
    tenMinWindowStartMs = now;
  }

  while ((now - lastSampleTickMs) >= SAMPLE_INTERVAL_MS) {
    if (latestAirData.valid) {
      accumulateSample(latestAirData.co2Raw, latestAirData.tvocRaw, latestAirData.hchoRaw);
    }
    lastSampleTickMs += SAMPLE_INTERVAL_MS;
  }

  while ((now - secondWindowStartMs) >= SECOND_WINDOW_MS) {
    finalizeSecondWindow();
    secondWindowStartMs += SECOND_WINDOW_MS;
  }

  while ((now - tenMinWindowStartMs) >= HISTORY_WINDOW_MS) {
    finalizeTenMinuteWindow();
    tenMinWindowStartMs += HISTORY_WINDOW_MS;
  }
}

uint16_t getRealtimeCountWithLive(uint8_t metric) {
  uint16_t c = realtimeSeries[metric].count;
  if (secondAccum[metric].count > 0) {
    c++;
  }
  if (c > REALTIME_SECONDS) {
    c = REALTIME_SECONDS;
  }
  return c;
}

ChartDataPoint getRealtimePointWithLive(uint8_t metric, uint16_t posFromOldest) {
  ChartDataPoint out;
  const RealtimeSeries& s = realtimeSeries[metric];
  const bool hasLive = secondAccum[metric].count > 0;
  uint16_t total = s.count + (hasLive ? 1 : 0);
  if (total == 0) {
    return out;
  }

  uint16_t trim = 0;
  if (total > REALTIME_SECONDS) {
    trim = total - REALTIME_SECONDS;
    total = REALTIME_SECONDS;
  }
  if (posFromOldest >= total) {
    return out;
  }

  const uint16_t normalizedPos = posFromOldest + trim;
  if (normalizedPos < s.count) {
    return getRealtimeByChrono(metric, static_cast<uint8_t>(normalizedPos));
  }

  // Live partial 1-second point.
  if (!hasLive) {
    return out;
  }
  const RunningAccum& live = secondAccum[metric];
  out.avg = static_cast<uint16_t>(live.sum / live.count);
  out.peak = live.peak;
  out.valid = true;
  return out;
}

uint16_t getHistoryCountWithLive(uint8_t metric) {
  uint16_t c = historySeries[metric].count;
  if (tenMinAccum[metric].count > 0) {
    c++;
  }
  if (c > HISTORY_SLOTS) {
    c = HISTORY_SLOTS;
  }
  return c;
}

ChartDataPoint getHistoryPointWithLive(uint8_t metric, uint16_t posFromOldest) {
  ChartDataPoint out;
  const HistorySeries& s = historySeries[metric];
  const bool hasLive = tenMinAccum[metric].count > 0;
  uint16_t total = s.count + (hasLive ? 1 : 0);
  if (total == 0) {
    return out;
  }

  uint16_t trim = 0;
  if (total > HISTORY_SLOTS) {
    trim = total - HISTORY_SLOTS;
    total = HISTORY_SLOTS;
  }
  if (posFromOldest >= total) {
    return out;
  }

  const uint16_t normalizedPos = posFromOldest + trim;
  if (normalizedPos < s.count) {
    return getHistoryByChrono(metric, normalizedPos);
  }

  if (!hasLive) {
    return out;
  }
  const RunningAccum& live = tenMinAccum[metric];
  out.avg = static_cast<uint16_t>(live.sum / live.count);
  out.peak = live.peak;
  out.valid = true;
  return out;
}

void formatTimeSpanLabel(AirItem item, PlotMode mode, char* out, size_t outLen) {
  if (mode == PlotMode::Realtime) {
    uint16_t seconds = getRealtimeCountWithLive(static_cast<uint8_t>(item));
    if (seconds > REALTIME_SECONDS) {
      seconds = REALTIME_SECONDS;
    }
    snprintf(out, outLen, "-%us", static_cast<unsigned>(seconds));
    return;
  }

  const uint16_t points = getHistoryCountWithLive(static_cast<uint8_t>(item));
  const uint32_t minutes = static_cast<uint32_t>(points) * 10UL;
  if (minutes < 60UL) {
    snprintf(out, outLen, "-%lum", static_cast<unsigned long>(minutes));
  } else {
    const uint32_t hoursRoundedUp = (minutes + 59UL) / 60UL;
    snprintf(out, outLen, "-%luh", static_cast<unsigned long>(hoursRoundedUp));
  }
}

void formatTimeHalfSpanLabel(AirItem item, PlotMode mode, char* out, size_t outLen) {
  if (mode == PlotMode::Realtime) {
    uint16_t seconds = getRealtimeCountWithLive(static_cast<uint8_t>(item));
    seconds = static_cast<uint16_t>(seconds / 2);
    snprintf(out, outLen, "-%us", static_cast<unsigned>(seconds));
    return;
  }

  const uint16_t points = getHistoryCountWithLive(static_cast<uint8_t>(item));
  const uint32_t minutes = (static_cast<uint32_t>(points) * 10UL) / 2UL;
  if (minutes < 60UL) {
    snprintf(out, outLen, "-%lum", static_cast<unsigned long>(minutes));
  } else {
    const uint32_t hoursRoundedUp = (minutes + 59UL) / 60UL;
    snprintf(out, outLen, "-%luh", static_cast<unsigned long>(hoursRoundedUp));
  }
}

void collectSeriesForPlot(uint8_t metric, PlotMode mode, uint16_t* outAvg, uint16_t* outPeak, uint8_t* outCount) {
  if (mode == PlotMode::Realtime) {
    const uint16_t count = getRealtimeCountWithLive(metric);
    if (count == 0) {
      *outCount = 0;
      return;
    }

    for (uint8_t x = 0; x < CHART_W; ++x) {
      const uint16_t start = static_cast<uint16_t>((static_cast<uint32_t>(x) * count) / CHART_W);
      uint16_t end = static_cast<uint16_t>((static_cast<uint32_t>(x + 1) * count) / CHART_W);
      if (end <= start) {
        end = static_cast<uint16_t>(start + 1);
      }
      if (end > count) {
        end = count;
      }

      uint32_t avgSum = 0;
      uint16_t peakMax = 0;
      uint16_t samples = 0;
      for (uint16_t p = start; p < end; ++p) {
        const ChartDataPoint point = getRealtimePointWithLive(metric, p);
        if (!point.valid) {
          continue;
        }
        avgSum += point.avg;
        if (samples == 0 || point.peak > peakMax) {
          peakMax = point.peak;
        }
        samples++;
      }
      if (samples == 0) {
        outAvg[x] = 0;
        outPeak[x] = 0;
      } else {
        outAvg[x] = static_cast<uint16_t>(avgSum / samples);
        outPeak[x] = peakMax;
      }
    }
    *outCount = CHART_W;
    return;
  }

  const uint16_t count = getHistoryCountWithLive(metric);
  if (count == 0) {
    *outCount = 0;
    return;
  }

  for (uint8_t x = 0; x < CHART_W; ++x) {
    const uint16_t start = static_cast<uint16_t>((static_cast<uint32_t>(x) * count) / CHART_W);
    uint16_t end = static_cast<uint16_t>((static_cast<uint32_t>(x + 1) * count) / CHART_W);
    if (end <= start) {
      end = static_cast<uint16_t>(start + 1);
    }
    if (end > count) {
      end = count;
    }

    uint32_t avgSum = 0;
    uint16_t peakMax = 0;
    uint16_t samples = 0;
    for (uint16_t p = start; p < end; ++p) {
      const ChartDataPoint point = getHistoryPointWithLive(metric, p);
      if (!point.valid) {
        continue;
      }
      avgSum += point.avg;
      if (samples == 0 || point.peak > peakMax) {
        peakMax = point.peak;
      }
      samples++;
    }
    if (samples == 0) {
      outAvg[x] = 0;
      outPeak[x] = 0;
    } else {
      outAvg[x] = static_cast<uint16_t>(avgSum / samples);
      outPeak[x] = peakMax;
    }
  }
  *outCount = CHART_W;
}

void drawDashedLine(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int step = 0;

  while (true) {
    if ((step & 1) == 0) {
      oled.drawPixel(x0, y0);
    }
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
    step++;
  }
}

GraphScale drawAirGraph(AirItem item, PlotMode mode) {
  GraphScale scale;
  uint16_t avg[CHART_W] = {0};
  uint16_t peak[CHART_W] = {0};
  uint8_t count = 0;
  collectSeriesForPlot(static_cast<uint8_t>(item), mode, avg, peak, &count);
  if (count == 0) {
    oled.drawStr(0, 23, "NO DATA");
    return scale;
  }

  uint16_t minVal = avg[0];
  uint16_t maxVal = peak[0];
  for (uint8_t i = 0; i < count; ++i) {
    if (avg[i] < minVal) {
      minVal = avg[i];
    }
    if (peak[i] < minVal) {
      minVal = peak[i];
    }
    if (avg[i] > maxVal) {
      maxVal = avg[i];
    }
    if (peak[i] > maxVal) {
      maxVal = peak[i];
    }
  }

  if (maxVal <= minVal) {
    maxVal = static_cast<uint16_t>(minVal + 1);
  }
  scale.valid = true;
  scale.minVal = minVal;
  scale.maxVal = maxVal;

  const int yBottom = CHART_Y + CHART_H - 1;
  const uint32_t range = static_cast<uint32_t>(maxVal - minVal);

  int prevAvgX = CHART_X;
  int prevAvgY = yBottom;
  int prevPeakX = CHART_X;
  int prevPeakY = yBottom;
  bool hasPrev = false;

  for (uint8_t x = 0; x < count; ++x) {
    const int px = CHART_X + x;
    const int avgY = yBottom - static_cast<int>((static_cast<uint32_t>(avg[x] - minVal) * (CHART_H - 1)) / range);
    const int peakY = yBottom - static_cast<int>((static_cast<uint32_t>(peak[x] - minVal) * (CHART_H - 1)) / range);

    if (hasPrev) {
      const int gapPrev = abs(prevAvgY - prevPeakY);
      const int gapCurr = abs(avgY - peakY);
      const bool mergeToWideAvg = (gapPrev <= 2 && gapCurr <= 2);

      if (mergeToWideAvg) {
        // When avg/peak are too close, draw only a 2px average solid line.
        oled.drawLine(prevAvgX, prevAvgY, px, avgY);
        if ((prevAvgY + 1) <= yBottom && (avgY + 1) <= yBottom) {
          oled.drawLine(prevAvgX, prevAvgY + 1, px, avgY + 1);
        } else if ((prevAvgY - 1) >= CHART_Y && (avgY - 1) >= CHART_Y) {
          oled.drawLine(prevAvgX, prevAvgY - 1, px, avgY - 1);
        }
      } else {
        oled.drawLine(prevAvgX, prevAvgY, px, avgY);      // solid = average
        drawDashedLine(prevPeakX, prevPeakY, px, peakY);  // dashed = peak
      }
    }
    prevAvgX = px;
    prevAvgY = avgY;
    prevPeakX = px;
    prevPeakY = peakY;
    hasPrev = true;
  }

  return scale;
}

void drawRootScreen() {
  oled.drawStr(0, 7, "MENU");
  oled.drawStr(0, 15, selectedRoot == 0 ? ">KEY" : " KEY");
  oled.drawStr(0, 23, selectedRoot == 1 ? ">AIR" : " AIR");
  oled.drawStr(0, 31, selectedRoot == 2 ? ">SET" : " SET");
  if (millis() < statusUntilMs) {
    oled.drawStr(0, 39, statusText);
  }
}

void drawKeyScreen() {
  oled.drawStr(0, 7, "KEY");

  uint8_t windowStart = 0;
  if (KEY_PRESET_COUNT > 3) {
    if (selectedKey >= 2) {
      windowStart = static_cast<uint8_t>(selectedKey - 2);
    }
    const uint8_t maxStart = static_cast<uint8_t>(KEY_PRESET_COUNT - 3);
    if (windowStart > maxStart) {
      windowStart = maxStart;
    }
  }

  for (uint8_t line = 0; line < 3; ++line) {
    const uint8_t idx = static_cast<uint8_t>(windowStart + line);
    if (idx >= KEY_PRESET_COUNT) {
      break;
    }
    char row[18];
    snprintf(row, sizeof(row), "%c%s", idx == selectedKey ? '>' : ' ', KEY_PRESETS[idx].name);
    oled.drawStr(0, static_cast<int16_t>(15 + line * 8), row);
  }
}

void drawAirSelectScreen() {
  oled.drawStr(0, 7, "AIR");
  oled.drawStr(0, 15, selectedAir == 0 ? ">CO2" : " CO2");
  oled.drawStr(0, 23, selectedAir == 1 ? ">TVOC" : " TVOC");
  oled.drawStr(0, 31, selectedAir == 2 ? ">HCHO" : " HCHO");
}

void drawSettingsScreen() {
  oled.drawStr(0, 7, "SET");

  char row1[18];
  char row2[18];
  snprintf(row1, sizeof(row1), "%cENT %s",
           selectedSetting == static_cast<uint8_t>(SettingItem::AutoEnter) ? '>' : ' ',
           settingAutoEnter ? "ON" : "OFF");
  snprintf(row2, sizeof(row2), "%cAIR %s",
           selectedSetting == static_cast<uint8_t>(SettingItem::AirStartMode) ? '>' : ' ',
           settingAirStartMode == PlotMode::Realtime ? "RT" : "HS");
  oled.drawStr(0, 15, row1);
  oled.drawStr(0, 23, row2);
  if (millis() < statusUntilMs) {
    oled.drawStr(0, 39, statusText);
  } else {
    oled.drawStr(0, 39, "Hold=Toggle");
  }
}

void drawAirViewScreen() {
  char valueText[20];
  formatCurrentMetricValue(activeAirSelection, valueText, sizeof(valueText));
  const char* unit = metricUnit(activeAirSelection);
  oled.drawStr(0, 7, valueText);
  const int unitW = oled.getStrWidth(unit);
  const int unitX = ((CHART_X + CHART_W) > unitW) ? ((CHART_X + CHART_W) - unitW) : 0;
  oled.drawStr(unitX, 7, unit);

  // Selected metric only: solid avg + dashed peak.
  const GraphScale scale = drawAirGraph(activeAirSelection, plotMode);

  // Y axis line and numeric labels
  oled.drawVLine(CHART_X, CHART_Y, CHART_H);
  oled.setFont(u8g2_font_3x5im_tr);
  if (scale.valid) {
    const uint16_t midRaw = static_cast<uint16_t>(scale.minVal + ((scale.maxVal - scale.minVal) / 2));
    char yTop[8];
    char yMid[8];
    char yBot[8];
    formatAxisMetricValue(activeAirSelection, scale.maxVal, yTop, sizeof(yTop));
    formatAxisMetricValue(activeAirSelection, midRaw, yMid, sizeof(yMid));
    formatAxisMetricValue(activeAirSelection, scale.minVal, yBot, sizeof(yBot));

    oled.drawStr(0, CHART_Y + 4, yTop);
    oled.drawStr(0, CHART_Y + (CHART_H / 2) + 2, yMid);
    oled.drawStr(0, CHART_Y + CHART_H, yBot);

    // Y-axis ticks at top/mid/bottom
    oled.drawHLine(CHART_X - 2, CHART_Y, 2);
    oled.drawHLine(CHART_X - 2, CHART_Y + (CHART_H / 2), 2);
    oled.drawHLine(CHART_X - 2, CHART_Y + CHART_H - 1, 2);
  }

  const int axisY = CHART_Y + CHART_H + 1;
  const int axisMidX = CHART_X + (CHART_W / 2);
  oled.drawHLine(CHART_X, axisY, CHART_W);
  oled.drawVLine(CHART_X, axisY - 1, 3);
  oled.drawVLine(axisMidX, axisY - 1, 3);
  oled.drawVLine(CHART_X + CHART_W - 1, axisY - 1, 3);

  char leftSpan[10];
  formatTimeSpanLabel(activeAirSelection, plotMode, leftSpan, sizeof(leftSpan));
  oled.drawStr(CHART_X, 39, leftSpan);
  char midSpan[10];
  formatTimeHalfSpanLabel(activeAirSelection, plotMode, midSpan, sizeof(midSpan));
  const int midW = oled.getStrWidth(midSpan);
  oled.drawStr(axisMidX - (midW / 2), 39, midSpan);
  const char* rightLabel = "now";
  const int rightW = oled.getStrWidth(rightLabel);
  oled.drawStr(CHART_X + CHART_W - rightW, 39, rightLabel);
}

void refreshDisplay() {
  const unsigned long now = millis();
  if ((now - lastOledRefreshMs) < OLED_REFRESH_MS) {
    return;
  }
  lastOledRefreshMs = now;

  oled.clearBuffer();
  oled.setFont(u8g2_font_5x7_tr);

  switch (currentScreen) {
    case Screen::Root:
      drawRootScreen();
      break;
    case Screen::Key:
      drawKeyScreen();
      break;
    case Screen::Air:
      drawAirSelectScreen();
      break;
    case Screen::AirView:
      drawAirViewScreen();
      break;
    case Screen::Settings:
      drawSettingsScreen();
      break;
  }
  oled.sendBuffer();
}

void handleSingleClick() {
  Serial.print("[UI] single click @");
  Serial.println(screenName(currentScreen));
  switch (currentScreen) {
    case Screen::Root:
      selectedRoot = (selectedRoot + 1) % 3;
      break;
    case Screen::Key:
      selectedKey = (selectedKey + 1) % KEY_PRESET_COUNT;
      break;
    case Screen::Air:
      selectedAir = (selectedAir + 1) % 3;
      break;
    case Screen::AirView:
      plotMode = (plotMode == PlotMode::Realtime) ? PlotMode::History : PlotMode::Realtime;
      break;
    case Screen::Settings:
      selectedSetting = (selectedSetting + 1) % 2;
      break;
  }
}

void handleDoubleClick() {
  Serial.print("[UI] double click @");
  Serial.println(screenName(currentScreen));
  switch (currentScreen) {
    case Screen::Root:
      break;
    case Screen::Key:
    case Screen::Air:
      currentScreen = Screen::Root;
      setStatus("Back");
      break;
    case Screen::AirView:
      currentScreen = Screen::Air;
      setStatus("Back");
      break;
    case Screen::Settings:
      currentScreen = Screen::Root;
      setStatus("Back");
      break;
  }
}

void handleLongPress() {
  Serial.print("[UI] long press @");
  Serial.println(screenName(currentScreen));
  switch (currentScreen) {
    case Screen::Root:
      if (selectedRoot == static_cast<uint8_t>(RootItem::Key)) {
        currentScreen = Screen::Key;
      } else if (selectedRoot == static_cast<uint8_t>(RootItem::Air)) {
        currentScreen = Screen::Air;
      } else {
        currentScreen = Screen::Settings;
      }
      break;
    case Screen::Key:
      Serial.print("[UI] send preset: ");
      Serial.println(KEY_PRESETS[selectedKey].name);
      sendTextToBle(KEY_PRESETS[selectedKey].text);
      break;
    case Screen::Air:
      activeAirSelection = static_cast<AirItem>(selectedAir);
      currentScreen = Screen::AirView;
      plotMode = settingAirStartMode;
      break;
    case Screen::AirView: {
      // Air metrics are display-only; do not type them over BLE keyboard.
      Serial.println("[UI] AIR VIEW long press: no BLE send");
      setStatus("AIR no send");
      break;
    }
    case Screen::Settings:
      if (selectedSetting == static_cast<uint8_t>(SettingItem::AutoEnter)) {
        settingAutoEnter = !settingAutoEnter;
      } else {
        settingAirStartMode =
            (settingAirStartMode == PlotMode::Realtime) ? PlotMode::History : PlotMode::Realtime;
      }
      saveSettingsToNvs();
      setStatus("Saved");
      break;
  }
}

void setup() {
  pinMode(TOUCH_PIN, INPUT);
  Serial.begin(115200);
  delay(100);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_5x7_tr);
  oled.drawStr(0, 15, "Booting...");
  oled.sendBuffer();

  AirSerial.begin(AIR_UART_BAUD, SERIAL_8N1, AIR_UART_RX_PIN, AIR_UART_TX_PIN);
  bleKeyboard.begin();
  bleKeyboard.setDelay(12);
#if defined(USE_NIMBLE)
  if (BLE_RELAXED_SECURITY) {
    // Reduce auth strictness to avoid repeated pair/disconnect loops on some hosts.
    NimBLEDevice::setSecurityAuth(true, false, true);
    NimBLEDevice::setSecurityIOCap(3);  // BLE_HS_IO_NO_INPUT_OUTPUT
  }
#endif

  loadSettingsFromNvs();
  loadHistoryFromNvs();
  lastBleConnectedState = bleKeyboard.isConnected();
  if (lastBleConnectedState) {
    bleConnectedSinceMs = millis();
  }

  lastSampleTickMs = millis();
  secondWindowStartMs = lastSampleTickMs;
  tenMinWindowStartMs = lastSampleTickMs;

  Serial.println("BLE + AIR + OLED menu started.");
  Serial.println("Single click: next, double click: back, long press: confirm.");
  Serial.println("AIR view: top value+unit, chart with time axis.");
  Serial.println("Serial cmd: help | status | send <text>");
#if defined(USE_NIMBLE)
  if (BLE_RELAXED_SECURITY) {
    Serial.println("BLE mode: NimBLE + relaxed security + 1.2s ready delay");
  } else {
    Serial.println("BLE mode: NimBLE + strict security + 1.2s ready delay");
  }
#else
  Serial.println("BLE mode: Bluedroid + 1.2s ready delay");
#endif
  Serial.println("Heartbeat: 1s interval");
  printCurrentBleStatus();
}

void loop() {
  pollAirSensor();
  processSampling();
  reportAirDataPeriodically();
  reportBleConnectionChange();
  reportHeartbeat();
  processSerialCommands();

  const ButtonEvent event = pollButtonEvent();
  if (event == ButtonEvent::SingleClick) {
    handleSingleClick();
  } else if (event == ButtonEvent::DoubleClick) {
    handleDoubleClick();
  } else if (event == ButtonEvent::LongPress) {
    handleLongPress();
  }

  refreshDisplay();
  delay(5);
}
