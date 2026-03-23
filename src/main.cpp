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

static const uint8_t BUTTON_UP_PIN = 0;           // Button to GND, uses internal pull-up
static const uint8_t BUTTON_DOWN_PIN = 1;         // Button to GND, uses internal pull-up
static const unsigned long DEBOUNCE_MS = 35;
static const unsigned long DOUBLE_CLICK_MS = 100; // Release-to-next-press gap for back
static const unsigned long TEST_MODE_EXIT_HOLD_MS = 1000;
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
static const unsigned long MENU_INDICATOR_ANIM_MS = 110;
static const unsigned long MENU_INDICATOR_MIN_MS = 82;
static const unsigned long MENU_INDICATOR_MAX_MS = 148;
static const unsigned long MENU_INDICATOR_STEP_MS = 16;
static const int ENV_I2C_SDA_PIN = 9;             // AHT20/BMP280 SDA
static const int ENV_I2C_SCL_PIN = 7;             // AHT20/BMP280 SCL
static const uint32_t ENV_I2C_FREQ = 100000;
static const unsigned long ENV_POLL_MS = 500;
static const unsigned long ENV_REPORT_MS = 5000;
static const unsigned long ENV_STALE_MS = 15000;
static const unsigned long ENV_REINIT_MS = 5000;
static const float ENV_PRESSURE_OFFSET_HPA = 300.0f;
static const uint8_t ENV_HIST_FMT_VER = 2;
static const uint8_t AHT20_I2C_ADDR = 0x38;
static const uint8_t BMP280_I2C_ADDR1 = 0x76;
static const uint8_t BMP280_I2C_ADDR2 = 0x77;

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

enum class Screen { Root, Key, Air, AirView, Env, EnvView, Settings, ButtonTest };
enum class RootItem : uint8_t { Key = 0, Air = 1, Env = 2, Settings = 3 };
enum class AirItem : uint8_t { Co2 = 0, Tvoc = 1, Hcho = 2 };
enum class EnvItem : uint8_t { Temp = 0, Humidity = 1, Pressure = 2 };
enum class PlotMode : uint8_t { Realtime = 0, History = 1 };
enum class ButtonEvent { None, Up, Down, Back, Confirm };
enum class SettingItem : uint8_t { AutoEnter = 0, AirStartMode = 1, IntervalTest = 2 };
enum class I2cOwner : uint8_t { Oled = 0, Env = 1 };
enum class ButtonId : uint8_t { None = 0, Up = 1, Down = 2 };

static const uint8_t METRIC_COUNT = 3;
static const uint8_t ROOT_ITEM_COUNT = 4;
static const uint16_t ENV_INVALID_POINT = 0xFFFF;
static const uint16_t AIR_INVALID_POINT = 0xFFFF;

struct AirData {
  bool valid = false;
  uint16_t tvocRaw = 0;
  uint16_t hchoRaw = 0;
  uint16_t co2Raw = 0;
  unsigned long updatedMs = 0;
};

struct EnvData {
  bool ahtValid = false;
  bool bmpValid = false;
  float temperatureC = 0.0f;
  float humidityPct = 0.0f;
  float pressureHpa = 0.0f;
  unsigned long updatedMs = 0;
};

struct Bmp280Calib {
  uint16_t digT1 = 0;
  int16_t digT2 = 0;
  int16_t digT3 = 0;
  uint16_t digP1 = 0;
  int16_t digP2 = 0;
  int16_t digP3 = 0;
  int16_t digP4 = 0;
  int16_t digP5 = 0;
  int16_t digP6 = 0;
  int16_t digP7 = 0;
  int16_t digP8 = 0;
  int16_t digP9 = 0;
  bool valid = false;
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

void processButtonInput();

struct MenuIndicatorAnim {
  bool initialized = false;
  Screen screen = Screen::Root;
  int16_t startY = 15;
  int16_t targetY = 15;
  unsigned long startMs = 0;
  unsigned long durationMs = 120;
};

struct DebouncedButton {
  bool stablePressed = false;
  bool rawPressed = false;
  unsigned long lastRawChangeMs = 0;
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
EnvData latestEnvData;
Bmp280Calib bmpCalib;

bool aht20Ready = false;
bool bmp280Ready = false;
uint8_t bmp280Address = 0;
I2cOwner currentI2cOwner = I2cOwner::Oled;

RealtimeSeries realtimeSeries[METRIC_COUNT];
HistorySeries historySeries[METRIC_COUNT];
RunningAccum secondAccum[METRIC_COUNT];
RunningAccum tenMinAccum[METRIC_COUNT];
RealtimeSeries envRealtimeSeries[METRIC_COUNT];
HistorySeries envHistorySeries[METRIC_COUNT];
RunningAccum envSecondAccum[METRIC_COUNT];
RunningAccum envTenMinAccum[METRIC_COUNT];
uint16_t lastAirRawValues[METRIC_COUNT] = {0};
bool airMetricReady[METRIC_COUNT] = {false, false, false};
uint16_t lastEnvScaledValues[METRIC_COUNT] = {0};
bool envMetricReady[METRIC_COUNT] = {false, false, false};

Screen currentScreen = Screen::Root;
uint8_t selectedRoot = static_cast<uint8_t>(RootItem::Key);
uint8_t selectedKey = 0;
uint8_t selectedAir = static_cast<uint8_t>(AirItem::Co2);
uint8_t selectedEnv = static_cast<uint8_t>(EnvItem::Temp);
uint8_t selectedSetting = static_cast<uint8_t>(SettingItem::AutoEnter);
AirItem activeAirSelection = AirItem::Co2;
PlotMode plotMode = PlotMode::Realtime;
PlotMode envPlotMode = PlotMode::Realtime;
bool settingAutoEnter = true;
PlotMode settingAirStartMode = PlotMode::Realtime;
bool lastBleConnectedState = false;
unsigned long bleConnectedSinceMs = 0;
uint32_t bleConnectCount = 0;
uint32_t bleDisconnectCount = 0;

char serialCmdBuf[128] = {0};
uint8_t serialCmdLen = 0;

DebouncedButton upButtonState;
DebouncedButton downButtonState;
bool previousUpPressed = false;
bool previousDownPressed = false;
unsigned long lastBackReleaseMs = 0;
ButtonId suppressNextReleaseButton = ButtonId::None;
bool comboConfirmTriggered = false;
bool comboLongHoldTriggered = false;
unsigned long comboPressStartMs = 0;
bool buttonTestUpGapValid = false;
bool buttonTestDownGapValid = false;
unsigned long buttonTestUpGapMs = 0;
unsigned long buttonTestDownGapMs = 0;
unsigned long buttonTestUpReleaseMs = 0;
unsigned long buttonTestDownReleaseMs = 0;
MenuIndicatorAnim menuIndicatorAnim;
unsigned long lastMenuNavigateMs = 0;
unsigned long smoothedMenuGapMs = 180;
unsigned long lastMenuAnimDurationMs = MENU_INDICATOR_ANIM_MS;

unsigned long lastBleSendMs = 0;
unsigned long lastReportMs = 0;
unsigned long lastEnvPollMs = 0;
unsigned long lastEnvReportMs = 0;
unsigned long lastAhtSuccessMs = 0;
unsigned long lastBmpSuccessMs = 0;
unsigned long lastEnvReinitAttemptMs = 0;
unsigned long lastAirHistorySaveMs = 0;
unsigned long lastEnvHistorySaveMs = 0;
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
    case Screen::Env:
      return "ENV";
    case Screen::EnvView:
      return "ENVV";
    case Screen::Settings:
      return "SET";
    case Screen::ButtonTest:
      return "TEST";
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

const char* envMetricName(EnvItem item) {
  switch (item) {
    case EnvItem::Temp:
      return "TEMP";
    case EnvItem::Humidity:
      return "HUM";
    case EnvItem::Pressure:
      return "PRES";
  }
  return "ENV";
}

const char* envMetricUnit(EnvItem item) {
  switch (item) {
    case EnvItem::Temp:
      return "C";
    case EnvItem::Humidity:
      return "%";
    case EnvItem::Pressure:
      return "hPa";
  }
  return "";
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

bool readEnvMetricScaled(EnvItem item, uint16_t& outScaled) {
  switch (item) {
    case EnvItem::Temp: {
      if (!latestEnvData.ahtValid && !latestEnvData.bmpValid) {
        return false;
      }
      const int raw = static_cast<int>((latestEnvData.temperatureC + 40.0f) * 100.0f + 0.5f);
      outScaled = static_cast<uint16_t>(constrain(raw, 0, 20000));
      return true;
    }
    case EnvItem::Humidity: {
      if (!latestEnvData.ahtValid) {
        return false;
      }
      const int raw = static_cast<int>(latestEnvData.humidityPct * 100.0f + 0.5f);
      outScaled = static_cast<uint16_t>(constrain(raw, 0, 10000));
      return true;
    }
    case EnvItem::Pressure: {
      if (!latestEnvData.bmpValid) {
        return false;
      }
      const int raw =
          static_cast<int>((latestEnvData.pressureHpa - ENV_PRESSURE_OFFSET_HPA) * 20.0f + 0.5f);
      outScaled = static_cast<uint16_t>(constrain(raw, 0, 20000));
      return true;
    }
  }
  return false;
}

void setStatus(const char* text, unsigned long holdMs = 1000) {
  snprintf(statusText, sizeof(statusText), "%s", text);
  statusUntilMs = millis() + holdMs;
}

void responsiveDelay(unsigned long waitMs, bool keepButtonSampling = true) {
  const unsigned long startMs = millis();
  while ((millis() - startMs) < waitMs) {
    if (keepButtonSampling) {
      processButtonInput();
    }
    delay(1);
  }
}

void resetButtonTestState() {
  buttonTestUpGapValid = false;
  buttonTestDownGapValid = false;
  buttonTestUpGapMs = 0;
  buttonTestDownGapMs = 0;
  buttonTestUpReleaseMs = 0;
  buttonTestDownReleaseMs = 0;
  lastBackReleaseMs = 0;
  suppressNextReleaseButton = ButtonId::None;
  comboConfirmTriggered = false;
  comboLongHoldTriggered = false;
  comboPressStartMs = 0;
}

bool switchI2cOwner(I2cOwner owner) {
  if (currentI2cOwner == owner) {
    return true;
  }
  Wire.end();
  const int sdaPin = (owner == I2cOwner::Oled) ? OLED_SDA_PIN : ENV_I2C_SDA_PIN;
  const int sclPin = (owner == I2cOwner::Oled) ? OLED_SCL_PIN : ENV_I2C_SCL_PIN;
  const bool ok = Wire.begin(sdaPin, sclPin, ENV_I2C_FREQ);
  if (!ok) {
    Serial.print("[I2C] switch failed sda=");
    Serial.print(sdaPin);
    Serial.print(" scl=");
    Serial.println(sclPin);
    return false;
  }
  currentI2cOwner = owner;
  return true;
}

bool i2cProbeAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cWriteBytes(uint8_t addr, const uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  if (Wire.write(data, len) != len) {
    Wire.endTransmission();
    return false;
  }
  return Wire.endTransmission() == 0;
}

bool i2cWriteReg8(uint8_t addr, uint8_t reg, uint8_t value) {
  const uint8_t cmd[2] = {reg, value};
  return i2cWriteBytes(addr, cmd, sizeof(cmd));
}

bool i2cReadRegBytes(uint8_t addr, uint8_t reg, uint8_t* out, size_t len) {
  Wire.beginTransmission(addr);
  if (Wire.write(reg) != 1) {
    Wire.endTransmission();
    return false;
  }
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const size_t readCount =
      static_cast<size_t>(Wire.requestFrom(static_cast<int>(addr), static_cast<int>(len), 1));
  if (readCount != len) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    const int b = Wire.read();
    if (b < 0) {
      return false;
    }
    out[i] = static_cast<uint8_t>(b);
  }
  return true;
}

uint16_t readLeU16(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[1] << 8) | p[0]);
}

int16_t readLeS16(const uint8_t* p) {
  return static_cast<int16_t>(readLeU16(p));
}

bool initAht20() {
  const uint8_t initCmd[3] = {0xBE, 0x08, 0x00};
  if (!i2cProbeAddress(AHT20_I2C_ADDR)) {
    return false;
  }
  if (!i2cWriteBytes(AHT20_I2C_ADDR, initCmd, sizeof(initCmd))) {
    return false;
  }
  responsiveDelay(10);
  return true;
}

bool readAht20(float& temperatureC, float& humidityPct) {
  const uint8_t triggerCmd[3] = {0xAC, 0x33, 0x00};
  if (!i2cWriteBytes(AHT20_I2C_ADDR, triggerCmd, sizeof(triggerCmd))) {
    return false;
  }
  responsiveDelay(80);
  const size_t expectedLen = 7;
  const size_t readCount = static_cast<size_t>(
      Wire.requestFrom(static_cast<int>(AHT20_I2C_ADDR), static_cast<int>(expectedLen), 1));
  if (readCount != expectedLen) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }
  uint8_t data[7] = {0};
  for (size_t i = 0; i < expectedLen; ++i) {
    const int b = Wire.read();
    if (b < 0) {
      return false;
    }
    data[i] = static_cast<uint8_t>(b);
  }

  if ((data[0] & 0x80) != 0) {
    return false;
  }

  const uint32_t humRaw = (static_cast<uint32_t>(data[1]) << 12) |
                          (static_cast<uint32_t>(data[2]) << 4) |
                          (static_cast<uint32_t>(data[3]) >> 4);
  const uint32_t tempRaw = ((static_cast<uint32_t>(data[3]) & 0x0F) << 16) |
                           (static_cast<uint32_t>(data[4]) << 8) |
                           static_cast<uint32_t>(data[5]);
  humidityPct = (static_cast<float>(humRaw) * 100.0f) / 1048576.0f;
  temperatureC = (static_cast<float>(tempRaw) * 200.0f) / 1048576.0f - 50.0f;
  return true;
}

bool initBmp280() {
  uint8_t addr = 0;
  if (i2cProbeAddress(BMP280_I2C_ADDR1)) {
    addr = BMP280_I2C_ADDR1;
  } else if (i2cProbeAddress(BMP280_I2C_ADDR2)) {
    addr = BMP280_I2C_ADDR2;
  } else {
    return false;
  }

  uint8_t chipId = 0;
  if (!i2cReadRegBytes(addr, 0xD0, &chipId, 1)) {
    return false;
  }
  if (chipId != 0x58) {
    return false;
  }

  uint8_t calibRaw[24] = {0};
  if (!i2cReadRegBytes(addr, 0x88, calibRaw, sizeof(calibRaw))) {
    return false;
  }
  bmpCalib.digT1 = readLeU16(&calibRaw[0]);
  bmpCalib.digT2 = readLeS16(&calibRaw[2]);
  bmpCalib.digT3 = readLeS16(&calibRaw[4]);
  bmpCalib.digP1 = readLeU16(&calibRaw[6]);
  bmpCalib.digP2 = readLeS16(&calibRaw[8]);
  bmpCalib.digP3 = readLeS16(&calibRaw[10]);
  bmpCalib.digP4 = readLeS16(&calibRaw[12]);
  bmpCalib.digP5 = readLeS16(&calibRaw[14]);
  bmpCalib.digP6 = readLeS16(&calibRaw[16]);
  bmpCalib.digP7 = readLeS16(&calibRaw[18]);
  bmpCalib.digP8 = readLeS16(&calibRaw[20]);
  bmpCalib.digP9 = readLeS16(&calibRaw[22]);
  bmpCalib.valid = true;
  bmp280Address = addr;

  if (!i2cWriteReg8(bmp280Address, 0xF4, 0x27)) {  // temp x1, press x1, normal mode
    return false;
  }
  if (!i2cWriteReg8(bmp280Address, 0xF5, 0xA0)) {  // standby + filter
    return false;
  }
  responsiveDelay(10);
  return true;
}

bool readBmp280(float& temperatureC, float& pressureHpa) {
  if (!bmpCalib.valid || bmp280Address == 0) {
    return false;
  }
  uint8_t raw[6] = {0};
  if (!i2cReadRegBytes(bmp280Address, 0xF7, raw, sizeof(raw))) {
    return false;
  }

  const int32_t adcP =
      (static_cast<int32_t>(raw[0]) << 12) | (static_cast<int32_t>(raw[1]) << 4) |
      (static_cast<int32_t>(raw[2]) >> 4);
  const int32_t adcT =
      (static_cast<int32_t>(raw[3]) << 12) | (static_cast<int32_t>(raw[4]) << 4) |
      (static_cast<int32_t>(raw[5]) >> 4);

  int32_t var1 = ((((adcT >> 3) - (static_cast<int32_t>(bmpCalib.digT1) << 1))) *
                  static_cast<int32_t>(bmpCalib.digT2)) >>
                 11;
  int32_t var2 = (((((adcT >> 4) - static_cast<int32_t>(bmpCalib.digT1)) *
                    ((adcT >> 4) - static_cast<int32_t>(bmpCalib.digT1))) >>
                   12) *
                  static_cast<int32_t>(bmpCalib.digT3)) >>
                 14;
  const int32_t tFine = var1 + var2;
  const int32_t t = (tFine * 5 + 128) >> 8;
  temperatureC = static_cast<float>(t) / 100.0f;

  int64_t pVar1 = static_cast<int64_t>(tFine) - 128000;
  int64_t pVar2 = pVar1 * pVar1 * static_cast<int64_t>(bmpCalib.digP6);
  pVar2 = pVar2 + ((pVar1 * static_cast<int64_t>(bmpCalib.digP5)) << 17);
  pVar2 = pVar2 + (static_cast<int64_t>(bmpCalib.digP4) << 35);
  pVar1 = ((pVar1 * pVar1 * static_cast<int64_t>(bmpCalib.digP3)) >> 8) +
          ((pVar1 * static_cast<int64_t>(bmpCalib.digP2)) << 12);
  pVar1 = (((static_cast<int64_t>(1) << 47) + pVar1) * static_cast<int64_t>(bmpCalib.digP1)) >> 33;
  if (pVar1 == 0) {
    return false;
  }

  int64_t p = 1048576 - adcP;
  p = (((p << 31) - pVar2) * 3125) / pVar1;
  pVar1 = (static_cast<int64_t>(bmpCalib.digP9) * (p >> 13) * (p >> 13)) >> 25;
  pVar2 = (static_cast<int64_t>(bmpCalib.digP8) * p) >> 19;
  p = ((p + pVar1 + pVar2) >> 8) + (static_cast<int64_t>(bmpCalib.digP7) << 4);
  pressureHpa = (static_cast<float>(p) / 256.0f) / 100.0f;
  return true;
}

void initEnvSensors() {
  if (!switchI2cOwner(I2cOwner::Env)) {
    return;
  }
  aht20Ready = initAht20();
  bmp280Ready = initBmp280();

  Serial.print("[ENV] AHT20=");
  Serial.print(aht20Ready ? "ok" : "none");
  Serial.print(", BMP280=");
  Serial.print(bmp280Ready ? "ok" : "none");
  if (bmp280Ready) {
    Serial.print(" @0x");
    Serial.print(bmp280Address, HEX);
  }
  Serial.println();

  const unsigned long now = millis();
  if (aht20Ready) {
    lastAhtSuccessMs = now;
  }
  if (bmp280Ready) {
    lastBmpSuccessMs = now;
  }
  switchI2cOwner(I2cOwner::Oled);
}

void pollEnvSensors(bool force = false) {
  const unsigned long now = millis();
  if ((!aht20Ready || !bmp280Ready) && (now - lastEnvReinitAttemptMs) >= ENV_REINIT_MS) {
    lastEnvReinitAttemptMs = now;
    if (switchI2cOwner(I2cOwner::Env)) {
      if (!aht20Ready) {
        aht20Ready = initAht20();
      }
      if (!bmp280Ready) {
        bmp280Ready = initBmp280();
      }
      switchI2cOwner(I2cOwner::Oled);
    }
  }

  if (!force && (now - lastEnvPollMs) < ENV_POLL_MS) {
    return;
  }
  lastEnvPollMs = now;

  if (!aht20Ready && !bmp280Ready) {
    return;
  }
  if (!switchI2cOwner(I2cOwner::Env)) {
    return;
  }

  bool updated = false;
  if (aht20Ready) {
    float t = 0.0f;
    float h = 0.0f;
    if (readAht20(t, h)) {
      latestEnvData.ahtValid = true;
      latestEnvData.temperatureC = t;
      latestEnvData.humidityPct = h;
      lastAhtSuccessMs = now;
      uint16_t tempScaled = 0;
      if (readEnvMetricScaled(EnvItem::Temp, tempScaled)) {
        lastEnvScaledValues[static_cast<uint8_t>(EnvItem::Temp)] = tempScaled;
        envMetricReady[static_cast<uint8_t>(EnvItem::Temp)] = true;
      }
      uint16_t humScaled = 0;
      if (readEnvMetricScaled(EnvItem::Humidity, humScaled)) {
        lastEnvScaledValues[static_cast<uint8_t>(EnvItem::Humidity)] = humScaled;
        envMetricReady[static_cast<uint8_t>(EnvItem::Humidity)] = true;
      }
      updated = true;
    }
  }

  if (bmp280Ready) {
    float t = 0.0f;
    float p = 0.0f;
    if (readBmp280(t, p)) {
      latestEnvData.bmpValid = true;
      latestEnvData.pressureHpa = p;
      if (!latestEnvData.ahtValid) {
        latestEnvData.temperatureC = t;
      }
      lastBmpSuccessMs = now;
      uint16_t tempScaled = 0;
      if (readEnvMetricScaled(EnvItem::Temp, tempScaled)) {
        lastEnvScaledValues[static_cast<uint8_t>(EnvItem::Temp)] = tempScaled;
        envMetricReady[static_cast<uint8_t>(EnvItem::Temp)] = true;
      }
      uint16_t pressureScaled = 0;
      if (readEnvMetricScaled(EnvItem::Pressure, pressureScaled)) {
        lastEnvScaledValues[static_cast<uint8_t>(EnvItem::Pressure)] = pressureScaled;
        envMetricReady[static_cast<uint8_t>(EnvItem::Pressure)] = true;
      }
      updated = true;
    }
  }

  if (latestEnvData.ahtValid && (now - lastAhtSuccessMs) > ENV_STALE_MS) {
    latestEnvData.ahtValid = false;
  }
  if (latestEnvData.bmpValid && (now - lastBmpSuccessMs) > ENV_STALE_MS) {
    latestEnvData.bmpValid = false;
  }

  if (updated) {
    latestEnvData.updatedMs = now;
  }
  switchI2cOwner(I2cOwner::Oled);
}

void reportEnvDataPeriodically() {
  if (!latestEnvData.ahtValid && !latestEnvData.bmpValid) {
    return;
  }
  const unsigned long now = millis();
  if ((now - lastEnvReportMs) < ENV_REPORT_MS) {
    return;
  }
  lastEnvReportMs = now;

  Serial.print("[ENV] T=");
  Serial.print(latestEnvData.temperatureC, 1);
  Serial.print(" C");
  if (latestEnvData.ahtValid) {
    Serial.print(", RH=");
    Serial.print(latestEnvData.humidityPct, 1);
    Serial.print(" %");
  }
  if (latestEnvData.bmpValid) {
    Serial.print(", P=");
    Serial.print(latestEnvData.pressureHpa, 1);
    Serial.print(" hPa");
  }
  Serial.println();
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

bool updateDebouncedButton(DebouncedButton& button, uint8_t pin) {
  const bool rawPressed = (digitalRead(pin) == LOW);
  if (rawPressed != button.rawPressed) {
    button.rawPressed = rawPressed;
    button.lastRawChangeMs = millis();
  }
  if ((millis() - button.lastRawChangeMs) >= DEBOUNCE_MS) {
    button.stablePressed = rawPressed;
  }
  return button.stablePressed;
}

ButtonEvent navigationEventForButton(ButtonId id) {
  if (id == ButtonId::Up) {
    return ButtonEvent::Up;
  }
  if (id == ButtonId::Down) {
    return ButtonEvent::Down;
  }
  return ButtonEvent::None;
}

ButtonEvent pollButtonEvent() {
  const unsigned long now = millis();
  const bool upPressed = updateDebouncedButton(upButtonState, BUTTON_UP_PIN);
  const bool downPressed = updateDebouncedButton(downButtonState, BUTTON_DOWN_PIN);

  const bool upPressedEdge = upPressed && !previousUpPressed;
  const bool downPressedEdge = downPressed && !previousDownPressed;
  ButtonId pressedButton = ButtonId::None;
  if (upPressedEdge) {
    pressedButton = ButtonId::Up;
  } else if (downPressedEdge) {
    pressedButton = ButtonId::Down;
  }

  if (comboConfirmTriggered) {
    if (!upPressed && !downPressed) {
      comboConfirmTriggered = false;
      comboLongHoldTriggered = false;
      comboPressStartMs = 0;
      suppressNextReleaseButton = ButtonId::None;
    } else if (currentScreen == Screen::ButtonTest && !comboLongHoldTriggered &&
               upPressed && downPressed && (now - comboPressStartMs) >= TEST_MODE_EXIT_HOLD_MS) {
      comboLongHoldTriggered = true;
      previousUpPressed = upPressed;
      previousDownPressed = downPressed;
      return ButtonEvent::Back;
    }
    previousUpPressed = upPressed;
    previousDownPressed = downPressed;
    return ButtonEvent::None;
  }

  if (upPressed && downPressed) {
    comboConfirmTriggered = true;
    comboLongHoldTriggered = false;
    comboPressStartMs = now;
    suppressNextReleaseButton = ButtonId::None;
    previousUpPressed = upPressed;
    previousDownPressed = downPressed;
    if (currentScreen == Screen::ButtonTest) {
      return ButtonEvent::None;
    }
    return ButtonEvent::Confirm;
  }

  if (pressedButton != ButtonId::None) {
    if (currentScreen == Screen::ButtonTest) {
      if (pressedButton == ButtonId::Up && buttonTestUpReleaseMs != 0) {
        buttonTestUpGapMs = now - buttonTestUpReleaseMs;
        buttonTestUpGapValid = true;
      } else if (pressedButton == ButtonId::Down && buttonTestDownReleaseMs != 0) {
        buttonTestDownGapMs = now - buttonTestDownReleaseMs;
        buttonTestDownGapValid = true;
      }
    }

    const unsigned long releaseGapMs = now - lastBackReleaseMs;
    const bool hasPreviousRelease = lastBackReleaseMs != 0;
    const bool isDoubleClick = (currentScreen != Screen::Root) &&
                               (currentScreen != Screen::ButtonTest) &&
                               hasPreviousRelease && (releaseGapMs <= DOUBLE_CLICK_MS);
    if (isDoubleClick) {
      lastBackReleaseMs = 0;
      suppressNextReleaseButton = pressedButton;
      previousUpPressed = upPressed;
      previousDownPressed = downPressed;
      return ButtonEvent::Back;
    }
  }

  ButtonId releasedButton = ButtonId::None;
  if (!upPressed && previousUpPressed) {
    releasedButton = ButtonId::Up;
  } else if (!downPressed && previousDownPressed) {
    releasedButton = ButtonId::Down;
  }

  if (releasedButton != ButtonId::None) {
    if (suppressNextReleaseButton == releasedButton) {
      suppressNextReleaseButton = ButtonId::None;
      previousUpPressed = upPressed;
      previousDownPressed = downPressed;
      return ButtonEvent::None;
    }
    if (currentScreen == Screen::ButtonTest) {
      if (releasedButton == ButtonId::Up) {
        buttonTestUpReleaseMs = now;
      } else {
        buttonTestDownReleaseMs = now;
      }
    } else {
      lastBackReleaseMs = now;
    }
    previousUpPressed = upPressed;
    previousDownPressed = downPressed;
    return navigationEventForButton(releasedButton);
  }

  previousUpPressed = upPressed;
  previousDownPressed = downPressed;
  return ButtonEvent::None;
}

uint8_t wrapSelection(uint8_t current, int8_t delta, uint8_t count) {
  int next = static_cast<int>(current) + static_cast<int>(delta);
  while (next < 0) {
    next += count;
  }
  while (next >= count) {
    next -= count;
  }
  return static_cast<uint8_t>(next);
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

void pushEnvRealtimePoint(uint8_t metric, uint16_t avg, uint16_t peak) {
  RealtimeSeries& s = envRealtimeSeries[metric];
  s.avg[s.head] = avg;
  s.peak[s.head] = peak;
  s.head = static_cast<uint8_t>((s.head + 1) % REALTIME_SECONDS);
  if (s.count < REALTIME_SECONDS) {
    s.count++;
  }
}

void pushEnvHistoryPoint(uint8_t metric, uint16_t avg, uint16_t peak) {
  HistorySeries& s = envHistorySeries[metric];
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

ChartDataPoint getEnvRealtimeByChrono(uint8_t metric, uint8_t posFromOldest) {
  ChartDataPoint out;
  const RealtimeSeries& s = envRealtimeSeries[metric];
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

ChartDataPoint getEnvHistoryByChrono(uint8_t metric, uint16_t posFromOldest) {
  ChartDataPoint out;
  const HistorySeries& s = envHistorySeries[metric];
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
  lastAirHistorySaveMs = millis();
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

void saveEnvHistoryToNvs() {
  prefs.begin("envhist", false);
  prefs.putUChar("fmt", ENV_HIST_FMT_VER);
  prefs.putBytes("h0", &envHistorySeries[0], sizeof(HistorySeries));
  prefs.putBytes("h1", &envHistorySeries[1], sizeof(HistorySeries));
  prefs.putBytes("h2", &envHistorySeries[2], sizeof(HistorySeries));
  prefs.end();
  lastEnvHistorySaveMs = millis();
}

void loadEnvHistoryFromNvs() {
  prefs.begin("envhist", true);
  const uint8_t fmt = prefs.getUChar("fmt", 0);
  if (fmt != ENV_HIST_FMT_VER) {
    memset(envHistorySeries, 0, sizeof(envHistorySeries));
    prefs.end();
    saveEnvHistoryToNvs();
    return;
  }
  if (prefs.getBytesLength("h0") == sizeof(HistorySeries)) {
    prefs.getBytes("h0", &envHistorySeries[0], sizeof(HistorySeries));
  }
  if (prefs.getBytesLength("h1") == sizeof(HistorySeries)) {
    prefs.getBytes("h1", &envHistorySeries[1], sizeof(HistorySeries));
  }
  if (prefs.getBytesLength("h2") == sizeof(HistorySeries)) {
    prefs.getBytes("h2", &envHistorySeries[2], sizeof(HistorySeries));
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

void formatEnvCurrentMetricValue(EnvItem item, char* out, size_t outLen) {
  switch (item) {
    case EnvItem::Temp:
      if (!latestEnvData.ahtValid && !latestEnvData.bmpValid) {
        snprintf(out, outLen, "TEMP --.-");
      } else {
        snprintf(out, outLen, "TEMP %.1f", latestEnvData.temperatureC);
      }
      return;
    case EnvItem::Humidity:
      if (!latestEnvData.ahtValid) {
        snprintf(out, outLen, "HUM --.-");
      } else {
        snprintf(out, outLen, "HUM %.1f", latestEnvData.humidityPct);
      }
      return;
    case EnvItem::Pressure:
      if (!latestEnvData.bmpValid) {
        snprintf(out, outLen, "PRES ----");
      } else {
        snprintf(out, outLen, "PRES %.1f", latestEnvData.pressureHpa);
      }
      return;
  }
}

void formatEnvAxisMetricValue(EnvItem item, uint16_t raw, char* out, size_t outLen) {
  switch (item) {
    case EnvItem::Temp: {
      const float c = static_cast<float>(raw) * 0.01f - 40.0f;
      snprintf(out, outLen, "%.2f", c);
      return;
    }
    case EnvItem::Humidity: {
      const float rh = static_cast<float>(raw) * 0.01f;
      snprintf(out, outLen, "%.2f", rh);
      if (out[0] == '0' && out[1] == '.') {
        memmove(out, out + 1, strlen(out));
      }
      return;
    }
    case EnvItem::Pressure: {
      const float p = ENV_PRESSURE_OFFSET_HPA + (static_cast<float>(raw) * 0.05f);
      snprintf(out, outLen, "%.2f", p);
      return;
    }
  }
}

float envScaledToValue(EnvItem item, uint16_t raw) {
  switch (item) {
    case EnvItem::Temp:
      return static_cast<float>(raw) * 0.01f - 40.0f;
    case EnvItem::Humidity:
      return static_cast<float>(raw) * 0.01f;
    case EnvItem::Pressure:
      return ENV_PRESSURE_OFFSET_HPA + static_cast<float>(raw) * 0.05f;
  }
  return 0.0f;
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
      lastAirRawValues[0] = latestAirData.co2Raw;
      lastAirRawValues[1] = latestAirData.tvocRaw;
      lastAirRawValues[2] = latestAirData.hchoRaw;
      airMetricReady[0] = true;
      airMetricReady[1] = true;
      airMetricReady[2] = true;
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
  Serial.print(" btnU=");
  Serial.print(upButtonState.stablePressed ? "1" : "0");
  Serial.print(" btnD=");
  Serial.print(downButtonState.stablePressed ? "1" : "0");
  Serial.print(" air=");
  Serial.print(latestAirData.valid ? "ok" : "none");
  Serial.print(" env=");
  Serial.println((latestEnvData.ahtValid || latestEnvData.bmpValid) ? "ok" : "none");
}

void printHistoryStats() {
  Serial.print("[HIST] AIR count CO2/TVOC/HCHO=");
  Serial.print(historySeries[0].count);
  Serial.print("/");
  Serial.print(historySeries[1].count);
  Serial.print("/");
  Serial.println(historySeries[2].count);

  Serial.print("[HIST] ENV count T/H/P=");
  Serial.print(envHistorySeries[0].count);
  Serial.print("/");
  Serial.print(envHistorySeries[1].count);
  Serial.print("/");
  Serial.println(envHistorySeries[2].count);

  Serial.print("[HIST] live AIR accum=");
  Serial.print(tenMinAccum[0].count);
  Serial.print("/");
  Serial.print(tenMinAccum[1].count);
  Serial.print("/");
  Serial.println(tenMinAccum[2].count);

  Serial.print("[HIST] live ENV accum=");
  Serial.print(envTenMinAccum[0].count);
  Serial.print("/");
  Serial.print(envTenMinAccum[1].count);
  Serial.print("/");
  Serial.println(envTenMinAccum[2].count);

  Serial.print("[HIST] last save ms AIR/ENV=");
  Serial.print(lastAirHistorySaveMs);
  Serial.print("/");
  Serial.println(lastEnvHistorySaveMs);
}

void printAirHistoryDump() {
  for (uint8_t i = 0; i < METRIC_COUNT; ++i) {
    const AirItem item = static_cast<AirItem>(i);
    const HistorySeries& s = historySeries[i];
    Serial.print("[HISTDUMP] AIR ");
    Serial.print(metricName(item));
    Serial.print(" count=");
    Serial.println(s.count);
    if (s.count == 0) {
      continue;
    }
    const uint16_t n = (s.count >= 5) ? 5 : s.count;
    for (uint16_t k = 0; k < n; ++k) {
      const uint16_t pos = static_cast<uint16_t>(s.count - n + k);
      const ChartDataPoint p = getHistoryByChrono(i, pos);
      Serial.print("  #");
      Serial.print(k + 1);
      Serial.print(" avg=");
      if (item == AirItem::Co2) {
        Serial.print(p.avg);
        Serial.print("ppm");
      } else {
        Serial.print(rawToMgPerM3(p.avg), 3);
        Serial.print("mg/m3");
      }
      Serial.print(" peak=");
      if (item == AirItem::Co2) {
        Serial.print(p.peak);
        Serial.println("ppm");
      } else {
        Serial.print(rawToMgPerM3(p.peak), 3);
        Serial.println("mg/m3");
      }
    }
  }
}

void printEnvHistoryDump() {
  for (uint8_t i = 0; i < METRIC_COUNT; ++i) {
    const EnvItem item = static_cast<EnvItem>(i);
    const HistorySeries& s = envHistorySeries[i];
    Serial.print("[HISTDUMP] ENV ");
    Serial.print(envMetricName(item));
    Serial.print(" count=");
    Serial.println(s.count);
    if (s.count == 0) {
      continue;
    }
    const uint16_t n = (s.count >= 5) ? 5 : s.count;
    for (uint16_t k = 0; k < n; ++k) {
      const uint16_t pos = static_cast<uint16_t>(s.count - n + k);
      const ChartDataPoint p = getEnvHistoryByChrono(i, pos);
      Serial.print("  #");
      Serial.print(k + 1);
      Serial.print(" avg=");
      Serial.print(envScaledToValue(item, p.avg), 3);
      Serial.print(envMetricUnit(item));
      Serial.print(" peak=");
      Serial.print(envScaledToValue(item, p.peak), 3);
      Serial.println(envMetricUnit(item));
    }
  }
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

  if (strcmp(cmdLine, "hist") == 0) {
    printHistoryStats();
    return;
  }

  if (strcmp(cmdLine, "histdump") == 0) {
    printHistoryStats();
    printAirHistoryDump();
    printEnvHistoryDump();
    return;
  }

  if (strcmp(cmdLine, "env") == 0) {
    if (!latestEnvData.ahtValid && !latestEnvData.bmpValid) {
      Serial.println("[ENV] no valid data yet");
      return;
    }
    Serial.print("[ENV] T=");
    Serial.print(latestEnvData.temperatureC, 1);
    Serial.print(" C");
    if (latestEnvData.ahtValid) {
      Serial.print(", RH=");
      Serial.print(latestEnvData.humidityPct, 1);
      Serial.print(" %");
    }
    if (latestEnvData.bmpValid) {
      Serial.print(", P=");
      Serial.print(latestEnvData.pressureHpa, 1);
      Serial.print(" hPa");
    }
    Serial.println();
    return;
  }

  if (strcmp(cmdLine, "help") == 0) {
    Serial.println("[CMD] help:");
    Serial.println("  status        -> show BLE status");
    Serial.println("  env           -> show AHT20/BMP280 data");
    Serial.println("  hist          -> show AIR/ENV history stats");
    Serial.println("  histdump      -> show latest history values");
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

void finalizeEnvSecondWindow() {
  for (uint8_t i = 0; i < METRIC_COUNT; ++i) {
    if (envSecondAccum[i].count == 0) {
      continue;
    }
    const uint16_t avg = static_cast<uint16_t>(envSecondAccum[i].sum / envSecondAccum[i].count);
    pushEnvRealtimePoint(i, avg, envSecondAccum[i].peak);
    envSecondAccum[i] = RunningAccum{};
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
    Serial.print("[HIST] AIR saved, counts CO2/TVOC/HCHO=");
    Serial.print(historySeries[0].count);
    Serial.print("/");
    Serial.print(historySeries[1].count);
    Serial.print("/");
    Serial.println(historySeries[2].count);
  }
}

void finalizeEnvTenMinuteWindow() {
  bool any = false;
  for (uint8_t i = 0; i < METRIC_COUNT; ++i) {
    if (envTenMinAccum[i].count == 0) {
      continue;
    }
    const uint16_t avg = static_cast<uint16_t>(envTenMinAccum[i].sum / envTenMinAccum[i].count);
    pushEnvHistoryPoint(i, avg, envTenMinAccum[i].peak);
    envTenMinAccum[i] = RunningAccum{};
    any = true;
  }
  if (any) {
    saveEnvHistoryToNvs();
    Serial.print("[HIST] ENV saved, counts T/H/P=");
    Serial.print(envHistorySeries[0].count);
    Serial.print("/");
    Serial.print(envHistorySeries[1].count);
    Serial.print("/");
    Serial.println(envHistorySeries[2].count);
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

void accumulateEnvSample() {
  for (uint8_t i = 0; i < METRIC_COUNT; ++i) {
    if (!envMetricReady[i]) {
      continue;
    }
    const uint16_t value = lastEnvScaledValues[i];

    envSecondAccum[i].sum += value;
    if (envSecondAccum[i].count == 0 || value > envSecondAccum[i].peak) {
      envSecondAccum[i].peak = value;
    }
    envSecondAccum[i].count++;

    envTenMinAccum[i].sum += value;
    if (envTenMinAccum[i].count == 0 || value > envTenMinAccum[i].peak) {
      envTenMinAccum[i].peak = value;
    }
    envTenMinAccum[i].count++;
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
    if (airMetricReady[0] && airMetricReady[1] && airMetricReady[2]) {
      accumulateSample(lastAirRawValues[0], lastAirRawValues[1], lastAirRawValues[2]);
    }
    accumulateEnvSample();
    lastSampleTickMs += SAMPLE_INTERVAL_MS;
  }

  while ((now - secondWindowStartMs) >= SECOND_WINDOW_MS) {
    finalizeSecondWindow();
    finalizeEnvSecondWindow();
    secondWindowStartMs += SECOND_WINDOW_MS;
  }

  while ((now - tenMinWindowStartMs) >= HISTORY_WINDOW_MS) {
    finalizeTenMinuteWindow();
    finalizeEnvTenMinuteWindow();
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

uint16_t getEnvRealtimeCountWithLive(uint8_t metric) {
  uint16_t c = envRealtimeSeries[metric].count;
  if (envSecondAccum[metric].count > 0) {
    c++;
  }
  if (c > REALTIME_SECONDS) {
    c = REALTIME_SECONDS;
  }
  return c;
}

ChartDataPoint getEnvRealtimePointWithLive(uint8_t metric, uint16_t posFromOldest) {
  ChartDataPoint out;
  const RealtimeSeries& s = envRealtimeSeries[metric];
  const bool hasLive = envSecondAccum[metric].count > 0;
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
    return getEnvRealtimeByChrono(metric, static_cast<uint8_t>(normalizedPos));
  }

  if (!hasLive) {
    return out;
  }
  const RunningAccum& live = envSecondAccum[metric];
  out.avg = static_cast<uint16_t>(live.sum / live.count);
  out.peak = live.peak;
  out.valid = true;
  return out;
}

uint16_t getEnvHistoryCountWithLive(uint8_t metric) {
  uint16_t c = envHistorySeries[metric].count;
  if (envTenMinAccum[metric].count > 0) {
    c++;
  }
  if (c > HISTORY_SLOTS) {
    c = HISTORY_SLOTS;
  }
  return c;
}

ChartDataPoint getEnvHistoryPointWithLive(uint8_t metric, uint16_t posFromOldest) {
  ChartDataPoint out;
  const HistorySeries& s = envHistorySeries[metric];
  const bool hasLive = envTenMinAccum[metric].count > 0;
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
    return getEnvHistoryByChrono(metric, normalizedPos);
  }

  if (!hasLive) {
    return out;
  }
  const RunningAccum& live = envTenMinAccum[metric];
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

void formatEnvTimeSpanLabel(EnvItem item, PlotMode mode, char* out, size_t outLen) {
  if (mode == PlotMode::Realtime) {
    uint16_t seconds = getEnvRealtimeCountWithLive(static_cast<uint8_t>(item));
    if (seconds > REALTIME_SECONDS) {
      seconds = REALTIME_SECONDS;
    }
    snprintf(out, outLen, "-%us", static_cast<unsigned>(seconds));
    return;
  }

  const uint16_t points = getEnvHistoryCountWithLive(static_cast<uint8_t>(item));
  const uint32_t minutes = static_cast<uint32_t>(points) * 10UL;
  if (minutes < 60UL) {
    snprintf(out, outLen, "-%lum", static_cast<unsigned long>(minutes));
  } else {
    const uint32_t hoursRoundedUp = (minutes + 59UL) / 60UL;
    snprintf(out, outLen, "-%luh", static_cast<unsigned long>(hoursRoundedUp));
  }
}

void formatEnvTimeHalfSpanLabel(EnvItem item, PlotMode mode, char* out, size_t outLen) {
  if (mode == PlotMode::Realtime) {
    uint16_t seconds = getEnvRealtimeCountWithLive(static_cast<uint8_t>(item));
    seconds = static_cast<uint16_t>(seconds / 2);
    snprintf(out, outLen, "-%us", static_cast<unsigned>(seconds));
    return;
  }

  const uint16_t points = getEnvHistoryCountWithLive(static_cast<uint8_t>(item));
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
        outAvg[x] = AIR_INVALID_POINT;
        outPeak[x] = AIR_INVALID_POINT;
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
      outAvg[x] = AIR_INVALID_POINT;
      outPeak[x] = AIR_INVALID_POINT;
    } else {
      outAvg[x] = static_cast<uint16_t>(avgSum / samples);
      outPeak[x] = peakMax;
    }
  }
  *outCount = CHART_W;
}

void collectEnvSeriesForPlot(uint8_t metric, PlotMode mode, uint16_t* outAvg, uint16_t* outPeak,
                             uint8_t* outCount) {
  if (mode == PlotMode::Realtime) {
    const uint16_t count = getEnvRealtimeCountWithLive(metric);
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
        const ChartDataPoint point = getEnvRealtimePointWithLive(metric, p);
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
        outAvg[x] = ENV_INVALID_POINT;
        outPeak[x] = ENV_INVALID_POINT;
      } else {
        outAvg[x] = static_cast<uint16_t>(avgSum / samples);
        outPeak[x] = peakMax;
      }
    }
    *outCount = CHART_W;
    return;
  }

  const uint16_t count = getEnvHistoryCountWithLive(metric);
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
      const ChartDataPoint point = getEnvHistoryPointWithLive(metric, p);
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
      outAvg[x] = ENV_INVALID_POINT;
      outPeak[x] = ENV_INVALID_POINT;
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

  uint16_t minVal = 0;
  uint16_t maxVal = 0;
  bool foundValid = false;
  for (uint8_t i = 0; i < count; ++i) {
    if (avg[i] == AIR_INVALID_POINT || peak[i] == AIR_INVALID_POINT) {
      continue;
    }
    if (!foundValid) {
      minVal = avg[i];
      maxVal = peak[i];
      foundValid = true;
      continue;
    }
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
  if (!foundValid) {
    oled.drawStr(0, 23, "NO DATA");
    return scale;
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
    if (avg[x] == AIR_INVALID_POINT || peak[x] == AIR_INVALID_POINT) {
      hasPrev = false;
      continue;
    }
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
    if (!hasPrev) {
      oled.drawPixel(px, avgY);
    }
    hasPrev = true;
  }

  return scale;
}

GraphScale drawEnvGraph(EnvItem item, PlotMode mode) {
  GraphScale scale;
  uint16_t avg[CHART_W] = {0};
  uint16_t peak[CHART_W] = {0};
  uint8_t count = 0;
  collectEnvSeriesForPlot(static_cast<uint8_t>(item), mode, avg, peak, &count);
  if (count == 0) {
    oled.drawStr(0, 23, "NO DATA");
    return scale;
  }

  uint16_t minVal = 0;
  uint16_t maxVal = 0;
  bool foundValid = false;
  for (uint8_t i = 0; i < count; ++i) {
    if (avg[i] == ENV_INVALID_POINT || peak[i] == ENV_INVALID_POINT) {
      continue;
    }
    if (!foundValid) {
      minVal = avg[i];
      maxVal = peak[i];
      foundValid = true;
      continue;
    }
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
  if (!foundValid) {
    oled.drawStr(0, 23, "NO DATA");
    return scale;
  }
  if (maxVal <= minVal) {
    maxVal = static_cast<uint16_t>(minVal + 1);
  }

  scale.valid = true;
  scale.minVal = minVal;
  scale.maxVal = maxVal;

  const uint16_t range = static_cast<uint16_t>(maxVal - minVal);
  const int yBottom = CHART_Y + CHART_H - 1;

  bool hasPrev = false;
  int prevAvgX = CHART_X;
  int prevAvgY = yBottom;
  int prevPeakX = CHART_X;
  int prevPeakY = yBottom;

  for (uint8_t x = 0; x < count; ++x) {
    if (avg[x] == ENV_INVALID_POINT || peak[x] == ENV_INVALID_POINT) {
      hasPrev = false;
      continue;
    }
    const int px = CHART_X + x;
    const int avgY = yBottom - static_cast<int>((static_cast<uint32_t>(avg[x] - minVal) * (CHART_H - 1)) / range);
    const int peakY = yBottom - static_cast<int>((static_cast<uint32_t>(peak[x] - minVal) * (CHART_H - 1)) / range);

    if (hasPrev) {
      const int gapPrev = abs(prevAvgY - prevPeakY);
      const int gapCurr = abs(avgY - peakY);
      const bool mergeToWideAvg = (gapPrev <= 2 && gapCurr <= 2);

      if (mergeToWideAvg) {
        oled.drawLine(prevAvgX, prevAvgY, px, avgY);
        if ((prevAvgY + 1) <= yBottom && (avgY + 1) <= yBottom) {
          oled.drawLine(prevAvgX, prevAvgY + 1, px, avgY + 1);
        } else if ((prevAvgY - 1) >= CHART_Y && (avgY - 1) >= CHART_Y) {
          oled.drawLine(prevAvgX, prevAvgY - 1, px, avgY - 1);
        }
      } else {
        oled.drawLine(prevAvgX, prevAvgY, px, avgY);
        drawDashedLine(prevPeakX, prevPeakY, px, peakY);
      }
    }
    prevAvgX = px;
    prevAvgY = avgY;
    prevPeakX = px;
    prevPeakY = peakY;
    if (!hasPrev) {
      oled.drawPixel(px, avgY);
    }
    hasPrev = true;
  }

  return scale;
}

uint8_t keyMenuWindowStart() {
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
  return windowStart;
}

bool getMenuIndicatorRow(Screen screen, uint8_t* outRow) {
  switch (screen) {
    case Screen::Root:
      *outRow = selectedRoot;
      return true;
    case Screen::Key:
      *outRow = static_cast<uint8_t>(selectedKey - keyMenuWindowStart());
      return true;
    case Screen::Air:
      *outRow = selectedAir;
      return true;
    case Screen::Env:
      *outRow = selectedEnv;
      return true;
    case Screen::Settings:
      *outRow = selectedSetting;
      return true;
    default:
      return false;
  }
}

int16_t menuRowBaseline(uint8_t row) {
  return static_cast<int16_t>(15 + row * 8);
}

unsigned long menuIndicatorAnimDuration() {
  const unsigned long now = millis();
  if (lastMenuNavigateMs == 0) {
    lastMenuNavigateMs = now;
    lastMenuAnimDurationMs = MENU_INDICATOR_ANIM_MS;
    return lastMenuAnimDurationMs;
  }

  const unsigned long rawGapMs = constrain(now - lastMenuNavigateMs, 70UL, 360UL);
  lastMenuNavigateMs = now;

  // Smooth the input rhythm first so nearby presses do not swing the animation too hard.
  smoothedMenuGapMs = (smoothedMenuGapMs * 3UL + rawGapMs) / 4UL;

  const unsigned long mappedDuration =
      MENU_INDICATOR_MIN_MS +
      ((smoothedMenuGapMs - 70UL) * (MENU_INDICATOR_MAX_MS - MENU_INDICATOR_MIN_MS)) / 290UL;

  unsigned long nextDuration = (lastMenuAnimDurationMs * 2UL + mappedDuration) / 3UL;
  if (nextDuration > (lastMenuAnimDurationMs + MENU_INDICATOR_STEP_MS)) {
    nextDuration = lastMenuAnimDurationMs + MENU_INDICATOR_STEP_MS;
  } else if (nextDuration + MENU_INDICATOR_STEP_MS < lastMenuAnimDurationMs) {
    nextDuration = lastMenuAnimDurationMs - MENU_INDICATOR_STEP_MS;
  }

  lastMenuAnimDurationMs = constrain(nextDuration, MENU_INDICATOR_MIN_MS, MENU_INDICATOR_MAX_MS);
  return lastMenuAnimDurationMs;
}

int16_t currentMenuIndicatorY() {
  if (!menuIndicatorAnim.initialized) {
    return menuIndicatorAnim.targetY;
  }
  const unsigned long elapsed = millis() - menuIndicatorAnim.startMs;
  if (elapsed >= menuIndicatorAnim.durationMs) {
    return menuIndicatorAnim.targetY;
  }

  const float t = static_cast<float>(elapsed) / static_cast<float>(menuIndicatorAnim.durationMs);
  const float inv = 1.0f - t;
  const float eased = 1.0f - (inv * inv * inv);  // ease-out cubic
  const float y = static_cast<float>(menuIndicatorAnim.startY) +
                  (static_cast<float>(menuIndicatorAnim.targetY - menuIndicatorAnim.startY) * eased);
  return static_cast<int16_t>(y + 0.5f);
}

void snapMenuIndicator(Screen screen, uint8_t row) {
  const int16_t y = menuRowBaseline(row);
  menuIndicatorAnim.initialized = true;
  menuIndicatorAnim.screen = screen;
  menuIndicatorAnim.startY = y;
  menuIndicatorAnim.targetY = y;
  menuIndicatorAnim.startMs = millis();
  menuIndicatorAnim.durationMs = 1;
}

void animateMenuIndicator(Screen screen) {
  uint8_t row = 0;
  if (!getMenuIndicatorRow(screen, &row)) {
    return;
  }
  const int16_t targetY = menuRowBaseline(row);
  if (!menuIndicatorAnim.initialized || menuIndicatorAnim.screen != screen) {
    snapMenuIndicator(screen, row);
    return;
  }
  if (menuIndicatorAnim.targetY == targetY) {
    return;
  }
  menuIndicatorAnim.startY = currentMenuIndicatorY();
  menuIndicatorAnim.targetY = targetY;
  menuIndicatorAnim.startMs = millis();
  menuIndicatorAnim.durationMs = menuIndicatorAnimDuration();
}

void drawMenuIndicator(Screen screen) {
  uint8_t row = 0;
  if (!getMenuIndicatorRow(screen, &row)) {
    return;
  }
  if (!menuIndicatorAnim.initialized || menuIndicatorAnim.screen != screen) {
    snapMenuIndicator(screen, row);
  }
  const int16_t baselineY = currentMenuIndicatorY();
  const int16_t centerY = baselineY - 3;
  oled.drawBox(0, centerY - 2, 2, 5);
  oled.drawLine(2, centerY - 2, 5, centerY);
  oled.drawLine(2, centerY + 2, 5, centerY);
}

void drawRootScreen() {
  static const char* ROOT_LABELS[ROOT_ITEM_COUNT] = {"KEY", "AIR", "ENV", "SET"};

  oled.drawStr(0, 7, "MENU");

  for (uint8_t line = 0; line < ROOT_ITEM_COUNT; ++line) {
    const uint8_t idx = line;
    oled.drawStr(8, static_cast<int16_t>(15 + line * 8), ROOT_LABELS[idx]);
  }
  drawMenuIndicator(Screen::Root);
}

void drawKeyScreen() {
  oled.drawStr(0, 7, "KEY");

  const uint8_t windowStart = keyMenuWindowStart();

  for (uint8_t line = 0; line < 3; ++line) {
    const uint8_t idx = static_cast<uint8_t>(windowStart + line);
    if (idx >= KEY_PRESET_COUNT) {
      break;
    }
    oled.drawStr(8, static_cast<int16_t>(15 + line * 8), KEY_PRESETS[idx].name);
  }
  drawMenuIndicator(Screen::Key);
}

void drawAirSelectScreen() {
  oled.drawStr(0, 7, "AIR");
  oled.drawStr(8, 15, "CO2");
  oled.drawStr(8, 23, "TVOC");
  oled.drawStr(8, 31, "HCHO");
  drawMenuIndicator(Screen::Air);
}

void drawSettingsScreen() {
  oled.drawStr(0, 7, "SET");

  char row1[18];
  char row2[18];
  char row3[18];
  snprintf(row1, sizeof(row1), "ENT %s", settingAutoEnter ? "ON" : "OFF");
  snprintf(row2, sizeof(row2), "AIR %s",
           settingAirStartMode == PlotMode::Realtime ? "RT" : "HS");
  snprintf(row3, sizeof(row3), "TST ENTER");
  oled.drawStr(8, 15, row1);
  oled.drawStr(8, 23, row2);
  oled.drawStr(8, 31, row3);
  drawMenuIndicator(Screen::Settings);
  if (millis() < statusUntilMs) {
    oled.drawStr(0, 39, statusText);
  } else {
    oled.drawStr(0, 39, "Both=OK 2x=Back");
  }
}

void drawButtonTestScreen() {
  oled.drawStr(0, 7, "TEST");

  char row1[18];
  char row2[18];
  if (buttonTestUpGapValid) {
    snprintf(row1, sizeof(row1), "UP  %lums", buttonTestUpGapMs);
  } else {
    snprintf(row1, sizeof(row1), "UP  --");
  }
  if (buttonTestDownGapValid) {
    snprintf(row2, sizeof(row2), "DN  %lums", buttonTestDownGapMs);
  } else {
    snprintf(row2, sizeof(row2), "DN  --");
  }
  oled.drawStr(0, 17, row1);
  oled.drawStr(0, 25, row2);

  if (comboConfirmTriggered) {
    const unsigned long heldMs = millis() - comboPressStartMs;
    char row3[18];
    snprintf(row3, sizeof(row3), "EXIT %lu/%lu", heldMs, TEST_MODE_EXIT_HOLD_MS);
    oled.drawStr(0, 39, row3);
  } else {
    oled.drawStr(0, 39, "Gap150 Hold1s");
  }
}

void drawEnvSelectScreen() {
  oled.drawStr(0, 7, "ENV");
  oled.drawStr(8, 15, "TEMP");
  oled.drawStr(8, 23, "HUM");
  oled.drawStr(8, 31, "PRES");
  drawMenuIndicator(Screen::Env);
}

void drawEnvViewScreen() {
  const EnvItem item = static_cast<EnvItem>(selectedEnv);

  char valueText[20];
  formatEnvCurrentMetricValue(item, valueText, sizeof(valueText));
  const char* unit = envMetricUnit(item);
  oled.drawStr(0, 7, valueText);
  const int unitW = oled.getStrWidth(unit);
  const int unitX = ((CHART_X + CHART_W) > unitW) ? ((CHART_X + CHART_W) - unitW) : 0;
  oled.drawStr(unitX, 7, unit);

  const GraphScale scale = drawEnvGraph(item, envPlotMode);

  oled.drawVLine(CHART_X, CHART_Y, CHART_H);
  oled.setFont(u8g2_font_3x5im_tr);
  if (scale.valid) {
    const uint16_t midRaw = static_cast<uint16_t>(scale.minVal + ((scale.maxVal - scale.minVal) / 2));
    char yTop[8];
    char yMid[8];
    char yBot[8];
    formatEnvAxisMetricValue(item, scale.maxVal, yTop, sizeof(yTop));
    formatEnvAxisMetricValue(item, midRaw, yMid, sizeof(yMid));
    formatEnvAxisMetricValue(item, scale.minVal, yBot, sizeof(yBot));

    oled.drawStr(0, CHART_Y + 4, yTop);
    oled.drawStr(0, CHART_Y + (CHART_H / 2) + 2, yMid);
    oled.drawStr(0, CHART_Y + CHART_H, yBot);

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
  formatEnvTimeSpanLabel(item, envPlotMode, leftSpan, sizeof(leftSpan));
  oled.drawStr(CHART_X, 39, leftSpan);
  char midSpan[10];
  formatEnvTimeHalfSpanLabel(item, envPlotMode, midSpan, sizeof(midSpan));
  const int midW = oled.getStrWidth(midSpan);
  oled.drawStr(axisMidX - (midW / 2), 39, midSpan);
  const char* rightLabel = "now";
  const int rightW = oled.getStrWidth(rightLabel);
  oled.drawStr(CHART_X + CHART_W - rightW, 39, rightLabel);
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
  if (!switchI2cOwner(I2cOwner::Oled)) {
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
    case Screen::Env:
      drawEnvSelectScreen();
      break;
    case Screen::EnvView:
      drawEnvViewScreen();
      break;
    case Screen::Settings:
      drawSettingsScreen();
      break;
    case Screen::ButtonTest:
      drawButtonTestScreen();
      break;
  }
  oled.sendBuffer();
}

void handleNavigate(int8_t delta) {
  Serial.print("[UI] ");
  Serial.print(delta < 0 ? "up" : "down");
  Serial.print(" @");
  Serial.println(screenName(currentScreen));
  switch (currentScreen) {
    case Screen::Root:
      selectedRoot = wrapSelection(selectedRoot, delta, ROOT_ITEM_COUNT);
      animateMenuIndicator(Screen::Root);
      break;
    case Screen::Key:
      selectedKey = wrapSelection(selectedKey, delta, KEY_PRESET_COUNT);
      animateMenuIndicator(Screen::Key);
      break;
    case Screen::Air:
      selectedAir = wrapSelection(selectedAir, delta, METRIC_COUNT);
      animateMenuIndicator(Screen::Air);
      break;
    case Screen::AirView:
      plotMode = (plotMode == PlotMode::Realtime) ? PlotMode::History : PlotMode::Realtime;
      break;
    case Screen::Env:
      selectedEnv = wrapSelection(selectedEnv, delta, METRIC_COUNT);
      animateMenuIndicator(Screen::Env);
      break;
    case Screen::EnvView:
      envPlotMode = (envPlotMode == PlotMode::Realtime) ? PlotMode::History : PlotMode::Realtime;
      break;
    case Screen::Settings:
      selectedSetting = wrapSelection(selectedSetting, delta, 3);
      animateMenuIndicator(Screen::Settings);
      break;
    case Screen::ButtonTest:
      break;
  }
}

void handleBack() {
  Serial.print("[UI] back @");
  Serial.println(screenName(currentScreen));
  switch (currentScreen) {
    case Screen::Root:
      break;
    case Screen::Key:
    case Screen::Air:
    case Screen::Env:
      currentScreen = Screen::Root;
      setStatus("Back");
      break;
    case Screen::AirView:
      currentScreen = Screen::Air;
      setStatus("Back");
      break;
    case Screen::EnvView:
      currentScreen = Screen::Env;
      setStatus("Back");
      break;
    case Screen::Settings:
      currentScreen = Screen::Root;
      setStatus("Back");
      break;
    case Screen::ButtonTest:
      currentScreen = Screen::Settings;
      setStatus("Back");
      break;
  }
}

void handleConfirm() {
  Serial.print("[UI] confirm @");
  Serial.println(screenName(currentScreen));
  switch (currentScreen) {
    case Screen::Root:
      if (selectedRoot == static_cast<uint8_t>(RootItem::Key)) {
        currentScreen = Screen::Key;
      } else if (selectedRoot == static_cast<uint8_t>(RootItem::Air)) {
        currentScreen = Screen::Air;
      } else if (selectedRoot == static_cast<uint8_t>(RootItem::Env)) {
        currentScreen = Screen::Env;
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
      Serial.println("[UI] AIR VIEW confirm: no BLE send");
      setStatus("AIR no send");
      break;
    }
    case Screen::Env:
      currentScreen = Screen::EnvView;
      envPlotMode = settingAirStartMode;
      pollEnvSensors(true);
      break;
    case Screen::EnvView: {
      Serial.println("[UI] ENV VIEW confirm: no BLE send");
      setStatus("ENV no send");
      break;
    }
    case Screen::Settings:
      if (selectedSetting == static_cast<uint8_t>(SettingItem::AutoEnter)) {
        settingAutoEnter = !settingAutoEnter;
        saveSettingsToNvs();
        setStatus("Saved");
      } else if (selectedSetting == static_cast<uint8_t>(SettingItem::AirStartMode)) {
        settingAirStartMode =
            (settingAirStartMode == PlotMode::Realtime) ? PlotMode::History : PlotMode::Realtime;
        saveSettingsToNvs();
        setStatus("Saved");
      } else {
        resetButtonTestState();
        currentScreen = Screen::ButtonTest;
        setStatus("Test", 800);
      }
      break;
    case Screen::ButtonTest:
      break;
  }
}

void processButtonInput() {
  const ButtonEvent event = pollButtonEvent();
  if (event == ButtonEvent::Up) {
    handleNavigate(-1);
  } else if (event == ButtonEvent::Down) {
    handleNavigate(1);
  } else if (event == ButtonEvent::Back) {
    handleBack();
  } else if (event == ButtonEvent::Confirm) {
    handleConfirm();
  }
}

void setup() {
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  delay(100);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN, ENV_I2C_FREQ);
  currentI2cOwner = I2cOwner::Oled;
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
  loadEnvHistoryFromNvs();
  lastBleConnectedState = bleKeyboard.isConnected();
  if (lastBleConnectedState) {
    bleConnectedSinceMs = millis();
  }

  lastSampleTickMs = millis();
  secondWindowStartMs = lastSampleTickMs;
  tenMinWindowStartMs = lastSampleTickMs;
  initEnvSensors();

  Serial.println("BLE + AIR + OLED menu started.");
  Serial.print("Buttons: up=GPIO");
  Serial.print(BUTTON_UP_PIN);
  Serial.print(", down=GPIO");
  Serial.println(BUTTON_DOWN_PIN);
  Serial.println("Up: previous, Down: next, both: confirm, double-click one: back.");
  Serial.println("AIR/ENV view: top value+unit, chart with time axis.");
  Serial.println("Serial cmd: help | status | env | send <text>");
  Serial.print("ENV hist fmt=");
  Serial.println(static_cast<unsigned>(ENV_HIST_FMT_VER));
  Serial.print("ENV I2C: SDA=");
  Serial.print(ENV_I2C_SDA_PIN);
  Serial.print(", SCL=");
  Serial.println(ENV_I2C_SCL_PIN);
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
  processButtonInput();
  pollAirSensor();
  pollEnvSensors();
  processButtonInput();
  processSampling();
  reportAirDataPeriodically();
  reportEnvDataPeriodically();
  reportBleConnectionChange();
  reportHeartbeat();
  processSerialCommands();
  processButtonInput();
  refreshDisplay();
  delay(1);
}
