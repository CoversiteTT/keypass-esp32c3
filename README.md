# ESP32-C3 BLE Menu Keyboard + UART Air Sensor

This PlatformIO project runs on your ESP32-C3 OLED board and provides:

- BLE HID keyboard output
- UART air sensor parsing (`0x2C 0xE4 ...` frame)
- I2C environment module support (AHT20 + BMP280)
- OLED menu controlled by one touch button

## Controls

- `Single click`: next item
- `Double click`: back
- `Long press (0.6s)`: confirm (fires while holding)

Menu tree:

- `KEY`
  - choose different text presets
  - long press to send selected preset via BLE
- `AIR`
  - choose `CO2`, `TVOC`, `HCHO`
  - long press to open full-screen chart for selected metric
- `ENV`
  - choose `TEMP`, `HUM`, `PRES`
  - long press to open full-screen chart for selected metric
- `SET`
  - `ENT`: auto Enter after BLE text send (`ON/OFF`)
  - `AIR`: default chart mode when entering AIR VIEW (`RT/HS`)
  - single click selects option, long press toggles and saves
- `AIR VIEW` (full-screen single metric)
  - top line: current value + unit (TVOC/HCHO keep 3 decimals)
  - bottom area: trend chart + x-axis (left span / middle half-span / right `now`)
  - left side shows Y-axis numeric labels (max / mid / min), non-CO2 removes leading zero (for example `.12`)
  - x-axis is separated from chart by 1px gap
  - single click: toggle `Realtime` / `History`
  - double click: back to `AIR` selection
  - long press: no BLE send (air data is display-only)
- `ENV VIEW` (full-screen single metric)
  - top line: current value + unit (`TEMP/HUM/PRES`)
  - bottom area: trend chart + x-axis (left span / middle half-span / right `now`)
  - single click: toggle `Realtime` / `History`
  - double click: back to `ENV` selection
  - long press: no BLE send (display-only)

## Data model

- Sampling updates at ~45Hz (`SAMPLE_INTERVAL_MS = 22`).
- Realtime mode span: `1 minute`.
- History mode span: up to `48 hours`.
- If history is shorter than 48h, span is from now to oldest recorded point.
- Three metrics are stored independently: `CO2`, `TVOC`, `HCHO`.
- Three ENV metrics are also stored independently: `TEMP`, `HUM`, `PRES`.

Storage strategy:

- Realtime chart:
  - solid line = per-second average
  - dashed line = per-second peak
- History chart:
  - solid line = per-10-minute average
  - dashed line = per-10-minute peak
- Persistent data:
  - only 10-minute average and peak are saved
  - auto-load from NVS after reboot/power loss
  - settings are also persisted in NVS (`auto enter`, `default air mode`)

## Wiring

### Touch (TTP223)

- `VCC` -> `3V3`
- `GND` -> `GND`
- `OUT` -> `GPIO0`

### Air sensor UART (`G / +5 / A(TX) / B(RX)`)

- Sensor `G` -> ESP `GND`
- Sensor `+5` -> ESP `5V`
- Sensor `A(TX)` -> ESP board pin `RX` (GPIO20)
- Sensor `B(RX)` -> ESP board pin `TX` (GPIO21, optional)

Important:

- Sensor TX is marked 5V UART.
- ESP32-C3 GPIO is not 5V tolerant.
- Add level shifting or a resistor divider on `Sensor TX -> GPIO20`.

### OLED (board built-in)

- I2C SDA = `GPIO5`
- I2C SCL = `GPIO6`
- Driver in code: `U8G2_SSD1306_72X40_ER_F_HW_I2C`

Based on your board screenshot:

- Use `RX/TX` pads for UART sensor.
- Keep `GPIO5/GPIO6` for built-in OLED.

### AHT20 + BMP280 module (new)

- Module `SDA` -> `GPIO9`
- Module `SCL` -> `GPIO7`
- Uses I2C polling in firmware (`2s` interval)
- Serial output includes `T/RH/P` when module data is valid

## UART Protocol

- `9600 8N1`
- 9-byte frame:
  - `B1=0x2C`, `B2=0xE4`
  - `B3/B4=TVOC`
  - `B5/B6=HCHO`
  - `B7/B8=CO2`
  - `B9=uint8(B1+...+B8)`
- Conversion:
  - `TVOC/HCHO = raw * 0.001` (mg/m3)
  - `CO2 = raw` (ppm)

## Build and Upload

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

## Private Preset Config (Do Not Commit)

To keep BLE preset text out of public repo:

1. Copy `include/user_secrets.h.example` to `include/user_secrets.h`.
2. Put your real preset text in `KEY_PRESET_1_TEXT/2/3`.
3. Build normally:

```bash
pio run
```

`include/user_secrets.h` is ignored by git.

## BLE debug via serial

After boot, serial monitor prints BLE status and send logs.
Heartbeat log prints every 1s (`[HB] ...`) to confirm serial is alive.
BLE send now waits for a 1.2s ready window after connection to reduce notify errors.

- `help` -> show command help
- `status` -> print current BLE connection and auto-enter state
- `env` -> print AHT20/BMP280 latest data
- `send <text>` -> manually send text through BLE keyboard

## Main Config

Edit constants in `src/main.cpp`:

- BLE name/manufacturer
- touch timing (`LONG_PRESS_MS`, `DOUBLE_CLICK_MS`)
- UART pins (`AIR_UART_RX_PIN`, `AIR_UART_TX_PIN`)
- I2C pins (`OLED_*` and `ENV_I2C_*`)
- key preset default macros (`KEY_PRESET_1/2/3_*`)
