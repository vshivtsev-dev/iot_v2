# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**iot_v2** — universal HTTP server for sensors and actuators on Arduino. A single `.ino` sketch supports two hardware targets via preprocessor directives:
- **Arduino Uno R4 WiFi** (`ARDUINO_UNOR4_WIFI`) — built-in WiFi (WiFiS3), hardware RTC
- **ESP32** — WebServer library, NTP synchronization

## Build and Upload

```bash
# Compile for ESP32
arduino-cli compile --fqbn esp32:esp32:esp32 iot_v2.ino

# Upload to ESP32
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 iot_v2.ino

# Compile for Arduino Uno R4 WiFi
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi iot_v2.ino

# Upload to Arduino Uno R4 WiFi
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi iot_v2.ino

# Monitor serial port (115200 baud)
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
```

## HTTP API Testing

```bash
curl http://[IP]/help      # API schema
curl http://[IP]/status    # device state
curl -X POST http://[IP]/ -H "Content-Type: application/json" -d @request.json
```

## Architecture

All code is in one file `iot_v2.ino` (~536 lines), organized into sections:

1. **Platform Abstraction** (lines ~68-98) — three functions isolating platform differences: `platformRestart()`, `platformUnixTime()`, `platformFreeHeap()`
2. **Task Scheduler** (~100-124) — periodic tasks via `millis()` (no interrupts/threads)
3. **WiFi Watchdog** (~126-152) — 3 reconnect attempts, then reboot
4. **Daily Restart** (~154-173) — daily reboot at `RESTART_HOUR:RESTART_MINUTE`, deduplication via NVS
5. **Sensor Drivers** (~187-255) — `sensorDht()`, `sensorBme280()`, `sensorSoil()` + dispatch table `SENSOR_TABLE`
6. **Function Drivers** (~268-294) — `funcPump()` + dispatch table `FUNC_TABLE`
7. **HTTP Endpoints** (~296-398) — GET `/help`, GET `/status`, POST `/`
8. **Setup / Loop** (~419-536) — WiFi+NTP initialization, route registration, main loop

**Key pattern — dispatch tables** instead of if/else chains:
```cpp
struct SensorEntry { const char* name; void (*fn)(JsonObject, JsonObject); };
static SensorEntry SENSOR_TABLE[] = { {"DHT_11", sensorDht}, ... };
```

**Platform branching** only in three places: platform functions, `setup()`, `loop()`.

## Configuration

All parameters are compile-time constants at the top of `iot_v2.ino` (~lines 39-49):
- `wifiSsid` / `wifiPass` — WiFi network
- `RESTART_HOUR` / `RESTART_MINUTE` — daily restart time
- `NTP_SERVER`, `GMT_OFFSET_SEC`, `DAYLIGHT_OFFSET` — NTP and timezone (Berlin/CET)

## Libraries

Installed in `/home/av/Arduino/libraries/`:
- **ArduinoJson** 7.4.2 — JSON
- **DHT sensor library** 1.4.6 — DHT11/DHT22
- **Adafruit BME280 Library** 2.3.0 — BME280
- **NTPClient** 3.2.1 — NTP (Uno R4 only)

Board support packages: `~/.arduino15/packages/` (renesas_uno 1.5.2, esp32 3.3.4)

## Implementation Notes

- **Relay is active-LOW**: `HIGH` = off, `LOW` = on (pump `funcPump`)
- **Rounding**: temperature/humidity rounded to 0.1
- **I2C on ESP32**: `Wire.begin(sda, scl)` called per-request (configurable pins)
- **Uno R4**: HTTP parsed manually via `WiFiServer`; ESP32 uses `WebServer` with routes
