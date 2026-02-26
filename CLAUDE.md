# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**iot** — HTTP server for sensors and actuators on ESP32. A single `.ino` sketch using WebServer library and NTP synchronization.

## Build and Upload

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32 iot.ino

# Upload
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 iot.ino

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

All code is in one file `iot.ino`, organized into sections:

1. **Platform Helpers** — `platformRestart()`, `platformUnixTime()`, `platformFreeHeap()`, `platformTimeStr()`
2. **Task Scheduler** — periodic tasks via `millis()` (no interrupts/threads)
3. **WiFi Watchdog** — 6 reconnect attempts, then reboot
4. **Daily Restart** — daily reboot at `RESTART_HOUR:RESTART_MINUTE`, deduplication via NVS
5. **Sensor Drivers** — `sensorDht()`, `sensorBme280()`, `sensorSoil()` + dispatch table `SENSOR_TABLE`
6. **Function Drivers** — `funcPump()` + dispatch table `FUNC_TABLE`
7. **HTTP Endpoints** — GET `/help`, GET `/status`, POST `/`
8. **Setup / Loop** — WiFi+NTP initialization, route registration, main loop

**Key pattern — dispatch tables** instead of if/else chains:
```cpp
struct SensorEntry { const char* name; void (*fn)(JsonObject, JsonObject); };
static SensorEntry SENSOR_TABLE[] = { {"DHT_11", sensorDht}, ... };
```

## Configuration

All parameters are compile-time constants at the top of `iot.ino`:
- `wifiSsid` / `wifiPass` — WiFi network
- `RESTART_HOUR` / `RESTART_MINUTE` — daily restart time
- `NTP_SERVER`, `TZ_BERLIN` — NTP server and POSIX timezone string (auto-DST)

## Libraries

- **ArduinoJson** 7.4.2 — JSON
- **DHT sensor library** 1.4.6 — DHT11/DHT22
- **Adafruit BME280 Library** 2.3.0 — BME280

## Implementation Notes

- **Relay is active-LOW**: `HIGH` = off, `LOW` = on (pump `funcPump`)
- **Rounding**: temperature/humidity rounded to 0.1
- **I2C**: `Wire.begin(sda, scl)` called per-request (configurable pins)
- **NTP**: uses `configTzTime(TZ_BERLIN, NTP_SERVER)` with POSIX string for automatic DST
- **Safe GPIO pins for relay**: 4, 13, 32, 33 — start LOW, no peripheral conflicts
- **Avoid for relay**: GPIO2 (WiFi PHY), GPIO18 (SPI CLK), GPIO25/26/27 (DAC)
