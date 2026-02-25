# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**iot_v2** — универсальный HTTP-сервер датчиков и исполнительных механизмов для Arduino. Единый `.ino`-скетч поддерживает два аппаратных таргета через препроцессорные директивы:
- **Arduino Uno R4 WiFi** (`ARDUINO_UNOR4_WIFI`) — встроенный WiFi (WiFiS3), аппаратный RTC
- **ESP32** — WebServer-библиотека, NTP-синхронизация

## Сборка и загрузка

```bash
# Компиляция для ESP32
arduino-cli compile --fqbn esp32:esp32:esp32 iot_v2.ino

# Загрузка на ESP32
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 iot_v2.ino

# Компиляция для Arduino Uno R4 WiFi
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi iot_v2.ino

# Загрузка на Arduino Uno R4 WiFi
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi iot_v2.ino

# Мониторинг серийного порта (115200 baud)
arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200
```

## Проверка HTTP API

```bash
curl http://[IP]/help      # схема API
curl http://[IP]/status    # состояние устройства
curl -X POST http://[IP]/ -H "Content-Type: application/json" -d @request.json
```

## Архитектура

Весь код — один файл `iot_v2.ino` (~536 строк), организованный в секции:

1. **Platform Abstraction** (строки ~68-98) — три функции, изолирующие различия платформ: `platformRestart()`, `platformUnixTime()`, `platformFreeHeap()`
2. **Task Scheduler** (~100-124) — периодические задачи через `millis()` (нет прерываний/потоков)
3. **WiFi Watchdog** (~126-152) — 3 попытки переподключения, затем перезагрузка
4. **Daily Restart** (~154-173) — ежедневная перезагрузка в `RESTART_HOUR:RESTART_MINUTE`, дедупликация через NVS
5. **Sensor Drivers** (~187-255) — `sensorDht()`, `sensorBme280()`, `sensorSoil()` + таблица диспетчеризации `SENSOR_TABLE`
6. **Function Drivers** (~268-294) — `funcPump()` + таблица диспетчеризации `FUNC_TABLE`
7. **HTTP Endpoints** (~296-398) — GET `/help`, GET `/status`, POST `/`
8. **Setup / Loop** (~419-536) — инициализация WiFi+NTP, регистрация роутов, основной цикл

**Ключевой паттерн — таблицы диспетчеризации** вместо цепочек if/else:
```cpp
struct SensorEntry { const char* name; void (*fn)(JsonObject, JsonObject); };
static SensorEntry SENSOR_TABLE[] = { {"DHT_11", sensorDht}, ... };
```

**Разветвление платформ** только в трёх местах: platform-функции, `setup()`, `loop()`.

## Конфигурация

Все параметры — compile-time константы в начале `iot_v2.ino` (~строки 39-49):
- `wifiSsid` / `wifiPass` — WiFi-сеть
- `RESTART_HOUR` / `RESTART_MINUTE` — время ежедневной перезагрузки
- `NTP_SERVER`, `GMT_OFFSET_SEC`, `DAYLIGHT_OFFSET` — NTP и часовой пояс (Berlin/CET)

## Библиотеки

Установлены в `/home/av/Arduino/libraries/`:
- **ArduinoJson** 7.4.2 — JSON
- **DHT sensor library** 1.4.6 — DHT11/DHT22
- **Adafruit BME280 Library** 2.3.0 — BME280
- **NTPClient** 3.2.1 — NTP (только Uno R4)

Board support packages: `~/.arduino15/packages/` (renesas_uno 1.5.2, esp32 3.3.4)

## Важные детали реализации

- **Реле активно-низкое**: `HIGH` = выкл, `LOW` = вкл (насос `funcPump`)
- **Округление**: температура/влажность до 0.1
- **I²C на ESP32**: `Wire.begin(sda, scl)` вызывается per-request (конфигурируемые пины)
- **Uno R4**: HTTP разбирается вручную через `WiFiServer`; ESP32 использует `WebServer` с роутами
