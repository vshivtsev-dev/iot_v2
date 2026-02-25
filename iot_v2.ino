/*
 * Универсальный сервер датчиков и исполнительных устройств — v2
 * Arduino Uno R4 WiFi + ESP32
 *
 * GET  /help   — образец максимального запроса
 * GET  /status — состояние платформы (uptime, heap, RSSI, IP)
 * POST /       — строгий JSON: датчики и/или функции
 *
 * Архитектура:
 *   - Таблицы диспетчеризации вместо цепочек if/else
 *   - Единый планировщик задач (WiFi watchdog + daily restart)
 *   - Платформенные различия изолированы в 3 функциях-обёртках
 */

#include <ArduinoJson.h>

#if defined(ARDUINO_UNOR4_WIFI)
  #include <WiFiS3.h>
  #include <WiFiServer.h>
  #include <WiFiUdp.h>
  #include <NTPClient.h>
  #include "RTC.h"
  #define PLATFORM_NAME "Arduino Uno R4 WiFi"
#elif defined(ESP32)
  #include <WiFi.h>
  #include <WebServer.h>
  #include <Wire.h>
  #include <time.h>
  #define PLATFORM_NAME "ESP32"
#else
  #error "Поддерживаются только Arduino Uno R4 WiFi и ESP32"
#endif

#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>
#include "secrets.h"

// ============== Конфигурация ==============

const char* wifiSsid = WIFI_SSID;
const char* wifiPass = WIFI_PASS;

#define RESTART_HOUR   3
#define RESTART_MINUTE 33

#define NTP_SERVER         "europe.pool.ntp.org"
#define GMT_OFFSET_SEC     (1 * 3600)
#define DAYLIGHT_OFFSET    (1 * 3600)

// ============== Глобальные объекты ==============

static Adafruit_BME280 bme280;
static Preferences prefs;
static const int PIN_MISSING = -1;
static const char* PREF_NS = "iot";
static const char* PREF_LAST_RESTART_DAY = "lastRestDay";
static unsigned long bootMs = 0;

#if defined(ARDUINO_UNOR4_WIFI)
  static WiFiUDP ntpUdp;
  static NTPClient ntpClient(ntpUdp, NTP_SERVER, GMT_OFFSET_SEC, 0);
  WiFiServer server(80);
#else
  WebServer server(80);
#endif

// ============== 1. Platform abstraction ==============

static void platformRestart() {
  Serial.flush();
  delay(200);
#if defined(ESP32)
  ESP.restart();
#else
  void (*resetFn)(void) = 0;
  resetFn();
#endif
}

static long platformUnixTime() {
#if defined(ESP32)
  time_t t = time(nullptr);
  return (t > 100000) ? (long)t : -1;
#else
  RTCTime t;
  if (!RTC.getTime(t)) return -1;
  return (long)t.getUnixTime();
#endif
}

static int platformFreeHeap() {
#if defined(ESP32)
  return (int)ESP.getFreeHeap();
#else
  return -1;
#endif
}

static String platformTimeStr() {
#if defined(ESP32)
  time_t now = time(nullptr);
  if (now < 100000) return "";
  struct tm* t = localtime(&now);
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
    t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
    t->tm_hour, t->tm_min, t->tm_sec);
  return String(buf);
#else
  RTCTime t;
  if (!RTC.getTime(t)) return "";
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
    t.getYear(), (int)t.getMonth(), t.getDayOfMonth(),
    t.getHour(), t.getMinutes(), t.getSeconds());
  return String(buf);
#endif
}

// ============== 2. Task scheduler ==============

static void tickWifi();
static void tickRestart();

struct Task {
  unsigned long intervalMs;
  unsigned long lastMs;
  void (*fn)();
};

static Task tasks[] = {
  {10000, 0, tickWifi},
  {60000, 0, tickRestart},
};

static void runTasks() {
  unsigned long now = millis();
  for (size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
    if (now - tasks[i].lastMs >= tasks[i].intervalMs) {
      tasks[i].lastMs = now;
      tasks[i].fn();
    }
  }
}

// ============== WiFi watchdog ==============

static int wifiAttempts = 0;
static const int WIFI_MAX_ATTEMPTS = 3;

static void tickWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiAttempts = 0;
    return;
  }
  wifiAttempts++;
  Serial.println("WiFi disconnected, attempt " + String(wifiAttempts) + "/" + String(WIFI_MAX_ATTEMPTS));
  if (wifiAttempts >= WIFI_MAX_ATTEMPTS) {
    Serial.println("Restarting...");
    delay(1000);
    platformRestart();
  }
  WiFi.disconnect();
  delay(500);
  WiFi.begin(wifiSsid, wifiPass);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t < 10000)) delay(500);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Reconnected: " + WiFi.localIP().toString());
    wifiAttempts = 0;
  }
}

// ============== Daily restart ==============

static void tickRestart() {
  long now = platformUnixTime();
  if (now < 0) return;

  unsigned long day = (unsigned long)now / 86400UL;
  unsigned long tod = (unsigned long)now % 86400UL;
  unsigned long restartTod = (unsigned long)(RESTART_HOUR * 3600 + RESTART_MINUTE * 60);
  if (tod < restartTod || tod > restartTod + 120) return;

  if (!prefs.begin(PREF_NS, false)) return;
  unsigned long lastDay = prefs.getULong(PREF_LAST_RESTART_DAY, 0);
  if (day <= lastDay) { prefs.end(); return; }
  prefs.putULong(PREF_LAST_RESTART_DAY, day);
  prefs.end();

  Serial.println(F("Daily restart"));
  platformRestart();
}

// ============== Вспомогательное ==============

static float round1(float v) {
  return roundf(v * 10.0f) / 10.0f;
}

static int pinFrom(JsonObject pins, const char* key) {
  if (pins.isNull()) return PIN_MISSING;
  if (pins[key].is<int>()) return pins[key].as<int>();
  return PIN_MISSING;
}

// ============== 3. Датчики ==============

static void sensorDht(JsonObject req, JsonObject res) {
  JsonObject pins = req["pins"].as<JsonObject>();
  int pin = pinFrom(pins, "DIGITAL");
  if (pin == PIN_MISSING) { res["error"] = "DHT_11: required pins.DIGITAL"; return; }

  int typ = 11;
  if (req["type"].is<const char*>()) {
    String t = req["type"].as<const char*>();
    typ = (t == "DHT22") ? 22 : 11;
  } else if (req["type"].is<int>()) {
    typ = req["type"].as<int>();
  }

  DHT dht(pin, typ == 22 ? DHT22 : DHT11);
  dht.begin();
  delay(50);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) { res["error"] = "Failed to read DHT"; return; }

  JsonObject d = res["DHT_11"].to<JsonObject>();
  d["temperature"] = round1(t);
  d["humidity"]    = round1(h);
}

static void sensorBme280(JsonObject req, JsonObject res) {
  JsonObject pins = req["pins"].as<JsonObject>();
  int sda = pinFrom(pins, "SDA");
  int scl = pinFrom(pins, "SCL");
  if (sda == PIN_MISSING || scl == PIN_MISSING) {
    res["error"] = "BME_280: required pins.SDA and pins.SCL";
    return;
  }
#if defined(ESP32)
  Wire.begin(sda, scl);
#endif
  if (!bme280.begin(0x76)) { res["error"] = "BME280 not available"; return; }

  float t = bme280.readTemperature();
  float h = bme280.readHumidity();
  if (isnan(h) || isnan(t)) { res["error"] = "Failed to read BME280"; return; }

  JsonObject d = res["BME_280"].to<JsonObject>();
  d["temperature"] = round1(t);
  d["humidity"]    = round1(h);
}

static void sensorSoil(JsonObject req, JsonObject res) {
  if (!req["id"].is<int>())              { res["error"] = "SOIL: required id"; return; }
  if (!req["dry"].is<int>() || !req["wet"].is<int>()) {
    res["error"] = "SOIL: required dry and wet"; return;
  }
  JsonObject pins = req["pins"].as<JsonObject>();
  int analogPin = pinFrom(pins, "ANALOG");
  if (analogPin == PIN_MISSING) { res["error"] = "SOIL: required pins.ANALOG"; return; }

  int id  = req["id"].as<int>();
  int dry = req["dry"].as<int>();
  int wet = req["wet"].as<int>();
  int raw = analogRead(analogPin);
  int pct = constrain(map(raw, wet, dry, 100, 0), 0, 100);

  JsonObject soil = res["SOIL"].to<JsonObject>();
  JsonObject s1   = soil[String(id)].to<JsonObject>();
  s1["moisture"]    = pct;
  s1["sensorValue"] = raw;
}

// ============== 3. Таблица датчиков ==============

struct SensorEntry { const char* name; void (*fn)(JsonObject, JsonObject); };

static SensorEntry SENSOR_TABLE[] = {
  {"DHT_11",  sensorDht},
  {"BME_280", sensorBme280},
  {"SOIL",    sensorSoil},
};
static const size_t SENSOR_COUNT = sizeof(SENSOR_TABLE) / sizeof(SENSOR_TABLE[0]);

// ============== 4. Функции ==============

static void funcPump(JsonObject req, JsonObject res) {
  JsonObject pins = req["pins"].as<JsonObject>();
  int pin = pinFrom(pins, "DIGITAL");
  if (pin == PIN_MISSING) { res["error"] = "pump: required pins.DIGITAL"; return; }
  if (!req["duration"].is<int>()) { res["error"] = "pump: required duration"; return; }

  int duration = req["duration"].as<int>();
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);   // active-LOW: убедиться что выкл
  digitalWrite(pin, LOW);    // включить
  delay(duration);
  digitalWrite(pin, HIGH);   // выключить

  res["duration"] = duration;
  res["executed"] = true;
}

// ============== 4. Таблица функций ==============

struct FuncEntry { const char* name; void (*fn)(JsonObject, JsonObject); };

static FuncEntry FUNC_TABLE[] = {
  {"PUMP", funcPump},
};
static const size_t FUNC_COUNT = sizeof(FUNC_TABLE) / sizeof(FUNC_TABLE[0]);

// ============== GET /help ==============

static String getHelpJson() {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  JsonArray sensors = root["sensors"].to<JsonArray>();

  JsonObject bme = sensors.add<JsonObject>();
  bme["name"] = "BME_280";
  JsonObject bmePins = bme["pins"].to<JsonObject>();
  bmePins["SDA"] = 21; bmePins["SCL"] = 22;

  JsonObject dht = sensors.add<JsonObject>();
  dht["name"] = "DHT_11";
  dht["pins"].to<JsonObject>()["DIGITAL"] = 4;

  JsonObject soil = sensors.add<JsonObject>();
  soil["name"] = "SOIL"; soil["id"] = 1; soil["dry"] = 610; soil["wet"] = 220;
  soil["pins"].to<JsonObject>()["ANALOG"] = 0;

  JsonObject pump1 = root["functions"]["PUMP"]["1"].to<JsonObject>();
  pump1["pins"].to<JsonObject>()["DIGITAL"] = 2;
  pump1["duration"] = 400;

  String out; serializeJson(doc, out); return out;
}

// ============== 5. GET /status ==============

static String getStatusJson() {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root["platform"] = PLATFORM_NAME;

  unsigned long upSec = (millis() - bootMs) / 1000UL;
  char upBuf[20];
  snprintf(upBuf, sizeof(upBuf), "%lu:%02lu:%02lu:%02lu",
    upSec / 86400, (upSec % 86400) / 3600, (upSec % 3600) / 60, upSec % 60);
  root["uptime"] = upBuf;

  String t = platformTimeStr();
  if (t.length() > 0) root["time"] = t;

  root["wifiRssi"] = WiFi.RSSI();
  root["ip"]       = WiFi.localIP().toString();
  int heap = platformFreeHeap();
  if (heap >= 0) root["freeHeap"] = heap;
  String out; serializeJson(doc, out); return out;
}

// ============== POST / — разбор и диспетчеризация ==============

static String processCommand(const String& body) {
  JsonDocument doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok)
    return "{\"ok\":false,\"error\":\"JSON parse error\"}";

  if (!doc["sensors"].is<JsonArray>() && !doc["functions"].is<JsonObject>())
    return "{\"ok\":false,\"error\":\"expected sensors (array) and/or functions (object)\"}";

  JsonDocument resDoc;
  JsonObject res = resDoc.to<JsonObject>();
  res["ok"] = true;
  JsonArray sensorsOut = res["sensors"].to<JsonArray>();
  JsonArray funcsOut   = res["functions"].to<JsonArray>();

  // --- Датчики ---
  if (doc["sensors"].is<JsonArray>()) {
    for (JsonVariant s : doc["sensors"].as<JsonArray>()) {
      JsonObject so = sensorsOut.add<JsonObject>();
      if (!s.is<JsonObject>()) { so["error"] = "each sensor must be an object with name and pins"; continue; }
      JsonObject o = s.as<JsonObject>();
      if (!o["name"].is<const char*>() && !o["name"].is<String>()) { so["error"] = "sensor: required name"; continue; }
      if (!o["pins"].is<JsonObject>()) { so["error"] = "sensor: required pins object"; continue; }

      String name = o["name"].as<String>();
      bool found = false;
      for (size_t i = 0; i < SENSOR_COUNT; i++) {
        if (name == SENSOR_TABLE[i].name) {
          SENSOR_TABLE[i].fn(o, so);
          found = true; break;
        }
      }
      if (!found) so["error"] = "unknown sensor: " + name;
    }
  }

  // --- Функции ---
  if (doc["functions"].is<JsonObject>()) {
    JsonObject reqF = doc["functions"].as<JsonObject>();
    JsonObject outF = funcsOut.add<JsonObject>();

    for (size_t i = 0; i < FUNC_COUNT; i++) {
      const char* fname = FUNC_TABLE[i].name;
      if (!reqF[fname].is<JsonObject>()) continue;

      JsonObject funcOut = outF[fname].to<JsonObject>();
      for (JsonPair kv : reqF[fname].as<JsonObject>()) {
        String idStr = kv.key().c_str();
        JsonObject itemReq = kv.value().as<JsonObject>();
        JsonObject itemRes = funcOut[idStr].to<JsonObject>();
        if (!itemReq["pins"].is<JsonObject>()) {
          itemRes["error"] = String(fname) + ": required pins object";
        } else {
          FUNC_TABLE[i].fn(itemReq, itemRes);
        }
      }
    }
  }

  String out; serializeJson(resDoc, out); return out;
}

// ============== HTTP helpers ==============

#if defined(ARDUINO_UNOR4_WIFI)
static void sendJsonResponse(WiFiClient& client, int code, const String& json) {
  client.println("HTTP/1.1 " + String(code) + " OK");
  client.println("Content-Type: application/json; charset=utf-8");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println("Content-Length: " + String(json.length()));
  client.println();
  client.print(json);
}
#else
static void sendJsonResponse(int code, const String& json) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(code, "application/json", json);
}
#endif

// ============== Setup ==============

void setup() {
  bootMs = millis();
  Serial.begin(115200);
  delay(500);
  Serial.println(F("Boot: " PLATFORM_NAME " v2"));

  WiFi.begin(wifiSsid, wifiPass);
  while (WiFi.status() != WL_CONNECTED) { Serial.print('.'); delay(500); }
  while (WiFi.localIP() == IPAddress(0, 0, 0, 0)) { Serial.print('.'); delay(500); }
  Serial.println();
  Serial.println(WiFi.localIP());

#if defined(ESP32)
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, NTP_SERVER);
  for (int i = 0; i < 20; i++) {
    if (time(nullptr) > 100000) break;
    delay(500);
  }
#else
  RTC.begin();
  ntpClient.begin();
  bool ntpSynced = false;
  for (int i = 0; i < 20 && !ntpSynced; i++) {
    if (ntpClient.update()) {
      RTCTime timeToSet((time_t)ntpClient.getEpochTime());
      RTC.setTime(timeToSet);
      ntpSynced = true;
    } else {
      delay(500);
    }
  }
  if (ntpSynced) {
    Serial.println(F("RTC synced from NTP"));
  } else {
    Serial.println(F("RTC: NTP sync failed"));
  }
#endif

#if defined(ESP32)
  server.on("/help", HTTP_GET, []() {
    sendJsonResponse(200, getHelpJson());
  });
  server.on("/status", HTTP_GET, []() {
    sendJsonResponse(200, getStatusJson());
  });
  server.on("/", HTTP_POST, []() {
    String body = server.arg("plain");
    body.trim();
    String json = body.isEmpty() ? "{\"ok\":false,\"error\":\"no data\"}" : processCommand(body);
    sendJsonResponse(200, json);
  });
  server.onNotFound([]() {
    server.send(404, "application/json", "{\"ok\":false,\"error\":\"not found\"}");
  });
#endif

  server.begin();
  Serial.print(F("Server started. GET /help /status, POST /. Daily restart at "));
  Serial.print(RESTART_HOUR); Serial.print(':');
  if (RESTART_MINUTE < 10) Serial.print('0');
  Serial.print(RESTART_MINUTE);
  Serial.println(F(" (NTP Berlin)"));
}

// ============== Loop ==============

#if defined(ARDUINO_UNOR4_WIFI)

void loop() {
  runTasks();
  WiFiClient client = server.available();
  if (!client) return;

  String reqLine = client.readStringUntil('\n');
  reqLine.trim();
  String method, path;
  int sp = reqLine.indexOf(' ');
  if (sp > 0) {
    method = reqLine.substring(0, sp);
    int sp2 = reqLine.indexOf(' ', sp + 1);
    path = sp2 > 0 ? reqLine.substring(sp + 1, sp2) : "";
  }

  int contentLength = 0;
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r" || line.isEmpty()) break;
    if (line.startsWith("Content-Length:") || line.startsWith("content-length:")) {
      contentLength = line.substring(line.indexOf(':') + 1).toInt();
    }
  }

  String body;
  if (contentLength > 0) {
    unsigned long to = millis() + 5000;
    while (client.connected() && (int)body.length() < contentLength && millis() < to) {
      if (client.available()) body += (char)client.read();
    }
    body.trim();
  }

  if (path == "/help" && method == "GET") {
    sendJsonResponse(client, 200, getHelpJson());
  } else if (path == "/status" && method == "GET") {
    sendJsonResponse(client, 200, getStatusJson());
  } else if (path == "/" && method == "POST") {
    String json = body.isEmpty() ? "{\"ok\":false,\"error\":\"no data\"}" : processCommand(body);
    sendJsonResponse(client, 200, json);
  } else {
    sendJsonResponse(client, 404, "{\"ok\":false,\"error\":\"not found\"}");
  }

  client.flush();
  delay(10);
  client.stop();
}

#else

void loop() {
  runTasks();
  server.handleClient();
}

#endif
