/*
 * ESP32 MQTT client — minimal base
 *
 * Topics:
 *   iot/<DEVICE_ID>/status  — retained heartbeat (uptime, heap, rssi, ip)
 *   iot/<DEVICE_ID>/cmd     — incoming JSON commands (subscribe)
 *
 * Libraries: PubSubClient, ArduinoJson, ArduinoOTA (built-in)
 */

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "secrets.h"

// ============== Configuration ==============

const char* wifiSsid = WIFI_SSID;
const char* wifiPass = WIFI_PASS;

const char* mqttHost = MQTT_HOST;
const int   mqttPort = 1883;
const char* mqttUser = MQTT_USER;
const char* mqttPass = MQTT_PASS;

static char deviceId[16]; // last IP octet

const char* ntpServer = NTP_SERVER;
const char* tzString  = TZ_STRING;

#define STATUS_INTERVAL_MS  60000
#define WIFI_RETRY_MAX      6

// ============== Topics ==============

static char topicStatus[64];
static char topicCmd[64];

static void buildTopics() {
    snprintf(topicStatus, sizeof(topicStatus), "iot/%s/status", deviceId);
    snprintf(topicCmd,    sizeof(topicCmd),    "iot/%s/cmd",    deviceId);
}

// ============== Platform helpers ==============

static unsigned long bootMs = 0;

static void platformRestart() { ESP.restart(); }

static String platformTimeStr() {
    struct tm t;
    if (!getLocalTime(&t)) return "?";
    char buf[20];
    strftime(buf, sizeof(buf), "%F %T", &t);
    return String(buf);
}

// ============== WiFi ==============

static int wifiRetries = 0;

static void wifiReconnect() {
    if (WiFi.status() == WL_CONNECTED) {
        wifiRetries = 0;
        return;
    }
    wifiRetries++;
    WiFi.reconnect();
    if (wifiRetries >= WIFI_RETRY_MAX) {
        platformRestart();
    }
}

// ============== MQTT ==============

static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);

static void mqttPublishStatus();

static void mqttOnMessage(char* topic, byte* payload, unsigned int len) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, len);
    if (err) return;

    // TODO: dispatch commands

}

static void mqttReconnect() {
    if (mqtt.connected()) return;
    if (WiFi.status() != WL_CONNECTED) return;

    char willPayload[] = "{\"online\":false}";
    bool ok = mqtt.connect(deviceId, mqttUser, mqttPass,
                           topicStatus, 0, /*retain=*/true, willPayload);
    if (ok) {
        mqtt.subscribe(topicCmd);
        mqttPublishStatus();
    }
}

static void mqttPublishStatus() {
    if (!mqtt.connected()) return;
    JsonDocument doc;
    doc["online"] = true;
    doc["time"]   = platformTimeStr();
    doc["uptime"] = (millis() - bootMs) / 1000;
    doc["heap"]   = ESP.getFreeHeap();
    doc["rssi"]   = WiFi.RSSI();
    doc["ip"]     = WiFi.localIP().toString();
    char buf[256];
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(topicStatus, buf, /*retain=*/true);
}

// ============== Setup / Loop ==============

void setup() {
    bootMs = millis();

    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid, wifiPass);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500);
    }

    snprintf(deviceId, sizeof(deviceId), "esp32-%d",
             WiFi.localIP()[3]); // last IP octet
    buildTopics();

    configTzTime(tzString, ntpServer);

    ArduinoOTA.setHostname(deviceId);
    ArduinoOTA.begin();

    mqtt.setServer(mqttHost, mqttPort);
    mqtt.setCallback(mqttOnMessage);
    mqtt.setBufferSize(512);
    mqttReconnect();
}

void loop() {
    ArduinoOTA.handle();
    mqtt.loop();

    unsigned long now = millis();

    static unsigned long lastWifi = 0;
    if (now - lastWifi >= 10000) { lastWifi = now; wifiReconnect(); }

    static unsigned long lastMqtt = 0;
    if (now - lastMqtt >= 5000) { lastMqtt = now; mqttReconnect(); }

    static unsigned long lastStatus = 0;
    if (now - lastStatus >= STATUS_INTERVAL_MS) { lastStatus = now; mqttPublishStatus(); }
}
