#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
  #include <driver/gpio.h>
  #include <esp_sleep.h>
#endif

#ifndef MMWAVE_RX_PIN
#error "MMWAVE_RX_PIN no definido"
#endif

#ifndef MMWAVE_TX_PIN
#error "MMWAVE_TX_PIN no definido"
#endif

#ifndef PRESENCE_LED_PIN
  #define PRESENCE_LED_PIN 8
#endif

// =====================================================
// MODO DE FUNCIONAMIENTO
// =====================================================
#define NODE_MODE_ALWAYS_ON   0
#define NODE_MODE_LIGHT_SLEEP 1

#ifndef NODE_MODE
#define NODE_MODE NODE_MODE_ALWAYS_ON
#endif

// =====================================================
// IDENTIDAD / CONFIG DEFAULT
// =====================================================
static const char* DEVICE_MODEL = "mmWave Presence Node";
static const char* DEVICE_MFR   = "Nahu Industries";

static const char* MQTT_ROOT = "mmwave";
static const char* HA_DISCOVERY_ROOT = "homeassistant";
static const char* CONFIG_PATH = "/config.json";
static const char* INDEX_PATH = "/index.html";

static const uint8_t LED_PWM_CHANNEL = 0;
static const uint32_t LED_PWM_FREQ = 5000;
static const uint8_t LED_PWM_RESOLUTION = 8;

static const char* DEFAULT_HA_HOST = "192.168.1.51";
static const char* HA_HOST_CANDIDATES[] = {
  "homeassistant.local",
  DEFAULT_HA_HOST
};

struct AppConfig {
  String deviceId = "s1";
  String uniqueBase = "mmwave_s1";
  String deviceName = "mmWave S1 - Dormitorio";
  String wifiSsid = "Fibra Nahuel";
  String wifiPass = "micaela1994";
  String haHost = "";
  String mqttHost = "";
  uint16_t mqttPort = 1883;
  String mqttUser = "mqtt";
  String mqttPass = "mqtt";
  bool usbUartMode = false;
  bool ledActiveLow = false;
  bool ledUsePwm = false;
  uint8_t ledPresenceDuty = 8;
};

AppConfig cfg;
String selectedHaHost;
String selectedMqttHost;

// =====================================================
// SENSOR mmWave por UART
// =====================================================
#define MMWAVE_BAUD 115200
HardwareSerial mmwaveSerial(1);

// Comando heredado de tu prueba anterior: poner modo reporte
static const char* REPORT_MODE_CMD = "FDFCFBFA0800120000000400000004030201";

// =====================================================
// PIN OUT digital del sensor (solo se usa en LIGHT_SLEEP)
// =====================================================
#ifndef SENSOR_OUT_PIN
#define SENSOR_OUT_PIN 4
#endif

static const bool SENSOR_OUT_ACTIVE_HIGH = true;

// =====================================================
// TIMINGS
// =====================================================
static const uint32_t POLL_MS = 50;
static const uint32_t HEARTBEAT_MS = 30000;
static const uint32_t PRESENCE_OFF_HOLD_MS = 1000;
static const uint32_t MIN_AWAKE_AFTER_CHANGE_MS = 1200;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 35000;
static const uint32_t WIFI_AP_RETRY_MS = 60000;

// =====================================================
// ESTADO
// =====================================================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer webServer(80);
WebSocketsServer webSocket(81);

bool rawPresence = false;
bool presence = false;
bool lastPublishedRaw = false;
bool lastPublishedPresence = false;
bool webAppStarted = false;
bool configApStarted = false;
bool presenceLedReady = false;
bool presenceLedPwmAttached = false;
bool ledTargetOn = false;
bool lastLedUsePwm = false;
bool lastLedActiveLow = false;
uint8_t lastLedPresenceDuty = 255;

uint32_t lastRawOnMs = 0;
uint32_t lastRawOffMs = 0;
uint32_t lastChangeMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastUartPresenceMs = 0;
uint32_t lastMqttAttemptMs = 0;
uint32_t lastWifiRetryMs = 0;
uint32_t ledTestUntilMs = 0;
uint32_t ledTestNextToggleMs = 0;
uint32_t ledPreviewUntilMs = 0;
bool ledTestState = false;

uint16_t targetDistance = 0;
uint16_t gateEnergy[16] = {0};

uint8_t frameBuf[64];
size_t framePos = 0;

// =====================================================
// FORWARD DECLARATIONS
// =====================================================
void loadConfig();
void saveConfig();
void selectHomeAssistantHost();
bool canConnectHomeAssistant(const String& host, String& connectHost);
String resolveLocalHost(const String& host);
String resolvedMqttHost();

void connectWiFi();
bool isWifiConnected();
void startConfigAccessPoint();
void printWifiScanForConfiguredSsid();
bool findBestConfiguredWifi(uint8_t* bssid, int32_t& channel, int32_t& rssi, wifi_auth_mode_t& authMode);
void handleWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
void connectMQTT();
void ensureConnections();

void startWebApp();
void handleWebRoot();
void handleWebSave();
void handleWebConfigJson();
void webSocketEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length);
void sendWebState(uint8_t clientNum, const char* message = nullptr);
void broadcastWebState(const char* message = nullptr);
void applyConfigFromJson(JsonObjectConst doc);
String htmlEscape(const String& s);
String checkedAttr(bool value);

void sendHexData(const char* hexString);
void readMmwaveUart();
void processUsbUartBridge();
bool readOutPresencePin();

void setRawPresence(bool newValue, const char* reason);
void updatePresenceLogic();
void setupPresenceLed();
void setPresenceLed(bool on);
void tickPresenceLed();
void tickLedTest();
void tickLedPreview();
uint8_t ledPwmValue(bool on);
void writePresenceLedDigital(bool on);
void publishState(bool force = false);
void publishDebug(const char* msg);
void publishHomeAssistantDiscovery();

bool mqttPublishRetained(const String& topic, const String& payload);
bool mqttPublish(const String& topic, const String& payload);

String topicBase();
String topicStatus();
String topicPresence();
String topicPresenceRaw();
String topicDistance();
String topicDebug();
String haConfigTopicPresence();
String haConfigTopicPresenceRaw();
String haConfigTopicDistance();
String mqttClientId();

#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
void configureWakeup();
void maybeSleep();
#endif

// =====================================================
// CONFIG
// =====================================================
void loadConfig() {
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS no inicio; usando defaults");
    return;
  }

  if (!LittleFS.exists(CONFIG_PATH)) {
    Serial.println("[FS] config.json no existe; creando default");
    saveConfig();
    return;
  }

  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file) {
    Serial.println("[FS] no pude abrir config.json; usando defaults");
    return;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();

  if (err) {
    Serial.printf("[FS] config.json invalido: %s\n", err.c_str());
    return;
  }

  cfg.deviceId = doc["deviceId"] | cfg.deviceId;
  cfg.uniqueBase = doc["uniqueBase"] | cfg.uniqueBase;
  cfg.deviceName = doc["deviceName"] | cfg.deviceName;
  cfg.wifiSsid = doc["wifiSsid"] | cfg.wifiSsid;
  cfg.wifiPass = doc["wifiPass"] | cfg.wifiPass;
  cfg.haHost = doc["haHost"] | cfg.haHost;
  cfg.mqttHost = doc["mqttHost"] | cfg.mqttHost;
  cfg.mqttPort = doc["mqttPort"] | cfg.mqttPort;
  cfg.mqttUser = doc["mqttUser"] | cfg.mqttUser;
  cfg.mqttPass = doc["mqttPass"] | cfg.mqttPass;
  cfg.usbUartMode = doc["usbUartMode"] | cfg.usbUartMode;
  cfg.ledActiveLow = doc["ledActiveLow"] | cfg.ledActiveLow;
  cfg.ledUsePwm = doc["ledUsePwm"] | cfg.ledUsePwm;
  cfg.ledPresenceDuty = doc["ledPresenceDuty"] | cfg.ledPresenceDuty;
}

void saveConfig() {
  JsonDocument doc;
  doc["deviceId"] = cfg.deviceId;
  doc["uniqueBase"] = cfg.uniqueBase;
  doc["deviceName"] = cfg.deviceName;
  doc["wifiSsid"] = cfg.wifiSsid;
  doc["wifiPass"] = cfg.wifiPass;
  doc["haHost"] = cfg.haHost;
  doc["mqttHost"] = cfg.mqttHost;
  doc["mqttPort"] = cfg.mqttPort;
  doc["mqttUser"] = cfg.mqttUser;
  doc["mqttPass"] = cfg.mqttPass;
  doc["usbUartMode"] = cfg.usbUartMode;
  doc["ledActiveLow"] = cfg.ledActiveLow;
  doc["ledUsePwm"] = cfg.ledUsePwm;
  doc["ledPresenceDuty"] = cfg.ledPresenceDuty;

  File file = LittleFS.open(CONFIG_PATH, "w");
  if (!file) {
    Serial.println("[FS] no pude guardar config.json");
    return;
  }

  serializeJsonPretty(doc, file);
  file.close();
  Serial.println("[FS] config.json guardado");
}

void selectHomeAssistantHost() {
  selectedHaHost = cfg.haHost;
  if (selectedHaHost.length() > 0) {
    String connectHost;
    Serial.printf("[HA] probando configurado %s:8123...", selectedHaHost.c_str());
    if (canConnectHomeAssistant(selectedHaHost, connectHost)) {
      selectedHaHost = connectHost;
      Serial.println("OK");
      return;
    }
    Serial.println("FAIL");
  }

  for (const char* host : HA_HOST_CANDIDATES) {
    String connectHost;
    Serial.printf("[HA] probando %s:8123...", host);
    if (canConnectHomeAssistant(host, connectHost)) {
      selectedHaHost = connectHost;
      Serial.println("OK");
      return;
    }
    Serial.println("FAIL");
  }

  selectedHaHost = cfg.haHost.length() > 0 ? cfg.haHost : DEFAULT_HA_HOST;
  Serial.printf("[HA] sin respuesta; fallback=%s\n", selectedHaHost.c_str());
}

bool canConnectHomeAssistant(const String& host, String& connectHost) {
  connectHost = resolveLocalHost(host);

  WiFiClient client;
  bool ok = client.connect(connectHost.c_str(), 8123, 1200);
  client.stop();
  return ok;
}

String resolveLocalHost(const String& host) {
  if (!host.endsWith(".local")) return host;

  static bool mdnsStarted = false;
  if (!mdnsStarted) {
    String mdnsName = cfg.uniqueBase;
    mdnsName.replace("_", "-");
    mdnsStarted = MDNS.begin(mdnsName.c_str());
    if (!mdnsStarted) {
      Serial.print("[mDNS] no inicio responder local, pruebo DNS normal; ");
      return host;
    }
  }

  String shortName = host.substring(0, host.length() - 6);
  IPAddress ip = MDNS.queryHost(shortName.c_str(), 1200);
  if (ip != INADDR_NONE) {
    Serial.printf(" mDNS=%s ", ip.toString().c_str());
    return ip.toString();
  }

  return host;
}

String resolvedMqttHost() {
  if (cfg.mqttHost.length() > 0) return cfg.mqttHost;
  if (selectedHaHost.length() > 0) return selectedHaHost;
  return DEFAULT_HA_HOST;
}

// =====================================================
// HELPERS TOPICS
// =====================================================
String topicBase() {
  return String(MQTT_ROOT) + "/" + cfg.deviceId;
}

String topicStatus() {
  return topicBase() + "/status";
}

String topicPresence() {
  return topicBase() + "/presence";
}

String topicPresenceRaw() {
  return topicBase() + "/presence_raw";
}

String topicDistance() {
  return topicBase() + "/distance";
}

String topicDebug() {
  return topicBase() + "/debug";
}

String haConfigTopicPresence() {
  return String(HA_DISCOVERY_ROOT) + "/binary_sensor/" + cfg.uniqueBase + "_presence/config";
}

String haConfigTopicPresenceRaw() {
  return String(HA_DISCOVERY_ROOT) + "/binary_sensor/" + cfg.uniqueBase + "_presence_raw/config";
}

String haConfigTopicDistance() {
  return String(HA_DISCOVERY_ROOT) + "/sensor/" + cfg.uniqueBase + "_distance/config";
}

String mqttClientId() {
  return cfg.uniqueBase + "_client";
}

// =====================================================
// WEBAPP
// =====================================================
String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (char c : s) {
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else out += c;
  }
  return out;
}

String checkedAttr(bool value) {
  return value ? F(" checked") : F("");
}

void startWebApp() {
  webServer.on("/", HTTP_GET, handleWebRoot);
  webServer.on("/save", HTTP_POST, handleWebSave);
  webServer.on("/config.json", HTTP_GET, handleWebConfigJson);
  webServer.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  webAppStarted = true;
  Serial.print("[WEB] http://");
  Serial.println(configApStarted ? WiFi.softAPIP() : WiFi.localIP());
  Serial.println("[WEB] websocket ws://<ip>:81/");
}

void handleWebRoot() {
  File file = LittleFS.open(INDEX_PATH, "r");
  if (!file) {
    webServer.send(500, "text/plain", "index.html no encontrado en LittleFS");
    return;
  }
  webServer.streamFile(file, "text/html");
  file.close();
}

void handleWebSave() {
  JsonDocument doc;
  doc["deviceId"] = webServer.arg("deviceId");
  doc["uniqueBase"] = webServer.arg("uniqueBase");
  doc["deviceName"] = webServer.arg("deviceName");
  doc["wifiSsid"] = webServer.arg("wifiSsid");
  doc["wifiPass"] = webServer.arg("wifiPass");
  doc["haHost"] = webServer.arg("haHost");
  doc["mqttHost"] = webServer.arg("mqttHost");
  doc["mqttPort"] = constrain(webServer.arg("mqttPort").toInt(), 1, 65535);
  doc["mqttUser"] = webServer.arg("mqttUser");
  doc["mqttPass"] = webServer.arg("mqttPass");
  doc["usbUartMode"] = webServer.hasArg("usbUartMode");
  doc["ledActiveLow"] = webServer.hasArg("ledActiveLow");
  doc["ledUsePwm"] = webServer.hasArg("ledUsePwm");
  doc["ledPresenceDuty"] = constrain(webServer.arg("ledPresenceDuty").toInt(), 0, 255);

  applyConfigFromJson(doc.as<JsonObjectConst>());

  saveConfig();
  selectHomeAssistantHost();
  selectedMqttHost = resolvedMqttHost();
  mqttClient.setServer(selectedMqttHost.c_str(), cfg.mqttPort);
  setupPresenceLed();
  setPresenceLed(presence);

  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleWebConfigJson() {
  File file = LittleFS.open(CONFIG_PATH, "r");
  if (!file) {
    webServer.send(404, "text/plain", "config.json no encontrado");
    return;
  }
  webServer.streamFile(file, "application/json");
  file.close();
}

void webSocketEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    sendWebState(clientNum, "Conectado.");
    return;
  }

  if (type != WStype_TEXT) return;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    sendWebState(clientNum, "JSON invalido.");
    return;
  }

  const char* msgType = doc["type"] | "";

  if (strcmp(msgType, "get") == 0) {
    sendWebState(clientNum);
    return;
  }

  if (strcmp(msgType, "save") == 0) {
    applyConfigFromJson(doc["config"].as<JsonObjectConst>());
    saveConfig();
    selectHomeAssistantHost();
    selectedMqttHost = resolvedMqttHost();
    mqttClient.setServer(selectedMqttHost.c_str(), cfg.mqttPort);
    setupPresenceLed();
    setPresenceLed(presence);
    broadcastWebState("Guardado.");
    return;
  }

  if (strcmp(msgType, "ledTest") == 0) {
    if (doc["config"].is<JsonObjectConst>()) {
      JsonObjectConst formConfig = doc["config"].as<JsonObjectConst>();
      cfg.ledActiveLow = formConfig["ledActiveLow"] | cfg.ledActiveLow;
      cfg.ledUsePwm = formConfig["ledUsePwm"] | cfg.ledUsePwm;
      cfg.ledPresenceDuty = constrain((int)(formConfig["ledPresenceDuty"] | cfg.ledPresenceDuty), 0, 255);
      setupPresenceLed();
      Serial.printf("[LED] test activeLow=%s usePwm=%s duty=%u pwmOn=%u pwmOff=%u\n",
                    cfg.ledActiveLow ? "true" : "false",
                    cfg.ledUsePwm ? "true" : "false",
                    cfg.ledPresenceDuty,
                    ledPwmValue(true),
                    ledPwmValue(false));
    }
    ledTestUntilMs = millis() + 5000;
    ledTestNextToggleMs = 0;
    ledTestState = false;
    tickLedTest();
    broadcastWebState("Probando LED 5s.");
    return;
  }

  if (strcmp(msgType, "ledPreview") == 0) {
    if (doc["config"].is<JsonObjectConst>()) {
      JsonObjectConst formConfig = doc["config"].as<JsonObjectConst>();
      cfg.ledActiveLow = formConfig["ledActiveLow"] | cfg.ledActiveLow;
      cfg.ledUsePwm = formConfig["ledUsePwm"] | cfg.ledUsePwm;
      cfg.ledPresenceDuty = constrain((int)(formConfig["ledPresenceDuty"] | cfg.ledPresenceDuty), 0, 255);
      setupPresenceLed();
      ledPreviewUntilMs = millis() + 1500;
      setPresenceLed(true);
      sendWebState(clientNum, "Preview LED.");
    }
    return;
  }

  sendWebState(clientNum, "Comando desconocido.");
}

void applyConfigFromJson(JsonObjectConst doc) {
  cfg.deviceId = doc["deviceId"] | cfg.deviceId;
  cfg.uniqueBase = doc["uniqueBase"] | cfg.uniqueBase;
  cfg.deviceName = doc["deviceName"] | cfg.deviceName;
  cfg.wifiSsid = doc["wifiSsid"] | cfg.wifiSsid;
  cfg.wifiPass = doc["wifiPass"] | cfg.wifiPass;
  cfg.haHost = doc["haHost"] | cfg.haHost;
  cfg.mqttHost = doc["mqttHost"] | cfg.mqttHost;
  cfg.mqttPort = constrain((int)(doc["mqttPort"] | cfg.mqttPort), 1, 65535);
  cfg.mqttUser = doc["mqttUser"] | cfg.mqttUser;
  cfg.mqttPass = doc["mqttPass"] | cfg.mqttPass;
  cfg.usbUartMode = doc["usbUartMode"] | cfg.usbUartMode;
  cfg.ledActiveLow = doc["ledActiveLow"] | cfg.ledActiveLow;
  cfg.ledUsePwm = doc["ledUsePwm"] | cfg.ledUsePwm;
  cfg.ledPresenceDuty = constrain((int)(doc["ledPresenceDuty"] | cfg.ledPresenceDuty), 0, 255);
}

void sendWebState(uint8_t clientNum, const char* message) {
  JsonDocument doc;
  JsonObject config = doc["config"].to<JsonObject>();
  config["deviceId"] = cfg.deviceId;
  config["uniqueBase"] = cfg.uniqueBase;
  config["deviceName"] = cfg.deviceName;
  config["wifiSsid"] = cfg.wifiSsid;
  config["wifiPass"] = cfg.wifiPass;
  config["haHost"] = cfg.haHost;
  config["mqttHost"] = cfg.mqttHost;
  config["mqttPort"] = cfg.mqttPort;
  config["mqttUser"] = cfg.mqttUser;
  config["mqttPass"] = cfg.mqttPass;
  config["usbUartMode"] = cfg.usbUartMode;
  config["ledActiveLow"] = cfg.ledActiveLow;
  config["ledUsePwm"] = cfg.ledUsePwm;
  config["ledPresenceDuty"] = cfg.ledPresenceDuty;
  doc["rawPresence"] = rawPresence;
  doc["presence"] = presence;
  doc["distance"] = targetDistance;
  doc["selectedHaHost"] = selectedHaHost;
  doc["selectedMqttHost"] = selectedMqttHost;
  doc["mqttConnected"] = mqttClient.connected();
  doc["ip"] = WiFi.localIP().toString();
  doc["ledPwmValue"] = ledPwmValue(ledTestUntilMs > millis() || presence);
  doc["message"] = message ? message : "";

  String payload;
  serializeJson(doc, payload);
  webSocket.sendTXT(clientNum, payload);
}

void broadcastWebState(const char* message) {
  JsonDocument doc;
  JsonObject config = doc["config"].to<JsonObject>();
  config["deviceId"] = cfg.deviceId;
  config["uniqueBase"] = cfg.uniqueBase;
  config["deviceName"] = cfg.deviceName;
  config["wifiSsid"] = cfg.wifiSsid;
  config["wifiPass"] = cfg.wifiPass;
  config["haHost"] = cfg.haHost;
  config["mqttHost"] = cfg.mqttHost;
  config["mqttPort"] = cfg.mqttPort;
  config["mqttUser"] = cfg.mqttUser;
  config["mqttPass"] = cfg.mqttPass;
  config["usbUartMode"] = cfg.usbUartMode;
  config["ledActiveLow"] = cfg.ledActiveLow;
  config["ledUsePwm"] = cfg.ledUsePwm;
  config["ledPresenceDuty"] = cfg.ledPresenceDuty;
  doc["rawPresence"] = rawPresence;
  doc["presence"] = presence;
  doc["distance"] = targetDistance;
  doc["selectedHaHost"] = selectedHaHost;
  doc["selectedMqttHost"] = selectedMqttHost;
  doc["mqttConnected"] = mqttClient.connected();
  doc["ip"] = WiFi.localIP().toString();
  doc["ledPwmValue"] = ledPwmValue(ledTestUntilMs > millis() || presence);
  doc["message"] = message ? message : "";

  String payload;
  serializeJson(doc, payload);
  webSocket.broadcastTXT(payload);
}

// =====================================================
// MQTT HELPERS
// =====================================================
bool mqttPublishRetained(const String& topic, const String& payload) {
  bool ok = mqttClient.publish(topic.c_str(), payload.c_str(), true);
  Serial.printf("[MQTT] retained %s -> %s (%s)\n",
                topic.c_str(), payload.c_str(), ok ? "OK" : "FAIL");
  return ok;
}

bool mqttPublish(const String& topic, const String& payload) {
  bool ok = mqttClient.publish(topic.c_str(), payload.c_str(), false);
  Serial.printf("[MQTT] publish %s -> %s (%s)\n",
                topic.c_str(), payload.c_str(), ok ? "OK" : "FAIL");
  return ok;
}

// =====================================================
// HOME ASSISTANT DISCOVERY
// =====================================================
void publishHomeAssistantDiscovery() {
  String deviceBlock =
      "\"device\":{"
      "\"identifiers\":[\"" + cfg.uniqueBase + "\"],"
      "\"name\":\"" + cfg.deviceName + "\" ,"
      "\"model\":\"" + String(DEVICE_MODEL) + "\" ,"
      "\"manufacturer\":\"" + String(DEVICE_MFR) + "\""
      "}";

  String availabilityBlock =
      "\"availability_topic\":\"" + topicStatus() + "\" ,"
      "\"payload_available\":\"online\" ,"
      "\"payload_not_available\":\"offline\" ,";

  String payloadPresence =
      "{"
      "\"name\":\"" + cfg.deviceName + " Presence\" ,"
      "\"unique_id\":\"" + cfg.uniqueBase + "_presence\" ,"
      "\"state_topic\":\"" + topicPresence() + "\" ,"
      "\"payload_on\":\"ON\" ,"
      "\"payload_off\":\"OFF\" ,"
      "\"device_class\":\"occupancy\" ,"
      + availabilityBlock +
      deviceBlock +
      "}";

  String payloadPresenceRaw =
      "{"
      "\"name\":\"" + cfg.deviceName + " Presence Raw\" ,"
      "\"unique_id\":\"" + cfg.uniqueBase + "_presence_raw\" ,"
      "\"state_topic\":\"" + topicPresenceRaw() + "\" ,"
      "\"payload_on\":\"ON\" ,"
      "\"payload_off\":\"OFF\" ,"
      "\"device_class\":\"motion\" ,"
      + availabilityBlock +
      deviceBlock +
      "}";

  String payloadDistance =
      "{"
      "\"name\":\"" + cfg.deviceName + " Distance\" ,"
      "\"unique_id\":\"" + cfg.uniqueBase + "_distance\" ,"
      "\"state_topic\":\"" + topicDistance() + "\" ,"
      "\"unit_of_measurement\":\"cm\" ,"
      "\"state_class\":\"measurement\" ,"
      + availabilityBlock +
      deviceBlock +
      "}";

  bool ok1 = mqttPublishRetained(haConfigTopicPresence(), payloadPresence);
  bool ok2 = mqttPublishRetained(haConfigTopicPresenceRaw(), payloadPresenceRaw);
  bool ok3 = mqttPublishRetained(haConfigTopicDistance(), payloadDistance);
  Serial.printf("[HA] discovery presence=%s raw=%s distance=%s\n",
                ok1 ? "OK" : "FAIL", ok2 ? "OK" : "FAIL", ok3 ? "OK" : "FAIL");
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  loadConfig();

#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
  pinMode(SENSOR_OUT_PIN, INPUT);
#endif

  Serial.println();
  Serial.println("=== mmWave MQTT node v3 ===");
  Serial.printf("Device: %s | unique: %s\n", cfg.deviceId.c_str(), cfg.uniqueBase.c_str());
  WiFi.onEvent(handleWiFiEvent);

#if NODE_MODE == NODE_MODE_ALWAYS_ON
  Serial.println("Mode: ALWAYS_ON (usa UART para presencia)");
#elif NODE_MODE == NODE_MODE_LIGHT_SLEEP
  Serial.println("Mode: LIGHT_SLEEP (usa pin OUT para wake/presencia cruda)");
#endif

  mmwaveSerial.begin(MMWAVE_BAUD, SERIAL_8N1, MMWAVE_RX_PIN, MMWAVE_TX_PIN);
  delay(100);

  connectWiFi();
  setupPresenceLed();
  setPresenceLed(false);
  startWebApp();

  if (isWifiConnected()) {
    selectHomeAssistantHost();
    selectedMqttHost = resolvedMqttHost();
    mqttClient.setServer(selectedMqttHost.c_str(), cfg.mqttPort);
    mqttClient.setBufferSize(1024);
  } else {
    selectedMqttHost = resolvedMqttHost();
    mqttClient.setBufferSize(1024);
  }

  if (!cfg.usbUartMode) {
    sendHexData(REPORT_MODE_CMD);
    Serial.println("REPORT_MODE_CMD sent");
  } else {
    Serial.println("USB>UART mode activo desde config");
  }

#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
  rawPresence = readOutPresencePin();
#else
  rawPresence = false;
#endif
  presence = rawPresence;
  setPresenceLed(presence);

  uint32_t now = millis();
  lastChangeMs = now;
  if (rawPresence) {
    lastRawOnMs = now;
    lastUartPresenceMs = now;
  } else {
    lastRawOffMs = now;
  }

  if (!cfg.usbUartMode && isWifiConnected()) {
    connectMQTT();
    publishState(true);
    publishDebug("boot");
  }

#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
  configureWakeup();
#endif
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  webServer.handleClient();
  webSocket.loop();
  tickLedTest();
  tickLedPreview();
  tickPresenceLed();

  if (cfg.usbUartMode) {
    processUsbUartBridge();
    delay(1);
    return;
  }

  ensureConnections();

#if NODE_MODE == NODE_MODE_ALWAYS_ON
  readMmwaveUart();
#else
  bool pinRaw = readOutPresencePin();
  if (pinRaw != rawPresence) {
    setRawPresence(pinRaw, pinRaw ? "pin_on" : "pin_off");
  }
#endif

  updatePresenceLogic();
  publishState(false);

  if (millis() - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = millis();
    publishDebug("heartbeat");
  }

#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
  maybeSleep();
#endif

  delay(POLL_MS);
}

// =====================================================
// WIFI / MQTT
// =====================================================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.printf("Connecting WiFi SSID='%s' passLen=%u",
                cfg.wifiSsid.c_str(), (unsigned)cfg.wifiPass.length());
  WiFi.mode(configApStarted ? WIFI_AP_STA : WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);
  delay(500);

  uint8_t bssid[6] = {0};
  int32_t channel = 0;
  int32_t rssi = -127;
  wifi_auth_mode_t authMode = WIFI_AUTH_OPEN;
  if (findBestConfiguredWifi(bssid, channel, rssi, authMode)) {
    Serial.printf(" channel=%ld rssi=%ld auth=%d", (long)channel, (long)rssi, (int)authMode);
    WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str(), channel, bssid, true);
  } else {
    Serial.print(" no-scan-match");
    WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());
  }
  lastWifiRetryMs = millis();

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < (int)(WIFI_CONNECT_TIMEOUT_MS / 250)) {
    if (webAppStarted) {
      webServer.handleClient();
      webSocket.loop();
    }
    delay(250);
    Serial.print('.');
    retries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
    configApStarted = false;
  } else {
    Serial.print("WiFi FAILED, status=");
    Serial.println(WiFi.status());
    printWifiScanForConfiguredSsid();
    startConfigAccessPoint();
  }
}

bool isWifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

void printWifiScanForConfiguredSsid() {
  Serial.println("[WiFi] scan...");
  int found = WiFi.scanNetworks(false, true);
  if (found < 0) {
    Serial.printf("[WiFi] scan fallo=%d\n", found);
    return;
  }

  bool matched = false;
  for (int i = 0; i < found; i++) {
    if (WiFi.SSID(i) != cfg.wifiSsid) continue;
    matched = true;
    Serial.printf("[WiFi] SSID encontrado RSSI=%d canal=%d auth=%d bssid=%s\n",
                  WiFi.RSSI(i), WiFi.channel(i), WiFi.encryptionType(i), WiFi.BSSIDstr(i).c_str());
  }

  if (!matched) {
    Serial.printf("[WiFi] SSID '%s' no aparece en scan (%d redes)\n", cfg.wifiSsid.c_str(), found);
  }
  WiFi.scanDelete();
}

bool findBestConfiguredWifi(uint8_t* bssid, int32_t& channel, int32_t& rssi, wifi_auth_mode_t& authMode) {
  int found = WiFi.scanNetworks(false, true);
  if (found <= 0) return false;

  int best = -1;
  int32_t bestRssi = -128;
  for (int i = 0; i < found; i++) {
    if (WiFi.SSID(i) != cfg.wifiSsid) continue;
    if (WiFi.RSSI(i) <= bestRssi) continue;
    best = i;
    bestRssi = WiFi.RSSI(i);
  }

  if (best < 0) {
    WiFi.scanDelete();
    return false;
  }

  const uint8_t* foundBssid = WiFi.BSSID(best);
  memcpy(bssid, foundBssid, 6);
  channel = WiFi.channel(best);
  rssi = WiFi.RSSI(best);
  authMode = WiFi.encryptionType(best);
  WiFi.scanDelete();
  return true;
}

void startConfigAccessPoint() {
  if (configApStarted) return;

  String apName = cfg.uniqueBase;
  apName.replace("_", "-");
  apName += "-setup";

  WiFi.mode(WIFI_AP_STA);
  bool ok = WiFi.softAP(apName.c_str(), "mmwave1234");
  configApStarted = ok;
  Serial.printf("[WiFi] AP config %s SSID='%s' pass='mmwave1234' IP=%s\n",
                ok ? "OK" : "FAIL", apName.c_str(), WiFi.softAPIP().toString().c_str());
}

void handleWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WiFi] STA connected");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("[WiFi] got IP ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[WiFi] STA disconnected reason=%u\n", info.wifi_sta_disconnected.reason);
      break;
    default:
      break;
  }
}

void connectMQTT() {
  if (!isWifiConnected()) return;
  if (mqttClient.connected()) return;
  if (millis() - lastMqttAttemptMs < 2000) return;
  lastMqttAttemptMs = millis();

  Serial.printf("Connecting MQTT %s:%u...", selectedMqttHost.c_str(), cfg.mqttPort);
  bool ok;

  if (cfg.mqttUser.length() > 0) {
    ok = mqttClient.connect(
      mqttClientId().c_str(),
      cfg.mqttUser.c_str(),
      cfg.mqttPass.c_str(),
      topicStatus().c_str(),
      1,
      true,
      "offline"
    );
  } else {
    ok = mqttClient.connect(mqttClientId().c_str());
  }

  if (ok) {
    Serial.println("OK");
    mqttPublishRetained(topicStatus(), "online");
    publishHomeAssistantDiscovery();
    publishState(true);
    publishDebug("mqtt_connected");
  } else {
    Serial.print("fail rc=");
    Serial.println(mqttClient.state());
  }
}

void ensureConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    if (configApStarted && (millis() - lastWifiRetryMs) < WIFI_AP_RETRY_MS) return;
    connectWiFi();
    if (!isWifiConnected()) return;
    selectHomeAssistantHost();
    selectedMqttHost = resolvedMqttHost();
    mqttClient.setServer(selectedMqttHost.c_str(), cfg.mqttPort);
  }
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();
}

// =====================================================
// SENSOR / UART
// =====================================================
void sendHexData(const char* hexString) {
  if (!hexString) return;

  size_t len = strlen(hexString);
  if (len == 0 || (len % 2 != 0)) {
    Serial.println("Invalid REPORT_MODE_CMD");
    return;
  }

  Serial.print("TX: ");
  for (size_t i = 0; i < len; i += 2) {
    char byteStr[3] = {hexString[i], hexString[i + 1], '\0'};
    uint8_t b = (uint8_t)strtoul(byteStr, nullptr, 16);
    mmwaveSerial.write(b);
    Serial.print(byteStr);
    Serial.print(' ');
  }
  Serial.println();
}

void processUsbUartBridge() {
  while (mmwaveSerial.available()) {
    Serial.write(mmwaveSerial.read());
  }

  while (Serial.available()) {
    mmwaveSerial.write(Serial.read());
  }
}

void readMmwaveUart() {
  static const uint8_t HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
  static const uint8_t FOOTER[4] = {0xF8, 0xF7, 0xF6, 0xF5};

  while (mmwaveSerial.available()) {
    uint8_t b = (uint8_t)mmwaveSerial.read();

    if (framePos < 4) {
      if (b == HEADER[framePos]) {
        frameBuf[framePos++] = b;
      } else {
        framePos = 0;
        if (b == HEADER[0]) {
          frameBuf[framePos++] = b;
        }
      }
      continue;
    }

    if (framePos < sizeof(frameBuf)) {
      frameBuf[framePos++] = b;
    } else {
      framePos = 0;
      continue;
    }

    if (framePos >= 6) {
      uint16_t payloadLen = ((uint16_t)frameBuf[5] << 8) | frameBuf[4];

      if (payloadLen != 35) {
        if (framePos > 10) {
          Serial.printf("Len raro LE=%u\n", payloadLen);
          framePos = 0;
        }
        continue;
      }

      size_t totalFrameLen = 4 + 2 + payloadLen + 4;

      if (framePos == totalFrameLen) {
        bool footerOk = true;
        for (int i = 0; i < 4; i++) {
          if (frameBuf[6 + payloadLen + i] != FOOTER[i]) {
            footerOk = false;
            break;
          }
        }

        if (!footerOk) {
          Serial.println("Footer invalido");
          framePos = 0;
          continue;
        }

        uint8_t presenceByte = frameBuf[6];
        bool detected = (presenceByte == 0x01);

        targetDistance = ((uint16_t)frameBuf[8] << 8) | frameBuf[7];

        for (int i = 0; i < 16; i++) {
          size_t idx = 9 + (i * 2);
          gateEnergy[i] = ((uint16_t)frameBuf[idx + 1] << 8) | frameBuf[idx];
        }

        lastUartPresenceMs = millis();
        setRawPresence(detected, detected ? "uart_on" : "uart_off");

        Serial.printf("Frame OK | presence=%u | distance=%u\n", presenceByte, targetDistance);

        framePos = 0;
      }
    }
  }
}

bool readOutPresencePin() {
  int level = digitalRead(SENSOR_OUT_PIN);
  return SENSOR_OUT_ACTIVE_HIGH ? (level == HIGH) : (level == LOW);
}

// =====================================================
// LOGICA DE PRESENCIA
// =====================================================
void setRawPresence(bool newValue, const char* reason) {
  if (newValue == rawPresence) return;

  rawPresence = newValue;
  lastChangeMs = millis();

  if (rawPresence) {
    lastRawOnMs = lastChangeMs;
    Serial.println("RAW -> ON");
    publishDebug(reason ? reason : "raw_on");
  } else {
    lastRawOffMs = lastChangeMs;
    Serial.println("RAW -> OFF");
    publishDebug(reason ? reason : "raw_off");
  }

  updatePresenceLogic();
  publishState(true);
}

void setupPresenceLed() {
  bool settingsChanged = !presenceLedReady ||
                         lastLedUsePwm != cfg.ledUsePwm ||
                         lastLedActiveLow != cfg.ledActiveLow ||
                         lastLedPresenceDuty != cfg.ledPresenceDuty;

  if (!settingsChanged) return;

  Serial.printf("[LED] pin=%d activeLow=%s usePwm=%s duty=%u\n",
                PRESENCE_LED_PIN,
                cfg.ledActiveLow ? "true" : "false",
                cfg.ledUsePwm ? "true" : "false",
                cfg.ledPresenceDuty);

  if (presenceLedPwmAttached) {
    ledcDetachPin(PRESENCE_LED_PIN);
    presenceLedPwmAttached = false;
  }

  pinMode(PRESENCE_LED_PIN, OUTPUT);

  if (cfg.ledUsePwm) {
    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_RESOLUTION);
    ledcAttachPin(PRESENCE_LED_PIN, LED_PWM_CHANNEL);
    presenceLedPwmAttached = true;
  }

  presenceLedReady = true;
  lastLedUsePwm = cfg.ledUsePwm;
  lastLedActiveLow = cfg.ledActiveLow;
  lastLedPresenceDuty = cfg.ledPresenceDuty;
  setPresenceLed(ledTargetOn);
}

uint8_t ledPwmValue(bool on) {
  uint8_t activeDuty = on ? cfg.ledPresenceDuty : 0;
  return cfg.ledActiveLow ? (255 - activeDuty) : activeDuty;
}

void setPresenceLed(bool on) {
  ledTargetOn = on;
  if (!presenceLedReady) return;

  if (cfg.ledUsePwm) {
    ledcWrite(LED_PWM_CHANNEL, ledPwmValue(on));
  } else {
    tickPresenceLed();
  }
}

void writePresenceLedDigital(bool on) {
  bool levelHigh = cfg.ledActiveLow ? !on : on;
  digitalWrite(PRESENCE_LED_PIN, levelHigh ? HIGH : LOW);
}

void tickPresenceLed() {
  if (!presenceLedReady || cfg.ledUsePwm) return;

  if (!ledTargetOn || cfg.ledPresenceDuty == 0) {
    writePresenceLedDigital(false);
    return;
  }

  if (cfg.ledPresenceDuty >= 255) {
    writePresenceLedDigital(true);
    return;
  }

  static const uint32_t SOFT_PWM_PERIOD_MS = 1000;
  uint32_t phase = millis() % SOFT_PWM_PERIOD_MS;
  uint32_t onMs = ((uint32_t)cfg.ledPresenceDuty * SOFT_PWM_PERIOD_MS) / 255;
  writePresenceLedDigital(phase < onMs);
}

void tickLedTest() {
  if (ledTestUntilMs == 0) return;
  uint32_t now = millis();
  if ((int32_t)(now - ledTestUntilMs) < 0) {
    if (ledTestNextToggleMs == 0 || (int32_t)(now - ledTestNextToggleMs) >= 0) {
      ledTestState = !ledTestState;
      setPresenceLed(ledTestState);
      ledTestNextToggleMs = now + 350;
    }
    return;
  }

  ledTestUntilMs = 0;
  ledTestNextToggleMs = 0;
  setPresenceLed(presence);
  broadcastWebState("Prueba LED terminada.");
}

void tickLedPreview() {
  if (ledPreviewUntilMs == 0 || ledTestUntilMs != 0) return;
  if ((int32_t)(millis() - ledPreviewUntilMs) < 0) return;

  ledPreviewUntilMs = 0;
  setPresenceLed(presence);
}

void updatePresenceLogic() {
  uint32_t now = millis();

  if (rawPresence) {
    presence = true;
    setPresenceLed(true);
    return;
  }

  if (PRESENCE_OFF_HOLD_MS == 0) {
    presence = false;
    setPresenceLed(false);
    return;
  }

  if ((now - lastRawOffMs) >= PRESENCE_OFF_HOLD_MS) {
    presence = false;
    setPresenceLed(false);
  }
}

void publishState(bool force) {
  if (!mqttClient.connected()) return;

  bool anyChange = false;

  if (force || rawPresence != lastPublishedRaw) {
    mqttPublishRetained(topicPresenceRaw(), rawPresence ? "ON" : "OFF");
    lastPublishedRaw = rawPresence;
    anyChange = true;
  }

  if (force || presence != lastPublishedPresence) {
    mqttPublishRetained(topicPresence(), presence ? "ON" : "OFF");
    lastPublishedPresence = presence;
    anyChange = true;
  }

  if (force) {
    mqttPublishRetained(topicDistance(), String(targetDistance));
  }

  if (anyChange) {
    broadcastWebState();
  }
}

void publishDebug(const char* msg) {
  if (!mqttClient.connected()) return;

  String s;
  s.reserve(160);
  s += "raw=";
  s += rawPresence ? "1" : "0";
  s += ";presence=";
  s += presence ? "1" : "0";
  s += ";msg=";
  s += msg ? msg : "-";
  mqttPublish(topicDebug(), s);
}

// =====================================================
// LIGHT SLEEP
// =====================================================
#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
void configureWakeup() {
  gpio_wakeup_enable(
    (gpio_num_t)SENSOR_OUT_PIN,
    SENSOR_OUT_ACTIVE_HIGH ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL
  );
  esp_sleep_enable_gpio_wakeup();
}

void maybeSleep() {
  if (rawPresence) return;
  if (presence) return;
  if ((millis() - lastChangeMs) < MIN_AWAKE_AFTER_CHANGE_MS) return;

  mqttClient.loop();
  delay(20);

  Serial.println("Entering light sleep...");
  configureWakeup();
  delay(20);

  esp_light_sleep_start();

  Serial.println("Woke up from light sleep");

  bool wakeRaw = readOutPresencePin();
  if (wakeRaw != rawPresence) {
    setRawPresence(wakeRaw, wakeRaw ? "wakeup_on" : "wakeup_off");
  }

  ensureConnections();
  publishState(true);
  publishDebug("wakeup");
}
#endif
