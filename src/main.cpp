#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <WebServer.h>
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

#ifndef BUILTIN_LED
#define BUILTIN_LED 8
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

static const char* HA_HOST_CANDIDATES[] = {
  "homeassistant.local",
  "192.168.1.40",
  "192.168.1.48"
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
  bool ledActiveLow = true;
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

// =====================================================
// ESTADO
// =====================================================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer webServer(80);

bool rawPresence = false;
bool presence = false;
bool lastPublishedRaw = false;
bool lastPublishedPresence = false;

uint32_t lastRawOnMs = 0;
uint32_t lastRawOffMs = 0;
uint32_t lastChangeMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastUartPresenceMs = 0;
uint32_t lastMqttAttemptMs = 0;

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
void connectMQTT();
void ensureConnections();

void startWebApp();
void handleWebRoot();
void handleWebSave();
void handleWebConfigJson();
String htmlEscape(const String& s);
String checkedAttr(bool value);

void sendHexData(const char* hexString);
void readMmwaveUart();
void processUsbUartBridge();
bool readOutPresencePin();

void setRawPresence(bool newValue, const char* reason);
void updatePresenceLogic();
void setPresenceLed(bool on);
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

  selectedHaHost = cfg.haHost.length() > 0 ? cfg.haHost : HA_HOST_CANDIDATES[2];
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
  return HA_HOST_CANDIDATES[2];
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
  Serial.print("[WEB] http://");
  Serial.println(WiFi.localIP());
}

void handleWebRoot() {
  String page;
  page.reserve(6200);
  page += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>mmWave config</title><style>");
  page += F("body{font-family:system-ui,Segoe UI,sans-serif;margin:0;background:#f5f7f8;color:#15202b}");
  page += F("main{max-width:760px;margin:0 auto;padding:20px}.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(220px,1fr))}");
  page += F("label{display:grid;gap:5px;font-size:13px}input{font:inherit;padding:9px;border:1px solid #b8c3cc;border-radius:6px;background:white}");
  page += F("section{margin:14px 0;padding:14px 0;border-top:1px solid #d7dee4}.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}");
  page += F("button{font:inherit;padding:10px 14px;border:0;border-radius:6px;background:#1f7a5c;color:white;cursor:pointer}.muted{color:#5a6872;font-size:13px}");
  page += F("input[type=checkbox]{width:20px;height:20px}.check{display:flex;gap:10px;align-items:center}.check span{font-size:14px}");
  page += F("</style></head><body><main><h1>mmWave config</h1>");
  page += F("<p class='muted'>HA: ");
  page += htmlEscape(selectedHaHost);
  page += F(" | MQTT: ");
  page += htmlEscape(selectedMqttHost);
  page += F(" | IP: ");
  page += WiFi.localIP().toString();
  page += F("</p><form method='post' action='/save'>");

  page += F("<section><h2>Identidad</h2><div class='grid'>");
  page += F("<label>Device ID<input name='deviceId' value='");
  page += htmlEscape(cfg.deviceId);
  page += F("'></label>");
  page += F("<label>Unique base<input name='uniqueBase' value='");
  page += htmlEscape(cfg.uniqueBase);
  page += F("'></label>");
  page += F("<label>Nombre<input name='deviceName' value='");
  page += htmlEscape(cfg.deviceName);
  page += F("'></label>");
  page += F("</div></section>");

  page += F("<section><h2>Red</h2><div class='grid'>");
  page += F("<label>WiFi SSID<input name='wifiSsid' value='");
  page += htmlEscape(cfg.wifiSsid);
  page += F("'></label>");
  page += F("<label>WiFi pass<input name='wifiPass' type='password' value='");
  page += htmlEscape(cfg.wifiPass);
  page += F("'></label>");
  page += F("<label>HA host<input name='haHost' placeholder='auto' value='");
  page += htmlEscape(cfg.haHost);
  page += F("'></label>");
  page += F("<label>MQTT host<input name='mqttHost' placeholder='usa HA detectado' value='");
  page += htmlEscape(cfg.mqttHost);
  page += F("'></label>");
  page += F("<label>MQTT port<input name='mqttPort' type='number' min='1' max='65535' value='");
  page += String(cfg.mqttPort);
  page += F("'></label>");
  page += F("<label>MQTT user<input name='mqttUser' value='");
  page += htmlEscape(cfg.mqttUser);
  page += F("'></label>");
  page += F("<label>MQTT pass<input name='mqttPass' type='password' value='");
  page += htmlEscape(cfg.mqttPass);
  page += F("'></label>");
  page += F("</div></section>");

  page += F("<section><h2>Modos</h2><div class='grid'>");
  page += F("<label class='check'><input type='checkbox' name='usbUartMode'");
  page += checkedAttr(cfg.usbUartMode);
  page += F("><span>USB &gt; UART</span></label>");
  page += F("<label class='check'><input type='checkbox' name='ledActiveLow'");
  page += checkedAttr(cfg.ledActiveLow);
  page += F("><span>LED onboard activo en LOW</span></label>");
  page += F("<label>Brillo LED presencia<input name='ledPresenceDuty' type='number' min='0' max='255' value='");
  page += String(cfg.ledPresenceDuty);
  page += F("'></label>");
  page += F("</div></section>");

  page += F("<div class='row'><button type='submit'>Guardar</button><span class='muted'>Algunos cambios aplican mejor reiniciando. Tranqui, no muerde.</span></div>");
  page += F("</form></main></body></html>");
  webServer.send(200, "text/html", page);
}

void handleWebSave() {
  cfg.deviceId = webServer.arg("deviceId");
  cfg.uniqueBase = webServer.arg("uniqueBase");
  cfg.deviceName = webServer.arg("deviceName");
  cfg.wifiSsid = webServer.arg("wifiSsid");
  cfg.wifiPass = webServer.arg("wifiPass");
  cfg.haHost = webServer.arg("haHost");
  cfg.mqttHost = webServer.arg("mqttHost");
  cfg.mqttPort = constrain(webServer.arg("mqttPort").toInt(), 1, 65535);
  cfg.mqttUser = webServer.arg("mqttUser");
  cfg.mqttPass = webServer.arg("mqttPass");
  cfg.usbUartMode = webServer.hasArg("usbUartMode");
  cfg.ledActiveLow = webServer.hasArg("ledActiveLow");
  cfg.ledPresenceDuty = constrain(webServer.arg("ledPresenceDuty").toInt(), 0, 255);

  saveConfig();
  selectHomeAssistantHost();
  selectedMqttHost = resolvedMqttHost();
  mqttClient.setServer(selectedMqttHost.c_str(), cfg.mqttPort);
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
  setPresenceLed(false);

#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
  pinMode(SENSOR_OUT_PIN, INPUT);
#endif

  Serial.println();
  Serial.println("=== mmWave MQTT node v3 ===");
  Serial.printf("Device: %s | unique: %s\n", cfg.deviceId.c_str(), cfg.uniqueBase.c_str());

#if NODE_MODE == NODE_MODE_ALWAYS_ON
  Serial.println("Mode: ALWAYS_ON (usa UART para presencia)");
#elif NODE_MODE == NODE_MODE_LIGHT_SLEEP
  Serial.println("Mode: LIGHT_SLEEP (usa pin OUT para wake/presencia cruda)");
#endif

  mmwaveSerial.begin(MMWAVE_BAUD, SERIAL_8N1, MMWAVE_RX_PIN, MMWAVE_TX_PIN);
  delay(100);

  connectWiFi();
  selectHomeAssistantHost();
  selectedMqttHost = resolvedMqttHost();
  mqttClient.setServer(selectedMqttHost.c_str(), cfg.mqttPort);
  mqttClient.setBufferSize(1024);
  startWebApp();

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

  if (!cfg.usbUartMode) {
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

  Serial.print("Connecting WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);
  delay(500);
  WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 200) {
    webServer.handleClient();
    delay(250);
    Serial.print('.');
    retries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.print("WiFi FAILED, status=");
    Serial.println(WiFi.status());
  }
}

void connectMQTT() {
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
    connectWiFi();
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

void setPresenceLed(bool on) {
  uint8_t activeDuty = on ? cfg.ledPresenceDuty : 0;
  uint8_t pwmValue = cfg.ledActiveLow ? (255 - activeDuty) : activeDuty;
  analogWrite(BUILTIN_LED, pwmValue);
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

  if (force || rawPresence != lastPublishedRaw) {
    mqttPublishRetained(topicPresenceRaw(), rawPresence ? "ON" : "OFF");
    lastPublishedRaw = rawPresence;
  }

  if (force || presence != lastPublishedPresence) {
    mqttPublishRetained(topicPresence(), presence ? "ON" : "OFF");
    lastPublishedPresence = presence;
  }

  if (force) {
    mqttPublishRetained(topicDistance(), String(targetDistance));
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
