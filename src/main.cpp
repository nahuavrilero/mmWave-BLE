#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
  #include <esp_sleep.h>
  #include <driver/gpio.h>
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
// IDENTIDAD DEL NODO
// Cambiá estas 3 líneas para cada sensor
// =====================================================
static const char* DEVICE_ID   = "s1";                // topic base: mmwave/s1/#
static const char* UNIQUE_BASE = "mmwave_s1";         // unique_id en Home Assistant
static const char* DEVICE_NAME = "mmWave S1";

static const char* DEVICE_MODEL = "mmWave Presence Node";
static const char* DEVICE_MFR   = "Nahu Industries";

// =====================================================
// WIFI / MQTT
// =====================================================
static const char* WIFI_SSID = "Fibra Nahuel";
static const char* WIFI_PASS = "micaela1994";

static const char* MQTT_HOST = "192.168.1.40";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_USER = "mqtt";
static const char* MQTT_PASS = "mqtt";

// =====================================================
// TOPICS BASE
// =====================================================
static const char* MQTT_ROOT = "mmwave";
static const char* HA_DISCOVERY_ROOT = "homeassistant";

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
static const uint32_t UART_SILENCE_TIMEOUT_MS = 1500;

// =====================================================
// ESTADO
// =====================================================
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

bool rawPresence = false;      // lectura cruda: UART en ALWAYS_ON, pin OUT en LIGHT_SLEEP
bool presence = false;         // presencia filtrada / útil
bool lastPublishedRaw = false;
bool lastPublishedPresence = false;

uint32_t lastRawOnMs = 0;
uint32_t lastRawOffMs = 0;
uint32_t lastChangeMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastUartPresenceMs = 0;

uint16_t targetDistance = 0;
uint16_t gateEnergy[16] = {0};

uint8_t frameBuf[64];
size_t framePos = 0;

// =====================================================
// FORWARD DECLARATIONS
// =====================================================
void connectWiFi();
void connectMQTT();
void ensureConnections();

void sendHexData(const char* hexString);
void readMmwaveUart();
bool readOutPresencePin();

void setRawPresence(bool newValue, const char* reason);
void updatePresenceLogic();
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
// HELPERS TOPICS
// =====================================================
String topicBase() {
  return String(MQTT_ROOT) + "/" + DEVICE_ID;
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
  return String(HA_DISCOVERY_ROOT) + "/binary_sensor/" + UNIQUE_BASE + "_presence/config";
}

String haConfigTopicPresenceRaw() {
  return String(HA_DISCOVERY_ROOT) + "/binary_sensor/" + UNIQUE_BASE + "_presence_raw/config";
}

String haConfigTopicDistance() {
  return String(HA_DISCOVERY_ROOT) + "/sensor/" + UNIQUE_BASE + "_distance/config";
}

String mqttClientId() {
  return String(UNIQUE_BASE) + "_client";
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
      "\"identifiers\":[\"" + String(UNIQUE_BASE) + "\"],"
      "\"name\":\"" + String(DEVICE_NAME) + "\" ,"
      "\"model\":\"" + String(DEVICE_MODEL) + "\" ,"
      "\"manufacturer\":\"" + String(DEVICE_MFR) + "\""
      "}";

  String availabilityBlock =
      "\"availability_topic\":\"" + topicStatus() + "\" ,"
      "\"payload_available\":\"online\" ,"
      "\"payload_not_available\":\"offline\" ,";

  String payloadPresence =
      "{"
      "\"name\":\"" + String(DEVICE_NAME) + " Presence\" ,"
      "\"unique_id\":\"" + String(UNIQUE_BASE) + "_presence\" ,"
      "\"state_topic\":\"" + topicPresence() + "\" ,"
      "\"payload_on\":\"ON\" ,"
      "\"payload_off\":\"OFF\" ,"
      "\"device_class\":\"occupancy\" ,"
      + availabilityBlock +
      deviceBlock +
      "}";

  String payloadPresenceRaw =
      "{"
      "\"name\":\"" + String(DEVICE_NAME) + " Presence Raw\" ,"
      "\"unique_id\":\"" + String(UNIQUE_BASE) + "_presence_raw\" ,"
      "\"state_topic\":\"" + topicPresenceRaw() + "\" ,"
      "\"payload_on\":\"ON\" ,"
      "\"payload_off\":\"OFF\" ,"
      "\"device_class\":\"motion\" ,"
      + availabilityBlock +
      deviceBlock +
      "}";

  String payloadDistance =
      "{"
      "\"name\":\"" + String(DEVICE_NAME) + " Distance\" ,"
      "\"unique_id\":\"" + String(UNIQUE_BASE) + "_distance\" ,"
      "\"state_topic\":\"" + topicDistance() + "\" ,"
      "\"unit_of_measurement\":\"cm\" ,"
      "\"state_class\":\"measurement\" ,"
      + availabilityBlock +
      deviceBlock +
      "}";

  bool ok1 = mqttPublishRetained(haConfigTopicPresence(), payloadPresence);
  bool ok2 = mqttPublishRetained(haConfigTopicPresenceRaw(), payloadPresenceRaw);
  bool ok3 = mqttPublishRetained(haConfigTopicDistance(), payloadDistance);
  Serial.printf("[HA] discovery presence=%s raw=%s distance=%s", ok1 ? "OK" : "FAIL", ok2 ? "OK" : "FAIL", ok3 ? "OK" : "FAIL");
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(8, OUTPUT);

#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
  pinMode(SENSOR_OUT_PIN, INPUT);
#endif

  Serial.println();
  Serial.println("=== mmWave MQTT node v2 ===");
  Serial.printf("Device: %s | unique: %s\n", DEVICE_ID, UNIQUE_BASE);

#if NODE_MODE == NODE_MODE_ALWAYS_ON
  Serial.println("Mode: ALWAYS_ON (usa UART para presencia)");
#elif NODE_MODE == NODE_MODE_LIGHT_SLEEP
  Serial.println("Mode: LIGHT_SLEEP (usa pin OUT para wake/presencia cruda)");
#endif

  mmwaveSerial.begin(MMWAVE_BAUD, SERIAL_8N1, MMWAVE_RX_PIN, MMWAVE_TX_PIN);
  delay(100);

  sendHexData(REPORT_MODE_CMD);
  Serial.println("REPORT_MODE_CMD sent");

#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
  rawPresence = readOutPresencePin();
#else
  rawPresence = false;
#endif
  presence = rawPresence;

  uint32_t now = millis();
  lastChangeMs = now;
  if (rawPresence) {
    lastRawOnMs = now;
    lastUartPresenceMs = now;
  } else {
    lastRawOffMs = now;
  }

  connectWiFi();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(1024);
  connectMQTT();

  publishState(true);
  publishDebug("boot");

#if NODE_MODE == NODE_MODE_LIGHT_SLEEP
  configureWakeup();
#endif
}

// =====================================================
// LOOP
// =====================================================
void loop() {
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
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 200) {
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

  Serial.print("Connecting MQTT...");
  while (!mqttClient.connected()) {
    bool ok;

    if (strlen(MQTT_USER) > 0) {
      ok = mqttClient.connect(
        mqttClientId().c_str(),
        MQTT_USER,
        MQTT_PASS,
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
      Serial.print(mqttClient.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}

void ensureConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
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
      uint16_t payloadLen = ((uint16_t)frameBuf[5] << 8) | frameBuf[4]; // little endian

      if (payloadLen != 35) {
        if (framePos > 10) {
          Serial.printf("Len raro LE=%u", payloadLen);
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

        targetDistance = ((uint16_t)frameBuf[8] << 8) | frameBuf[7]; // little endian

        for (int i = 0; i < 16; i++) {
          size_t idx = 9 + (i * 2);
          gateEnergy[i] = ((uint16_t)frameBuf[idx + 1] << 8) | frameBuf[idx]; // little endian
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
// LÓGICA DE PRESENCIA
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

void updatePresenceLogic() {
  uint32_t now = millis();

  if (rawPresence) {
    presence = true;
    digitalWrite(8, HIGH);
    return;
  }

  if (PRESENCE_OFF_HOLD_MS == 0) {
    presence = false;
    digitalWrite(8, LOW);
    return;
  }

  if ((now - lastRawOffMs) >= PRESENCE_OFF_HOLD_MS) {
    presence = false;
    digitalWrite(8, LOW);
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
