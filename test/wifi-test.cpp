#include <Arduino.h>
#include <WiFi.h>

const char* ssid = "Fibra Nahuel";
const char* pass = "micaela1994";

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);
  delay(1000);

  Serial.print("RSSI scan previo: ");
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == ssid) {
      Serial.println(WiFi.RSSI(i));
    }
  }

  Serial.println("Conectando...");
  WiFi.begin(ssid, pass);

  for (int i = 0; i < 40; i++) {
    delay(500);
    Serial.printf("Pin 0: %i\n",digitalRead(0));
    Serial.printf("Pin 1: %i\n",digitalRead(1));
    Serial.printf("status=%d\n", WiFi.status());
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      return;
    }
  }

  Serial.println("No conecto.");
}

void loop() {}