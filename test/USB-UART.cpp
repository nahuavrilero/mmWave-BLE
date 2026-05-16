//UART->USB

#include <Arduino.h>

#define RX_PIN 6
#define TX_PIN 5

HardwareSerial SensorSerial(1);

void setup() {
  Serial.begin(115200); // USB
  SensorSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("UART bridge ready");
}

void loop() {
  // Sensor → PC
  while (SensorSerial.available()) {
    Serial.write(SensorSerial.read());
  }

  // PC → Sensor
  while (Serial.available()) {
    SensorSerial.write(Serial.read());
  }
}