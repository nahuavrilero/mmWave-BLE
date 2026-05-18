# ESP32 mmWave Presence Sensor

ESP32-based presence sensor using a Waveshare 24 GHz mmWave radar module.

The project is built with PlatformIO and supports both ESP32-S3 Super Mini and ESP32-C3 Super Mini boards. It publishes presence data over MQTT and includes Home Assistant MQTT Discovery support.

> This project was developed with AI assistance. The code was manually reviewed, adapted, and tested for my own use case, but it may still contain bugs, questionable decisions, or small digital demons hiding in the loop.

## Features

- Presence detection using a Waveshare 24 GHz mmWave radar module
- ESP32-S3 Super Mini support
- ESP32-C3 Super Mini support
- MQTT status publishing
- Home Assistant MQTT Discovery
- PlatformIO-based build system

## Hardware

- ESP32-S3 Super Mini or ESP32-C3 Super Mini
- Waveshare 24 GHz mmWave radar module
- USB / 5 V power supply

## Pinout

### ESP32-S3 Super Mini

| Sensor | ESP32-S3 |
|---|---|
| TX | GPIO 33 / RX |
| RX | GPIO 34 / TX |
| VCC | 5V / 3V3* |
| GND | GND |

### ESP32-C3 Super Mini

| Sensor | ESP32-C3 |
|---|---|
| TX | GPIO 6 / RX |
| RX | GPIO 5 / TX |
| VCC | 5V / 3V3* |
| GND | GND |

\*Check the voltage requirements of your specific sensor module.

## MQTT

The device publishes presence status through MQTT and supports Home Assistant MQTT Discovery.

Example topic:

```txt
presence/sensor/status
```

Example payload:

```json
{
  "presence": true
}
```

Adjust WiFi, MQTT, device name, and topic settings in the project configuration before flashing.

## Home Assistant

Home Assistant MQTT Discovery is supported.

Once the device is flashed and connected to the MQTT broker, the presence sensor should appear automatically in Home Assistant, assuming MQTT discovery is enabled.

## Build

Install PlatformIO and build the desired environment:

```bash
pio run -e esp32-s3-supermini
```

or:

```bash
pio run -e esp32-c3-supermini
```

## Upload

```bash
pio run -e esp32-s3-supermini -t upload
```

or:

```bash
pio run -e esp32-c3-supermini -t upload
```

Depending on the board, you may need to hold the BOOT button while flashing.

## Status

Experimental personal project.

It works for my use case, but use it at your own risk. No guarantees, no magic, no enterprise-grade promises, no Kubernetes cluster required.

## AI Disclosure

This project was created with the help of AI tools.

The code and structure were reviewed and adapted manually before publishing, but the project should still be considered experimental.

## License

No license specified yet.
