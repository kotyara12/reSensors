# Driver DHTxx for ESP32 and ESP-IDF framework

Sensor driver DHT11, DHT12, DHT21, DHT22 for ESP32-based devices running FreeRTOS. The option with waiting for a change of the signal level in a cycle (as in Adafruit)

## Dependencies:
  - https://github.com/kotyara12/rLog
  - https://github.com/kotyara12/rSensor

### Notes:
  - libraries starting with the <b>re</b> prefix are only suitable for ESP32 and ESP-IDF
  - libraries starting with the <b>ra</b> prefix are only suitable for ARDUINO compatible code
  - libraries starting with the <b>r</b> prefix can be used in both cases (in ESP-IDF and in ARDUINO)