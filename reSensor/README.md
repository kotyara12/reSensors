# rSensor

Base class for various temperature and humidity sensors. Features: built-in filters (mean and median), extremums and JSON generation for posting to MQTT

## Dependencies:
  - https://github.com/kotyara12/rLog
  - https://github.com/kotyara12/rStrings
  - https://github.com/kotyara12/reEsp32

### Notes:
  - libraries starting with the <b>re</b> prefix are only suitable for ESP32 and ESP-IDF
  - libraries starting with the <b>ra</b> prefix are only suitable for ARDUINO compatible code
  - libraries starting with the <b>r</b> prefix can be used in both cases (in ESP-IDF and in ARDUINO)