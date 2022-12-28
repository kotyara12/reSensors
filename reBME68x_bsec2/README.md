# Driver BME680 and BME688 sensors for ESP32 and ESP-IDF framework

Module for receiving data from BME680 and BME688 sensors via I2C bus from ESP32. 
Bosch BSEC2 API (https://github.com/boschsensortec/Bosch-BSEC2-Library). 
Air quality data are presented as sensor resistance (not converted). 
Bus I2C only.

## Dependencies:
  - https://github.com/kotyara12/rLog
  - https://github.com/kotyara12/rSensor
  - https://github.com/kotyara12/reI2C
  - https://github.com/boschsensortec/Bosch-BSEC2-Library

### Notes:
  - libraries starting with the <b>re</b> prefix are only suitable for ESP32 and ESP-IDF
  - libraries starting with the <b>ra</b> prefix are only suitable for ARDUINO compatible code
  - libraries starting with the <b>r</b> prefix can be used in both cases (in ESP-IDF and in ARDUINO)