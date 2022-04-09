# Driver BMP280 sensors for ESP32 and ESP-IDF framework

Driver for BMP280 sensor over BMP280 sensor API: https://github.com/BoschSensortec/BMP280_driver
Bus I2C only.

## Dependencies:
  - https://github.com/kotyara12/rLog
  - https://github.com/kotyara12/rSensor
  - https://github.com/kotyara12/reI2C
  - https://github.com/BoschSensortec/BMP280_driver

### Notes:
  - libraries starting with the <b>re</b> prefix are only suitable for ESP32 and ESP-IDF
  - libraries starting with the <b>ra</b> prefix are only suitable for ARDUINO compatible code
  - libraries starting with the <b>r</b> prefix can be used in both cases (in ESP-IDF and in ARDUINO)