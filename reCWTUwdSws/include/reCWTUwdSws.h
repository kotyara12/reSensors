/* 
   RU: Драйвер метеостанции CWT-UWD RS485 Small Weather Station для ESP32
   EN: Sensor driver CWT-UWD series RS485 Small Weather Station for ESP32
   --------------------------------------------------------------------------------
   (с) 2024 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_CWTUWDSWS_H__
#define __RE_CWTUWDSWS_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>

#ifdef __cplusplus
extern "C" {
#endif

class reCWTUwdSws : public rSensor {
  public:
    reCWTUwdSws(uint8_t eventId, 
      void* modbus, const uint8_t address,
      const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    void setSensorItems(rSensorItem* itemTemperature, rSensorItem* itemHumidity, 
      rSensorItem* itemPressure, 
      rSensorItem* itemIlluminance, 
      rSensorItem* itemWindSpeed, rSensorItem* itemWindStrength, rSensorItem* itemWindDirection, 
      rSensorItem* itemNoise, 
      rSensorItem* itemPM25, rSensorItem* itemPM10, 
      rSensorItem* itemRainfall);

    sensor_status_t sensorReset() override;

    sensor_value_t getTemperature(const bool readSensor);
    sensor_value_t getHumidity(const bool readSensor);
    sensor_value_t getPressure(const bool readSensor);
    sensor_value_t getIlluminance(const bool readSensor);
    sensor_value_t getWindSpeed(const bool readSensor);
    sensor_value_t getWindStrength(const bool readSensor);
    sensor_value_t getWindDirection(const bool readSensor);
    sensor_value_t getNoise(const bool readSensor);
    sensor_value_t getPM25(const bool readSensor);
    sensor_value_t getPM10(const bool readSensor);
    sensor_value_t getRainfall(const bool readSensor);

    sensor_status_t setBaudRate(uint16_t value);
    sensor_status_t setSlaveId(uint16_t value);
    sensor_status_t setWindSpeedZero();
    sensor_status_t setRainfallZero();
  protected:
    sensor_status_t readRawData() override;  

    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
  private:
    void*           _modbus = nullptr;
    uint8_t         _address = 1;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_CWTUWDSWS_H__
