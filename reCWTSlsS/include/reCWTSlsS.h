/* 
   RU: Драйвер датчика CWT-SLS-S (подключение через RS485 Modbus RTU) для ESP32
   EN: Sensor driver CWT-SLS-S (connection via RS485 Modbus RTU) for ESP32
   --------------------------------------------------------------------------------
   (с) 2024 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_CWTSLSS_H__
#define __RE_CWTSLSS_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>

#ifdef __cplusplus
extern "C" {
#endif

class reCWTSlsS : public rSensor {
  public:
    reCWTSlsS(uint8_t eventId, 
      void* modbus, const uint8_t address,
      const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    void setSensorItems(rSensorItem* itemTemperature, rSensorItem* itemHumidity, rSensorItem* itemIlluminance);

    sensor_status_t sensorReset() override;

    sensor_value_t getHumidity(const bool readSensor);
    sensor_value_t getTemperature(const bool readSensor);
    sensor_value_t getIlluminance(const bool readSensor);
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

#endif // __RE_CWTSLSS_H__
