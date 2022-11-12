/* 
   RU: Модуль для получения данных с датчиков XY-MD02 (SHT20 через RS485) из ESP32
   EN: Module for receiving data from sensors XY-MD02 (SHT20 over RS485) from ESP32
   --------------------------------------------------------------------------------
   (с) 2022 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_MD02_H__
#define __RE_MD02_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>


#ifdef __cplusplus
extern "C" {
#endif

class xyMD02 : public rSensorHT {
  public:
    xyMD02(uint8_t eventId);

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      void* modbus, const uint8_t address,
      // humidity filter
      const sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, const uint16_t filterSize1 = 0, 
      // temperature filter
      const sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, const uint16_t filterSize2 = 0,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      void* modbus, const uint8_t address,
      // humidity filter
      rSensorItem* item1, 
      // temperature filter
      rSensorItem* item2,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    sensor_status_t sensorReset() override;
  protected:
    sensor_status_t readRawData() override;  
  private:
    void*          _modbus = nullptr;
    uint8_t        _address = 1;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_MD02_H__
