/* 
   RU: Драйвер датчика QDY30A (подключение через RS485 Modbus RTU) для ESP32
   EN: Sensor driver QDY30A (connection via RS485 Modbus RTU) for ESP32
   --------------------------------------------------------------------------------
   (с) 2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_QDY30A_H__
#define __RE_QDY30A_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>

#ifdef __cplusplus
extern "C" {
#endif

class reQDY30A : public rSensorX1 {
  public:
    reQDY30A(uint8_t eventId);

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      void* modbus, const uint8_t address,
      // level filter
      const sensor_filter_t filterMode = SENSOR_FILTER_RAW, const uint16_t filterSize = 0,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      void* modbus, const uint8_t address, 
      // level item
      rSensorItem* item, 
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    sensor_status_t sensorReset() override;
  protected:
    void createSensorItems(const sensor_filter_t filterMode = SENSOR_FILTER_RAW, const uint16_t filterSize = 0);
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
    sensor_status_t readRawData() override;  
  private:
    void*            _modbus = nullptr;
    uint8_t          _address = 1;
    
    esp_err_t callModbusRegister(uint8_t cmd, uint16_t reg, int16_t* value);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_QDY30A_H__
