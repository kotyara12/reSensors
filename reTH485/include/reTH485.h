/* 
   RU: Универсальный драйвер для датчиков температуры и влажности, работающих по шине RS485 и протоколу ModbusRTU
   EN: Universal driver for temperature and humidity sensors operating via RS485 bus and ModbusRTU protocol
   --------------------------------------------------------------------------------
   (с) 2022-2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_TH485_H__
#define __RE_TH485_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>


#ifdef __cplusplus
extern "C" {
#endif

class reTH485 : public rSensorHT {
  public:
    reTH485(uint8_t eventId);

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      void* modbus, const uint8_t address, const uint8_t read_cmd, const uint16_t reg_temp, const uint16_t reg_humd,
      // humidity filter
      const sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, const uint16_t filterSize1 = 0, 
      // temperature filter
      const sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, const uint16_t filterSize2 = 0,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      void* modbus, const uint8_t address, const uint8_t read_cmd, const uint16_t reg_temp, const uint16_t reg_humd,
      // humidity filter
      rSensorItem* item1, 
      // temperature filter
      rSensorItem* item2,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    // Sensor reset
    sensor_status_t sensorReset() override;

    // Update (read / write) modbus register
    esp_err_t callModbusRegister(uint8_t cmd, uint16_t reg, void* value);
  protected:
    sensor_status_t readRawData() override;  
  private:
    void*          _modbus = nullptr;
    uint8_t        _address = 1;
    uint8_t        _command = 0;
    uint16_t       _reg_temp = 0;
    uint16_t       _reg_humd = 0;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_TH485_H__
