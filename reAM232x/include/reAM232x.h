/* 
   RU: Модуль для получения данных с датчиков AM2320/AM2321 по I2C шине из ESP32
   EN: Module for receiving data from sensors AM2320/AM2321 from ESP32
   --------------------------------------------------------------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_AM232X_H__
#define __RE_AM232X_H__

#include <stdint.h>
#include <esp_err.h>
#include "driver/i2c.h"
#include <reSensor.h>

#define AM232x_ADDRESS        0x5C // I2C fixed address

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t model;
  uint8_t  version;
  uint32_t device_id;
} am232x_dev_info_t;

class AM232x : public rSensorHT {
  public:
    AM232x();

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, 
      // hardware properties
      const i2c_port_t numI2C, 
      // humidity filter
      const sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, const uint16_t filterSize1 = 0, 
      // temperature filter
      const sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, const uint16_t filterSize2 = 0,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, 
      // hardware properties
      const i2c_port_t numI2C, 
      // humidity filter
      rSensorItem* item1, 
      // temperature filter
      rSensorItem* item2,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
  protected:
    sensor_status_t readRawData() override;  
  private:
    i2c_port_t        _I2C_num;
    am232x_dev_info_t _devInfo;
    
    bool initHardware(const int numI2C);
    uint16_t CRC16(uint8_t *buffer, uint8_t nbytes);
    sensor_status_t readRegister(uint8_t reg, uint8_t len, uint8_t * buf);
    sensor_status_t readDeviceInfo();
};

#ifdef __cplusplus
}
#endif

#endif // __RE_AM232X_H__
