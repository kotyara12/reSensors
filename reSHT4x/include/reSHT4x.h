/* 
   RU: Модуль для получения данных с датчиков SHT40D/SHT41D/SHT45D из ESP32
   EN: Module for receiving data from sensors SHT40D/SHT41D/SHT45D from ESP32
   --------------------------------------------------------------------------------
   (с) 2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_SHT4X_H__
#define __RE_SHT4X_H__

#include <stdint.h>
#include <esp_err.h>
#include "driver/i2c.h"
#include "reSensor.h"

#define SHT4xA_ADDRESS 0x44
#define SHT4xB_ADDRESS 0x45

typedef enum {
	SHT4x_PRECISION_LOW,
	SHT4x_PRECISION_MEDIUM,
	SHT4x_PRECISION_HIGH
} SHT4x_PRECISION;

typedef enum {
	SHT4x_HEATER_200mW_1s,
	SHT4x_HEATER_200mW_100ms,
	SHT4x_HEATER_110mW_1s,
	SHT4x_HEATER_110mW_100ms,
	SHT4x_HEATER_20mW_1s,
	SHT4x_HEATER_20mW_100ms
} SHT4x_HEATER;


#ifdef __cplusplus
extern "C" {
#endif

class SHT4x : public rSensorHT {
  public:
    SHT4x(uint8_t eventId);

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, const SHT4x_PRECISION mode,
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
      const i2c_port_t numI2C, const uint8_t addrI2C, const SHT4x_PRECISION mode,
      // humidity filter
      rSensorItem* item1, 
      // temperature filter
      rSensorItem* item2,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    // Reset hardware
    sensor_status_t sensorReset() override;
    // Soft reset
    sensor_status_t softReset();
    // Reading serial number
    uint32_t readSerialNumber();
    // Built-in heater
    sensor_status_t activateHeater(const SHT4x_HEATER heater_mode);
  protected:
    sensor_status_t readRawData() override;  
  private:
    i2c_port_t       _I2C_num;
    uint8_t          _I2C_address;
    SHT4x_PRECISION  _mode;
    uint8_t          _bufData[6] = {0};

    esp_err_t sendCommand(uint8_t command);
    esp_err_t readBuffer(uint8_t command, const uint32_t usWaitData, const uint8_t bytes);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_SHT4X_H__
