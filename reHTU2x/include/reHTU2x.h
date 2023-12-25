/* 
   RU: Модуль для получения данных с датчиков HTU2xD/SHT2x/Si7021 из ESP32
       Основан на библиотеке от enjoyneering79 (https://github.com/enjoyneering/HTU21D)
   EN: Module for receiving data from sensors HTU2xD / SHT2x / Si7021 from ESP32
       Based on a library from enjoyneering79 (https://github.com/enjoyneering/HTU21D)
   --------------------------------------------------------------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_HTU2X_H__
#define __RE_HTU2X_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>
#include "driver/i2c.h"

#define HTU2X_ADDRESS               0x40       // chip i2c address
#define HTU2X_ERROR                 0xFF       // returns 255, if communication error is occurred

typedef enum {
  HTU2X_NULL       = 0,                        // could not determine 
  HTU2X_SHT20      = 1,                        // SHT20
  HTU2X_HTU2x      = 2,                        // HTU2x/SHT21
  HTU2X_SI7013     = 3,                        // Si7013
  HTU2X_SI7020     = 4,                        // Si7020
  HTU2X_SI7021     = 5,                        // Si7021
  HTU2X_SI7021FAKE = 6,                        // Si7021 fake: serial number is always 15ffffff, the CRC8 of serial number does not match
  HTU2X_UNKNOWN    = 7                         // something other
} HTU2X_TYPE;

typedef enum : uint8_t {
  HTU2X_RES_RH12_TEMP14 = 0x00,                // resolution, temperature: 14-Bit & humidity: 12-Bit
  HTU2X_RES_RH8_TEMP12  = 0x01,                // resolution, temperature: 12-Bit & humidity: 8-Bit
  HTU2X_RES_RH10_TEMP13 = 0x80,                // resolution, temperature: 13-Bit & humidity: 10-Bit
  HTU2X_RES_RH11_TEMP11 = 0x81                 // resolution, temperature: 11-Bit & humidity: 11-Bit
} HTU2X_RESOLUTION;

#ifdef __cplusplus
extern "C" {
#endif

class HTU2x : public rSensorHT {
  public:
    HTU2x(uint8_t eventId);

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      const i2c_port_t numI2C, const HTU2X_RESOLUTION resolution, bool compensated_humidity,
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
      const i2c_port_t numI2C, const HTU2X_RESOLUTION resolution, bool compensated_humidity,
      // humidity filter
      rSensorItem* item1, 
      // temperature filter
      rSensorItem* item2,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    sensor_status_t sensorReset() override;
    sensor_status_t softReset();
    sensor_status_t setResolution(HTU2X_RESOLUTION sensorResolution);
    sensor_status_t setHeater(const bool heaterMode);
    sensor_status_t setHeaterPower(const uint8_t heaterPower);
    bool batteryStatus(void);
    HTU2X_TYPE getType();
    uint32_t getSerialB();
  protected:
    sensor_status_t readRawData() override;  
  private:
    i2c_port_t       _I2C_num;
    HTU2X_RESOLUTION _resolution;
    bool             _compensated;
    bool             _heater;
    uint32_t         _serialB = 0;
    HTU2X_TYPE       _deviceType = HTU2X_NULL;

    sensor_status_t readDeviceID(void);
    esp_err_t sendCommand(uint8_t command);
    esp_err_t sendU8(uint8_t command, uint8_t data);
    esp_err_t readU8(uint8_t command, const uint32_t usWaitData, uint8_t* value);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_HTU2X_H__
