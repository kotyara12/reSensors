/* 
   Драйвер для получения данных с датчиков ASAIR AHT10/AHT15/AHT20 из ESP32
   Основан на библиотеке от enjoyneering79 (https://github.com/enjoyneering/)
   --------------------------
   (с) 2021-2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_AHT1x_H__
#define __RE_AHT1x_H__

#include <stdint.h>
#include <reSensor.h>
#include "driver/i2c.h"

#define AHT10_ADDRESS_0X38         0x38  // chip I2C address no.1 for AHT10/AHT15/AHT20, address pin connected to GND
#define AHT10_ADDRESS_0X39         0x39  // chip I2C address no.2 for AHT10 only, address pin connected to Vcc

typedef enum {
  AHT1X_SENSOR = 0x00,
  AHT2X_SENSOR = 0x02
} ASAIR_I2C_SENSOR;

typedef enum {
  AHT1x_MODE_NORMAL  = 0x00,
  AHT1x_MODE_CYCLED  = 0x20,
  AHT1x_MODE_COMMAND = 0x40 
} AHT1x_MODE;

#ifdef __cplusplus
extern "C" {
#endif

class AHT1x : public rSensorHT {
  public:
    AHT1x(uint8_t eventId);
    
    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
      // hardware properties
      ASAIR_I2C_SENSOR sensorType, const i2c_port_t numI2C, const uint8_t addrI2C, const AHT1x_MODE sensorMode,
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
      ASAIR_I2C_SENSOR sensorType, const i2c_port_t numI2C, const uint8_t addrI2C, const AHT1x_MODE sensorMode,
      // humidity filter
      rSensorItem* item1, 
      // temperature filter
      rSensorItem* item2,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    sensor_status_t sensorReset() override;
    sensor_status_t softReset(const AHT1x_MODE sensorMode);
  protected:
    sensor_status_t readRawData() override;  
  private:
    i2c_port_t       _I2C_num;
    uint8_t          _I2C_addr;
    ASAIR_I2C_SENSOR _sensorType;
    AHT1x_MODE       _sensorMode = AHT1x_MODE_NORMAL;
    uint8_t          _rawCmdBuffer[3] = {0, 0, 0};
    uint8_t          _rawDataBuffer[7] = {0, 0, 0, 0, 0, 0, 0};

    uint8_t readStatus();
    uint8_t waitBusy(uint32_t delay);
    sensor_status_t setMode(const AHT1x_MODE newMode);
    bool checkCRC8();
};

#ifdef __cplusplus
}
#endif

#endif // __RE_AHT1x_H__

