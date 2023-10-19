/* 
   RU: Модуль для получения данных с датчиков BH1750 из ESP32
   EN: Module for receiving data from sensors BH1750 from ESP32
   --------------------------------------------------------------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_BH1750_H__
#define __RE_BH1750_H__

#include <stdbool.h>
#include <stdint.h>
#include <driver/gpio.h>
#include "reI2C.h"
#include "reSensor.h"

#define BH1750_ADDR_LO 0x23   // I2C address when ADDR pin floating/low
#define BH1750_ADDR_HI 0x5C   // I2C address when ADDR pin high

/**
 * Measurement mode
 */
typedef enum {
  BH1750_MODE_ONE_TIME = 0,         // One time measurement
  BH1750_MODE_CONTINUOUS            // Continuous measurement
} bh1750_mode_t;

/**
 * Measurement resolution
 */
typedef enum {
  BH1750_RES_LOW = 0,               // 4 lx resolution, measurement time is usually 16 ms
  BH1750_RES_HIGH,                  // 1 lx resolution, measurement time is usually 120 ms
  BH1750_RES_HIGH2                  // 0.5 lx resolution, measurement time is usually 120 ms
} bh1750_resolution_t;

#define BH1750_TIME_RES_LOW   24    // Wait to complete L-resolution mode measurement.( max. 24ms. )
#define BH1750_TIME_RES_HIGH  180   // Wait to complete H-resolution mode measurement.( max. 180ms. )

#ifdef __cplusplus
extern "C" {
#endif

class BH1750 : public rSensorX1 {
  public:
    BH1750(uint8_t eventId);
    
    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, const bh1750_mode_t mode = BH1750_MODE_ONE_TIME, const bh1750_resolution_t resolution = BH1750_RES_HIGH,
      // illumination filter
      const sensor_filter_t filterMode = SENSOR_FILTER_RAW, const uint16_t filterSize = 0,
      // limits
      const uint32_t minReadInterval = 100, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, const bh1750_mode_t mode = BH1750_MODE_ONE_TIME, const bh1750_resolution_t resolution = BH1750_RES_HIGH,
      // illumination filter
      rSensorItem* item = nullptr,
      // limits
      const uint32_t minReadInterval = 100, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    sensor_status_t sensorReset() override;

    esp_err_t powerOn();
    esp_err_t powerOff();
    esp_err_t setup(bh1750_mode_t mode, bh1750_resolution_t resolution);
    esp_err_t setMeasurementTime(uint8_t time);
  protected:
    void createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
    sensor_status_t readRawData() override;
  private:
    i2c_port_t               _I2C_num;
    uint8_t                  _I2C_address;
    uint8_t                  _meas_time = 0;
    bh1750_mode_t            _mode = BH1750_MODE_ONE_TIME;
    bh1750_resolution_t      _resolution = BH1750_RES_HIGH;

    esp_err_t sendCommand(uint8_t cmd);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_BH1750_H__
