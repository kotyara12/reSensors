/* 
   RU: Модуль для получения данных с датчиков DHTxx из ESP32
   EN: Module for receiving data from sensors DHTxx from ESP32
   --------------------------------------------------------------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_DHTXX_H__
#define __RE_DHTXX_H__

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include <driver/gpio.h>
#include <reSensor.h>

typedef enum {
  DHT_DHT11         = 0,  // DHT11
  DHT_DHT12         = 1,  // DHT12
  DHT_DHT21         = 2,  // DHT21 / AM2301
  DHT_DHT22         = 3   // DHT22 / AM2302
} DHTxx_TYPE;

#ifdef __cplusplus
extern "C" {
#endif

class DHTxx : public rSensorHT {
  public:
    DHTxx(uint8_t eventId);
    
    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
      // hardware properties
      DHTxx_TYPE sensorType, const uint8_t gpioNum, const bool gpioPullup, const int8_t gpioReset, const uint8_t levelReset,
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
      DHTxx_TYPE sensorType, const uint8_t gpioNum, const bool gpioPullup, const int8_t gpioReset, const uint8_t levelReset,
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
    DHTxx_TYPE       _sensorType = DHT_DHT22;
    gpio_num_t       _sensorGPIO;
    gpio_num_t       _resetGPIO;
    uint8_t          _resetLevel;
    time_t           _resetTime;
    bool             _gpioPullup;
    uint32_t         _maxCycles;

    uint32_t expectPulse(bool level);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_DHTXX_H__

