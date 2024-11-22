/* 
   RU: Модуль для получения данных с датчиков DHTxx из ESP32
   EN: Module for receiving data from sensors DHTxx from ESP32
   --------------------------------------------------------------------------------
   (с) 2021-2024 Разживин Александр | Razzhivin Alexander
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
  DHT_DHT22         = 3,  // DHT22 / AM2302
  DHT_AM2320        = 4,  // AM2320
  DHT_MW33          = 5,  // MW33
} DHTxx_TYPE;

#ifdef __cplusplus
extern "C" {
#endif

class DHTxx : public rSensorHT {
  public:
    DHTxx(uint8_t eventId,
      DHTxx_TYPE sensorType, const gpio_num_t gpioNum, const bool gpioPullup, const gpio_num_t gpioReset, const uint8_t levelReset,
      const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
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

