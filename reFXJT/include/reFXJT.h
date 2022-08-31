/* 
   RU: Модуль для получения данных c флюгера RS-FXJT-N01 по шине RX485 из ESP32
   EN: Module for receiving data from RS-FXJT-N01 wind via RX485 bus from ESP32
   --------------------------------------------------------------------------------
   (с) 2022 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_FXJT_H__
#define __RE_FXJT_H__

#include <stdbool.h>
#include <stdint.h>
#include "mbcontroller.h"
#include "reSensor.h"
#include "reEsp32.h"

#ifdef __cplusplus
extern "C" {
#endif

class rDirectionItem: public rSensorItem {
  public:
    rDirectionItem(rSensor *sensor, const char* itemName,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString 
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , const char* formatTimestamp
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , const char* formatTimestampValue, const char* formatStringTimeValue
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
      );
    char* asString(const char* format, const value_t value, bool nan_brackets) override;
};

class FXJT485 : public rSensorX1 {
  public:
    FXJT485(uint8_t eventId);
    
    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      uart_port_t uartPort, int16_t uartRx, int16_t uartTx, 
      // temperature filter
      const sensor_filter_t filterMode = SENSOR_FILTER_RAW, const uint16_t filterSize = 0,
      // limits
      const uint32_t minReadInterval = 0, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      uart_port_t uartPort, int16_t uartRx, int16_t uartTx, 
      // temperature filter
      rSensorItem* item,
      // limits
      const uint32_t minReadInterval = 0, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    sensor_status_t sensorReset() override;
  protected:
    void createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
    sensor_status_t readRawData() override;
  private:
    uart_port_t    _uartPort;
    int16_t        _uartRx;
    int16_t        _uartTx;
    void*          _modbus = nullptr;

    esp_err_t initModbus();
    uint16_t calcCRC16(uint8_t *buf, uint16_t len);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_FXJT_H__