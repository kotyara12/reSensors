/* 
   EN: Library for reading data from Capacitive Soil Moisture Sensor v1.2 or similar
   RU: Библиотека для считывания данных с Capacitive Soil Moisture Sensor v1.2 или аналогичных
   --------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_MOISTURE_H__
#define __RE_MOISTURE_H__

#include <stdint.h>
#include <driver/gpio.h>
#include <reParams.h>
#include <reSensor.h>
#include "project_config.h"

#ifdef __cplusplus
extern "C" {
#endif

class rMoistureItem: public rSensorItem {
  public:
    rMoistureItem(const char* itemName, 
      const value_t levelMin, const value_t levelMax, const type_bounds_t typeBounds, const value_t sizeRange,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString 
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , const char* formatTimestamp
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , const char* formatTimestampValue, const char* formatStringTimeValue
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
      );
    value_t convertValue(const value_t rawValue) override;
  protected:
    void setLevelMin(const value_t newValue);
    void setLevelMax(const value_t newValue);
    void registerItemParameters(paramsGroup_t * group) override;
  private:
    value_t _level_min;
    paramsEntryHandle_t _prm_level_min;
    value_t _level_max;
    paramsEntryHandle_t _prm_level_max;
    type_bounds_t _type;
    paramsEntryHandle_t _prm_type;
    value_t _range;
    paramsEntryHandle_t _prm_range;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_MOISTURE_H__