/* 
   EN: Voltage measurement at ADC inputs and conversion to real values
   RU: Измерение напряжения на ADC входах и пересчет в реальные значения
   --------------------------
   (с) 2022 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_ADC_H__
#define __RE_ADC_H__

#include <stdint.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include <reParams.h>
#include <reSensor.h>
#include "project_config.h"

#ifdef __cplusplus
extern "C" {
#endif

class reADC1: public rSensorItem {
  public:
    reADC1(rSensor *sensor, const char* itemName, 
      const adc1_channel_t channel, const adc_atten_t atten, const bool cal_enabled, const double coefficient,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString 
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , const char* formatTimestamp
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , const char* formatTimestampValue, const char* formatStringTimeValue
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
      );
    bool initItem() override;
    value_t convertValue(const value_t rawValue) override;
  protected:
    void registerItemParameters(paramsGroup_t * group) override;
    sensor_status_t getRawValue(value_t * rawValue) override;
  private:
    adc1_channel_t _channel;
    adc_atten_t _atten;
    bool _cal_enable = false;
    esp_adc_cal_characteristics_t _chars;
    double _coefficient = 1.0;
};

class reADC2: public rSensorItem {
  public:
    reADC2(rSensor *sensor, const char* itemName, 
      const adc2_channel_t channel, const adc_atten_t atten, bool cal_enabled, const double coefficient,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString 
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , const char* formatTimestamp
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , const char* formatTimestampValue, const char* formatStringTimeValue
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
      );
    bool initItem() override;
    value_t convertValue(const value_t rawValue) override;
  protected:
    void registerItemParameters(paramsGroup_t * group) override;
    sensor_status_t getRawValue(value_t * rawValue) override;
  private:
    adc2_channel_t _channel;
    adc_atten_t _atten;
    bool _cal_enable = false;
    esp_adc_cal_characteristics_t _chars;
    double _coefficient = 1.0;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_ADC_H__