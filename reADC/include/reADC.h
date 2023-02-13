/* 
   EN: Voltage measurement at ADC inputs and conversion to real values
   RU: Измерение напряжения на ADC входах и пересчет в реальные значения
   --------------------------
   (с) 2022-2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_ADC_H__
#define __RE_ADC_H__

#include <stdint.h>
#include <driver/gpio.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include <reParams.h>
#include <reSensor.h>
#include "project_config.h"

#ifdef __cplusplus
extern "C" {
#endif

class reADC: public rSensorItem {
  public:
    reADC(rSensor *sensor, const char* itemName, 
      const int io_num, const adc_ulp_mode_t ulp_mode, 
      const adc_atten_t atten, const adc_bitwidth_t bitwidth, 
      const bool cal_enabled, const double coefficient,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString 
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , const char* formatTimestamp
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , const char* formatTimestampValue, const char* formatStringTimeValue
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
      );
    reADC(rSensor *sensor, const char* itemName, 
      const adc_unit_t unit, const adc_channel_t channel, const adc_ulp_mode_t ulp_mode, 
      const adc_atten_t atten, const adc_bitwidth_t bitwidth, 
      const bool cal_enabled, const double coefficient,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString 
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , const char* formatTimestamp
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , const char* formatTimestampValue, const char* formatStringTimeValue
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
      );
    virtual ~reADC();
    bool initItem() override;
    adc_unit_t getUnit();
    value_t convertValue(const value_t rawValue) override;
  protected:
    void registerItemParameters(paramsGroup_t * group) override;
    sensor_status_t getRawValue(value_t * rawValue) override;
  private:
    adc_unit_t      _unit;
    adc_channel_t   _channel;
    adc_ulp_mode_t  _ulp_mode;
    adc_atten_t     _atten;
    adc_bitwidth_t  _bitwidth;
    double          _coefficient = 1.0;
    bool            _cal_enabled = false;
    adc_oneshot_unit_handle_t _adc_handle = nullptr;
    adc_cali_handle_t _cal_handle = nullptr;
    
    void initADC(const adc_unit_t unit, const adc_channel_t channel, const adc_ulp_mode_t ulp_mode, 
      const adc_atten_t atten, const adc_bitwidth_t bitwidth, 
      const bool cal_enabled, const double coefficient);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_ADC_H__