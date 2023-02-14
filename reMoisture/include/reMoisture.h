/* 
   EN: Library for reading data from Capacitive Soil Moisture Sensor v1.2 or similar
   RU: Библиотека для считывания данных с Capacitive Soil Moisture Sensor v1.2 или аналогичных
   --------------------------
   (с) 2021-2002 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_MOISTURE_H__
#define __RE_MOISTURE_H__

#include <stdint.h>
#include <driver/gpio.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "reParams.h"
#include "reSensor.h"
#include "reADC.h"
#include "project_config.h"

#ifdef __cplusplus
extern "C" {
#endif

class rMoistureItem: public rMapItem {
  public:
    rMoistureItem(rSensor *sensor, const char* itemName, 
      const adc_oneshot_unit_handle_t adc_unit_handle, 
      const adc_channel_t channel, const adc_atten_t atten, const adc_bitwidth_t bitwidth, 
      const type_bounds_t in_bounds, const value_t in_min, const value_t in_max,
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

    // Correction of the value depending on the temperature
    void setTempCorrection(double coefficent, value_t base_point);
    void setTemperature(value_t temperature);
  protected:
    void setChannel(adc_oneshot_unit_handle_t unit_handle, adc_channel_t channel);

    void registerItemParameters(paramsGroup_t * group) override;
    sensor_status_t getRawValue(value_t * rawValue) override;
  private:
    // ADC data 
    adc_oneshot_unit_handle_t _unit_handle = nullptr;
    adc_channel_t             _channel;
    adc_atten_t               _atten;
    adc_bitwidth_t            _bitwidth;
    
    // Temperature correction
    double                    _temp_coef = 0.0;
    paramsEntryHandle_t       _prm_temp_coef;
    value_t                   _temp_base = 0.0;
    paramsEntryHandle_t       _prm_temp_base;
    value_t                   _temp_curr = 0.0;
};

class rMoistureGpio: public rMoistureItem {
  public: 
    rMoistureGpio(rSensor *sensor, const char* itemName, 
      const int adc_gpio, const adc_atten_t atten, const adc_bitwidth_t bitwidth, 
      const type_bounds_t in_bounds, const value_t in_min, const value_t in_max,
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
  private:
    int _adc_gpio = -1;
};


#ifdef __cplusplus
}
#endif

#endif // __RE_MOISTURE_H__