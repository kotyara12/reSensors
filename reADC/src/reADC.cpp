#include "reADC.h"
#include "reEsp32.h"
#include <string.h>

static const char* logTAG = "ADC";

#define CONFIG_IDF_ADC_DMAX 4095

// ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

// Constructor
reADC1::reADC1(rSensor *sensor, const char* itemName,
  const adc1_channel_t channel, const adc_atten_t atten, const double coefficient,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString 
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , const char* formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , const char* formatTimestampValue, const char* formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
// inherited constructor
):rSensorItem(sensor, itemName, filterMode, filterSize, formatNumeric, formatString
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , formatTimestampValue, formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
) {
  _channel = channel;
  _atten = atten;
  _cal_enable = false;
  memset(&_chars, 0, sizeof(_chars));
  _coefficient = coefficient;
};

bool reADC1::initItem()
{
  _cal_enable = false;
  RE_OK_CHECK_FIRST(logTAG, adc1_config_width((adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT), return false);
  RE_OK_CHECK(logTAG, adc1_config_channel_atten(_channel, _atten), return false);
  RE_OK_CHECK(logTAG, esp_adc_cal_check_efuse(ADC_CALI_SCHEME), return true);
  _cal_enable = (esp_adc_cal_characterize(ADC_UNIT_1, _atten, (adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT, 0, &_chars) != ESP_ADC_CAL_VAL_NOT_SUPPORTED);
  if (_cal_enable) {
    rlog_i(logTAG, "ADC channel [ %s ] initialized, calibration enabled", getName());
  } else {
    rlog_w(logTAG, "ADC channel [ %s ] initialized, calibration disabled", getName());
  };
  return rSensorItem::initItem();
}

void reADC1::registerItemParameters(paramsGroup_t * group)
{
  rSensorItem::registerItemParameters(group);
  paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_DOUBLE, nullptr, group, 
    CONFIG_SENSOR_PARAM_COEF_KEY, CONFIG_SENSOR_PARAM_COEF_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_coefficient);
}

sensor_status_t reADC1::getRawValue(value_t * rawValue)
{
  *rawValue = (value_t)adc1_get_raw(_channel);
  return SENSOR_STATUS_OK;
}

value_t reADC1::convertValue(const value_t rawValue)
{
  if (_cal_enable) {
    return (value_t)esp_adc_cal_raw_to_voltage((uint32_t)rawValue, &_chars) * _coefficient;
  } else {
    switch (_atten) {
      case ADC_ATTEN_DB_2_5: 
        return (rawValue * 1250 / 4095) * _coefficient;
      case ADC_ATTEN_DB_6:   
        return (rawValue * 1750 / 4095) * _coefficient;
      case ADC_ATTEN_DB_11:
        return (rawValue * 2450 / 4095) * _coefficient;
      default: 
        return (rawValue * 950 / 4095) * _coefficient;
    };
  };
}

