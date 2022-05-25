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

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- ADC1 --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// Constructor
reADC1::reADC1(rSensor *sensor, const char* itemName,
  const adc1_channel_t channel, const adc_atten_t atten, const bool cal_enabled, const double coefficient,
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
  _cal_enable = cal_enabled;
  memset(&_chars, 0, sizeof(_chars));
  _coefficient = coefficient;
};

bool reADC1::initItem()
{
  RE_OK_CHECK(adc1_config_width((adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT), return false);
  RE_OK_CHECK(adc1_config_channel_atten(_channel, _atten), return false);
  if (_cal_enable && (esp_adc_cal_check_efuse(ADC_CALI_SCHEME) == ESP_OK)) {
    _cal_enable = (esp_adc_cal_characterize(ADC_UNIT_1, _atten, (adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT, 0, &_chars) != ESP_ADC_CAL_VAL_NOT_SUPPORTED);
  };
  if (_cal_enable) {
    rlog_i(logTAG, "ADC1 channel [ %s ] initialized, calibration enabled", getName());
  } else {
    rlog_w(logTAG, "ADC1 channel [ %s ] initialized, calibration disabled", getName());
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
  if (rawValue != nullptr) {
    *rawValue = (value_t)adc1_get_raw(_channel);
  };
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

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- ADC2 --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// Constructor
reADC2::reADC2(rSensor *sensor, const char* itemName,
  const adc2_channel_t channel, const adc_atten_t atten, bool cal_enabled, const double coefficient,
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
  _cal_enable = cal_enabled;
  memset(&_chars, 0, sizeof(_chars));
  _coefficient = coefficient;
};

bool reADC2::initItem()
{
  _cal_enable = false;
  RE_OK_CHECK(adc2_config_channel_atten(_channel, _atten), return false);
  if (_cal_enable && (esp_adc_cal_check_efuse(ADC_CALI_SCHEME) == ESP_OK)) {
    _cal_enable = (esp_adc_cal_characterize(ADC_UNIT_2, _atten, (adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT, 0, &_chars) != ESP_ADC_CAL_VAL_NOT_SUPPORTED);
  };
  if (_cal_enable) {
    rlog_i(logTAG, "ADC2 channel [ %s ] initialized, calibration enabled", getName());
  } else {
    rlog_w(logTAG, "ADC2 channel [ %s ] initialized, calibration disabled", getName());
  };
  return rSensorItem::initItem();
}

void reADC2::registerItemParameters(paramsGroup_t * group)
{
  rSensorItem::registerItemParameters(group);
  paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_DOUBLE, nullptr, group, 
    CONFIG_SENSOR_PARAM_COEF_KEY, CONFIG_SENSOR_PARAM_COEF_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_coefficient);
}

sensor_status_t reADC2::getRawValue(value_t * rawValue)
{
  if (rawValue != nullptr) {
    int buf;
    esp_err_t err = adc2_get_raw(_channel, (adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT, &buf);
    if (err == ESP_OK) {
      *rawValue = (value_t)buf;
    } else if (err == ESP_ERR_INVALID_STATE) {
      rlog_i(logTAG, "ADC2 not initialized yet: %d, %s", err, esp_err_to_name(err));
      return SENSOR_STATUS_NO_INIT;
    } else if (err == ESP_ERR_TIMEOUT) {
      rlog_i(logTAG, "ADC2 is in use by Wi-Fi: %d, %s", err, esp_err_to_name(err));
      return SENSOR_STATUS_CONN_ERROR;
    } else {
      rlog_i(logTAG, "ADC2 read error: %d %s", err, esp_err_to_name(err));
      return SENSOR_STATUS_ERROR;
    };
  };
  return SENSOR_STATUS_OK;
}

value_t reADC2::convertValue(const value_t rawValue)
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

