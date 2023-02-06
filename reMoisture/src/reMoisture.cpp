#include "reMoisture.h"

static const char* logTAG = "ADCM";

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

// =======================================================================================================================
// =======================================================================================================================
// ==================================================== rMoistureItem ====================================================
// =======================================================================================================================
// =======================================================================================================================

#define CONFIG_RAW_VALUE_THRESCHOLD 1024

// Constructor
rMoistureItem::rMoistureItem(rSensor *sensor, const char* itemName, 
  const adc1_channel_t channel, const adc_atten_t atten, const bool cal_enabled, const double coefficient,
  const type_bounds_t in_bounds, const value_t in_min, const value_t in_max,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString 
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , const char* formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , const char* formatTimestampValue, const char* formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
// inherited constructor
):rMapItem(sensor, itemName, in_bounds, in_min, in_max, 0, 100,
  filterMode, filterSize, formatNumeric, formatString
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

  _temp_coef = 0.0;
  _prm_temp_coef = nullptr;
  _temp_base = 0.0;
  _prm_temp_base = nullptr;
  _temp_curr = 0.0;
};

bool rMoistureItem::initItem()
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

void rMoistureItem::registerItemParameters(paramsGroup_t * group)
{
  rSensorItem::registerItemParameters(group);

  _prm_temp_coef = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_DOUBLE, nullptr, group, 
    CONFIG_SENSOR_PARAM_VTC_COEF_KEY, CONFIG_SENSOR_PARAM_VTC_COEF_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_temp_coef);
  _prm_temp_base = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_VTC_BASE_KEY, CONFIG_SENSOR_PARAM_VTC_BASE_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_temp_base);
}

void rMoistureItem::setTempCorrection(double coefficent, value_t base_point)
{
  _temp_coef = coefficent;
  paramsValueStore(_prm_temp_coef, false);
  _temp_base = _temp_base;
  paramsValueStore(_prm_temp_base, false);
}

void rMoistureItem::setTemperature(value_t temperature)
{
  _temp_curr = temperature;
}

sensor_status_t rMoistureItem::getRawValue(value_t * rawValue)
{
  if (rawValue != nullptr) {
    *rawValue = (value_t)adc1_get_raw(_channel);
    if (*rawValue > CONFIG_RAW_VALUE_THRESCHOLD) {
      return SENSOR_STATUS_OK;
    } else {
      *rawValue = NAN;
      return SENSOR_STATUS_CONN_ERROR;
    };
  };
  return SENSOR_STATUS_NOT_SUPPORTED;
}

value_t rMoistureItem::convertValue(const value_t rawValue)
{
  if (isnan(rawValue)) return NAN;

  // Correction of the value depending on the temperature + Converting voltage to % moisture
  return rMapItem::convertValue(rawValue + rawValue * (double)((_temp_curr - _temp_base) * _temp_coef));
}

