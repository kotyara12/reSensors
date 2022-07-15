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
  const value_t levelMin, const value_t levelMax, const type_bounds_t typeBounds, const value_t sizeRange,
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

  _level_min = levelMin;
  _prm_level_min = nullptr;
  _level_max = levelMax;
  _prm_level_max = nullptr;
  _type = typeBounds;
  _prm_type = nullptr;
  _range = sizeRange;
  _prm_range = nullptr;

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

  _prm_type = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, group, 
    CONFIG_SENSOR_PARAM_RANGETYPE_KEY, CONFIG_SENSOR_PARAM_RANGETYPE_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_type);
  _prm_range = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_RANGESIZE_KEY, CONFIG_SENSOR_PARAM_RANGESIZE_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_range);
  _prm_level_min = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_LEVELMIN_KEY, CONFIG_SENSOR_PARAM_LEVELMIN_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_level_min);
  _prm_level_max = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_LEVELMAX_KEY, CONFIG_SENSOR_PARAM_LEVELMAX_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_level_max);
  _prm_temp_coef = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_DOUBLE, nullptr, group, 
    CONFIG_SENSOR_PARAM_VTC_COEF_KEY, CONFIG_SENSOR_PARAM_VTC_COEF_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_temp_coef);
  _prm_temp_base = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_VTC_BASE_KEY, CONFIG_SENSOR_PARAM_VTC_BASE_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_temp_base);
}

void rMoistureItem::setLevelMin(const value_t newValue)
{
  _level_min = newValue;
  paramsValueStore(_prm_level_min, false);
}

void rMoistureItem::setLevelMax(const value_t newValue)
{
  _level_max = newValue;
  paramsValueStore(_prm_level_max, false);
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

  // Correction of the value depending on the temperature
  value_t _value = rawValue + rawValue * (double)((_temp_curr - _temp_base) * _temp_coef);

  // Converting voltage to % moisture
  switch (_type) {
    case BOUNDS_AUTOEXPANDING_DIRECT:
      if (_value < _level_min) setLevelMin(_value);
      if (_value > _level_max) setLevelMax(_value);
      break;
    case BOUNDS_AUTOEXPANDING_INVERT:
      if (_value < _level_max) setLevelMax(_value);
      if (_value > _level_min) setLevelMin(_value);
      break;
    case BOUNDS_SLIDING:
      if (_level_min < _level_max) {
        if (_value < _level_min) {
          setLevelMin(_value);
          setLevelMax(_value + _range);
        };
        if (_value > _level_max) {
          setLevelMax(_value);
          setLevelMin(_value - _range);
        };
      } else {
        if (_value < _level_max) {
          setLevelMax(_value);
          setLevelMin(_value + _range);
        };
        if (_value > _level_min) {
          setLevelMin(_value);
          setLevelMax(_value - _range);
        };
      };
      break;
    default:
      break;
  };
  
  value_t ret = NAN;
  if (_level_min != _level_max) {
    // Conversion in % of the specified range
    if (_level_min < _level_max) {
      ret = 100.0 * ((_value - _level_min) / (_level_max - _level_min));
    } else {
      ret = 100.0 * ((_level_min - _value) / (_level_min - _level_max));
    };
    
    // Normalization
    if (ret < 0) ret = 0.0;
    if (ret > 100) ret = 100.0;
  };

  return ret;
}

