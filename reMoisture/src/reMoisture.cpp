#include "reMoisture.h"

// =======================================================================================================================
// =======================================================================================================================
// ==================================================== rMoistureItem ====================================================
// =======================================================================================================================
// =======================================================================================================================

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
):reADC1(sensor, itemName, 
  channel, atten, cal_enabled, coefficient, 
  filterMode, filterSize, formatNumeric, formatString
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , formatTimestampValue, formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
) {
  _level_min = levelMin;
  _prm_level_min = nullptr;
  _level_max = levelMax;
  _prm_level_max = nullptr;
  _type = typeBounds;
  _prm_type = nullptr;
  _range = sizeRange;
  _prm_range = nullptr;
};

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
}

void rMoistureItem::setLevelMin(const value_t newValue)
{
  _level_min = newValue;
  paramsValueApply(_prm_level_min, false);
}

void rMoistureItem::setLevelMax(const value_t newValue)
{
  _level_max = newValue;
  paramsValueApply(_prm_level_max, false);
}

value_t rMoistureItem::convertValue(const value_t rawValue)
{
  if (isnan(rawValue)) return NAN;
  
  // Converting voltage to % moisture
  switch (_type) {
    case BOUNDS_AUTOEXPANDING_DIRECT:
      if (rawValue < _level_min) setLevelMin(rawValue);
      if (rawValue > _level_max) setLevelMax(rawValue);
      break;
    case BOUNDS_AUTOEXPANDING_INVERT:
      if (rawValue < _level_max) setLevelMax(rawValue);
      if (rawValue > _level_min) setLevelMin(rawValue);
      break;
    case BOUNDS_SLIDING:
      if (_level_min < _level_max) {
        if (rawValue < _level_min) {
          setLevelMin(rawValue);
          setLevelMax(rawValue + _range);
        };
        if (rawValue > _level_max) {
          setLevelMax(rawValue);
          setLevelMin(rawValue - _range);
        };
      } else {
        if (rawValue < _level_max) {
          setLevelMax(rawValue);
          setLevelMin(rawValue + _range);
        };
        if (rawValue > _level_min) {
          setLevelMin(rawValue);
          setLevelMax(rawValue - _range);
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
      ret = 100.0 * (rawValue - _level_min) / (_level_max - _level_min);
    } else {
      ret = 100.0 * (rawValue - _level_max) / (_level_min - _level_max);
    };
    
    // Normalization
    if (ret < 0) ret = 0.0;
    if (ret > 100) ret = 100.0;
  };
  return ret;
}

