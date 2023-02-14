#include "reMoisture.h"
#include "reADCIntf.h"

static const char* logTAG = "ADCM";

#define CONFIG_RAW_VALUE_THRESCHOLD 1024

// =======================================================================================================================
// ==================================================== rMoistureItem ====================================================
// =======================================================================================================================

// Constructor
rMoistureItem::rMoistureItem(rSensor *sensor, const char* itemName, 
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
  _unit_handle = adc_unit_handle;
  _channel = channel;
  _atten = atten;
  _bitwidth = bitwidth;

  _temp_coef = 0.0;
  _prm_temp_coef = nullptr;
  _temp_base = 0.0;
  _prm_temp_base = nullptr;
  _temp_curr = 0.0;
};

bool rMoistureItem::initItem()
{
  RE_LINK_CHECK_EVENT(_unit_handle, "adc_unit_handle", return false);

  // Configure ADC channel
  adc_oneshot_chan_cfg_t cfg_channel;
  memset(&cfg_channel, 0, sizeof(cfg_channel));
  cfg_channel.atten = _atten;
  cfg_channel.bitwidth = _bitwidth;
  RE_OK_CHECK(adc_oneshot_config_channel(_unit_handle, _channel, &cfg_channel), return false);

  // Logging
  rlog_i(logTAG, "ADC channel [ %s ] initialized, calibration disabled", getName());

  // Inherited init
  return rSensorItem::initItem();
}

void rMoistureItem::setChannel(adc_oneshot_unit_handle_t unit_handle, adc_channel_t channel)
{
  _unit_handle = unit_handle;
  _channel = channel;
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
    int out_raw;
    RE_OK_CHECK(adc_oneshot_read(_unit_handle, _channel, &out_raw), return SENSOR_STATUS_CONN_ERROR);
    *rawValue = (value_t)out_raw;
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

// =======================================================================================================================
// ==================================================== rMoistureGpio ====================================================
// =======================================================================================================================

rMoistureGpio::rMoistureGpio(rSensor *sensor, const char* itemName, 
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
):rMoistureItem(sensor, itemName, 
  nullptr, ADC_CHANNEL_0, atten, bitwidth, 
  in_bounds, in_min, in_max,
  filterMode, filterSize, 
  formatNumeric, formatString 
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , formatTimestampValue, formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
) {
  _adc_gpio = adc_gpio;
}

bool rMoistureGpio::initItem()
{
  adc_unit_t unit_id;
  adc_channel_t channel;
  RE_OK_CHECK(adc_oneshot_io_to_channel(_adc_gpio, &unit_id, &channel), return false);
  adc_oneshot_unit_handle_t unit = adcUnitGet(unit_id);
  if (unit == nullptr) return false;
  setChannel(unit, channel);

  return rMoistureItem::initItem();
}
