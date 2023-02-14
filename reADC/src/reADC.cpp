#include "reADC.h"
#include "reADCIntf.h"
#include "reEsp32.h"
#include <string.h>

static const char* logTAG = "ADC";

#define CONFIG_IDF_ADC_DMAX 4095

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------- ADC Item ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

reADCItem::reADCItem(rSensor *sensor, const char* itemName, 
  const adc_oneshot_unit_handle_t adc_unit_handle, const adc_cali_handle_t adc_cali_handle,
  const adc_channel_t channel, const adc_atten_t atten, const adc_bitwidth_t bitwidth, 
  const double coefficient,
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
  _unit_handle = adc_unit_handle;
  _cali_handle = adc_cali_handle;
  _channel = channel;
  _atten = atten;
  _bitwidth = bitwidth;
  _coefficient = coefficient;
};

bool reADCItem::initItem()
{
  RE_LINK_CHECK_EVENT(_unit_handle, "adc_unit_handle", return false);

  // Configure ADC channel
  adc_oneshot_chan_cfg_t cfg_channel;
  memset(&cfg_channel, 0, sizeof(cfg_channel));
  cfg_channel.atten = _atten;
  cfg_channel.bitwidth = _bitwidth;
  RE_OK_CHECK(adc_oneshot_config_channel(_unit_handle, _channel, &cfg_channel), return false);

  // Logging
  if (_cali_handle) {
    rlog_i(logTAG, "ADC channel [ %s ] initialized, calibration enabled", getName());
  } else {
    rlog_w(logTAG, "ADC channel [ %s ] initialized, calibration disabled", getName());
  };

  // Inherited init
  return rSensorItem::initItem();
}

adc_channel_t reADCItem::getChannel()
{
  return _channel;
}

adc_atten_t reADCItem::getAtten()
{
  return _atten;
}

adc_bitwidth_t reADCItem::getBitwidth()
{
  return _bitwidth;
}

void reADCItem::setChannel(adc_oneshot_unit_handle_t unit_handle, adc_channel_t channel)
{
  _unit_handle = unit_handle;
  _channel = channel;
}

void reADCItem::registerItemParameters(paramsGroup_t * group)
{
  rSensorItem::registerItemParameters(group);
  paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_DOUBLE, nullptr, group, 
    CONFIG_SENSOR_PARAM_COEF_KEY, CONFIG_SENSOR_PARAM_COEF_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_coefficient);
}

sensor_status_t reADCItem::getRawValue(value_t * rawValue)
{
  if (rawValue != nullptr) {
    int out_raw;
    RE_OK_CHECK(adc_oneshot_read(_unit_handle, _channel, &out_raw), return SENSOR_STATUS_ERROR);
    *rawValue = (value_t)out_raw;
  };
  return SENSOR_STATUS_OK;
}

value_t reADCItem::convertValue(const value_t rawValue)
{
  // Convert ADC raw data to calibrated voltage
  if (_cali_handle) {
    int outValue;
    esp_err_t res = adc_cali_raw_to_voltage(_cali_handle, (int)rawValue, &outValue);
    if (res == ESP_OK) {
      return (value_t)outValue * _coefficient;
    };
  };

  // Default calculation
  switch (_atten) {
    case ADC_ATTEN_DB_2_5: 
      return (rawValue * 1250 / CONFIG_IDF_ADC_DMAX) * _coefficient;
    case ADC_ATTEN_DB_6:   
      return (rawValue * 1750 / CONFIG_IDF_ADC_DMAX) * _coefficient;
    case ADC_ATTEN_DB_11:
      return (rawValue * 2450 / CONFIG_IDF_ADC_DMAX) * _coefficient;
    default: 
      return (rawValue * 950 / CONFIG_IDF_ADC_DMAX) * _coefficient;
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- ADC ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

reADCGpio::reADCGpio(rSensor *sensor, const char* itemName, 
  const int adc_gpio, const bool use_calibration,
  const adc_atten_t atten, const adc_bitwidth_t bitwidth, 
  const double coefficient,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString 
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , const char* formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , const char* formatTimestampValue, const char* formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
):reADCItem(sensor, itemName, 
  nullptr, nullptr, ADC_CHANNEL_0, atten, bitwidth, coefficient,
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
  _calibration = use_calibration;
};

reADCGpio::~reADCGpio()
{
  adcCaliFree(_cali_handle);
}

bool reADCGpio::initItem() 
{
  adc_unit_t unit_id;
  adc_channel_t channel;
  RE_OK_CHECK(adc_oneshot_io_to_channel(_adc_gpio, &unit_id, &channel), return false);
  adc_oneshot_unit_handle_t unit = adcUnitGet(unit_id);
  if (unit == nullptr) return false;
  setChannel(unit, channel);

  if (_calibration) {
    _calibration = adcCaliCreate(unit_id, getAtten(), getBitwidth(), &_cali_handle);
  };
  
  return reADCItem::initItem();
}