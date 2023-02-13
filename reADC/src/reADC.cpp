#include "reADC.h"
#include "reEsp32.h"
#include <string.h>
#include "adc_cali_interface.h"
#include "esp_adc/adc_cali_scheme.h"

static const char* logTAG = "ADC";

#define CONFIG_IDF_ADC_DMAX 4095

// ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
  #define ADC_CALI_VREF     ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF
#else
  #define ADC_CALI_VREF     ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP
#endif

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- ADC1 --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// Constructor
reADC::reADC(rSensor *sensor, const char* itemName,
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
// inherited constructor
):rSensorItem(sensor, itemName, filterMode, filterSize, formatNumeric, formatString
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , formatTimestampValue, formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
) {
  initADC(unit, channel, ulp_mode, atten, bitwidth, cal_enabled, coefficient);
};

reADC::reADC(rSensor *sensor, const char* itemName,
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
// inherited constructor
):rSensorItem(sensor, itemName, filterMode, filterSize, formatNumeric, formatString
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , formatTimestampValue, formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
) {
  adc_unit_t unit;
  adc_channel_t channel;
  adc_oneshot_io_to_channel(io_num, &unit, &channel);
  initADC(unit, channel, ulp_mode, atten, bitwidth, cal_enabled, coefficient);
}

reADC::~reADC()
{
  if (_cal_handle != nullptr) {
    adc_cali_delete_scheme_line_fitting(_cal_handle);
  };
  if (_adc_handle != nullptr) {
    adc_oneshot_del_unit(_adc_handle);
  };

  rSensorItem::~rSensorItem();
}

void reADC::initADC(const adc_unit_t unit, const adc_channel_t channel, const adc_ulp_mode_t ulp_mode, 
      const adc_atten_t atten, const adc_bitwidth_t bitwidth, 
      const bool cal_enabled, const double coefficient)
{
  _unit = unit;
  _channel = channel;
  _ulp_mode = ulp_mode;
  _atten = atten;
  _bitwidth = bitwidth;
  _cal_enabled = cal_enabled;
  _coefficient = coefficient;
}

bool reADC::initItem()
{
  // Create an ADC unit handle under normal oneshot mode
  if (_adc_handle == nullptr) {
    adc_oneshot_unit_init_cfg_t adcCfg;
    memset(&adcCfg, 0, sizeof(adcCfg));
    adcCfg.unit_id = _unit;
    adcCfg.ulp_mode = _ulp_mode;
    RE_OK_CHECK(adc_oneshot_new_unit(&adcCfg, &_adc_handle), return false);
  };
  RE_MEM_CHECK(_adc_handle, return false);

  // Configure ADC channel
  adc_oneshot_chan_cfg_t chnCfg;
  memset(&chnCfg, 0, sizeof(chnCfg));
  chnCfg.atten = _atten;
  chnCfg.bitwidth = _bitwidth;
  RE_OK_CHECK(adc_oneshot_config_channel(_adc_handle, _channel, &chnCfg), return false);

  // Configure calibration
  if (_cal_handle == nullptr) {
    if (_cal_enabled) {
      bool calibrated = false;
      #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED      
        if (!calibrated) {
          adc_cali_curve_fitting_config_t calCfg;
          memset(&calCfg, 0, sizeof(calCfg));
          calCfg.atten = _atten;
          calCfg.bitwidth = _bitwidth;
          calCfg.unit_id = _unit;
          calibrated = adc_cali_create_scheme_curve_fitting(&calCfg, &_cal_handle) == ESP_OK;
        };
      #endif
      #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED      
        if (!calibrated) {
          adc_cali_line_fitting_config_t calCfg;
          memset(&calCfg, 0, sizeof(calCfg));
          calCfg.atten = _atten;
          calCfg.bitwidth = _bitwidth;
          calCfg.unit_id = _unit;
          calCfg.default_vref = ADC_CALI_VREF;
          calibrated = adc_cali_create_scheme_line_fitting(&calCfg, &_cal_handle) == ESP_OK;
        };
      #endif
      _cal_enabled = calibrated;
    };
  };

  // Logging
  if (_cal_enabled) {
    rlog_i(logTAG, "ADC channel [ %s ] initialized, calibration enabled", getName());
  } else {
    rlog_w(logTAG, "ADC channel [ %s ] initialized, calibration disabled", getName());
  };

  // Inherited init
  return rSensorItem::initItem();
}

void reADC::registerItemParameters(paramsGroup_t * group)
{
  rSensorItem::registerItemParameters(group);
  paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_DOUBLE, nullptr, group, 
    CONFIG_SENSOR_PARAM_COEF_KEY, CONFIG_SENSOR_PARAM_COEF_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_coefficient);
}

adc_unit_t reADC::getUnit()
{
  return _unit;
}

sensor_status_t reADC::getRawValue(value_t * rawValue)
{
  if (rawValue != nullptr) {
    int out_raw;
    RE_OK_CHECK(adc_oneshot_read(_adc_handle, _channel, &out_raw), return SENSOR_STATUS_ERROR);
    *rawValue = (value_t)out_raw;
  };
  return SENSOR_STATUS_OK;
}

value_t reADC::convertValue(const value_t rawValue)
{
  // Convert ADC raw data to calibrated voltage
  if (_cal_enabled) {
    int outValue;
    esp_err_t res = adc_cali_raw_to_voltage(_cal_handle, (int)rawValue, &outValue);
    if (res == ESP_OK) {
      return (value_t)outValue;
    };
  };

  // Default calculation
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
}