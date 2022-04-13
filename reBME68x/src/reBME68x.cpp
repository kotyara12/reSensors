#include "reBME68x.h"
#include "freertos/FreeRTOS.h"
#include "rStrings.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "reParams.h"
#include "rLog.h"
#include "string.h"
#include "driver/i2c.h"
#include "def_consts.h"

#define BME68X_I2C_TIMEOUT    3000

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Callbacks ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static BME68X_INTF_RET_TYPE BME68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  BME68x* sensor = (BME68x*)intf_ptr;
  if (sensor) {
    esp_err_t err = readI2C(sensor->getI2CNum(), sensor->getI2CAddress(), &reg_addr, sizeof(reg_addr), reg_data, length, 0, BME68X_I2C_TIMEOUT); 
    if (err == ESP_OK) {
      return BME68X_OK;
    } else {
      return BME68X_E_COM_FAIL;
    };
  };
  return BME68X_E_NULL_PTR;
}

static BME68X_INTF_RET_TYPE BME68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  BME68x* sensor = (BME68x*)intf_ptr;
  if (sensor) {
    esp_err_t err = writeI2C(sensor->getI2CNum(), sensor->getI2CAddress(), &reg_addr, sizeof(reg_addr), (uint8_t*)reg_data, length, BME68X_I2C_TIMEOUT); 
    if (err == ESP_OK) {
      return BME68X_OK;
    } else {
      return BME68X_E_COM_FAIL;
    };
  };
  return BME68X_E_NULL_PTR;
}

static void BME68x_delay_us(uint32_t period, void *intf_ptr)
{
  ets_delay_us(period);
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ rBME68xHeaterHandler -------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rBME68xHeaterHandler::rBME68xHeaterHandler(rSensor *sensor)
{
  _sensor = sensor;
}

void rBME68xHeaterHandler::onChange(param_change_mode_t mode)
{
  BME68x* bme = (BME68x*)_sensor;
  bme->sendHeaterMode();
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ rIAQItem -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// Constructor
rIAQItem::rIAQItem(rSensor *sensor, const char* itemName, rSensorItem *humidityItem,
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
  _forcedRawPublish = true;

  _humidity = humidityItem; 

  _limit_bad = BME68x_DEFAULT_GAS_LIMIT_BAD;
  _limit_good = BME68x_DEFAULT_GAS_LIMIT_GOOD;
  _limits_autoshift = LIM_AUTOSHIFT_OFF;
};

void rIAQItem::registerItemParameters(paramsGroup_t * group)
{
  rSensorItem::registerItemParameters(group);

  _prm_hum_ratio = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_GAS_HUM_RATIO_KEY, CONFIG_SENSOR_PARAM_GAS_HUM_RATIO_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_hum_ratio);
  paramsSetLimitsFloat(_prm_hum_ratio, 0.0, 0.9);

  _prm_limit_bad = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_GAS_LIMIT_BAD_KEY, CONFIG_SENSOR_PARAM_GAS_LIMIT_BAD_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_limit_bad);
  paramsSetLimitsFloat(_prm_limit_bad, 0, 999999);

  _prm_limit_good = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_GAS_LIMIT_GOOD_KEY, CONFIG_SENSOR_PARAM_GAS_LIMIT_GOOD_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_limit_good);
  paramsSetLimitsFloat(_prm_limit_good, 0, 999999);

  _prm_autoshift = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, group, 
    CONFIG_SENSOR_PARAM_GAS_LIMIT_AUTOSHIFT_KEY, CONFIG_SENSOR_PARAM_GAS_LIMIT_AUTOSHIFT_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_limits_autoshift);
  paramsSetLimitsU8(_prm_autoshift, LIM_AUTOSHIFT_OFF, LIM_AUTOSHIFT_HIGH);
}

void rIAQItem::checkLimits(float newValue) 
{
  if ((_limits_autoshift == LIM_AUTOSHIFT_ALL) || (_limits_autoshift == LIM_AUTOSHIFT_LOW)) {
    if (_limit_bad < _limit_good) {
      if (newValue < _limit_bad) {
        _limit_bad = newValue;
        paramsValueStore(_prm_limit_bad, false);
        rlog_v(_name, "A new value for the lower range limit has been set: %f", _limit_bad);
      };
    } else {
      if (newValue > _limit_bad) {
        _limit_bad = newValue;
        paramsValueStore(_prm_limit_bad, false);
        rlog_v(_name, "A new value for the lower range limit has been set: %f", _limit_bad);
      };
    };
  };

  if ((_limits_autoshift == LIM_AUTOSHIFT_ALL) || (_limits_autoshift == LIM_AUTOSHIFT_HIGH)) {
    if (_limit_bad < _limit_good) {
      if (newValue > _limit_good) {
        _limit_good = newValue;
        paramsValueStore(_prm_limit_good, false);
        rlog_v(_name, "A new value for the higher range limit has been set: %f", _limit_good);
      };
    } else {
      if (newValue < _limit_good) {
        _limit_good = newValue;
        paramsValueStore(_prm_limit_good, false);
        rlog_v(_name, "A new value for the higher range limit has been set: %f", _limit_good);
      };
    };
  };
};

value_t rIAQItem::getHumidityScore(value_t humValue) 
{  
  if (_hum_ratio > 0) {
    float hum_reference = 40;
    if ((humValue >= 38) && (humValue <= 42))
      // Humidity +/-5% around optimum
      return _hum_ratio * 100;
    else { 
      // Humidity is sub-optimal
      if (humValue < 38)
        return _hum_ratio / hum_reference * humValue * 100;
      else {
        return ((1.6666667 * _hum_ratio) - (_hum_ratio / (100 - hum_reference) * humValue)) * 100;
      };
    }
  };
  return 0;
}

value_t rIAQItem::getGasScore(value_t gasValue) 
{
  value_t gas_score = 0.0;
  value_t gas_ratio = 1.0 - _hum_ratio;
  if (_limit_bad < _limit_good) {
    gas_score = (gas_ratio / (_limit_good - _limit_bad) * gasValue - (_limit_bad * (gas_ratio / (_limit_good - _limit_bad)))) * 100.00;
  } else {
    gas_score = (gas_ratio / (_limit_bad - _limit_good) * gasValue - (_limit_good * (gas_ratio / (_limit_bad - _limit_good)))) * 100.00;
  };
  // Sometimes gas readings can go outside of expected scale maximum
  if (gas_score > (gas_ratio * 100)) gas_score = gas_ratio * 100;
  // Sometimes gas readings can go outside of expected scale minimum
  if (gas_score < 0) gas_score = 0;
  return gas_score;
}

value_t rIAQItem::calculateIAQ(value_t humValue, value_t gasValue)
{
  return (100.0 - (getHumidityScore(humValue) + getGasScore(gasValue))) * 5.0;
}

value_t rIAQItem::convertValue(const value_t rawValue)
{
  if (_limits_autoshift != LIM_AUTOSHIFT_OFF) {
    checkLimits(rawValue);
  };

  return calculateIAQ(_humidity->getValue().rawValue, rawValue);
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------- BME68x --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

BME68x::BME68x():rSensorX4()
{
  _I2C_num = 0;
  _I2C_address = 0;

  memset(&_dev, 0, sizeof(_dev));
  _dev.chip_id = 0;
  _dev.amb_temp = 25;
  _dev.intf_ptr = this;
  _dev.intf = BME68X_I2C_INTF;
  _dev.read = &BME68x_i2c_read;
  _dev.write = &BME68x_i2c_write;
  _dev.delay_us = &BME68x_delay_us;

  memset(&_conf, 0, sizeof(_conf));

  memset(&_heatr_conf, 0, sizeof(_heatr_conf));
  _heatr_conf.heatr_temp = 320;
  _heatr_conf.heatr_dur = 150;
  _prm_heatr_temp = nullptr;
  _prm_heatr_dur = nullptr;
  _heatr_hndl = nullptr;
}

// Destructor
BME68x::~BME68x()
{
  if (_heatr_hndl) delete _heatr_hndl;
  _heatr_hndl = nullptr;
}

// Displaying multiple values in one topic
#if CONFIG_SENSOR_DISPLAY_ENABLED

char* BME68x::getDisplayValue()
{
  char* ret = nullptr;
  if (_item2) { 
    ret = _item2->getStringFiltered(); 
  };
  if (_item3) {
    ret = concat_strings_div(ret, _item3->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

#if CONFIG_SENSOR_AS_PLAIN

bool BME68x::publishCustomValues()
{
  bool ret = rSensor::publishCustomValues();

  #if CONFIG_SENSOR_DEWPOINT_ENABLE
    if ((ret) && (_item2) && (_item3)) {
      ret = _item2->publishDataValue(CONFIG_SENSOR_DEWPOINT, 
        calcDewPoint(_item2->getValue().filteredValue, _item3->getValue().filteredValue));
    };
  #endif // CONFIG_SENSOR_DEWPOINT_ENABLE

  return ret;
} 

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* BME68x::jsonCustomValues()
{
  #if CONFIG_SENSOR_DEWPOINT_ENABLE
    if ((_item2) && (_item3)) {
      char * _dew_point = _item2->jsonDataValue(true, calcDewPoint(_item2->getValue().filteredValue, _item3->getValue().filteredValue));
      char * ret = malloc_stringf("\"%s\":%s", CONFIG_SENSOR_DEWPOINT, _dew_point);
      if (_dew_point) free(_dew_point);
      return ret;  
    };
  #endif // CONFIG_SENSOR_DEWPOINT_ENABLE
  return nullptr;
}

#endif // CONFIG_SENSOR_AS_JSON

/**
 * Dynamically creating internal items on the heap
 * */
bool BME68x::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const int numI2C, const uint8_t addrI2C, 
  BME68x_STANDBYTIME odr, BME68x_IIR_FILTER filter,
  BME68x_OVERSAMPLING osPress, BME68x_OVERSAMPLING osTemp, BME68x_OVERSAMPLING osHum,
  sensor_filter_t filterMode1, uint16_t filterSize1, 
  sensor_filter_t filterMode2, uint16_t filterSize2,
  sensor_filter_t filterMode3, uint16_t filterSize3,
  sensor_filter_t filterMode4, uint16_t filterSize4,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  _conf.odr = odr;
  _conf.filter = filter;
  _conf.os_pres = osPress;
  _conf.os_temp = osTemp;
  _conf.os_hum = osHum;
   // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX4::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2, filterMode3, filterSize3, filterMode4, filterSize4)) {
    // Start device
    return sensorStart();
  };
  return false;
}

void BME68x::createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                               const sensor_filter_t filterMode2, const uint16_t filterSize2,
                               const sensor_filter_t filterMode3, const uint16_t filterSize3,
                               const sensor_filter_t filterMode4, const uint16_t filterSize4)
{
  // Pressure
  _item1 = new rPressureItem(this, CONFIG_SENSOR_PRESSURE_NAME, CONFIG_FORMAT_PRESSURE_UNIT,
    filterMode1, filterSize1,
    CONFIG_FORMAT_PRESSURE_VALUE, CONFIG_FORMAT_PRESSURE_STRING,
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    CONFIG_FORMAT_TIMESTAMP_L, 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
    CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  if (_item1) {
    rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item1->getName(), _name);
  };

  // Temperature
  _item2 = new rTemperatureItem(this, CONFIG_SENSOR_TEMP_NAME, CONFIG_FORMAT_TEMP_UNIT,
    filterMode2, filterSize2,
    CONFIG_FORMAT_TEMP_VALUE, CONFIG_FORMAT_TEMP_STRING,
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    CONFIG_FORMAT_TIMESTAMP_L, 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
    CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  if (_item2) {
    rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item2->getName(), _name);
  };

  // Humidity
  _item3 = new rSensorItem(this, CONFIG_SENSOR_HUMIDITY_NAME, 
    filterMode3, filterSize3,
    CONFIG_FORMAT_HUMIDITY_VALUE, CONFIG_FORMAT_HUMIDITY_STRING,
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    CONFIG_FORMAT_TIMESTAMP_L, 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
    CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  if (_item3) {
    rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item3->getName(), _name);
  };

  // Gas
  _item4 = new rIAQItem(this, CONFIG_SENSOR_IAQ_NAME, _item3,
    filterMode4, filterSize4,
    CONFIG_FORMAT_IAQ_VALUE, CONFIG_FORMAT_IAQ_STRING,
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    CONFIG_FORMAT_TIMESTAMP_L, 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
    CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  if (_item4) {
    rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item4->getName(), _name);
  };
}

/**
 *  Register internal parameters
 * */
void BME68x::registerCustomParameters(paramsGroupHandle_t sensor_group)
{
  this->rSensorX4::registerCustomParameters(sensor_group);

  _heatr_hndl = new rBME68xHeaterHandler(this);

  _prm_heater = paramsRegisterGroup(sensor_group, 
    CONFIG_SENSOR_PARAM_HEATER_GROUP_KEY, CONFIG_SENSOR_PARAM_HEATER_GROUP_TOPIC, CONFIG_SENSOR_PARAM_HEATER_GROUP_FRIENDLY);

  _prm_heatr_temp = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U16, _heatr_hndl, _prm_heater, 
    CONFIG_SENSOR_PARAM_HEATER_TEMP_KEY, CONFIG_SENSOR_PARAM_HEATER_TEMP_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_heatr_conf.heatr_temp);
  paramsSetLimitsU16(_prm_heatr_temp, BME68X_LOW_TEMP, BME68X_HIGH_TEMP);

  _prm_heatr_dur = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U16, _heatr_hndl, _prm_heater, 
    CONFIG_SENSOR_PARAM_HEATER_DUR_KEY, CONFIG_SENSOR_PARAM_HEATER_DUR_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_heatr_conf.heatr_dur);
  paramsSetLimitsU16(_prm_heatr_dur, 10, 2000);
}

void BME68x::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  // Pressure
  if (_item1) {
    _item1->registerParameters(parent_group, CONFIG_SENSOR_PRESSURE_KEY, CONFIG_SENSOR_PRESSURE_NAME, CONFIG_SENSOR_PRESSURE_FRIENDLY);
  };
  // Temperature
  if (_item2) {
    _item2->registerParameters(parent_group, CONFIG_SENSOR_TEMP_KEY, CONFIG_SENSOR_TEMP_NAME, CONFIG_SENSOR_TEMP_FRIENDLY);
  };
  // Humidity
  if (_item3) {
    _item3->registerParameters(parent_group, CONFIG_SENSOR_HUMIDITY_KEY, CONFIG_SENSOR_HUMIDITY_NAME, CONFIG_SENSOR_HUMIDITY_FRIENDLY);
  };
  // Gas
  if (_item4) {
    _item4->registerParameters(parent_group, CONFIG_SENSOR_IAQ_KEY, CONFIG_SENSOR_IAQ_NAME, CONFIG_SENSOR_IAQ_FRIENDLY);
  };
}

/**
 * Connecting external previously created items, for example statically declared
 * */
bool BME68x::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const int numI2C, const uint8_t addrI2C, 
  BME68x_STANDBYTIME odr, BME68x_IIR_FILTER filter,
  BME68x_OVERSAMPLING osPress, BME68x_OVERSAMPLING osTemp, BME68x_OVERSAMPLING osHum,
  rSensorItem* item1, rSensorItem* item2, rSensorItem* item3, rSensorItem* item4,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  _conf.odr = odr;
  _conf.filter = filter;
  _conf.os_pres = osPress;
  _conf.os_temp = osTemp;
  _conf.os_hum = osHum;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX4::setSensorItems(item1, item2, item3, item4);
  // Start device
  return sensorStart();
}

/**
 * Get I2C parameters
 * */
int BME68x::getI2CNum()
{
  return _I2C_num;
}

uint8_t BME68x::getI2CAddress()
{
  return _I2C_address;
}

// API error handling
bool BME68x::checkApiCode(const char* api_name, int8_t rslt)
{
  switch (rslt) {
    case BME68X_OK:
      setRawStatus(SENSOR_STATUS_OK, false);
      return true;
    case BME68X_E_NULL_PTR:
      setRawStatus(SENSOR_STATUS_ERROR, false);
      rlog_e(_name, "API name [%s] error [%d]: Null pointer", api_name, rslt);
      return false;
    case BME68X_E_COM_FAIL:
      setRawStatus(SENSOR_STATUS_CONN_ERROR, false);
      rlog_e(_name, "API name [%s] error [%d]: Communication failure\r\n", api_name, rslt);
      return false;
    case BME68X_E_INVALID_LENGTH:
      setRawStatus(SENSOR_STATUS_ERROR, false);
      rlog_e(_name, "API name [%s] error [%d]: Incorrect length parameter\r\n", api_name, rslt);
      return false;
    case BME68X_E_DEV_NOT_FOUND:
      setRawStatus(SENSOR_STATUS_CONN_ERROR, false);
      rlog_e(_name, "API name [%s] error [%d]: Device not found\r\n", api_name, rslt);
      return false;
    case BME68X_E_SELF_TEST:
      setRawStatus(SENSOR_STATUS_CAL_ERROR, false);
      rlog_e(_name, "API name [%s] error [%d]: Self test error\r\n", api_name, rslt);
      return false;
    case BME68X_W_NO_NEW_DATA:
      setRawStatus(SENSOR_STATUS_NO_DATA, false);
      rlog_w(_name, "API name [%s] earning [%d]: No new data found\r\n", api_name, rslt);
      return false;
    default:
      setRawStatus(SENSOR_STATUS_ERROR, false);
      rlog_e(_name, "API name [%s] error [%d]: Unknown error code\r\n", api_name, rslt);
      return false;
  };
}

/**
 * Start device
 * */
bool BME68x::sensorReset()
{
  int8_t rslt = bme68x_init(&_dev); // bme68x_soft_reset() inline
  if (checkApiCode("bme68x_init", rslt)) {
    return sendConfiguration() && sendHeaterMode();
  };
  return false;
}

/**
 * Setting parameters
 * */
bool BME68x::sendConfiguration()
{
  rlog_i(_name, "Send configuration");
  int8_t rslt = bme68x_set_conf(&_conf, &_dev);
  return checkApiCode("bme68x_set_conf", rslt);
}

bool BME68x::sendHeaterMode()
{
  
  if ((_heatr_conf.heatr_temp == 0) || (_heatr_conf.heatr_dur == 0)) {
    _heatr_conf.enable = BME68X_DISABLE;
    rlog_i(_name, "Send heater mode: off");
  } else {
    _heatr_conf.enable = BME68X_ENABLE;
    if (_heatr_conf.heatr_temp < BME68X_LOW_TEMP) _heatr_conf.heatr_temp = BME68X_LOW_TEMP;
    if (_heatr_conf.heatr_temp > BME68X_HIGH_TEMP) _heatr_conf.heatr_temp = BME68X_HIGH_TEMP;
    rlog_i(_name, "Send heater mode: temperature = %d C, duration = %d ms", _heatr_conf.heatr_temp, _heatr_conf.heatr_dur);
  };

  int8_t rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &_heatr_conf, &_dev);
  return checkApiCode("bme68x_set_heatr_conf", rslt);
}

bool BME68x::setConfiguration(BME68x_STANDBYTIME odr, BME68x_IIR_FILTER filter,
      BME68x_OVERSAMPLING osPress, BME68x_OVERSAMPLING osTemp, BME68x_OVERSAMPLING osHum)
{
  _conf.os_pres = osPress;
  _conf.os_temp = osTemp;
  _conf.os_hum = osHum;
  _conf.filter = filter;
  _conf.odr = odr;
  return sendConfiguration();
}

bool BME68x::setOversampling(BME68x_OVERSAMPLING osPress, BME68x_OVERSAMPLING osTemp, BME68x_OVERSAMPLING osHum)
{
  _conf.os_pres = osPress;
  _conf.os_temp = osTemp;
  _conf.os_hum = osHum;
  return sendConfiguration();
};

bool BME68x::setIIRFilterSize(BME68x_IIR_FILTER filter)
{
  _conf.filter = filter;
  return sendConfiguration();
};

bool BME68x::setODR(BME68x_STANDBYTIME odr)
{
  _conf.odr = odr;
  return sendConfiguration();
};

bool BME68x::setGasHeater(uint16_t heaterTemp, uint16_t heaterTime)
{
  _heatr_conf.heatr_temp = heaterTemp;
  _heatr_conf.heatr_dur = heaterTime;

  if (_prm_heatr_temp) paramsValueStore(_prm_heatr_temp, false);
  if (_prm_heatr_dur) paramsValueStore(_prm_heatr_dur, false);

  return sendHeaterMode();
};

/**
 * Soft reset
 * */
bool BME68x::softReset()
{
  int8_t rslt = bme68x_soft_reset(&_dev);
  return checkApiCode("bme68x_soft_reset", rslt);
};

/**
 * Reading values from sensor
 * */
sensor_status_t BME68x::readRawData()
{
  int8_t rslt;
  uint32_t measure_delay;
  struct bme68x_data data;
  uint8_t n_fields;

  // Set operation mode
  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &_dev);
  if (!checkApiCode("bme68x_set_op_mode", rslt)) return _lastStatus;

  // Calculate delay period in microseconds
  measure_delay = bme68x_get_meas_dur(BME68X_FORCED_MODE, &_conf, &_dev) + (_heatr_conf.heatr_dur * 1000);
  _dev.delay_us(measure_delay, this);

  // Get data
  rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &_dev);
  if (!checkApiCode("bme68x_get_data", rslt)) return _lastStatus;

  if ((n_fields) && (data.status & BME68X_NEW_DATA_MSK) && (data.status & BME68X_GASM_VALID_MSK) && (data.status & BME68X_HEAT_STAB_MSK)) {
    setRawValues(data.pressure, data.temperature, data.humidity, data.gas_resistance);
  } else {
    setRawStatus(SENSOR_STATUS_ERROR, false);
  };

  return _lastStatus;
};
