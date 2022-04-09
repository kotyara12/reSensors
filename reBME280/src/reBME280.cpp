#include "reBME280.h"
#include "freertos/FreeRTOS.h"
#include "rStrings.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "reParams.h"
#include "rLog.h"
#include "string.h"
#include "driver/i2c.h"
#include "def_consts.h"

#define BME280_I2C_TIMEOUT    3000

static const char* logTAG = "BME280";

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Callbacks ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static BME280_INTF_RET_TYPE BME280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  BME280* sensor = (BME280*)intf_ptr;
  if (sensor) {
    esp_err_t err = readI2C(sensor->getI2CNum(), sensor->getI2CAddress(), &reg_addr, sizeof(reg_addr), reg_data, length, 0, BME280_I2C_TIMEOUT); 
    if (err == ESP_OK) {
      return BME280_OK;
    } else {
      return BME280_E_COMM_FAIL;
    };
  };
  return BME280_E_NULL_PTR;
}

static BME280_INTF_RET_TYPE BME280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  BME280* sensor = (BME280*)intf_ptr;
  if (sensor) {
    esp_err_t err = writeI2C(sensor->getI2CNum(), sensor->getI2CAddress(), &reg_addr, sizeof(reg_addr), (uint8_t*)reg_data, length, BME280_I2C_TIMEOUT); 
    if (err == ESP_OK) {
      return BME280_OK;
    } else {
      return BME280_E_COMM_FAIL;
    };
  };
  return BME280_E_NULL_PTR;
}

static void BME280_delay_us(uint32_t period, void *intf_ptr)
{
  ets_delay_us(period);
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------- BME280 --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

BME280::BME280():rSensorX3()
{
  _I2C_num = 0;
  _I2C_address = 0;
  memset(&_dev, 0, sizeof(_dev));
}

// Destructor
BME280::~BME280()
{
}

/**
 * Dynamically creating internal items on the heap
 * */
bool BME280::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const int numI2C, const uint8_t addrI2C, 
  BME280_MODE mode, BME280_STANDBYTIME odr, BME280_IIR_FILTER filter,
  BME280_OVERSAMPLING osPress, BME280_OVERSAMPLING osTemp, BME280_OVERSAMPLING osHumd,
  sensor_filter_t filterMode1, uint16_t filterSize1, 
  sensor_filter_t filterMode2, uint16_t filterSize2,
  sensor_filter_t filterMode3, uint16_t filterSize3,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX3::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2, filterMode3, filterSize3)) {
    // Start device
    return (initHardware(numI2C, addrI2C) && setConfiguration(mode, odr, filter, osPress, osTemp, osHumd));
  };
  return false;
}

void BME280::createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                               const sensor_filter_t filterMode2, const uint16_t filterSize2,
                               const sensor_filter_t filterMode3, const uint16_t filterSize3)
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
}

void BME280::registerItemsParameters(paramsGroupHandle_t parent_group)
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
}

/**
 * Connecting external previously created items, for example statically declared
 * */
bool BME280::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const int numI2C, const uint8_t addrI2C, 
  BME280_MODE mode, BME280_STANDBYTIME odr, BME280_IIR_FILTER filter,
  BME280_OVERSAMPLING osPress, BME280_OVERSAMPLING osTemp, BME280_OVERSAMPLING osHumd,
  rSensorItem* item1, rSensorItem* item2, rSensorItem* item3,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX3::setSensorItems(item1, item2, item3);
  // Start device
  return (initHardware(numI2C, addrI2C) && setConfiguration(mode, odr, filter, osPress, osTemp, osHumd));
}

/**
 * Get I2C parameters
 * */
int BME280::getI2CNum()
{
  return _I2C_num;
}

uint8_t BME280::getI2CAddress()
{
  return _I2C_address;
}

// Displaying multiple values in one topic
#if CONFIG_SENSOR_DISPLAY_ENABLED

char* BME280::getDisplayValue()
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

bool BME280::publishCustomValues()
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

char* BME280::jsonCustomValues()
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

// API error handling
bool BME280::checkApiCode(const char* api_name, int8_t rslt)
{
  switch (rslt) {
    case BME280_OK:
      setRawStatus(SENSOR_STATUS_OK, false);
      return true;
    case BME280_E_NULL_PTR:
      setRawStatus(SENSOR_STATUS_ERROR, false);
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Null pointer", _name, api_name, rslt);
      return false;
    case BME280_E_COMM_FAIL:
      setRawStatus(SENSOR_STATUS_TIMEOUT, false);
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Communication failure\r\n", _name, api_name, rslt);
      return false;
    case BME280_E_INVALID_LEN:
      setRawStatus(SENSOR_STATUS_ERROR, false);
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Incorrect length parameter\r\n", _name, api_name, rslt);
      return false;
    case BME280_E_DEV_NOT_FOUND:
      setRawStatus(SENSOR_STATUS_TIMEOUT, false);
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Device not found\r\n", _name, api_name, rslt);
      return false;
    case BME280_E_NVM_COPY_FAILED:
      setRawStatus(SENSOR_STATUS_CAL_ERROR, false);
      rlog_w(logTAG, "%s: API name [%s] earning [%d]: Invalid temperature\r\n", _name, api_name, rslt);
      return false;
    default:
      setRawStatus(SENSOR_STATUS_ERROR, false);
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Unknown error code\r\n", _name, api_name, rslt);
      return false;
  };
}

/**
 * Start device
 * */
bool BME280::initHardware(const int numI2C, const uint8_t addrI2C)
{
  int8_t rslt;

  _dev.chip_id = 0;
  _dev.intf_ptr = this;
  _dev.intf = BME280_I2C_INTF;
  _dev.read = &BME280_i2c_read;
  _dev.write = &BME280_i2c_write;
  _dev.delay_us = &BME280_delay_us;
  _mode = BME280_MODE_SLEEP;

  rslt = bme280_init(&_dev);
  return checkApiCode("bme280_init", rslt);
}

/**
 * Setting parameters
 * */
bool BME280::sendPowerMode(BME280_MODE mode)
{
  rlog_i(_name, "Send power mode");
  int8_t rslt = bme280_set_sensor_mode(mode, &_dev);
  if (rslt == BME280_OK) { _mode = mode; };
  return checkApiCode("bme280_set_sensor_mode", rslt);
}

bool BME280::sendConfiguration(uint8_t settings_sel)
{
  rlog_i(_name, "Send configuration");
  int8_t rslt = bme280_set_sensor_settings(settings_sel, &_dev);
  if (rslt == BME280_OK) { _mode = BME280_MODE_SLEEP; };
  return checkApiCode("bme280_set_sensor_settings", rslt);
}

bool BME280::setConfiguration(BME280_MODE mode, 
  BME280_STANDBYTIME odr, BME280_IIR_FILTER filter,
  BME280_OVERSAMPLING osPress, BME280_OVERSAMPLING osTemp, BME280_OVERSAMPLING osHumd)
{
  _dev.settings.osr_p = osPress;
  _dev.settings.osr_t = osTemp;
  _dev.settings.osr_h = osHumd;
  _dev.settings.filter = filter;
  _dev.settings.standby_time = odr;
  return sendConfiguration(BME280_ALL_SETTINGS_SEL) && sendPowerMode(mode);
}

bool BME280::setOversampling(BME280_OVERSAMPLING osPress, BME280_OVERSAMPLING osTemp, BME280_OVERSAMPLING osHumd)
{
  _dev.settings.osr_h = osPress;
  _dev.settings.osr_h = osTemp;
  _dev.settings.osr_h = osHumd;
  return sendConfiguration(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL);
};

bool BME280::setIIRFilterSize(BME280_IIR_FILTER filter)
{
  _dev.settings.filter = filter;
  return sendConfiguration(BME280_FILTER_SEL);
};

bool BME280::setODR(BME280_STANDBYTIME odr)
{
  _dev.settings.standby_time = odr;
  return sendConfiguration(BME280_STANDBY_SEL);
};

/**
 * Soft reset
 * */
bool BME280::softReset()
{
  int8_t rslt = bme280_soft_reset(&_dev);
  return checkApiCode("bme280_soft_reset", rslt);
};

/**
 * Reading values from sensor
 * */
sensor_status_t BME280::readRawData()
{
  int8_t rslt;
  struct bme280_data comp_data;

  // Send a request for FORCED mode if the previously configured mode is different from cyclic (NORMAL)
  if (_mode != BME280_MODE_NORMAL) {
    rslt = bme280_set_sensor_mode(BME280_MODE_FORCED, &_dev);
    if (!checkApiCode("bme280_set_sensor_mode", rslt)) return _lastStatus; 
    _mode = BME280_MODE_FORCED;

    // Calculate the minimum delay required between consecutive measurement based upon the sensor enabled and the oversampling configuration
    _dev.delay_us(bme280_cal_meas_delay(&_dev.settings), this);
  };

  // Reading the raw data from sensor
  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &_dev);
  if (!checkApiCode("bme280_get_sensor_data", rslt)) return _lastStatus;

  setRawValues(comp_data.pressure, comp_data.temperature, comp_data.humidity);

  return _lastStatus;
};
