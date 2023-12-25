#include "reBMP280.h"
#include "freertos/FreeRTOS.h"
#include "rStrings.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "reParams.h"
#include "rLog.h"
#include "string.h"
#include "driver/i2c.h"
#include "rom/ets_sys.h"
#include "def_consts.h"

#define BMP280_I2C_TIMEOUT    3000

static const char* logTAG = "BMP280";

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Callbacks ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static int8_t BMP280_i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  return readI2C((i2c_port_t)(i2c_addr >> 7), i2c_addr & 0x7F, &reg_addr, sizeof(reg_addr), reg_data, length, 0, BMP280_I2C_TIMEOUT); 
}

static int8_t BMP280_i2c_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  return writeI2C((i2c_port_t)(i2c_addr >> 7), i2c_addr & 0x7F, &reg_addr, sizeof(reg_addr), (uint8_t*)reg_data, length, BMP280_I2C_TIMEOUT); 
}

static void BMP280_delay_ms(uint32_t period_ms)
{
  ets_delay_us(period_ms*1000);
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------- BMP280 --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

BMP280::BMP280(uint8_t eventId):rSensorX2(eventId)
{
  _I2C_num = 0;
  _I2C_address = 0;
  _meas_wait = 16;
  memset(&_dev, 0, sizeof(_dev));
  memset(&_conf, 0, sizeof(_conf));

  _dev.chip_id = 0;
  _dev.intf = BMP280_I2C_INTF;
  _dev.read = &BMP280_i2c_read;
  _dev.write = &BMP280_i2c_write;
  _dev.delay_ms = &BMP280_delay_ms;
}

// Destructor
BMP280::~BMP280()
{
}

/**
 * Dynamically creating internal items on the heap
 * */
bool BMP280::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const int numI2C, const uint8_t addrI2C, 
  BMP280_MODE mode, BMP280_STANDBYTIME odr, BMP280_IIR_FILTER filter,
  BMP280_OVERSAMPLING osPress, BMP280_OVERSAMPLING osTemp,
  sensor_filter_t filterMode1, uint16_t filterSize1, 
  sensor_filter_t filterMode2, uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  _mode = mode;
  _conf.odr = odr;
  _conf.filter = filter;
  _conf.os_pres = osPress;
  _conf.os_temp = osTemp;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX2::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2)) {
    // Start device
    return sensorStart();
  };
  return false;
}

/**
 * Connecting external previously created items, for example statically declared
 * */
bool BMP280::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const int numI2C, const uint8_t addrI2C, 
  BMP280_MODE mode, BMP280_STANDBYTIME odr, BMP280_IIR_FILTER filter,
  BMP280_OVERSAMPLING osPress, BMP280_OVERSAMPLING osTemp,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  _mode = mode;
  _conf.odr = odr;
  _conf.filter = filter;
  _conf.os_pres = osPress;
  _conf.os_temp = osTemp;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX2::setSensorItems(item1, item2);
  // Start device
  return sensorStart();
}

void BMP280::createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                               const sensor_filter_t filterMode2, const uint16_t filterSize2)
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
}

void BMP280::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  // Pressure
  if (_item1) {
    _item1->registerParameters(parent_group, CONFIG_SENSOR_PRESSURE_KEY, CONFIG_SENSOR_PRESSURE_NAME, CONFIG_SENSOR_PRESSURE_FRIENDLY);
  };
  // Temperature
  if (_item2) {
    _item2->registerParameters(parent_group, CONFIG_SENSOR_TEMP_KEY, CONFIG_SENSOR_TEMP_NAME, CONFIG_SENSOR_TEMP_FRIENDLY);
  };
}

// Displaying multiple values in one topic
#if CONFIG_SENSOR_DISPLAY_ENABLED

char* BMP280::getDisplayValue()
{
  char* ret = nullptr;
  if (_item1) { 
    ret = _item1->getStringFiltered(); 
  };
  if (_item2) {
    ret = concat_strings_div(ret, _item2->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

// API error handling
sensor_status_t BMP280::checkApiCode(const char* api_name, int8_t rslt)
{
  switch (rslt) {
    case BMP280_OK:
      return SENSOR_STATUS_OK;
    case BMP280_E_NULL_PTR:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Null pointer", _name, api_name, rslt);
      return SENSOR_STATUS_ERROR;
    case BMP280_E_COMM_FAIL:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Communication failure\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_CONN_ERROR;
    case BMP280_E_INVALID_LEN:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Incorrect length parameter\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_ERROR;
    case BMP280_E_DEV_NOT_FOUND:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Device not found\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_CONN_ERROR;
    case BMP280_E_INVALID_MODE:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Invalid mode\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_NOT_SUPPORTED;
    case BMP280_E_IMPLAUS_TEMP:
    case BMP280_E_IMPLAUS_PRESS:
      rlog_w(logTAG, "%s: API name [%s] earning [%d]: Invalid temperature\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_NO_DATA;
    case BMP280_E_CAL_PARAM_RANGE:
      rlog_w(logTAG, "%s: API name [%s] earning [%d]: Invalid temperature\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_CAL_ERROR;
    default:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Unknown error code\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_ERROR;
  };
}

/**
 * Start device
 * */
sensor_status_t BMP280::sensorReset()
{
  _dev.dev_id = (_I2C_num << 7) | (_I2C_address & 0x7F);
  _mode = BMP280_MODE_SLEEP;

  sensor_status_t rslt = checkApiCode("bmp280_init", bmp280_init(&_dev));
  if (rslt == SENSOR_STATUS_OK) {
    rslt = sendConfiguration();
    if (rslt == SENSOR_STATUS_OK) {
      rslt = sendPowerMode(_mode);
    };
  };
  return rslt;
}

/**
 * Setting parameters
 * */
uint8_t BMP280::osr2int(BMP280_OVERSAMPLING osr)
{
  switch (osr) {
    case BMP280_OSM_X1:  return 1;
    case BMP280_OSM_X2:  return 2;
    case BMP280_OSM_X4:  return 4;
    case BMP280_OSM_X8:  return 8;
    case BMP280_OSM_X16: return 16;
    default: return 0;
  };
}

sensor_status_t BMP280::sendPowerMode(BMP280_MODE mode)
{
  rlog_i(logTAG, RSENSOR_LOG_MSG_SET_MODE_HEADER, _name, mode);
  int8_t rslt = bmp280_set_power_mode(mode, &_dev);
  if (rslt == BMP280_OK) { _mode = mode; };
  return checkApiCode("bmp280_set_power_mode", rslt);
}

sensor_status_t BMP280::sendConfiguration()
{
  rlog_i(logTAG, RSENSOR_LOG_MSG_SEND_CONFIG, _name);
  uint8_t _meas_wait_pres = osr2int((BMP280_OVERSAMPLING)_conf.os_pres);
  uint8_t _meas_wait_temp = osr2int((BMP280_OVERSAMPLING)_conf.os_temp);
  _meas_wait_pres < _meas_wait_temp ? _meas_wait = _meas_wait_temp : _meas_wait = _meas_wait_pres;
  int8_t rslt = bmp280_set_config(&_conf, &_dev);
  if (rslt == BMP280_OK) { _mode = BMP280_MODE_SLEEP; };
  return checkApiCode("bmp280_set_config", rslt);
}

bool BMP280::setConfiguration(BMP280_MODE mode, 
  BMP280_STANDBYTIME odr, BMP280_IIR_FILTER filter,
  BMP280_OVERSAMPLING osPress, BMP280_OVERSAMPLING osTemp)
{
  _conf.odr = odr;
  _conf.filter = filter;
  _conf.os_pres = osPress;
  _conf.os_temp = osTemp;
  if (sendConfiguration() == SENSOR_STATUS_OK) {
    return (sendPowerMode(mode) == SENSOR_STATUS_OK);
  };
  return false;
}

bool BMP280::setOversampling(BMP280_OVERSAMPLING osPress, BMP280_OVERSAMPLING osTemp)
{
  _conf.os_pres = osPress;
  _conf.os_temp = osTemp;
  return (sendConfiguration() == SENSOR_STATUS_OK);
};

bool BMP280::setIIRFilterSize(BMP280_IIR_FILTER filter)
{
  _conf.filter = filter;
  return (sendConfiguration() == SENSOR_STATUS_OK);
};

bool BMP280::setODR(BMP280_STANDBYTIME odr)
{
  _conf.odr = odr;
  return (sendConfiguration() == SENSOR_STATUS_OK);
};

/**
 * Reading values from sensor
 * */
sensor_status_t BMP280::readRawData()
{
  sensor_status_t rslt;
  struct bmp280_uncomp_data ucomp_data;
  double temperature, pressure;

  // Send a request for FORCED mode if the previously configured mode is different from cyclic (NORMAL)
  if (_mode != BMP280_MODE_NORMAL) {
    rslt = checkApiCode("bmp280_set_power_mode", bmp280_set_power_mode(BMP280_MODE_FORCED, &_dev));
    if (rslt != SENSOR_STATUS_OK) return rslt;
    _mode = BMP280_MODE_FORCED;

    // Calculate the minimum delay required between consecutive measurement based upon the sensor enabled and the oversampling configuration
    _dev.delay_ms(bmp280_compute_meas_time(&_dev));
  };

  // Reading the raw data from sensor
  rslt = checkApiCode("bmp280_get_uncomp_data", bmp280_get_uncomp_data(&ucomp_data, &_dev));
  if (rslt != SENSOR_STATUS_OK) return rslt;

  // Getting the compensated pressure
  rslt = checkApiCode("bmp280_get_comp_pres_double", bmp280_get_comp_pres_double(&pressure, ucomp_data.uncomp_press, &_dev));
  if (rslt != SENSOR_STATUS_OK) return rslt;

  // Getting the compensated temperature
  rslt = checkApiCode("bmp280_get_comp_temp_double", bmp280_get_comp_temp_double(&temperature, ucomp_data.uncomp_temp, &_dev));
  if (rslt != SENSOR_STATUS_OK) return rslt;

  if (_meas_wait > 0) {
    _meas_wait--;
    return SENSOR_STATUS_NO_DATA;
  };
  return setRawValues(pressure, temperature);
};
