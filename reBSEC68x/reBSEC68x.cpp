#include "reBSEC68x.h"
#include "freertos/FreeRTOS.h"
#include "rStrings.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "reParams.h"
#include "rLog.h"
#include "string.h"
#include "rom/ets_sys.h"
#include "driver/i2c.h"
#include "def_consts.h"
#include "reTgSend.h"

#define BME68X_I2C_TIMEOUT            3000
#define BSEC_TOTAL_HEAT_DUR           UINT16_C(140)
#define BSEC_CHECK_INPUT(x, shift)		(x & (1 << (shift-1)))
#define BSEC_MAX_VIRTUAL_SENSOR       5

static const char* logTAG = "BME680";

#define BSEC_CHECK_ERROR(a, msg) do { \
  bsec_library_return_t bsec_status = (a); \
  if (bsec_status != BSEC_OK) { \
    if (bsec_status > 0) { \
      rlog_w(logTAG, "Sensor [%s]: \"%s()\" completed with code %d", _name, msg, bsec_status); \
    } else { \
      rlog_e(logTAG, "Sensor [%s]: \"%s()\" failed with code %d", _name, msg, bsec_status); \
      tgSend(MK_SERVICE, MP_HIGH, true, "BME680", "Error: <b>%s</b> failed with code #<b>%d</b>", msg, bsec_status); \
      return SENSOR_STATUS_ERROR; \
    }; \
  }; \
} while (0);

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Callbacks ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static BME68X_INTF_RET_TYPE BME68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  BSEC68x* sensor = (BSEC68x*)intf_ptr;
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
  BSEC68x* sensor = (BSEC68x*)intf_ptr;
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
// ------------------------------------------------------- BME68x --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

BSEC68x::BSEC68x(uint8_t eventId):rSensorX4(eventId)
{
  _I2C_num = I2C_NUM_0;
  _I2C_address = 0;

  _filter = BME68X_FILTER_OFF;
  _odr = BME68X_ODR_125_MS;
  _os_press = BME68X_OS_4X;
  _os_temp = BME68X_OS_4X;
  _os_humd = BME68X_OS_4X;

  memset(&_dev, 0, sizeof(_dev));
  _dev.chip_id = 0;
  _dev.amb_temp = 25;
  _dev.intf_ptr = this;
  _dev.intf = BME68X_I2C_INTF;
  _dev.read = &BME68x_i2c_read;
  _dev.write = &BME68x_i2c_write;
  _dev.delay_us = &BME68x_delay_us;

  memset(&_dev_conf, 0, sizeof(_dev_conf));
  memset(&_bsec_version, 0, sizeof(_bsec_version));
  memset(&_bsec_conf, 0, sizeof(_bsec_conf));
}

// Destructor
BSEC68x::~BSEC68x()
{
  // nothing
}

// Displaying multiple values in one topic
#if CONFIG_SENSOR_DISPLAY_ENABLED

char* BSEC68x::getDisplayValue()
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

bool BSEC68x::publishCustomValues()
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

char* BSEC68x::jsonCustomValues()
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
bool BSEC68x::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const i2c_port_t numI2C, const uint8_t addrI2C, 
  BME68x_STANDBYTIME odr, BME68x_IIR_FILTER filter, BME68x_OVERSAMPLING osPress, BME68x_OVERSAMPLING osTemp, BME68x_OVERSAMPLING osHum,
  BME68x_BSEC2_OUTPUT bsec_output, float bsec_rate,
  sensor_filter_t filterMode1, uint16_t filterSize1, 
  sensor_filter_t filterMode2, uint16_t filterSize2,
  sensor_filter_t filterMode3, uint16_t filterSize3,
  sensor_filter_t filterMode4, uint16_t filterSize4,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  // Set configuration
  _odr = (uint8_t)odr;
  _filter = (uint8_t)filter;
  _os_press = (uint8_t)osPress;
  _os_temp = (uint8_t)osTemp;
  _os_humd = (uint8_t)osHum;
  // Initialize BSEC2
  _bsec_output_type = bsec_output;
  _bsec_sample_rate = bsec_rate;
   // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX4::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2, filterMode3, filterSize3, filterMode4, filterSize4)) {
    // Start device
    return sensorStart();
  };
  return false;
}

void BSEC68x::createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
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
  _item4 = new rSensorItem(this, CONFIG_SENSOR_IAQ_NAME, 
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

void BSEC68x::registerItemsParameters(paramsGroupHandle_t parent_group)
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
bool BSEC68x::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const i2c_port_t numI2C, const uint8_t addrI2C, 
  BME68x_STANDBYTIME odr, BME68x_IIR_FILTER filter, BME68x_OVERSAMPLING osPress, BME68x_OVERSAMPLING osTemp, BME68x_OVERSAMPLING osHum,
  BME68x_BSEC2_OUTPUT bsec_output, float bsec_rate,
  rSensorItem* item1, rSensorItem* item2, rSensorItem* item3, rSensorItem* item4,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  // Set configuration
  _odr = (uint8_t)odr;
  _filter = (uint8_t)filter;
  _os_press = (uint8_t)osPress;
  _os_temp = (uint8_t)osTemp;
  _os_humd = (uint8_t)osHum;
  // Initialize BSEC2
  _bsec_output_type = bsec_output;
  _bsec_sample_rate = bsec_rate;
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
i2c_port_t BSEC68x::getI2CNum()
{
  return _I2C_num;
}

uint8_t BSEC68x::getI2CAddress()
{
  return _I2C_address;
}

// API error handling
sensor_status_t BSEC68x::checkApiCode(const char* api_name, int8_t rslt)
{
  switch (rslt) {
    case BME68X_OK:
      return SENSOR_STATUS_OK;
    case BME68X_E_NULL_PTR:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Null pointer", _name, api_name, rslt);
      return SENSOR_STATUS_ERROR;
    case BME68X_E_COM_FAIL:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Communication failure\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_CONN_ERROR;
    case BME68X_E_INVALID_LENGTH:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Incorrect length parameter\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_ERROR;
    case BME68X_E_DEV_NOT_FOUND:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Device not found\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_CONN_ERROR;
    case BME68X_E_SELF_TEST:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Self test error\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_CAL_ERROR;
    case BME68X_W_NO_NEW_DATA:
      rlog_w(logTAG, "%s: API name [%s] earning [%d]: No new data found\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_NO_DATA;
    default:
      rlog_e(logTAG, "%s: API name [%s] error [%d]: Unknown error code\r\n", _name, api_name, rslt);
      return SENSOR_STATUS_ERROR;
  };
}

/**
 * Start device
 * */
sensor_status_t BSEC68x::sensorReset()
{
  sensor_status_t rslt = checkApiCode("bme68x_init", bme68x_init(&_dev)); // bme68x_soft_reset() inline
  if (rslt == SENSOR_STATUS_OK) {
    // Reset sensor configuration
    memset(&_bsec_conf, 0, sizeof(_bsec_conf));
    // Init BSEC2
    bsec_library_return_t retb = bsec_init();
    if (retb != BSEC_OK) {
      return SENSOR_STATUS_ERROR;
    };
    // Get version BSEC2
    retb = bsec_get_version(&_bsec_version);
    if (retb != BSEC_OK) {
      return SENSOR_STATUS_ERROR;
    };
    rlog_d(logTAG, "Sensor [%s]: BSEC version %d.%d.%d.%d", _name, _bsec_version.major, _bsec_version.major_bugfix, _bsec_version.minor, _bsec_version.minor_bugfix);
    // Update subscriptions
    rslt = updateSubscription();
    // Initial configuration
    setInitConfiguration();
  };
  return rslt;
}

/**
 * Sensor parameters
 * */
uint32_t BSEC68x::getMeasureDuration(uint8_t op_mode)
{
	if (op_mode == BME68X_SLEEP_MODE)
		op_mode = _bsec_conf.op_mode;

	return bme68x_get_meas_dur(op_mode, &_dev_conf, &_dev);
}

sensor_status_t BSEC68x::setFilterAndOdr(uint8_t new_filter, uint8_t new_odr)
{

  int8_t retcode = bme68x_get_conf(&_dev_conf, &_dev);
  if (retcode != BME68X_OK) {
    return checkApiCode("bme68x_get_conf",  retcode);
  };

  _dev_conf.filter = new_filter;
  _dev_conf.odr = new_odr;
  
  retcode = bme68x_set_conf(&_dev_conf, &_dev);
  return checkApiCode("bme68x_set_conf",  retcode);
}

sensor_status_t BSEC68x::setInitConfiguration()
{
  _dev_conf.filter = _filter;
  _dev_conf.odr = _odr;
  _dev_conf.os_pres = _os_press;
  _dev_conf.os_temp = _os_temp;
  _dev_conf.os_hum = _os_humd;

  int8_t retcode = bme68x_set_conf(&_dev_conf, &_dev);
  if (retcode != BME68X_OK) {
    return checkApiCode("bme68x_set_conf", retcode);
  };

  retcode = bme68x_set_op_mode(BME68X_SLEEP_MODE, &_dev);
  return checkApiCode("bme68x_set_op_mode", retcode);
}

sensor_status_t BSEC68x::setBsecConfiguration()
{
  int8_t retcode = BME68X_OK;

  if (_bsec_conf.op_mode != BME68X_SLEEP_MODE) {
    retcode = bme68x_get_conf(&_dev_conf, &_dev);
    if (retcode != BME68X_OK) {
      return checkApiCode("bme68x_get_conf", retcode);
    };

    // Set filters
    bool configChanged = false;
    if (_dev_conf.filter != _filter) {
      _dev_conf.filter = _filter;
      configChanged = true;
    };
    if (_dev_conf.odr != _odr) {
      _dev_conf.odr = _odr;
      configChanged = true;
    };

    // Set oversamplings
    // At low oversampling values, the sensor may return incorrect data, especially pressure
    if (_bsec_conf.pressure_oversampling > _os_press) {
      if (_bsec_conf.pressure_oversampling != _dev_conf.os_pres) {
        _dev_conf.os_pres = _bsec_conf.pressure_oversampling;
        configChanged = true;
      };
    } else {
      if (_os_press != _dev_conf.os_pres) {
        _dev_conf.os_pres = _os_press;
        configChanged = true;
      };
    };
    if (_bsec_conf.temperature_oversampling > _os_temp) {
      if (_bsec_conf.temperature_oversampling != _dev_conf.os_temp) {
        _dev_conf.os_temp = _bsec_conf.temperature_oversampling;
        configChanged = true;
      };
    } else {
      if (_os_temp != _dev_conf.os_temp) {
        _dev_conf.os_temp = _os_temp;
        configChanged = true;
      };
    };
    if (_bsec_conf.humidity_oversampling > _os_humd) {
      if (_bsec_conf.humidity_oversampling != _dev_conf.os_hum) {
        _dev_conf.os_hum = _bsec_conf.humidity_oversampling;
        configChanged = true;
      };
    } else {
      if (_os_humd != _dev_conf.os_hum) {
        _dev_conf.os_hum = _os_humd;
        configChanged = true;
      };
    };

    // Send the configuration to the sensor if it has been changed
    if (configChanged) {
      retcode = bme68x_set_conf(&_dev_conf, &_dev);
      if (retcode != BME68X_OK) {
        return checkApiCode("bme68x_set_conf", retcode);
      };
    };

    // The heater is fully controlled by BSEC, without our intervention
    bme68x_heatr_conf heatr_conf;
    memset(&heatr_conf, 0, sizeof(heatr_conf));

    if (_bsec_conf.run_gas == 1) {
      heatr_conf.enable = BME68X_ENABLE;
      heatr_conf.heatr_temp = _bsec_conf.heater_temperature;
      heatr_conf.heatr_temp_prof = _bsec_conf.heater_temperature_profile;
      heatr_conf.heatr_dur = _bsec_conf.heater_duration;
      heatr_conf.heatr_dur_prof = _bsec_conf.heater_duration_profile;
      heatr_conf.profile_len = _bsec_conf.heater_profile_len;
      if (_bsec_conf.op_mode == BME68X_PARALLEL_MODE) {
        heatr_conf.shared_heatr_dur = BSEC_TOTAL_HEAT_DUR - (getMeasureDuration(BME68X_PARALLEL_MODE) / INT64_C(1000));
      };
    } else {
      heatr_conf.enable = BME68X_DISABLE;
    };

    retcode = bme68x_set_heatr_conf(_bsec_conf.op_mode, &heatr_conf, &_dev);
    if (retcode != BME68X_OK) {
      return checkApiCode("bme68x_set_heatr_conf", retcode);
    };
  };

  // Set operation mode
  retcode = bme68x_set_op_mode(_bsec_conf.op_mode, &_dev);
  return checkApiCode("bme68x_set_op_mode", retcode);
}

/**
 * BSEC
 * */
sensor_status_t BSEC68x::updateSubscription()
{
  uint8_t numPhysical = BSEC_MAX_PHYSICAL_SENSOR;
  bsec_sensor_configuration_t sensorsPhysical[BSEC_MAX_PHYSICAL_SENSOR];
  uint8_t numVirtual = BSEC_MAX_VIRTUAL_SENSOR;
  bsec_sensor_configuration_t sensorsVirtual[BSEC_MAX_VIRTUAL_SENSOR];
  
  switch (_bsec_output_type) {
    case BME68x_BSEC2_GAS_PERCENTAGE:
      sensorsVirtual[0].sensor_id = BSEC_OUTPUT_GAS_PERCENTAGE;
      sensorsVirtual[0].sample_rate = _bsec_sample_rate;
      break;
    case BME68x_BSEC2_IAQ:
      sensorsVirtual[0].sensor_id = BSEC_OUTPUT_IAQ;
      sensorsVirtual[0].sample_rate = _bsec_sample_rate;
      break;
    case BME68x_BSEC2_STATIC_IAQ:
      sensorsVirtual[0].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
      sensorsVirtual[0].sample_rate = _bsec_sample_rate;
      break;
    case BME68x_BSEC2_CO2_EQUIVALENT:
      sensorsVirtual[0].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
      sensorsVirtual[0].sample_rate = _bsec_sample_rate;
      break;
    case BME68x_BSEC2_BREATH_VOC_EQUIVALENT:
      sensorsVirtual[0].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
      sensorsVirtual[0].sample_rate = _bsec_sample_rate;
      break;
    default:
      sensorsVirtual[0].sensor_id = BSEC_OUTPUT_IAQ;
      sensorsVirtual[0].sample_rate = BSEC_SAMPLE_RATE_DISABLED;
      break;
  };
  sensorsVirtual[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
  sensorsVirtual[1].sample_rate = _bsec_sample_rate;
  sensorsVirtual[2].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
  sensorsVirtual[2].sample_rate = _bsec_sample_rate;
  sensorsVirtual[3].sensor_id = BSEC_OUTPUT_STABILIZATION_STATUS;
  sensorsVirtual[3].sample_rate = _bsec_sample_rate;
  sensorsVirtual[4].sensor_id = BSEC_OUTPUT_RUN_IN_STATUS;
  sensorsVirtual[4].sample_rate = _bsec_sample_rate;

  BSEC_CHECK_ERROR(bsec_update_subscription(sensorsVirtual, numVirtual, sensorsPhysical, &numPhysical), "bsec_update_subscription");
  return SENSOR_STATUS_OK;
}

/**
 * Reading values from sensor
 * */
sensor_status_t BSEC68x::readRawData()
{
  sensor_status_t rslt;
  struct bme68x_data data;
  uint8_t last_op_mode;
  uint8_t n_reads, n_inputs, n_outputs;
  bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
  bsec_output_t outputs[BSEC_MAX_VIRTUAL_SENSOR];

  last_op_mode = _bsec_conf.op_mode;
  int64_t curr_time_ms = esp_timer_get_time();
  if ((_bsec_conf.next_call / INT64_C(1000) - curr_time_ms) < 0) {
    // Read sensor configuration from BSEC2
    // Exampled: op_mode=1, next_call=+3000000ms, heater_temp=320, heater_dur=197, process_data=8206, run_gas=1, trigger_measurement=1
    BSEC_CHECK_ERROR(bsec_sensor_control(curr_time_ms * INT64_C(1000), &_bsec_conf), "bsec_sensor_control");
    
    // Set configuration and operation mode
    if ((_bsec_conf.op_mode == BME68X_FORCED_MODE) || (last_op_mode != _bsec_conf.op_mode)) {
      rslt = setBsecConfiguration();
      if (rslt != SENSOR_STATUS_OK) return rslt;
    };

    // Waiting for the end of all measurements
    uint32_t measure_delay = bme68x_get_meas_dur(_bsec_conf.op_mode, &_dev_conf, &_dev);
    _dev.delay_us(measure_delay, this);

    // Read RAW data from sensor
    if (_bsec_conf.trigger_measurement && (_bsec_conf.op_mode != BME68X_SLEEP_MODE)) {
      rslt = checkApiCode("bme68x_get_data", bme68x_get_data(_bsec_conf.op_mode, &data, &n_reads, &_dev));
      if (rslt != SENSOR_STATUS_OK) return rslt;
      if (n_reads == 0) return SENSOR_STATUS_NO_DATA;

      // Prepare RAW data for BSEC
      n_inputs = 0;
      if (BSEC_CHECK_INPUT(_bsec_conf.process_data, BSEC_INPUT_PRESSURE)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[n_inputs].signal = data.pressure;
        inputs[n_inputs].time_stamp = curr_time_ms * INT64_C(1000);
        n_inputs++;
      };
      if (BSEC_CHECK_INPUT(_bsec_conf.process_data, BSEC_INPUT_HUMIDITY)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
        inputs[n_inputs].signal = data.humidity;
        inputs[n_inputs].time_stamp = curr_time_ms * INT64_C(1000);
        n_inputs++;
      };
      if (BSEC_CHECK_INPUT(_bsec_conf.process_data, BSEC_INPUT_TEMPERATURE)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[n_inputs].signal = extTempOffset;
        inputs[n_inputs].time_stamp = curr_time_ms * INT64_C(1000);
        n_inputs++;
        inputs[n_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
        inputs[n_inputs].signal = data.temperature;
        inputs[n_inputs].time_stamp = curr_time_ms * INT64_C(1000);
        n_inputs++;
      };
      if (BSEC_CHECK_INPUT(_bsec_conf.process_data, BSEC_INPUT_GASRESISTOR) && (data.status & BME68X_GASM_VALID_MSK)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[n_inputs].signal = data.gas_resistance;
        inputs[n_inputs].time_stamp = curr_time_ms * INT64_C(1000);
        n_inputs++;
      };
      if (BSEC_CHECK_INPUT(_bsec_conf.process_data, BSEC_INPUT_PROFILE_PART) && (data.status & BME68X_GASM_VALID_MSK)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[n_inputs].signal = (_bsec_conf.op_mode == BME68X_FORCED_MODE) ? 0 : data.gas_index;
        inputs[n_inputs].time_stamp = curr_time_ms * INT64_C(1000);
        n_inputs++;
      };

      // Process BSEC virtual data
      if (n_inputs > 0) {
        n_outputs = BSEC_MAX_VIRTUAL_SENSOR;
        memset(outputs, 0, sizeof(outputs));
        BSEC_CHECK_ERROR(bsec_do_steps(inputs, n_inputs, outputs, &n_outputs), "bsec_do_steps");
        
        // Get BSEC virtual data
        value_t out_pressure = data.pressure;
        value_t out_temperature = data.temperature;
        value_t out_humidity = data.humidity;
        value_t out_iaq = 0.0;
        for (uint8_t i = 0; i < n_outputs; i++) {
          if (outputs[i].sensor_id == BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE) {
            out_temperature = outputs[i].signal;
          } else if (outputs[i].sensor_id == BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY) {
            out_humidity = outputs[i].signal;
          } else if ((outputs[i].sensor_id == BSEC_OUTPUT_GAS_PERCENTAGE) && (_bsec_output_type == BME68x_BSEC2_GAS_PERCENTAGE)) {
            out_iaq = outputs[i].signal;
          } else if ((outputs[i].sensor_id == BSEC_OUTPUT_IAQ) && (_bsec_output_type == BME68x_BSEC2_IAQ)) {
            out_iaq = outputs[i].signal;
          } else if ((outputs[i].sensor_id == BSEC_OUTPUT_STATIC_IAQ) && (_bsec_output_type == BME68x_BSEC2_STATIC_IAQ)) {
            out_iaq = outputs[i].signal;
          } else if ((outputs[i].sensor_id == BSEC_OUTPUT_CO2_EQUIVALENT) && (_bsec_output_type == BME68x_BSEC2_CO2_EQUIVALENT)) {
            out_iaq = outputs[i].signal;
          } else if ((outputs[i].sensor_id == BSEC_OUTPUT_BREATH_VOC_EQUIVALENT) && (_bsec_output_type == BME68x_BSEC2_BREATH_VOC_EQUIVALENT)) {
            out_iaq = outputs[i].signal;
          };
        };
        
        time_t timestamp = time(nullptr);
        _item1->setRawValue(data.pressure, timestamp);
        _item2->setRawAndConvertedValue(data.temperature, out_temperature, timestamp);
        _item3->setRawAndConvertedValue(data.humidity, out_humidity, timestamp);
        _item4->setRawAndConvertedValue(data.gas_resistance, out_iaq, timestamp);
        return SENSOR_STATUS_OK;
      };
    };
  };
  return SENSOR_STATUS_NO_DATA;
};
