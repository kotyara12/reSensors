#include "reCCS811.h"
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

static const char* logTAG = "CCS811";

// CCS811 driver error codes ORed with error codes for I2C the interface
#define CCS811_ERR_BASE            0xa000
#define CCS811_ERR_BOOT_MODE       (CCS811_ERR_BASE + 1)  // firmware is in boot mode
#define CCS811_ERR_NO_APP          (CCS811_ERR_BASE + 2)  // no application firmware loaded
#define CCS811_ERR_NO_NEW_DATA     (CCS811_ERR_BASE + 3)  // no new data samples are ready
#define CCS811_ERR_NO_IAQ_DATA     (CCS811_ERR_BASE + 4)  // no new data samples are ready
#define CCS811_ERR_HW_ID           (CCS811_ERR_BASE + 5)  // wrong hardware ID
#define CCS811_ERR_INV_SENS        (CCS811_ERR_BASE + 6)  // invalid sensor ID
#define CCS811_ERR_WR_REG_INV      (CCS811_ERR_BASE + 7)  // invalid register addr on write
#define CCS811_ERR_RD_REG_INV      (CCS811_ERR_BASE + 8)  // invalid register addr on read
#define CCS811_ERR_MM_INV          (CCS811_ERR_BASE + 9)  // invalid measurement mode
#define CCS811_ERR_MAX_RESIST      (CCS811_ERR_BASE + 10) // max sensor resistance reached
#define CCS811_ERR_HEAT_FAULT      (CCS811_ERR_BASE + 11) // heater current not in range
#define CCS811_ERR_HEAT_SUPPLY     (CCS811_ERR_BASE + 12) // heater voltage not correct
#define CCS811_ERR_WRONG_MODE      (CCS811_ERR_BASE + 13) // wrong measurement mode
#define CCS811_ERR_RD_STAT_FAILED  (CCS811_ERR_BASE + 14) // read status register failed
#define CCS811_ERR_RD_DATA_FAILED  (CCS811_ERR_BASE + 15) // read sensor data failed
#define CCS811_ERR_APP_START_FAIL  (CCS811_ERR_BASE + 16) // sensor app start failure
#define CCS811_ERR_WRONG_PARAMS    (CCS811_ERR_BASE + 17) // wrong parameters used

// ranges
#define CCS811_ECO2_RANGE_MIN      400
#define CCS811_ECO2_RANGE_MAX      8192
#define CCS811_TVOC_RANGE_MIN      0
#define CCS811_TVOC_RANGE_MAX      1187

#define CCS811_I2C_TIMEOUT         3000

// CCS811 register addresses
#define CCS811_REG_STATUS          0x00
#define CCS811_REG_MEAS_MODE       0x01
#define CCS811_REG_ALG_RESULT_DATA 0x02
#define CCS811_REG_RAW_DATA        0x03
#define CCS811_REG_ENV_DATA        0x05
#define CCS811_REG_NTC             0x06
#define CCS811_REG_THRESHOLDS      0x10
#define CCS811_REG_BASELINE        0x11

#define CCS811_REG_HW_ID           0x20
#define CCS811_REG_HW_VER          0x21
#define CCS811_REG_FW_BOOT_VER     0x23
#define CCS811_REG_FW_APP_VER      0x24

#define CCS811_REG_ERROR_ID        0xe0

#define CCS811_REG_APP_ERASE       0xf1
#define CCS811_REG_APP_DATA        0xf2
#define CCS811_REG_APP_VERIFY      0xf3
#define CCS811_REG_APP_START       0xf4
#define CCS811_REG_SW_RESET        0xff

// status register bits
#define CCS811_STATUS_ERROR        0x01  // error, details in CCS811_REG_ERROR
#define CCS811_STATUS_DATA_RDY     0x08  // new data sample in ALG_RESULT_DATA
#define CCS811_STATUS_APP_VALID    0x10  // valid application firmware loaded
#define CCS811_STATUS_FW_MODE      0x80  // firmware is in application mode

// error register bits
#define CCS811_ERR_WRITE_REG_INV   0x01  // invalid register address on write
#define CCS811_ERR_READ_REG_INV    0x02  // invalid register address on read
#define CCS811_ERR_MEASMODE_INV    0x04  // invalid requested measurement mode
#define CCS811_ERR_MAX_RESISTANCE  0x08  // maximum sensor resistance exceeded 
#define CCS811_ERR_HEATER_FAULT    0x10  // heater current not in range
#define CCS811_ERR_HEATER_SUPPLY   0x20  // heater voltage not applied correctly

// Timings
#define CCS811_WAIT_RESET_MS       100   // The CCS811 needs a wait after reset
#define CCS811_WAIT_APPSTART_MS    100   // The CCS811 needs a wait after app start
#define CCS811_WAIT_MODE_MS        100   // The CCS811 needs a wait after set new mode
#define CCS811_WAIT_WAKE_US        50    // The CCS811 needs a wait after WAKE signal
#define CCS811_WAIT_APPERASE_MS    500   // The CCS811 needs a wait after app erase (300ms from spec not enough)
#define CCS811_WAIT_APPVERIFY_MS   70    // The CCS811 needs a wait after app verify
#define CCS811_WAIT_APPDATA_MS     50    // The CCS811 needs a wait after writing app data

#define CCS811_ALG_DATA_ECO2_HB    0
#define CCS811_ALG_DATA_ECO2_LB    1
#define CCS811_ALG_DATA_TVOC_HB    2
#define CCS811_ALG_DATA_TVOC_LB    3
#define CCS811_ALG_DATA_STATUS     4
#define CCS811_ALG_DATA_ERROR_ID   5
#define CCS811_ALG_DATA_RAW_HB     6
#define CCS811_ALG_DATA_RAW_LB     7

#define CCS811_EXEC(a, msg_failed) if (wakeUp()) { \
  esp_err_t err = a; \
  if (err == ESP_OK) { \
    setRawStatus(SENSOR_STATUS_OK, false); \
    wakeDown(); \
  } else { \
    setEspError(err, false); \
    wakeDown(); \
    rlog_e(logTAG, msg_failed, _name, err, esp_err_to_name(err)); \
    return false; \
  }; \
} else { \
  setRawStatus(SENSOR_STATUS_ERROR, false); \
  return false; \
}; 

CCS811::CCS811(uint8_t eventId):rSensorX2(eventId)
{
  _I2C_num = I2C_NUM_0;
  _I2C_address = 0;
  _wakePin = GPIO_NUM_NC;
}

// Destructor
CCS811::~CCS811()
{
}

/**
 * Dynamically creating internal items on the heap
 * */
bool CCS811::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const i2c_port_t numI2C, const uint8_t addrI2C, const ccs811_mode_t mode, const gpio_num_t wake_pin,
  sensor_filter_t filterMode1, uint16_t filterSize1, 
  sensor_filter_t filterMode2, uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  _mode = mode;
  _wakePin = wake_pin;
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
bool CCS811::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const i2c_port_t numI2C, const uint8_t addrI2C, const ccs811_mode_t mode, const gpio_num_t wake_pin,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  _mode = mode;
  _wakePin = wake_pin;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX2::setSensorItems(item1, item2);
  // Start device
  return sensorStart();
}

void CCS811::createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                               const sensor_filter_t filterMode2, const uint16_t filterSize2)
{
  // IAQ TVOC
  _item1 = new rSensorItem(this, CONFIG_SENSOR_IAQ_NAME, 
    filterMode1, filterSize1,
    CONFIG_FORMAT_IAQ_VALUE, CONFIG_FORMAT_IAQ_STRING,
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

  // IAQ eCO2
  _item2 = new rSensorItem(this, CONFIG_SENSOR_CO2_NAME, 
    filterMode2, filterSize2,
    CONFIG_FORMAT_CO2_VALUE, CONFIG_FORMAT_CO2_STRING,
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

void CCS811::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  // IAQ TVOC
  if (_item1) {
    _item1->registerParameters(parent_group, CONFIG_SENSOR_IAQ_KEY, CONFIG_SENSOR_IAQ_NAME, CONFIG_SENSOR_IAQ_FRIENDLY);
  };
  // IAQ eCO2
  if (_item2) {
    _item2->registerParameters(parent_group, CONFIG_SENSOR_CO2_KEY, CONFIG_SENSOR_CO2_NAME, CONFIG_SENSOR_CO2_FRIENDLY);
  };
}

/**
 * Wake UP
 * */
bool CCS811::wakeUp()
{
  if (_wakePin != GPIO_NUM_NC) {
    RE_OK_CHECK(gpio_set_level(_wakePin, 0), return false);
    ets_delay_us(CCS811_WAIT_WAKE_US);
  };
  return true;
}

bool CCS811::wakeDown()
{
  if (_wakePin != GPIO_NUM_NC) {
    RE_OK_CHECK(gpio_set_level(_wakePin, 1), return false);
  };
  return true;
}

/**
 * Displaying multiple values in one topic
 * */
#if CONFIG_SENSOR_DISPLAY_ENABLED

char* CCS811::getDisplayValue()
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

/**
 * I2C
 * */
esp_err_t CCS811::readReg(uint8_t reg, uint8_t *data, uint32_t len)
{
  return readI2C(_I2C_num, _I2C_address, &reg, 1, data, len, 0, CCS811_I2C_TIMEOUT);
}

esp_err_t CCS811::writeReg(uint8_t reg, uint8_t *data, uint32_t len)
{
  return writeI2C(_I2C_num, _I2C_address, &reg, 1, data, len, CCS811_I2C_TIMEOUT);
}

/**
 * Start device
 * */
sensor_status_t CCS811::sendMode(ccs811_mode_t mode) 
{
  rlog_i(logTAG, RSENSOR_LOG_MSG_SET_MODE_HEADER, _name, mode);
  uint8_t reg;
  // Read measurement mode register value
  SENSOR_ERR_CHECK(readReg(CCS811_REG_MEAS_MODE, &reg, 1), RSENSOR_LOG_MSG_READ_MODE_FAILED);
  // Clear mode bits (4-6) // 1xxx1111 
  reg = reg & 0x8F; 
  // Set new bits
  reg = reg | (uint8_t)mode << 4;
  // Write back measurement mode register
  SENSOR_ERR_CHECK(writeReg(CCS811_REG_MEAS_MODE, &reg, 1), RSENSOR_LOG_MSG_SET_MODE_FAILED);
  // Check whether setting measurement mode were succesfull
  vTaskDelay(pdMS_TO_TICKS(CCS811_WAIT_MODE_MS));
  SENSOR_ERR_CHECK(readReg(CCS811_REG_MEAS_MODE, &reg, 1), RSENSOR_LOG_MSG_READ_MODE_FAILED);
  if (((reg & 0x70) >> 4) == mode) {
    _mode = mode;
    return SENSOR_STATUS_OK;
  } else {
    rlog_e(logTAG, RSENSOR_LOG_MSG_SET_MODE_UNCONFIRMED, _name, mode);
    return SENSOR_STATUS_ERROR;
  };
}

sensor_status_t CCS811::sensorResetEx(ccs811_mode_t mode)
{
  // Invoke a SW reset (bring CCS811 in a know state)
  const static uint8_t sw_reset[4] = { 0x11, 0xe5, 0x72, 0x8a };
  SENSOR_ERR_CHECK(writeReg(CCS811_REG_SW_RESET, (uint8_t *)sw_reset, 4), RSENSOR_LOG_MSG_RESET_FAILED);
  rlog_i(logTAG, RSENSOR_LOG_MSG_RESET, _name);
  // Wait 100 ms after the reset
  vTaskDelay(pdMS_TO_TICKS(CCS811_WAIT_RESET_MS));
  
  // Check that HW_ID is 0x81
  uint8_t hw_id;
  uint8_t hw_version;
  uint8_t boot_version;
  uint16_t app_version;
  uint8_t status;
  SENSOR_ERR_CHECK(readReg(CCS811_REG_HW_ID, &hw_id, 1), RSENSOR_LOG_MSG_READ_HADRWARE_ID);
  if (hw_id != 0x81) {
    rlog_e(logTAG, "Wrong hardware ID %02x, should be 0x81", hw_id);
    return SENSOR_STATUS_ERROR;
  };

  // Check that HW_VERSION is 0x1X
  SENSOR_ERR_CHECK(readReg(CCS811_REG_HW_VER, &hw_version, 1), RSENSOR_LOG_MSG_READ_HADRWARE_VERSION);
  if ((hw_version & 0xF0) != 0x10) {
    rlog_w(logTAG, "Wrong hardware version %02x", hw_version);
    return SENSOR_STATUS_ERROR;
  };

  // Check status (after reset, CCS811 should be in boot mode with valid app)
  SENSOR_ERR_CHECK(readReg(CCS811_REG_STATUS, &status, 1), RSENSOR_LOG_MSG_READ_STATUS_FAILED);
  if (!(status & CCS811_STATUS_APP_VALID)) {
    rlog_e(logTAG, "Sensor is in boot mode, but has no valid application");
    return SENSOR_STATUS_ERROR;
  };

  // Read the application version
  SENSOR_ERR_CHECK(readReg(CCS811_REG_FW_BOOT_VER, &boot_version, 1), "Failed to read application boot version from sensor [%s]: %d %s");
  SENSOR_ERR_CHECK(readReg(CCS811_REG_FW_APP_VER, (uint8_t*)&app_version, 2), "Failed to read application version from sensor [%s]: %d %s");
  rlog_i(logTAG, "Found CSS811, hardware version: %02x, firmware boot version: %02x, firmware app version: %04x", 
    hw_version, boot_version, app_version);

  // If the application is not running, you need to start it
  if (!(status & CCS811_STATUS_FW_MODE)) {
    // Switch CCS811 from boot mode into app mode
    SENSOR_ERR_CHECK(writeReg(CCS811_REG_APP_START, nullptr, 0), "Could not start application on [%s]: %d %s");
    // Wait 100 ms after starting the app
    vTaskDelay(pdMS_TO_TICKS(CCS811_WAIT_APPSTART_MS));
    // Get the status to check whether sensor switched to application mode
    SENSOR_ERR_CHECK(readReg(CCS811_REG_STATUS, &status, 1), RSENSOR_LOG_MSG_READ_STATUS_FAILED);
    if ((status & CCS811_STATUS_FW_MODE)) {
      rlog_i(logTAG, "Application started");
    } else {
      rlog_e(logTAG, "Could not start application, invalid status 0x%02x.", status);
      return SENSOR_STATUS_ERROR;
    };
  };

  return sendMode(mode);
}

bool CCS811::setMode(ccs811_mode_t mode) 
{
  bool ret = false;
  if (wakeUp()) {
    ret = (sendMode(mode) == SENSOR_STATUS_OK);
    wakeDown();
  };
  return ret;
}

sensor_status_t CCS811::sensorReset()
{
  // Init wakeup GPIO
  if (_wakePin != GPIO_NUM_NC) {
    gpio_reset_pin(_wakePin);
    SENSOR_ERR_CHECK(gpio_set_direction(_wakePin, GPIO_MODE_OUTPUT), "Failed to set wakeup GPIO mode for sensor [%s]: %d %s");
  };

  sensor_status_t ret = SENSOR_STATUS_ERROR;
  if (wakeUp()) {
    ret = sensorResetEx(_mode);
    wakeDown();
  };
  return ret;
}

/**
 * Set / get baseline
 * */
bool CCS811::getBaseline(uint16_t *baseline)
{
  uint8_t data[2] = {0, 0};
  CCS811_EXEC(readReg(CCS811_REG_BASELINE, data, 2), "Failed to read baseline from sensor [%s]: %d %s");
  *baseline = (uint16_t) (data[0]) << 8 | data[1];
  rlog_i(logTAG, "Sensor [%s]: read baseline = 0x%04X", _name, *baseline);
  return true;
}

bool CCS811::setBaseline(uint16_t baseline)
{
  uint8_t data[2] = { (uint8_t)(baseline >> 8), (uint8_t)(baseline & 0xff) };
  CCS811_EXEC(writeReg(CCS811_REG_BASELINE, data, 2), "Failed to write baseline for sensor [%s]: %d %s");
  rlog_i(logTAG, "Sensor [%s]: set baseline: 0x%04X", _name, baseline);
  return true;
}

/**
 * Set environmental temperature and humidity
 * */
bool CCS811::setEnvironmentalData(float temperature, float humidity)
{
  if ((_mode != CCS811_MODE_IDLE) && !isnan(temperature) && !isnan(humidity)) {
    rlog_i(logTAG, "Send environmental data to sensor [ %s ]", _name);
    uint16_t hum_conv = humidity * 512.0f + 0.5f;
    uint16_t temp_conv = (temperature + 25.0f) * 512.0f + 0.5f;
    uint8_t data[4] = {
      (uint8_t)((hum_conv >> 8) & 0xFF), (uint8_t)(hum_conv & 0xFF),
      (uint8_t)((temp_conv >> 8) & 0xFF), (uint8_t)(temp_conv & 0xFF)
    };
    CCS811_EXEC(writeReg(CCS811_REG_ENV_DATA, data, 4), "Failed to write environmental data for sensor [%s]: %d %s");
    return true;
  };
  return false;
}

/**
 * Get the resistance of connected external NTC thermistor
 * */
bool CCS811::getNtcResistance(uint32_t r_ref, uint32_t *res)
{
  uint8_t data[4] = {0, 0, 0, 0};
  CCS811_EXEC(readReg(CCS811_REG_NTC, data, 4), "Failed to read NTC register from sensor [%s]: %d %s");
  // Calculation from application note ams AN000372
  uint16_t v_ref = (uint16_t) (data[0]) << 8 | data[1];
  uint16_t v_ntc = (uint16_t) (data[2]) << 8 | data[3];
  *res = (v_ntc * r_ref / v_ref);
  return true;
}

// Error handling
sensor_status_t CCS811::checkErrorStatus()
{
  uint8_t status;
  uint8_t err_reg;
  // Check the status register
  SENSOR_ERR_CHECK(readReg(CCS811_REG_STATUS, &status, 1), RSENSOR_LOG_MSG_READ_STATUS_FAILED);
  if (!(status & CCS811_STATUS_ERROR)) {
    // Everything is fine
    return SENSOR_STATUS_OK;
  };
  // Check the error register
  if (readReg(CCS811_REG_ERROR_ID, &err_reg, 1)) {
    if (err_reg & CCS811_ERR_WRITE_REG_INV) {
      rlog_e(logTAG, "Sensor [%s]: received an invalid register for write", _name);
      return SENSOR_STATUS_ERROR;
    };
    if (err_reg & CCS811_ERR_READ_REG_INV) {
      rlog_e(logTAG, "Sensor [%s]: received an invalid register for read", _name);
      return SENSOR_STATUS_ERROR;
    };
    if (err_reg & CCS811_ERR_MEASMODE_INV) {
      rlog_e(logTAG, "Sensor [%s]: received an invalid measurement mode request", _name);
      return SENSOR_STATUS_ERROR;
    };
    if (err_reg & CCS811_ERR_MAX_RESISTANCE) {
      rlog_e(logTAG, "Sensor [%s]: sensor resistance measurement has reached or exceeded the maximum range", _name);
      return SENSOR_STATUS_CAL_ERROR;
    };
    if (err_reg & CCS811_ERR_HEATER_FAULT) {
      rlog_e(logTAG, "Sensor [%s]: heater current not in range", _name);
      return SENSOR_STATUS_CAL_ERROR;
    }
    if (err_reg & CCS811_ERR_HEATER_SUPPLY) {
      rlog_e(logTAG, "Sensor [%s]: heater voltage is not being applied correctly", _name);
      return SENSOR_STATUS_CAL_ERROR;
    };
  };
  rlog_e(logTAG, "Sensor [%s]: unknown error", _name);
  return SENSOR_STATUS_ERROR;
}

/**
 * Reading values from sensor
 * */
/*
sensor_status_t CCS811::readRawData()
{
  // Check mode
  if (_mode == CCS811_MODE_IDLE) {
    rlog_e(logTAG, "Sensor is in idle mode and not performing measurements");
    setRawStatus(SENSOR_STATUS_NO_DATA, true);
    return _lastStatus; 
  };
  if (_mode == CCS811_MODE_250MS) {
    rlog_e(logTAG, "Sensor is in constant power mode, only raw data are available every 250ms");
    setRawStatus(SENSOR_STATUS_NOT_SUPPORTED, true);
    return _lastStatus; 
  };
  
  // Wake up sensor
  if (wakeUp()) {
    uint8_t data[8];
    if (readReg(CCS811_REG_ALG_RESULT_DATA, data, 8)) {
      // Check for errors
      if (data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_ERROR) {
        if (!checkErrorStatus()) { 
          wakeDown();
          return _lastStatus; 
        };
      };
      // Check whether new data are ready, if not, latest values are read from sensor
      if ((data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_DATA_RDY)) {
        // Set data
        uint16_t iaq_tvoc = data[CCS811_ALG_DATA_TVOC_HB] << 8 | data[CCS811_ALG_DATA_TVOC_LB];
        uint16_t iaq_eco2 = data[CCS811_ALG_DATA_ECO2_HB] << 8 | data[CCS811_ALG_DATA_ECO2_LB];
        setRawValues((value_t)iaq_tvoc, (value_t)iaq_eco2);
      } else {
        rlog_w(logTAG, "No new data");
        setErrorStatus(SENSOR_STATUS_NO_DATA, false);
      };
    } else {
      setErrorStatus(SENSOR_STATUS_ERROR, false); 
    };
    
    wakeDown();
  } else { 
    setErrorStatus(SENSOR_STATUS_ERROR, false); 
  };
  return _lastStatus; 
}
*/
sensor_status_t CCS811::readRawData()
{
  // Check mode
  if (_mode == CCS811_MODE_IDLE) {
    rlog_e(logTAG, "Sensor [%s] is in idle mode and not performing measurements", _name);
    return SENSOR_STATUS_NO_DATA; 
  };
  if (_mode == CCS811_MODE_250MS) {
    rlog_e(logTAG, "Sensor [%s] is in constant power mode, only raw data are available every 250ms", _name);
    return SENSOR_STATUS_NOT_SUPPORTED; 
  };
  
  // Wake up sensor
  sensor_status_t ret = SENSOR_STATUS_ERROR;
  if (wakeUp()) {
    uint8_t data[8];
    ret = convertEspError(readReg(CCS811_REG_ALG_RESULT_DATA, data, 8));
    if (ret == SENSOR_STATUS_OK) {
      // Check for errors
      if (data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_ERROR) {
        ret = checkErrorStatus();
      };
      // Check whether new data are ready, if not, latest values are read from sensor
      if (ret == SENSOR_STATUS_OK) {
        if (data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_DATA_RDY) {
          // Set data
          uint16_t iaq_tvoc = data[CCS811_ALG_DATA_TVOC_HB] << 8 | data[CCS811_ALG_DATA_TVOC_LB];
          uint16_t iaq_eco2 = data[CCS811_ALG_DATA_ECO2_HB] << 8 | data[CCS811_ALG_DATA_ECO2_LB];
          ret = setRawValues((value_t)iaq_tvoc, (value_t)iaq_eco2);
        } else {
          rlog_w(logTAG, RSENSOR_LOG_MSG_NO_DATA, _name);
          ret = SENSOR_STATUS_NO_DATA;
        };
      };
    };
    wakeDown();
  };
  return ret; 
}

/**
 * eCO2 interrupt
 * */
bool CCS811::enableInterrupt(bool enabled)
{
  uint8_t reg;
  // Read measurement mode register value
  CCS811_EXEC(readReg(CCS811_REG_MEAS_MODE, &reg, 1), RSENSOR_LOG_MSG_INTERRUPT_GET_FAILED);
  // Clear interrupt and threshold bits (2-3) // 1111xx11 
  reg = reg & 0xF3; 
  // Set new bits (threshold DISABLED!)
  reg = reg | enabled << 3;
  // Write back measurement mode register
  CCS811_EXEC(writeReg(CCS811_REG_MEAS_MODE, &reg, 1), RSENSOR_LOG_MSG_INTERRUPT_SET_FAILED);
  rlog_i(logTAG, RSENSOR_LOG_MSG_INTERRUPT_STATE, _name, enabled ? RSENSOR_LOG_MSG_INTERRUPT_ON : RSENSOR_LOG_MSG_INTERRUPT_OFF);
  return true;
}

/**
 * eCO2 thresholds
 * */
bool CCS811::enableThreshold(bool enabled) 
{
  uint8_t reg;
  // Read measurement mode register value
  CCS811_EXEC(readReg(CCS811_REG_MEAS_MODE, &reg, 1), RSENSOR_LOG_MSG_THRESHOLD_GET_FAILED);
  // Clear interrupt and threshold bits (2-3) // 1111xx11 
  reg = reg & 0xF3; 
  // Set new bits
  reg = reg | enabled << 2;
  reg = reg | enabled << 3;
  // Write back measurement mode register
  CCS811_EXEC(writeReg(CCS811_REG_MEAS_MODE, &reg, 1), RSENSOR_LOG_MSG_THRESHOLD_SET_FAILED);
  rlog_i(logTAG, RSENSOR_LOG_MSG_THRESHOLD_STATE, _name, enabled ? RSENSOR_LOG_MSG_THRESHOLD_ON : RSENSOR_LOG_MSG_THRESHOLD_OFF);
  return true;
}

bool CCS811::setCO2Thresholds(uint16_t low, uint16_t high, uint8_t hysteresis)
{
  rlog_i(logTAG, "Sensor [%s]: set threshold interrupt data: low=%d, high=%d, hysteresis=%d", _name, low, high, hysteresis);

  // Check whether interrupt has to be disabled
  if (!low && !high && !hysteresis) return enableThreshold(false);

  // Check parameters
  if (low < CCS811_ECO2_RANGE_MIN || high > CCS811_ECO2_RANGE_MAX || low > high || !hysteresis) {
    rlog_e(logTAG, "Sensor [%s]: wrong threshold parameters");
    enableThreshold(false);
    return false;
  }

  // Send threshold parameters and enable threshold
  uint8_t data[5] = { (uint8_t)(low >> 8), (uint8_t)(low & 0xff), (uint8_t)(high >> 8), (uint8_t)(high & 0xff), hysteresis };
  CCS811_EXEC(writeReg(CCS811_REG_THRESHOLDS, data, 5), "Failed to write threshold interrupt data for sensor [%s]: %d %s");
  return enableThreshold(true);
}

