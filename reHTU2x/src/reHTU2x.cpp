#include "reHTU2x.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "rLog.h"
#include "driver/i2c.h"
#include <time.h>

#define HTU2X_SOFT_RESET            0xFE      //soft reset

#define HTU2X_USER_REGISTER_WRITE   0xE6      // write user register
#define HTU2X_USER_REGISTER_READ    0xE7      // read  user register

#define HTU2X_HEATER_REGISTER_WRITE 0x51      // write heater control register
#define HTU2X_HEATER_REGISTER_READ  0x11      // read  heater control register

#define HTU2X_SERIAL1_READ1         0xFA      // read 1-st two serial bytes
#define HTU2X_SERIAL1_READ2         0x0F      // read 2-nd two serial bytes
#define HTU2X_SERIAL2_READ1         0xFC      // read 3-rd two serial bytes
#define HTU2X_SERIAL2_READ2         0xC9      // read 4-th two serial bytes

#define HTU2X_FIRMWARE_READ1        0x84      // read firmware revision, 1-st part of the command
#define HTU2X_FIRMWARE_READ2        0xB8      // read firmware revision, 2-nd part of the command

#define HTU2X_HUMD_MEASURE_HOLD     0xE5      // humidity measurement with hold master
#define HTU2X_HUMD_MEASURE_NOHOLD   0xF5      // temperature measurement with no hold master

#define HTU2X_TEMP_MEASURE_HOLD     0xE3      // temperature measurement with hold master
#define HTU2X_TEMP_MEASURE_NOHOLD   0xF3      // temperature measurement with no hold master
#define SI70xx_TEMP_READ_AFTER_RH_MEASURMENT 0xE0 // read temperature value from previous RH measurement, for Si7021 only

#define HTU2X_FIRMWARE_V1           0xFF      // sensor firmware v1.0
#define HTU2X_FIRMWARE_V2           0x20      // sensor firmware v2.0

#define HTU2X_HEATER_ON             0x04      // heater ON
#define HTU2X_HEATER_OFF            0xFB      // heater OFF

#define CHIPID_SI7013               0x0D      // device id SI7013
#define CHIPID_SI7020               0x14      // device id SI7020
#define CHIPID_SI7021               0x15      // device id SI7021
#define CHIPID_HTU21D               0x32      // device id HTU2x/SHT21
#define CHIPID_SHT20                0x01      // device id SHT20

#define HTU2X_SOFT_RESET_DELAY      15        // in milliseconds
#define HTU2X_TIMEOUT               1000      // default timeout 

static const char* logTAG = "HTU2x";
static const char* HTU2X_TYPES [] = {"NULL", "SHT20", "HTU2x / SHT21", "Si7013", "Si7020", "Si7021", "Si7021 FAKE", "UNKNOWN"};

HTU2x::HTU2x(uint8_t eventId):rSensorHT(eventId)
{
  _resolution = HTU2X_RES_RH12_TEMP14;
  _compensated = false;
  _heater = false;
  _deviceType = HTU2X_NULL;
  _serialB = 0;
}

// Dynamically creating internal items on the heap
bool HTU2x::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
  const i2c_port_t numI2C, const HTU2X_RESOLUTION resolution, bool compensated_humidity,
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _resolution = resolution;
  _compensated = compensated_humidity;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX2::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2)) {
    // Start device
    return sensorStart();
  };
  return false;
}

// Connecting external previously created items, for example statically declared
bool HTU2x::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  const i2c_port_t numI2C, const HTU2X_RESOLUTION resolution, bool compensated_humidity,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _resolution = resolution;
  _compensated = compensated_humidity;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX2::setSensorItems(item1, item2);
  // Start device
  return sensorStart();
}

// Start device
sensor_status_t HTU2x::sensorReset()
{
  sensor_status_t ret = softReset();
  if (ret == SENSOR_STATUS_OK) {
    ret = readDeviceID();
    if (ret == SENSOR_STATUS_OK) {
      ret = setResolution(_resolution);
      if (ret == SENSOR_STATUS_OK) {
        ret = setHeater(false);
      };
    };
  };
  return ret;
};

// Send command 1 byte
esp_err_t HTU2x::sendCommand(uint8_t command) 
{
  return writeI2C(_I2C_num, HTU2X_ADDRESS, &command, 1, nullptr, 0, HTU2X_TIMEOUT);
}

// Send command and 1 byte
esp_err_t HTU2x::sendU8(uint8_t command, uint8_t data) 
{
  return writeI2C(_I2C_num, HTU2X_ADDRESS, &command, 1, &data, 1, HTU2X_TIMEOUT);
}

// Send command and receive response 1 byte
esp_err_t HTU2x::readU8(uint8_t command, const uint32_t usWaitData, uint8_t* value)
{
  return readI2C(_I2C_num, HTU2X_ADDRESS, &command, 1, value, 1, usWaitData, HTU2X_TIMEOUT);
}

/**
 * Soft reset, switch sensor OFF & ON
 * NOTE:
 *  - takes ~15ms
 *  - all registers & bits except heater bit will set to default
 * */
sensor_status_t HTU2x::softReset()
{
  SENSOR_ERR_CHECK(sendCommand(HTU2X_SOFT_RESET), RSENSOR_LOG_MSG_RESET_FAILED);
  rlog_i(logTAG, RSENSOR_LOG_MSG_RESET, _name);
  sys_delay_ms(HTU2X_SOFT_RESET_DELAY);
	return SENSOR_STATUS_OK;
}

/**
 * Sets sensor's resolution
 * */
sensor_status_t HTU2x::setResolution(HTU2X_RESOLUTION sensorResolution)
{
  rlog_i(logTAG, RSENSOR_LOG_MSG_SET_RESOLUTION_HEADER, _name, sensorResolution);
  uint8_t data = 0;
  SENSOR_ERR_CHECK(readU8(HTU2X_USER_REGISTER_READ, 0, &data), RSENSOR_LOG_MSG_READ_RESOLUTION_FAILED);
  data &= 0x7E; // 01111110, clear D7;D0                            
  data |= sensorResolution;            
  SENSOR_ERR_CHECK(sendU8(HTU2X_USER_REGISTER_WRITE, data), RSENSOR_LOG_MSG_SET_RESOLUTION_FAILED);
  _resolution = sensorResolution;
  return SENSOR_STATUS_OK;
}

/**
 * Turn ON/OFF build-in heater
 * NOTE:
 *  - prolonged exposure to high humidity will result gradual upward drift
 *    of the RH reading, the heater is used to drive off condensation &
 *    reverse drift effect.
 *  - heater consumtion is 3.09mA - 94.20mA @ 3.3v.
 * */ 
sensor_status_t HTU2x::setHeater(const bool heaterMode)
{
  uint8_t data = 0;
  SENSOR_ERR_CHECK(readU8(HTU2X_USER_REGISTER_READ, 0, &data), RSENSOR_LOG_MSG_HEATER_GET_FAILED);
  heaterMode ? data |= HTU2X_HEATER_ON : data &= HTU2X_HEATER_OFF;
  _heater = data & HTU2X_HEATER_ON;
  SENSOR_ERR_CHECK(sendU8(HTU2X_USER_REGISTER_WRITE, data), RSENSOR_LOG_MSG_HEATER_SET_FAILED);
  _heater = data & HTU2X_HEATER_ON;
  if (_heater == heaterMode) {
    rlog_i(logTAG, RSENSOR_LOG_MSG_HEATER_STATE, _name, _heater ? RSENSOR_LOG_MSG_HEATER_ON : RSENSOR_LOG_MSG_HEATER_OFF);
    return SENSOR_STATUS_OK;
  } else {
    rlog_i(logTAG, RSENSOR_LOG_MSG_HEATER_UNCONFIRMED, _name);
    return SENSOR_STATUS_ERROR;
  };
}

/**
 * Set power of build-in heater
 * NOTE:
 *   - for Si70XX only
 *   - values from 0 to 15 (0x0F) can be used
 *   - heater consumtion is 3.09mA - 94.20mA @ 3.3v.
 * */
sensor_status_t HTU2x::setHeaterPower(const uint8_t heaterPower)
{
  if (_deviceType == HTU2X_NULL) {
    readDeviceID();
  };
  if (_deviceType <= HTU2X_HTU2x) {
    rlog_w(logTAG, RSENSOR_LOG_MSG_CMD_NOT_SUPPORTED, _name);
    return SENSOR_STATUS_NOT_SUPPORTED;
  };
  uint8_t data = 0;
  SENSOR_ERR_CHECK(readU8(HTU2X_HEATER_REGISTER_READ, 0, &data), RSENSOR_LOG_MSG_HEATER_GET_FAILED);
  data &= 0xF0;  // 11110000, clear D3-D0
  data |= (heaterPower & 0x0F);
  SENSOR_ERR_CHECK(sendU8(HTU2X_HEATER_REGISTER_WRITE, data), RSENSOR_LOG_MSG_HEATER_SET_FAILED);
  rlog_i(logTAG, "Sensor [%s]: built-in heater power is set to %d", _name, heaterPower & 0x0F);
	return SENSOR_STATUS_OK;
}

/**
 * Checks the battery status.
 * NOTE:
 * - for SHT21, HTU21D:
 *   - if VDD > 2.25v ±0.1v return TRUE
 *   - if VDD < 2.25v ±0.1v return FALSE
 * - for Si70xx:
 *   - if VDD > 1.9v ±0.1v return TRUE
 *   - if VDD < 1.9v ±0.1v return FALSE
 * */
bool HTU2x::batteryStatus(void)
{
  uint8_t data = 0;
  SENSOR_ERR_CHECK_BOOL(readU8(HTU2X_USER_REGISTER_READ, 0, &data), RSENSOR_LOG_MSG_READ_USER_REGISTER_FAILED);
  data &= 0x40;
  return data == 0x00;
}

/**
 * Reads device id
 * NOTE:
 *  - see p.23 of Si7021 datasheet for details
 *  - full serial number is {SNA3, SNA2, SNA1, SNA0, SNB3**, SNB2, SNB1, SNB0}
 *  - **chip ID:
 *    - 0x0D: Si7013
 *    - 0x14: Si7020
 *    - 0x15: Si7021 
 *    - 0x32: HTU21D & SHT21
 * */
sensor_status_t HTU2x::readDeviceID(void)
{
  _serialB = 0;
  _deviceType = HTU2X_NULL;
  bool _crcIsBad = false;
  uint8_t cmds[2];
  uint8_t data[6];
  // request serialB -> SNAB**, SNB2, SNB1, SNB0
  cmds[0] = HTU2X_SERIAL2_READ1;
  cmds[1] = HTU2X_SERIAL2_READ2;
  SENSOR_ERR_CHECK(readI2C(_I2C_num, HTU2X_ADDRESS, cmds, 2, data, 6, 0, HTU2X_TIMEOUT), RSENSOR_LOG_MSG_READ_HADRWARE_ID);
  // error handler, checksum verification
  if ((CRC8(0x00, data[0], data[1]) != data[2]) || (CRC8(0x00, data[3], data[4]) != data[5])) {
    // For some reason, CRC8 returns incorrect on some instances. Probably fake sensors
    _crcIsBad = true;
    rlog_w(logTAG, "Sensor [%s]: serial B CRC error: CRC8(0x%.2X, 0x%.2X) != 0x%.2X or CRC8(0x%.2X, 0x%.2X) != 0x%.2X!", _name, data[0], data[1], data[2], data[3], data[4], data[5]);
    // return SENSOR_STATUS_CRC_ERROR;
  };
  _serialB = (((((data[0] << 8) | data[1]) << 8) | data[3]) << 8) | data[4];
  // decode sensor type
  switch (data[0]) {
    case CHIPID_SHT20:
      _deviceType = HTU2X_SHT20;
      break;
    case CHIPID_HTU21D:
      _deviceType = HTU2X_HTU2x;
      break;
    case CHIPID_SI7013:
      _deviceType = HTU2X_SI7013;
      break;
    case CHIPID_SI7020:
      _deviceType = HTU2X_SI7020;
      break;
    case CHIPID_SI7021:
      (_crcIsBad || (_serialB == 0x15ffffff)) ? _deviceType = HTU2X_SI7021FAKE : _deviceType = HTU2X_SI7021;
      break;
    default:
      _deviceType = HTU2X_UNKNOWN;
      break;
  };
  // show log
  rlog_i(logTAG, "Detected sensor %s on bus %d at address 0x%.2X, serial number: 0x%.8x", HTU2X_TYPES[_deviceType], _I2C_num, HTU2X_ADDRESS, _serialB);
  return SENSOR_STATUS_OK;
}

HTU2X_TYPE HTU2x::getType()
{
  if (_deviceType == HTU2X_NULL) {
    readDeviceID();
  };
  return _deviceType;
}

uint32_t HTU2x::getSerialB()
{
  if (_deviceType == HTU2X_NULL) {
    readDeviceID();
  };
  return _serialB;
}

/**
 * Read humidity and temperature data
 * NOTE: 
 *  - all measurement are made in nohold mode
 *  - for Si80xx use SI70xx_TEMP_READ_AFTER_RH_MEASURMENT mode
 * */
sensor_status_t HTU2x::readRawData()
{
  uint8_t cmd;
  uint8_t data[3];
  uint32_t meas_delay;
  esp_err_t err;

  // It is pointless to take measurements while the heater is operating.
  if (_heater) {
    rlog_w(logTAG, RSENSOR_LOG_MSG_HEATER_STATE, _name, RSENSOR_LOG_MSG_HEATER_ON);
    return SENSOR_STATUS_NO_DATA;
  };

  // Request humidity measurement
  cmd = HTU2X_HUMD_MEASURE_NOHOLD;
  // Humidity measurement delay (minimum step = portTICK_RATE_MS = 10 uS)
  switch(_resolution) {
    // Si7021 - 10..12msec, HTU21D - 14..16msec, SHT21 - 22..29msec
    case HTU2X_RES_RH12_TEMP14:
      switch (_deviceType) {
        case HTU2X_SI7021: meas_delay = 15000; break;
        case HTU2X_SI7021FAKE: meas_delay = 50000; break;
        default: meas_delay = 30000; break;
      };
      break;
    // Si7021 - 6..7msec, HTU21D - 7..8msec, SHT21 - 12..15msec
    case HTU2X_RES_RH11_TEMP11:
      switch (_deviceType) {
        case HTU2X_SI7021: meas_delay = 7000; break;
        case HTU2X_SI7021FAKE: meas_delay = 20000; break;
        default: meas_delay = 15000; break;
      };
      break;
    // Si7021 - 4..5msec, HTU21D - 4..5msec, SHT21 - 7..9msec
    case HTU2X_RES_RH10_TEMP13:
      switch (_deviceType) {
        case HTU2X_SI7021: meas_delay = 5000; break;
        case HTU2X_SI7021FAKE: meas_delay = 10000; break;
        default: meas_delay = 9000; break;
      };
      break;
    // Si7021 - 3..4msec, HTU21D - 2..3msec, SHT21 - 3..4msec
    default:
      meas_delay = 4000;
      break;
  };

  // Read humidity measurement to buffer
  SENSOR_ERR_CHECK(readI2C_CRC8(_I2C_num, HTU2X_ADDRESS, &cmd, 1, data, 3, meas_delay, 0x00, HTU2X_TIMEOUT), RSENSOR_LOG_MSG_READ_HUMD_FAILED);
  // Calculate humidity value
  value_t humdValue = 125.0 * (float)(((data[0] << 8) | data[1]) ^ 0x02) / 65536 - 6;
  if (humdValue < 0) humdValue = 0;
  if (humdValue > 100) humdValue = 100;

  // Request temperature measurement
  if ((_deviceType == HTU2X_HTU2x) || (_deviceType == HTU2X_SHT20)) {
    cmd = HTU2X_TEMP_MEASURE_NOHOLD;
    // Humidity measurement delay (minimum step = portTICK_RATE_MS = 10 uS)
    switch(_resolution) {
      // HTU21D - 44..50msec, Si7021 - 7..11msec, SHT21 - 66..85msec
      case HTU2X_RES_RH12_TEMP14:
        meas_delay = 85000;
        break;
      // HTU21D - 22..25msec, Si7021 - 4..7msec,  SHT21 - 33..43msec
      case HTU2X_RES_RH11_TEMP11:
        meas_delay = 43000;
        break;
      // HTU21D - 11..13msec, Si7021 - 3..4msec,  SHT21 - 17..22msec
      case HTU2X_RES_RH10_TEMP13:
        meas_delay = 22000;
        break;
      // HTU21D - 6..7msec,   Si7021 - 2..3msec,  SHT21 - 9..11msec
      default:
        meas_delay = 11000;
        break;
    };
    // Read temperature measurement to buffer
    SENSOR_ERR_CHECK(readI2C_CRC8(_I2C_num, HTU2X_ADDRESS, &cmd, 1, data, 3, meas_delay, 0x00, HTU2X_TIMEOUT), RSENSOR_LOG_MSG_READ_TEMP_FAILED);
  } else {
    // Read temperature measurement to buffer without checksum
    cmd = SI70xx_TEMP_READ_AFTER_RH_MEASURMENT;
    SENSOR_ERR_CHECK(readI2C(_I2C_num, HTU2X_ADDRESS, &cmd, 1, data, 3, 0, HTU2X_TIMEOUT), RSENSOR_LOG_MSG_READ_TEMP_FAILED);
  };
  // Calculate temperature value
  value_t tempValue = 175.72 * (float)((data[0] << 8) | data[1]) / 65536 - 46.85;

  // Calculate temperature compensated humidity (only for HTU21D & SHT21)
  // Si70xx automatically compensates temperature influence on RH every humidity measurement
  if ((_compensated) && (_deviceType == HTU2X_HTU2x)) {
    if (tempValue > 0 && tempValue < 80) humdValue = humdValue + -0.15 * (25.0 - tempValue);
  };

  // Store values in sensors
  return setRawValues(humdValue, tempValue);
};
