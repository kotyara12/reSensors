#include "reAHT1x.h"
#include "freertos/FreeRTOS.h"
#include "reI2C.h"
#include "rLog.h"
#include "driver/i2c.h"
#include <time.h>
#include <math.h>

/* List of command registers */
#define AHT1X_CMD_INIT                    0xE1  // Initialization command, for AHT1x only
#define AHT2X_CMD_INIT                    0xBE  // Initialization command, for AHT2x only
#define AHTXX_CMD_SOFTRESET               0xBA  // Soft reset command
#define AHTXX_CMD_MEASURMENT              0xAC  // Start measurment command
#define AHTXX_STATUS_REG                  0x71  // Read status byte command

/* Calibration register controls */
#define AHT1X_INIT_CTRL_NORMAL_MODE       0x00  // Enable normal mode, for AHT1x only
#define AHT1X_INIT_CTRL_CYCLE_MODE        0x20  // Enable cycle mode, for AHT1x only
#define AHT1X_INIT_CTRL_CMD_MODE          0x40  // Enable command mode, for AHT1x only
#define AHT1X_INIT_CTRL_CAL_ENABLE        0x08  // Load factory calibration coeff
#define AHTXX_INIT_CTRL_NOP               0x00  // NOP control, send after any "AHT1X_INIT_CTRL..."

/* Status byte register controls */
#define AHTXX_STATUS_CTRL_BUSY            0x80  // Busy                      bit[7]
#define AHT1X_STATUS_CTRL_NORMAL_MODE     0x00  // Normal mode status        bit[6:5], for AHT1x only
#define AHT1X_STATUS_CTRL_CYCLE_MODE      0x20  // Cycle mode status         bit[6:5], for AHT1x only
#define AHT1X_STATUS_CTRL_CMD_MODE        0x40  // Command mode status       bit[6:5], for AHT1x only
#define AHTXX_STATUS_CTRL_CRC             0x10  // CRC8 status               bit[4], no info in datasheet
#define AHTXX_STATUS_CTRL_CAL_ON          0x08  // Calibration coeff status  bit[3]
#define AHTXX_STATUS_CTRL_FIFO_ON         0x04  // FIFO on status            bit[2], no info in datasheet
#define AHTXX_STATUS_CTRL_FIFO_FULL       0x02  // FIFO full status          bit[1], no info in datasheet
#define AHTXX_STATUS_CTRL_FIFO_EMPTY      0x02  // FIFO empty status         bit[1], no info in datasheet

/* Measurement register controls */
#define AHTXX_START_MEASUREMENT_CTRL      0x33  // Measurement controls, suspect this is temperature & humidity DAC resolution
#define AHTXX_MEASUREMENT_CTRL_NOP        0x00  // NOP control, send after any "AHTXX_DATA_MEASURMENT..."

/* Sensor delays */
#define AHTXX_CMD_DELAY                   300   // Delay between commands, at least 300 milliseconds, no info in datasheet!!!
#define AHTXX_MEASURMENT_DELAY            75    // Wait for measurement to complete, at least 75 milliseconds
#define AHT1X_POWER_ON_DELAY              40    // Wait for AHT1x to initialize after power-on, at least 20..40 milliseconds
#define AHT2X_POWER_ON_DELAY              100   // Wait for AHT2x to initialize after power-on, in milliseconds
#define AHTXX_SOFT_RESET_DELAY            20    // Less than 20 milliseconds 
#define AHTXX_REPEAT_DELAY                500   // Waiting before re-requesting data
#define AHTXX_TIMEOUT                     500   // Default i2c timeout 
#define AHTXX_CHANGE_LIMIT_TEMP           5.0   // The maximum change in humidity between adjacent measurements above which the value is considered suspicious
#define AHTXX_CHANGE_LIMIT_HUMD           5.0   // The maximum change in temperature between adjacent measurements above which the value is considered suspicious
#define AHTXX_ERROR                       0xFF  // Returns 255, if communication error is occurred

static const char* logTAG = "AHT1x";

AHT1x::AHT1x():rSensorHT() 
{
}

// Dynamically creating internal items on the heap
bool AHT1x::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  ASAIR_I2C_SENSOR sensorType, const int numI2C, const uint8_t addrI2C, const AHT1x_MODE sensorMode,
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX2::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2)) {
    // Start device
    return initHardware(sensorType, numI2C, addrI2C, sensorMode);
  };
  return false;
}

// Connecting external previously created items, for example statically declared
bool AHT1x::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,  
  ASAIR_I2C_SENSOR sensorType, const int numI2C, const uint8_t addrI2C, const AHT1x_MODE sensorMode,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX2::setSensorItems(item1, item2);
  // Start device
  return initHardware(sensorType, numI2C, addrI2C, sensorMode);
}

// Start device
bool AHT1x::initHardware(ASAIR_I2C_SENSOR sensorType, const int numI2C, const uint8_t addrI2C, const AHT1x_MODE sensorMode)
{
  _I2C_num = numI2C;
  _I2C_addr = addrI2C;
  _sensorType = sensorType;
  // Wait for sensor to initialize 
  if (sensorType == AHT2X_SENSOR) {
    vTaskDelay(AHT2X_POWER_ON_DELAY / portTICK_PERIOD_MS);
  } else {
    vTaskDelay(AHT1X_POWER_ON_DELAY / portTICK_PERIOD_MS);
  };
  // Set operation mode
  if (setMode(sensorMode) != SENSOR_STATUS_OK) return false;
  rlog_d(_name, RSENSOR_LOG_MSG_INIT_OK, _name);
  return true;
}

uint8_t AHT1x::readStatus() 
{
  uint8_t status;
  i2c_cmd_handle_t cmd = prepareI2C(_I2C_addr, false);
  i2c_master_read_byte(cmd, &status, I2C_MASTER_NACK);
  if (execI2C(_I2C_num, cmd, AHTXX_TIMEOUT) != ESP_OK) {
    this->rSensor::setRawStatus(SENSOR_STATUS_TIMEOUT, false);
    return AHTXX_ERROR;
  };
  return status;
} 

uint8_t AHT1x::waitBusy(uint32_t delay)
{
  vTaskDelay(delay / portTICK_RATE_MS);
  uint8_t status = readStatus();
  while ((status != AHTXX_ERROR) && (status & AHTXX_STATUS_CTRL_BUSY)) {
    vTaskDelay(AHTXX_CMD_DELAY / portTICK_RATE_MS);
    status = readStatus();
  };
  return status;
};

sensor_status_t AHT1x::setMode(const AHT1x_MODE newMode) 
{
  if (_sensorType == AHT2X_SENSOR) {
    // First command: init sensor
    _rawCmdBuffer[0] = AHT2X_CMD_INIT;
    // Second command: CAL enable
    _rawCmdBuffer[1] = AHT1X_INIT_CTRL_CAL_ENABLE;
  } else {
    // First command: init sensor
    _rawCmdBuffer[0] = AHT1X_CMD_INIT;
    // Second command: CAL enable and set mode
    _rawCmdBuffer[1] = AHT1X_INIT_CTRL_CAL_ENABLE | newMode;
  };
  // Third command: NOP
  _rawCmdBuffer[2] = AHTXX_INIT_CTRL_NOP;
  // Send commands
  esp_err_t err = writeI2C(_I2C_num, _I2C_addr, _rawCmdBuffer, 3, nullptr, 0, AHTXX_TIMEOUT);
  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_SET_MODE_FAILED, _name, err, esp_err_to_name(err));
    return this->rSensor::setEspError(err, true);
  };
  // Wait for a response from the sensor
  waitBusy(AHTXX_CMD_DELAY);
  // Check calibration enable
  if (!(readStatus() & AHTXX_STATUS_CTRL_CAL_ON)) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_CAL_FAILED, _name);
    this->rSensor::setRawStatus(SENSOR_STATUS_CAL_ERROR, false);
    return SENSOR_STATUS_CAL_ERROR;
  };
  _sensorMode = newMode;
  this->rSensor::setRawStatus(SENSOR_STATUS_OK, true);
  rlog_i(logTAG, "For sensor [%s] has been set mode 0x%.2X", _name, newMode);
  return SENSOR_STATUS_OK;
}

sensor_status_t AHT1x::softReset(const AHT1x_MODE sensorMode)
{
  // Send reset command
  _rawCmdBuffer[0] = AHTXX_CMD_SOFTRESET;
  esp_err_t err = writeI2C(_I2C_num, _I2C_addr, _rawCmdBuffer, 1, nullptr, 0, AHTXX_TIMEOUT);
	if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_SOFT_RESET_FAILED_N, _name, err, esp_err_to_name(err));
    return this->rSensor::setEspError(err, true);
  };
  // Wait for sensor to initialize 
  vTaskDelay(AHTXX_SOFT_RESET_DELAY / portTICK_RATE_MS);
  rlog_i(logTAG, RSENSOR_LOG_MSG_SOFT_RESET, _name);
  // Set operation mode
  return setMode(sensorMode);
}

bool AHT1x::checkCRC8()
{
  if (_sensorType == AHT2X_SENSOR) {
    // Initial value
    uint8_t crc = 0xFF;                                      
    // 6-bytes in data, {status, RH, RH, RH+T, T, T, CRC}
    for (uint8_t byteIndex = 0; byteIndex < 6; byteIndex ++) {
      crc ^= _rawDataBuffer[byteIndex];
      // 8-bits in byte
      for (uint8_t bitIndex = 8; bitIndex > 0; --bitIndex) {
        //0x31=CRC seed/polynomial 
        if (crc & 0x80) {
          crc = (crc << 1) ^ 0x31;
        } else {
          crc = (crc << 1);
        };
      };
    };
    return (crc == _rawDataBuffer[6]);
  };
  return true;
}

sensor_status_t AHT1x::readRawDataEx(bool resetBus)
{
  uint32_t hData, tData;
  float hValue, tValue;
  esp_err_t err = ESP_OK;

  // Reset bus
  if (resetBus) {
    generalCallResetI2C(_I2C_num);
    softReset(_sensorMode);
  };
  
  // Send measurment command
  _rawCmdBuffer[0] = AHTXX_CMD_MEASURMENT;
  _rawCmdBuffer[1] = AHTXX_START_MEASUREMENT_CTRL;
  _rawCmdBuffer[2] = AHTXX_MEASUREMENT_CTRL_NOP;
  err = writeI2C(_I2C_num, _I2C_addr, _rawCmdBuffer, 3, nullptr, 0, AHTXX_TIMEOUT);
	if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_SEND_MEASURMENT, _name, err, esp_err_to_name(err));
    return this->rSensor::convertEspError(err);
  };
  
  // Measurement delay
  uint8_t status = waitBusy(AHTXX_MEASURMENT_DELAY);
  if (status != AHTXX_ERROR) {
    // Check calibration enable
    if (!(status & AHTXX_STATUS_CTRL_CAL_ON)) {
      rlog_e(logTAG, RSENSOR_LOG_MSG_CAL_FAILED, _name);
      return SENSOR_STATUS_CAL_ERROR;
    };
    
    // Read data from sensor
    if (_sensorType == AHT2X_SENSOR) {
      // Read 7-bytes from sensor {status, RH, RH, RH+T, T, T, CRC}, CRC for AHT2x only
      err = readI2C(_I2C_num, _I2C_addr, nullptr, 0, _rawDataBuffer, 7, 0, AHTXX_TIMEOUT);
    } else {
      // Read 6-bytes from sensor {status, RH, RH, RH+T, T, T}
      err = readI2C(_I2C_num, _I2C_addr, nullptr, 0, _rawDataBuffer, 6, 0, AHTXX_TIMEOUT);
    };
    if (err != ESP_OK) {
      rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
      return this->rSensor::convertEspError(err);
    };

    // Check CRC8
    if ((_sensorType == AHT2X_SENSOR) && !checkCRC8()) {
      rlog_e(logTAG, RSENSOR_LOG_MSG_CRC_FAILED, _name);
      return SENSOR_STATUS_CRC_ERROR;
    };

    // Decode 20-bit raw humidity data
    hData = (((uint32_t)_rawDataBuffer[1] << 16) | ((uint16_t)_rawDataBuffer[2] << 8) | (_rawDataBuffer[3])) >> 4;
    hValue = ((float)hData * 100) / 0x100000;
    // Decode 20-bit raw temperature data
    tData = ((uint32_t)(_rawDataBuffer[3] & 0x0F) << 16) | ((uint16_t)_rawDataBuffer[4] << 8) | _rawDataBuffer[5]; 
    tValue = ((float)tData / 0x100000) * 200 - 50;

    // Check values
    if ((_sensorType != AHT2X_SENSOR) 
     && ((hData == 0x0) || (tData = 0x0) 
      || (!resetBus && ((fabsf(hValue-getHandle1()->lastValue.rawValue)>AHTXX_CHANGE_LIMIT_HUMD) 
                     || (fabsf(tValue-getHandle2()->lastValue.rawValue)>AHTXX_CHANGE_LIMIT_TEMP))))) {
      rlog_e(logTAG, RSENSOR_LOG_MSG_BAD_VALUE, _name);
      return SENSOR_STATUS_NAN;
    };
    
    // Set raw values
    this->rSensorX2::setRawValues(hValue, tValue);
    return SENSOR_STATUS_OK;
  };
  return SENSOR_STATUS_ERROR;
}

sensor_status_t AHT1x::readRawData()
{
  sensor_status_t statusRead = readRawDataEx(false);
  if (statusRead != SENSOR_STATUS_OK) {
    vTaskDelay(pdMS_TO_TICKS(AHTXX_REPEAT_DELAY));
    statusRead = readRawDataEx(true);
  };
  this->rSensor::setRawStatus(statusRead, false);
  return statusRead;
}


