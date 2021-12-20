#include "reAHT1x.h"
#include "freertos/FreeRTOS.h"
#include "reI2C.h"
#include "rLog.h"
#include "driver/i2c.h"
#include <time.h>

#define AHT10_CMD_INIT              0xE1  // Calibration command for AHT10/AHT15
#define AHT20_CMD_INIT              0xBE  // Calibration command for AHT10/AHT15
#define AHTX0_CMD_SOFTRESET         0xBA  // Soft reset command
#define AHTX0_CMD_MEASURMENT        0xAC  // Start measurment command
#define AHTX0_CMD_NORMAL            0xA8  // Normal cycle mode command, no info in datasheet!!!

#define AHTX0_INIT_NORMAL_MODE      0x00  // Enable normal mode
#define AHTX0_INIT_CYCLE_MODE       0x20  // Enable cycle mode
#define AHTX0_INIT_CAL_ENABLE       0x08  // Load factory calibration coeff
#define AHTX0_INIT_CMD_MODE         0x40  // Enable command mode

#define AHTX0_STATUS_CALIBRATED     0x08  // Status bit for calibrated
#define AHTX0_STATUS_BUSY           0x80  // Status bit for busy

#define AHTX0_DATA_MEASURMENT       0x33  // No info in datasheet!!! my guess it is DAC resolution, saw someone send 0x00 instead
#define AHTX0_DATA_NOP              0x00  // No info in datasheet!!!

#define AHTX0_MEASURMENT_DELAY      80    // at least 75 milliseconds
#define AHTX0_POWER_ON_DELAY        40    // at least 20..40 milliseconds
#define AHTX0_SOFT_RESET_DELAY      20    // less than 20 milliseconds 
#define AHTX0_CMD_DELAY             350   // at least 300 milliseconds, no info in datasheet!!!
#define AHTX0_TIMEOUT               1000  // default timeout 
#define AHT10_ERROR                 0xFF  // returns 255, if communication error is occurred

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
  vTaskDelay(AHTX0_POWER_ON_DELAY / portTICK_PERIOD_MS);
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
  if (execI2C(_I2C_num, cmd, AHTX0_TIMEOUT) != ESP_OK) {
    this->rSensor::setRawStatus(SENSOR_STATUS_TIMEOUT, true);
    return AHT10_ERROR;
  };
  return status;
} 

void AHT1x::waitBusy(uint32_t delay)
{
  vTaskDelay(delay / portTICK_RATE_MS);
  while (readStatus() & AHTX0_STATUS_BUSY) {
    vTaskDelay(1);
  };
};

sensor_status_t AHT1x::setMode(const AHT1x_MODE newMode) 
{
  // First command: init sensor
  _sensorType == AHT20_SENSOR ? _rawCmdBuffer[0] = AHT20_CMD_INIT : _rawCmdBuffer[0] = AHT10_CMD_INIT;
  // Second command: CAL enable and set mode
  _rawCmdBuffer[1] = AHTX0_INIT_CAL_ENABLE | newMode;
  // Third command: NOP
  _rawCmdBuffer[2] = AHTX0_DATA_NOP;
  // Send commands
  esp_err_t err = writeI2C(_I2C_num, _I2C_addr, _rawCmdBuffer, 3, nullptr, 0, AHTX0_TIMEOUT);
  if (err != ESP_OK) {
    rlog_e(_name, "Failed to set mode: %d %s!", err, esp_err_to_name(err));
    return this->rSensor::setEspError(err, true);
  };
  // Wait for a response from the sensor
  waitBusy(AHTX0_CMD_DELAY);
  // Check calibration enable
  if (!(readStatus() & AHTX0_STATUS_CALIBRATED)) {
    rlog_e(_name, "Failed to load factory calibration data!");
    this->rSensor::setRawStatus(SENSOR_STATUS_CAL_ERROR, true);
    return SENSOR_STATUS_CAL_ERROR;
  };
  this->rSensor::setRawStatus(SENSOR_STATUS_OK, true);
  rlog_i(_name, "For sensor [%s] has been set mode 0x%.2X", _name, newMode);
  return SENSOR_STATUS_OK;
}

sensor_status_t AHT1x::softReset(const AHT1x_MODE sensorMode)
{
  // Send reset command
  _rawCmdBuffer[0] = AHTX0_CMD_SOFTRESET;
  esp_err_t err = writeI2C(_I2C_num, _I2C_addr, _rawCmdBuffer, 1, nullptr, 0, AHTX0_TIMEOUT);
	if (err != ESP_OK) {
    rlog_e(_name, RSENSOR_LOG_MSG_SOFT_RESET_FAILED, err, esp_err_to_name(err));
    return this->rSensor::setEspError(err, true);
  };
  // Wait for sensor to initialize 
  vTaskDelay(AHTX0_SOFT_RESET_DELAY / portTICK_RATE_MS);
  rlog_i(_name, RSENSOR_LOG_MSG_SOFT_RESET, _name);
  // Set operation mode
  return setMode(sensorMode);
}

sensor_status_t AHT1x::readRawData()
{
  static uint32_t hData, tData;
  // Send measurment command
  _rawCmdBuffer[0] = AHTX0_CMD_MEASURMENT;
  _rawCmdBuffer[1] = AHTX0_DATA_MEASURMENT;
  _rawCmdBuffer[2] = AHTX0_DATA_NOP;
  esp_err_t err = writeI2C(_I2C_num, _I2C_addr, _rawCmdBuffer, 3, nullptr, 0, AHTX0_TIMEOUT);
	if (err != ESP_OK) {
    rlog_e(_name, "Failed to send measurment command: %d %s!", err, esp_err_to_name(err));
    return this->rSensor::setEspError(err, true);
  };
  // Get sensor status
  uint8_t status = readStatus();
  // Check calibration enable
  if (!(status & AHTX0_STATUS_CALIBRATED)) {
    rlog_e(_name, "Failed to load factory calibration data!");
    this->rSensor::setRawStatus(SENSOR_STATUS_CAL_ERROR, true);
    return SENSOR_STATUS_CAL_ERROR;
  };
  // Measurement delay
  if (status & AHTX0_STATUS_BUSY) {
    vTaskDelay(AHTX0_MEASURMENT_DELAY / portTICK_RATE_MS); 
  };
  // Read 6-bytes from sensor
  err = readI2C(_I2C_num, _I2C_addr, nullptr, 0, _rawDataBuffer, 6, 0, AHTX0_TIMEOUT);
  if (err != ESP_OK) {
    rlog_e(_name, "Failed to read data from sensor: %d %s!", err, esp_err_to_name(err));
    return this->rSensor::setEspError(err, true);
  };
  // Decode 20-bit raw humidity data
  hData = (((uint32_t)_rawDataBuffer[1] << 16) | ((uint16_t)_rawDataBuffer[2] << 8) | (_rawDataBuffer[3])) >> 4;
  // Decode 20-bit raw temperature data
  tData = ((uint32_t)(_rawDataBuffer[3] & 0x0F) << 16) | ((uint16_t)_rawDataBuffer[4] << 8) | _rawDataBuffer[5]; 
  // Set raw values
  this->rSensorX2::setRawValues(((float)hData * 100) / 0x100000, ((float)tData * 200 / 0x100000) - 50);
  return SENSOR_STATUS_OK;
}
