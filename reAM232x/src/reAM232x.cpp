#include "reAM232x.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "rLog.h"
#include "driver/i2c.h"

#define AM232x_CMD_READ_REGISTERS    0x03     // Read one or more data registers
#define AM232x_CMD_WRITE_REGISTERS   0x10     // Multiple sets of binary data to write multiple registers

#define AM232x_TIMEOUT               3000     // Time to read a communication bus for a maximum of 3 S
#define AM232x_WAKE_DELAY            1000     // Waiting time of at least 800 μs, the maximum 3ms
#define AM232x_READ_DELAY            1500     // Reading device information delay in microseconds

AM232x::AM232x():rSensorHT()
{
  _I2C_num = I2C_NUM_0;
}

/**
 * Dynamically creating internal items on the heap
 * */
bool AM232x::initIntItems(const char* sensorName, const char* topicName, 
  const i2c_port_t numI2C, 
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX2::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2)) {
    // Start device
    return initHardware(numI2C);
  };
  return false;
}

/**
 * Connecting external previously created items, for example statically declared
 * */
bool AM232x::initExtItems(const char* sensorName, const char* topicName, 
  const i2c_port_t numI2C, 
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX2::setSensorItems(item1, item2);
  // Start device
  return initHardware(numI2C);
}

/**
 * Start device
 * */
bool AM232x::initHardware(const int numI2C)
{
  _I2C_num = numI2C;
	return readDeviceInfo() == SENSOR_STATUS_OK;
};

/**
 * Calculate CRC16
 * */
uint16_t AM232x::CRC16(uint8_t *buffer, uint8_t nbytes) 
{
  uint16_t crc = 0xffff;
  for (int i = 0; i < nbytes; i++) {
    uint8_t b = buffer[i];
    crc ^= b;
    for (int x = 0; x < 8; x++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

sensor_status_t AM232x::readRegister(uint8_t reg, uint8_t len, uint8_t * buf)
{
  esp_err_t error_code = ESP_ERR_NO_MEM;
  
  // Step one: wake sensor
  i2c_cmd_handle_t cmdLink = i2c_cmd_link_create();
  i2c_master_start(cmdLink);
  i2c_master_write_byte(cmdLink, (AM232x_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_DIS);
  i2c_master_stop(cmdLink);
  error_code = i2c_master_cmd_begin(_I2C_num, cmdLink, AM232x_TIMEOUT / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmdLink);
  if (error_code != ESP_OK) {
    rlog_e(_name, "Failed to wake sensor: %d %s!", error_code, esp_err_to_name(error_code));
    return this->rSensor::setEspError(error_code, false);
  };
  ets_delay_us(AM232x_WAKE_DELAY);

  // Step two: send the read command
  cmdLink = i2c_cmd_link_create();
  i2c_master_start(cmdLink);
  i2c_master_write_byte(cmdLink, (AM232x_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmdLink, AM232x_CMD_READ_REGISTERS, ACK_CHECK_EN);
  i2c_master_write_byte(cmdLink, reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmdLink, len, ACK_CHECK_EN);
  i2c_master_stop(cmdLink);
  error_code = i2c_master_cmd_begin(_I2C_num, cmdLink, AM232x_TIMEOUT / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmdLink);
  if (error_code != ESP_OK) {
    rlog_e(_name, "Failed to send command 0x%.2x: %d %s!", reg, error_code, esp_err_to_name(error_code));
    return this->rSensor::setEspError(error_code, false);
  };
  ets_delay_us(AM232x_READ_DELAY);

  // Steps three : to return the data read
  cmdLink = i2c_cmd_link_create();
  i2c_master_start(cmdLink);
  i2c_master_write_byte(cmdLink, (AM232x_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read(cmdLink, buf, len+4, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmdLink);
  error_code = i2c_master_cmd_begin(_I2C_num, cmdLink, AM232x_TIMEOUT / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmdLink);
  if (error_code != ESP_OK) {
    rlog_e(_name, "Failed to read register 0x%.2x: %d %s!", reg, error_code, esp_err_to_name(error_code));
    return this->rSensor::setEspError(error_code, false);
  };

  // Check resonse
  if ((buf[0] != AM232x_CMD_READ_REGISTERS) || (buf[1] != len)) {
    rlog_e(_name, "Failed to read register 0x%.2x: bad response 0x%.2x 0x%.2x!", reg, buf[0], buf[1]);
    this->rSensor::setRawStatus(SENSOR_STATUS_ERROR, false);
    return SENSOR_STATUS_ERROR;
  };
  // Check CRC16
  uint16_t crcRcvd = (buf[len+3] << 8) | buf[len+2];
  if (CRC16(buf, len+2) != crcRcvd) {
    rlog_e(_name, "CRC check failed: recieved 0x%.4x, calculated 0x%.4x!", crcRcvd, CRC16(buf, len+2));
    this->rSensor::setRawStatus(SENSOR_STATUS_CRC_ERROR, false);
    return SENSOR_STATUS_CRC_ERROR;
  };
  return this->rSensor::setEspError(error_code, false);
}

/**
 * Reading device information
 * */
sensor_status_t AM232x::readDeviceInfo()
{
  // Return：0x03 + length (0x07) + Model (16 bit) + version number (8 bit) + ID (32 bit) + CRC  
  uint8_t bufData[11] = {0};
  sensor_status_t ret = readRegister(0x08, 7, bufData);
  if (ret == SENSOR_STATUS_OK) {
    _devInfo.model = (bufData[2] << 8) | bufData[3];
    _devInfo.version = bufData[4];
    _devInfo.device_id = (((((bufData[5] << 8) | bufData[6]) << 8) | bufData[7]) << 8) | bufData[8];
    // If the Chinese fuckers didn't even bother to register the sensor model...
    if (_devInfo.model == 0) _devInfo.model = 2320; 
    rlog_i(_name, "Detected sensor AM%d on bus %d at address 0x%.2X, version %d, device id: 0x%.8x", _devInfo.model, _I2C_num, AM232x_ADDRESS, _devInfo.version, _devInfo.device_id);
  };
  return ret;
}

/**
 * Read temperature and humidity
 * */
sensor_status_t AM232x::readRawData()
{
  // Return：0x03 + length (0x04) + humidity high + humidity low + temperature high + temperature low + CRC
  uint8_t bufData[8] = {0};
  sensor_status_t ret = readRegister(0x00, 4, bufData);
  if (ret == SENSOR_STATUS_OK) {
    // Convering data
    float valueHum = ((bufData[2] << 8) | bufData[3]) / 10.0;
    float valueTemp = (((bufData[4] & 0x7F) << 8) | bufData[5]) / 10.0;
    if (bufData[4] & 0x80) valueTemp = -valueTemp;
    this->rSensorX2::setRawValues(valueHum, valueTemp);
  };
  return ret;
}

