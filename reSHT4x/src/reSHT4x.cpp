#include "reSHT4x.h"
#include "freertos/FreeRTOS.h"
#include "rLog.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "rom/ets_sys.h"
#include "driver/i2c.h"

// soft reset [ACK]
#define SHT4x_CMD_SOFT_RESET 								0x94

// read serial number [2 * 8-bit data; 8-bit CRC; 2 * 8-bit data; 8-bit CRC]
#define SHT4x_CMD_READ_SERIAL_NUMBER				0x89

// measure T & RH [2 * 8-bit T-data; 8-bit CRC; 2 * 8-bit RH-data; 8-bit CRC]
// lowest precision (low repeatability)
#define SHT4x_CMD_MEASURE_LOW_PRECISION 		0xE0
// medium precision (medium repeatability)
#define SHT4x_CMD_MEASURE_MEDIUM_PRECISION 	0xF6
// high precision (high repeatability) 
#define SHT4x_CMD_MEASURE_HIGH_PRECISION 		0xFD

// activate heater, including a high precision measurement just before deactivation [2 * 8-bit T-data; 8-bit CRC; 2 * 8-bit RH-data; 8-bit CRC]
// activate heater with 200mW for 1s
#define SHT4x_CMD_HEATER_200MW_1000MS				0x39
// activate heater with 200mW for 0.1s, including a high precision measurement just before deactivation
#define SHT4x_CMD_HEATER_200MW_100MS				0x32
// activate heater with 110mW for 1s
#define SHT4x_CMD_HEATER_110MW_1000MS				0x2F
// activate heater with 110mW for 0.1s, including a high precision measurement just before deactivation
#define SHT4x_CMD_HEATER_110MW_100MS				0x24
// activate heater with 20mW for 1s
#define SHT4x_CMD_HEATER_20MW_1000MS				0x1E
// activate heater with 20mW for 0.1s, including a high precision measurement just before deactivation
#define SHT4x_CMD_HEATER_20MW_100MS					0x15

#define SHT4x_CRC_BASE		           				0xFF
#define SHT4x_TIMEOUT                				3000
#define SHT4x_DELAY_SOFT_RESET_uS    				1000
#define SHT4x_DELAY_READ_SERIAL_uS   				1000
#define SHT4x_DELAY_READ_LOW_uS							1600
#define SHT4x_DELAY_READ_MEDIUM_uS					4500
#define SHT4x_DELAY_READ_HIGH_uS						8300
#define SHT4x_DELAY_HEATER_1000_MS 	 				1100
#define SHT4x_DELAY_HEATER_100_MS 	 				110

#define SHT4x_LOG_MSG_HEATER_ACTIVATE				"Sensor [%s]: heater activate for %d mW and %f s"

static const char* logTAG = "SHT4x";

SHT4x::SHT4x(uint8_t eventId):rSensorHT(eventId)
{
  _I2C_num = I2C_NUM_0;
  _I2C_address = 0;
	_mode = SHT4x_PRECISION_HIGH;
}

/**
 * Dynamically creating internal items on the heap
 * */
bool SHT4x::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
  const i2c_port_t numI2C, const uint8_t addrI2C, const SHT4x_PRECISION mode,
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
	_mode = mode;
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
bool SHT4x::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  const i2c_port_t numI2C, const uint8_t addrI2C, const SHT4x_PRECISION mode,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
	_mode = mode;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX2::setSensorItems(item1, item2);
  // Start device
  return sensorStart();
}

/**
 * Start device
 * */
sensor_status_t SHT4x::sensorReset()
{
	readSerialNumber();
	return softReset();
}

/**
 * Soft reset
 * */
sensor_status_t SHT4x::softReset()
{
  SENSOR_ERR_CHECK(sendCommand(SHT4x_CMD_SOFT_RESET), RSENSOR_LOG_MSG_RESET_FAILED);
  rlog_i(logTAG, RSENSOR_LOG_MSG_RESET, _name);
	ets_delay_us(SHT4x_DELAY_SOFT_RESET_uS);
	return SENSOR_STATUS_OK;
}

/**
 * Reading serial number
 * */
uint32_t SHT4x::readSerialNumber()
{
	esp_err_t err = readBuffer(SHT4x_CMD_READ_SERIAL_NUMBER, SHT4x_DELAY_READ_SERIAL_uS, 6);
	if (err == ESP_OK) {
		uint32_t serialNum = (((((_bufData[0] << 8) | _bufData[1]) << 8) | _bufData[3]) << 8) | _bufData[4];
		rlog_i(logTAG, "Sensor [%s] serial number: %.8x", _name, serialNum);
		return serialNum;
	};
	rlog_w(logTAG, "Failed to read serial number for sensor [%s]: %d %s!", _name, err, esp_err_to_name(err));
	return 0;
}

/**
 * Sending commands and receiving data from the sensor
 * */
esp_err_t SHT4x::sendCommand(uint8_t command)
{
	return writeI2C(_I2C_num, _I2C_address, &command, 1, nullptr, 0, SHT4x_TIMEOUT);
}

esp_err_t SHT4x::readBuffer(uint8_t command, const uint32_t usWaitData, const uint8_t bytes)
{
	return readI2C_CRC8(_I2C_num, _I2C_address, &command, 1, _bufData, bytes, usWaitData, SHT4x_CRC_BASE, SHT4x_TIMEOUT);
}

/**
 * Heater
 * */
sensor_status_t SHT4x::activateHeater(const SHT4x_HEATER heater_mode)
{
	// Acivate heater
	switch (heater_mode) {
		case SHT4x_HEATER_200mW_1s:
			SENSOR_ERR_CHECK(readBuffer(SHT4x_CMD_HEATER_200MW_1000MS, SHT4x_DELAY_HEATER_1000_MS, 6), RSENSOR_LOG_MSG_HEATER_SET_FAILED);
			rlog_i(logTAG, SHT4x_LOG_MSG_HEATER_ACTIVATE, _name, 200, 1);
			break;
		case SHT4x_HEATER_200mW_100ms:
			SENSOR_ERR_CHECK(readBuffer(SHT4x_CMD_HEATER_200MW_100MS, SHT4x_DELAY_HEATER_100_MS, 6), RSENSOR_LOG_MSG_HEATER_SET_FAILED);
			rlog_i(logTAG, SHT4x_LOG_MSG_HEATER_ACTIVATE, _name, 200, 0.1);
			break;
		case SHT4x_HEATER_110mW_1s:
			SENSOR_ERR_CHECK(readBuffer(SHT4x_CMD_HEATER_110MW_1000MS, SHT4x_DELAY_HEATER_1000_MS, 6), RSENSOR_LOG_MSG_HEATER_SET_FAILED);
			rlog_i(logTAG, SHT4x_LOG_MSG_HEATER_ACTIVATE, _name, 110, 1);
			break;
		case SHT4x_HEATER_110mW_100ms:
			SENSOR_ERR_CHECK(readBuffer(SHT4x_CMD_HEATER_110MW_100MS, SHT4x_DELAY_HEATER_100_MS, 6), RSENSOR_LOG_MSG_HEATER_SET_FAILED);
			rlog_i(logTAG, SHT4x_LOG_MSG_HEATER_ACTIVATE, _name, 110, 0.1);
			break;
		case SHT4x_HEATER_20mW_1s:
			SENSOR_ERR_CHECK(readBuffer(SHT4x_CMD_HEATER_20MW_1000MS, SHT4x_DELAY_HEATER_1000_MS, 6), RSENSOR_LOG_MSG_HEATER_SET_FAILED);
			rlog_i(logTAG, SHT4x_LOG_MSG_HEATER_ACTIVATE, _name, 20, 1);
			break;
		case SHT4x_HEATER_20mW_100ms:
			SENSOR_ERR_CHECK(readBuffer(SHT4x_CMD_HEATER_20MW_100MS, SHT4x_DELAY_HEATER_100_MS, 6), RSENSOR_LOG_MSG_HEATER_SET_FAILED);
			rlog_i(logTAG, SHT4x_LOG_MSG_HEATER_ACTIVATE, _name, 20, 0.1);
			break;
		default:
			return SENSOR_STATUS_NOT_SUPPORTED;
			break;
	}
	return SENSOR_STATUS_OK;
}

/**
 * Reading data from the sensor
 * */
sensor_status_t SHT4x::readRawData()
{
	switch (_mode) {
		case SHT4x_PRECISION_LOW:
			SENSOR_ERR_CHECK(readBuffer(SHT4x_CMD_MEASURE_LOW_PRECISION, SHT4x_DELAY_READ_LOW_uS, 6), RSENSOR_LOG_MSG_READ_DATA_FAILED); 
			break;
		case SHT4x_PRECISION_MEDIUM: 
			SENSOR_ERR_CHECK(readBuffer(SHT4x_CMD_MEASURE_MEDIUM_PRECISION, SHT4x_DELAY_READ_MEDIUM_uS, 6), RSENSOR_LOG_MSG_READ_DATA_FAILED); 
			break;
		case SHT4x_PRECISION_HIGH: 
			SENSOR_ERR_CHECK(readBuffer(SHT4x_CMD_MEASURE_HIGH_PRECISION, SHT4x_DELAY_READ_HIGH_uS, 6), RSENSOR_LOG_MSG_READ_DATA_FAILED); 
			break;
	};

	uint16_t rawTemp = ((uint16_t)_bufData[0] << 8) | (uint16_t)_bufData[1];
	uint16_t rawHumd = ((uint16_t)_bufData[3] << 8) | (uint16_t)_bufData[4];

	return setRawValues(125.0 * (float)rawHumd / 65535.0 - 6.0, 175.0 * (float)rawTemp / 65535.0 - 45.0);
};
