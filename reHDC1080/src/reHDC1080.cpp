#include "reHDC1080.h"
#include "freertos/FreeRTOS.h"
#include "rLog.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "rom/ets_sys.h"
#include "driver/i2c.h"

#define	HDC1080_REG_TEMPERATURE								0x00
#define HDC1080_REG_HUMIDITY									0x01
#define HDC1080_REG_CONFIGURATION							0x02
#define HDC1080_REG_MANUFACTURER_ID 					0xFE
#define HDC1080_REG_DEVICE_ID									0xFF
#define HDC1080_REG_SERIAL_ID_FIRST						0xFB
#define HDC1080_REG_SERIAL_ID_MID							0xFC
#define HDC1080_REG_SERIAL_ID_LAST						0xFD

#define HDC1080_VAL_MANUFACTURER_ID						0x5449
#define HDC1080_VAL_DEVICE_ID									0x1050

#define HDC1080_TIMEOUT                				3000
#define HDC1080_DELAY_SOFT_RESET_MS    				15

#define HDC1080_DELAY_READ_HUMD_8BIT_uS   		2500
#define HDC1080_DELAY_READ_HUMD_11BIT_uS			3850
#define HDC1080_DELAY_READ_HUMD_14BIT_uS			6500
#define HDC1080_DELAY_READ_TEMP_11BIT_uS			3650
#define HDC1080_DELAY_READ_TEMP_14BIT_uS			6350

static const char* logTAG = "HDC1080";

typedef union {
	uint8_t raw[2];
	struct {
		uint8_t humidity_resolution : 2;
		uint8_t temperature_resolution : 1;
		uint8_t battery_status : 1;
		uint8_t acquisition_mode : 1;
		uint8_t heater : 1;
		uint8_t reserved : 1;
		uint8_t software_reset : 1;
    uint8_t unused;
	};
} hdc1080_registers_t; 

HDC1080::HDC1080(uint8_t eventId):rSensorHT(eventId)
{
  _I2C_num = I2C_NUM_0;
	_hres = hdc1080_hres_14bit;
	_tres = hdc1080_tres_14bit;
}

/**
 * Dynamically creating internal items on the heap
 * */
bool HDC1080::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
  const i2c_port_t numI2C, const hdc1080_humidity_resolution_t humidity_resolution, const hdc1080_temperature_resolution_t temperature_resolution,
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
	_hres = humidity_resolution;
	_tres = temperature_resolution;
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
bool HDC1080::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  const i2c_port_t numI2C, const hdc1080_humidity_resolution_t humidity_resolution, const hdc1080_temperature_resolution_t temperature_resolution,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
	_hres = humidity_resolution;
	_tres = temperature_resolution;
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
sensor_status_t HDC1080::sensorReset()
{
	readSerialNumber();
	return softReset();
}

/**
 * Reading serial number
 * */
hdc1080_serial_t HDC1080::readSerialNumber()
{
	hdc1080_serial_t ret = {0};
	uint8_t buffer[2];
	uint8_t cmd = HDC1080_REG_SERIAL_ID_FIRST;
	esp_err_t err = readI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &buffer[0], 2, 0, HDC1080_TIMEOUT);
	if (err == ESP_OK) {
		ret.serialFirst = buffer[0] << 8 | buffer[1];
		uint8_t cmd = HDC1080_REG_SERIAL_ID_MID;
		err = readI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &buffer[0], 2, 0, HDC1080_TIMEOUT);
	};
	if (err == ESP_OK) {
		ret.serialMid = buffer[0] << 8 | buffer[1];
		uint8_t cmd = HDC1080_REG_SERIAL_ID_LAST;
		err = readI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &buffer[0], 2, 0, HDC1080_TIMEOUT);
	};
	if (err == ESP_OK) {
		ret.serialLast = buffer[0] << 8 | buffer[1];
		uint8_t cmd = HDC1080_REG_MANUFACTURER_ID;
		err = readI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &buffer[0], 2, 0, HDC1080_TIMEOUT);
	};
	if (err == ESP_OK) {
		ret.manufacturerID = buffer[0] << 8 | buffer[1];
		uint8_t cmd = HDC1080_REG_DEVICE_ID;
		err = readI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &buffer[0], 2, 0, HDC1080_TIMEOUT);
	};
	if (err == ESP_OK) {
		ret.deviceID = buffer[0] << 8 | buffer[1];
		if ((ret.manufacturerID == HDC1080_VAL_MANUFACTURER_ID) && (ret.deviceID == HDC1080_VAL_DEVICE_ID)) {
			rlog_i(logTAG, "Found HDC1080 sensor: manufacturer ID %.4X, device ID %.4X, serial %.4x-%.4x-%.4x", 
				ret.manufacturerID, ret.deviceID, ret.serialFirst, ret.serialMid, ret.serialLast);
		} else {
			rlog_w(logTAG, "Found fake HDC1080 sensor: manufacturer ID %.4X, device ID %.4X, serial %.4x-%.4x-%.4x", 
				ret.manufacturerID, ret.deviceID, ret.serialFirst, ret.serialMid, ret.serialLast);
		};
		return ret;
	};
	rlog_w(logTAG, RSENSOR_LOG_MSG_READ_HADRWARE_ID, _name, err, esp_err_to_name(err));
	return ret;
}

/**
 * Soft reset
 * */
sensor_status_t HDC1080::softReset()
{
  uint8_t cmd = HDC1080_REG_CONFIGURATION;
	hdc1080_registers_t config = {0};
	config.software_reset = 0x01;
	config.acquisition_mode = 0x01;
	config.temperature_resolution = _tres;
	config.humidity_resolution = _hres;
	config.heater = 0x00;
	// Reset sensors
	SENSOR_ERR_CHECK(writeI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &config.raw[0], 2, HDC1080_TIMEOUT), RSENSOR_LOG_MSG_RESET_FAILED);
	vTaskDelay(pdMS_TO_TICKS(HDC1080_DELAY_SOFT_RESET_MS));
	// Set resolutions
	config.software_reset = 0x00;
	SENSOR_ERR_CHECK(writeI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &config.raw[0], 2, HDC1080_TIMEOUT), RSENSOR_LOG_MSG_SET_RESOLUTION_FAILED);
	return SENSOR_STATUS_OK;
}

/**
 * Set resolutions
 * */
sensor_status_t HDC1080::setResolutions(const hdc1080_humidity_resolution_t humidity_resolution, const hdc1080_temperature_resolution_t temperature_resolution)
{
  uint8_t cmd = HDC1080_REG_CONFIGURATION;
	hdc1080_registers_t config = {0};
	config.acquisition_mode = 0x01;
	config.temperature_resolution = temperature_resolution;
	config.humidity_resolution = humidity_resolution;
	SENSOR_ERR_CHECK(writeI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &config.raw[0], 2, HDC1080_TIMEOUT), RSENSOR_LOG_MSG_SET_RESOLUTION_FAILED);
	_tres = temperature_resolution;
	_hres = humidity_resolution;
	return SENSOR_STATUS_OK;
}

/**
 * Heater
 * */
sensor_status_t HDC1080::setHeater(const bool heater_enable)
{
	uint8_t cmd = HDC1080_REG_CONFIGURATION;
	hdc1080_registers_t config = {0};
	// Reading the current state of the configuration register
	esp_err_t err = readI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &config.raw[0], 2, 0, HDC1080_TIMEOUT);
	if (err == ESP_OK) {
		// If the state does not correspond to the current one, send a command to change
		if (config.heater != heater_enable) {
			config.heater = heater_enable;
			SENSOR_ERR_CHECK(writeI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &config.raw[0], 2, HDC1080_TIMEOUT), RSENSOR_LOG_MSG_HEATER_SET_FAILED);
			rlog_i(logTAG, RSENSOR_LOG_MSG_HEATER_STATE, _name, heater_enable ? RSENSOR_LOG_MSG_HEATER_ON : RSENSOR_LOG_MSG_HEATER_OFF);
			return SENSOR_STATUS_OK;
		};
	};
	rlog_e(logTAG, RSENSOR_LOG_MSG_HEATER_GET_FAILED, _name, err, esp_err_to_name(err));
	return SENSOR_STATUS_CONN_ERROR;
}

/**
 * Reading data from the sensor
 * */
sensor_status_t HDC1080::readRawData()
{
	uint8_t cmd = HDC1080_REG_TEMPERATURE;
	uint8_t rawData[4];

	// Determining the time delay for measurement
	uint32_t waitData = 0;
	switch (_tres)	{
		case hdc1080_tres_11bit:
			waitData += HDC1080_DELAY_READ_TEMP_11BIT_uS;
			break;
		default:
			waitData += HDC1080_DELAY_READ_TEMP_14BIT_uS;
			break;
	};
	switch (_hres)	{
		case hdc1080_hres_8bit:
			waitData += HDC1080_DELAY_READ_HUMD_8BIT_uS;
			break;
		case hdc1080_hres_11bit:
			waitData += HDC1080_DELAY_READ_HUMD_11BIT_uS;
			break;
		default:
			waitData += HDC1080_DELAY_READ_HUMD_14BIT_uS;
			break;
	};

	// Reading data
	SENSOR_ERR_CHECK(readI2C(_I2C_num, HDC1080_ADDRESS, &cmd, 1, &rawData[0], 4, waitData, HDC1080_TIMEOUT), RSENSOR_LOG_MSG_READ_DATA_FAILED);

	return setRawValues(((float)((rawData[2] << 8) | rawData[3]) / 65535.0) * 100.0, ((float)((rawData[0] << 8) | rawData[1]) / 65535.0) * 165.0 - 40.0);
};
