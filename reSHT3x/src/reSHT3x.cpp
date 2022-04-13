#include "reSHT3x.h"
#include "freertos/FreeRTOS.h"
#include "rLog.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "driver/i2c.h"

#define SHT3xD_CMD_READ_SERIAL_NUMBER 0x3780

#define SHT3xD_CMD_READ_STATUS        0xF32D
#define SHT3xD_CMD_CLEAR_STATUS       0x3041

#define SHT3xD_CMD_HEATER_ENABLE      0x306D
#define SHT3xD_CMD_HEATER_DISABLE     0x3066

#define SHT3xD_CMD_SOFT_RESET         0x30A2

#define SHT3xD_CMD_CLOCK_STRETCH_H    0x2C06
#define SHT3xD_CMD_CLOCK_STRETCH_M    0x2C0D
#define SHT3xD_CMD_CLOCK_STRETCH_L    0x2C10

#define SHT3xD_CMD_POLLING_H          0x2400
#define SHT3xD_CMD_POLLING_M          0x240B
#define SHT3xD_CMD_POLLING_L          0x2416

#define SHT3xD_CMD_ART                0x2B32

#define SHT3xD_CMD_PERIODIC_HALF_H    0x2032
#define SHT3xD_CMD_PERIODIC_HALF_M    0x2024
#define SHT3xD_CMD_PERIODIC_HALF_L    0x202F
#define SHT3xD_CMD_PERIODIC_1_H       0x2130
#define SHT3xD_CMD_PERIODIC_1_M       0x2126
#define SHT3xD_CMD_PERIODIC_1_L       0x212D
#define SHT3xD_CMD_PERIODIC_2_H       0x2236
#define SHT3xD_CMD_PERIODIC_2_M       0x2220
#define SHT3xD_CMD_PERIODIC_2_L       0x222B
#define SHT3xD_CMD_PERIODIC_4_H       0x2334
#define SHT3xD_CMD_PERIODIC_4_M       0x2322
#define SHT3xD_CMD_PERIODIC_4_L       0x2329
#define SHT3xD_CMD_PERIODIC_10_H      0x2737
#define SHT3xD_CMD_PERIODIC_10_M      0x2721
#define SHT3xD_CMD_PERIODIC_10_L      0x272A

#define SHT3xD_CMD_FETCH_DATA         0xE000
#define SHT3xD_CMD_STOP_PERIODIC      0x3093

#define SHT3xD_CMD_READ_ALR_LIMIT_LS  0xE102
#define SHT3xD_CMD_READ_ALR_LIMIT_LC  0xE109
#define SHT3xD_CMD_READ_ALR_LIMIT_HS  0xE11F
#define SHT3xD_CMD_READ_ALR_LIMIT_HC  0xE114

#define SHT3xD_CMD_WRITE_ALR_LIMIT_HS 0x611D
#define SHT3xD_CMD_WRITE_ALR_LIMIT_HC 0x6116
#define SHT3xD_CMD_WRITE_ALR_LIMIT_LC 0x610B
#define SHT3xD_CMD_WRITE_ALR_LIMIT_LS 0x6100

#define SHT3xD_CMD_NO_SLEEP           0x303E

#define SHT3xD_CRC_BASE		            0xFF
#define SHT3xD_TIMEOUT                3000
#define SHT3xD_DELAY_SOFT_RESET       1500
#define SHT3xD_DELAY_STOP_PERIODIC    1000
#define SHT3xD_DELAY_MEASURE_L	      4500
#define SHT3xD_DELAY_MEASURE_M	      6500
#define SHT3xD_DELAY_MEASURE_H	      15500

static const char* logTAG = "SHT3x";

SHT3xD::SHT3xD():rSensorHT()
{
  _I2C_num = 0;
  _I2C_address = 0;
	_frequency = SHT3xD_SINGLE;
	_mode = SHT3xD_MODE_NOHOLD;
	_repeatability = SHT3xD_REPEATABILITY_MEDIUM;
}

/**
 * Dynamically creating internal items on the heap
 * */
bool SHT3xD::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
  const int numI2C, const uint8_t addrI2C, const SHT3xD_FREQUENCY frequency, const SHT3xD_MODE mode, const SHT3xD_REPEATABILITY repeatability, 
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
	_frequency = frequency;
	_mode = mode;
	_repeatability = repeatability;
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
bool SHT3xD::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  const int numI2C, const uint8_t addrI2C, const SHT3xD_FREQUENCY frequency, const SHT3xD_MODE mode, const SHT3xD_REPEATABILITY repeatability,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
	_frequency = frequency;
	_mode = mode;
	_repeatability = repeatability;
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
bool SHT3xD::sensorReset()
{
	readSerialNumber();
	sensor_status_t err = softReset();
	if (err != SENSOR_STATUS_OK) return false;
 	err = setMode(_frequency, _mode, _repeatability);
	if (err != SENSOR_STATUS_OK) return false;
	err = clearStatusRegister();
	if (err != SENSOR_STATUS_OK) return false;
	err = setHeater(false);
	if (err != SENSOR_STATUS_OK) return false;
	return true;
}

/**
 * Soft reset
 * */
sensor_status_t SHT3xD::softReset()
{
  esp_err_t err = sendCommand(SHT3xD_CMD_SOFT_RESET);
	if (err == ESP_OK) {
		ets_delay_us(SHT3xD_DELAY_SOFT_RESET);
		rlog_i(logTAG, "Sensor [%s] has been reset!", _name);
	}	else {
	  rlog_e(logTAG, "Soft reset failed: %d %s!", err, esp_err_to_name(err));
	};
	return this->rSensor::setEspError(err, true);
}

/**
 * Reading serial number
 * */
uint32_t SHT3xD::readSerialNumber()
{
	esp_err_t err = readBuffer(SHT3xD_CMD_READ_SERIAL_NUMBER, 50, 6);
	this->rSensor::setEspError(err, false);
	if (err == ESP_OK) {
		uint32_t serialNum = (((((_bufData[0] << 8) | _bufData[1]) << 8) | _bufData[3]) << 8) | _bufData[4];
		rlog_i(logTAG, "Sensor [ %s ] serial number: %.8x", _name, serialNum);
		return serialNum;
	}
	else rlog_w(logTAG, "Failed to read serial number for sensor [ %s ]: %d %s!", _name, err, esp_err_to_name(err));
	return 0;
}

/**
 * Changing the operating mode
 * */
sensor_status_t SHT3xD::setMode(const SHT3xD_FREQUENCY frequency, const SHT3xD_MODE mode, const SHT3xD_REPEATABILITY repeatability)
{
	_mode = mode;
  return startPeriodicMode(frequency, repeatability);
}

/**
 * Sending commands and receiving data from the sensor
 * */
esp_err_t SHT3xD::sendCommand(const uint16_t command)
{
	_bufCmd[0] = command >> 8;
	_bufCmd[1] = command & 0xFF;
	return writeI2C(_I2C_num, _I2C_address, _bufCmd, 2, nullptr, 0, SHT3xD_TIMEOUT);
}

esp_err_t SHT3xD::readBuffer(const uint16_t command, const uint32_t usWaitData, const uint8_t bytes)
{
	_bufCmd[0] = command >> 8;
	_bufCmd[1] = command & 0xFF;
	return readI2C_CRC8(_I2C_num, _I2C_address, _bufCmd, 2, _bufData, bytes, usWaitData, SHT3xD_CRC_BASE, SHT3xD_TIMEOUT);
}

/**
 * Status register
 * */
sensor_status_t SHT3xD::clearStatusRegister()
{
	esp_err_t err = sendCommand(SHT3xD_CMD_CLEAR_STATUS);
	if (err == ESP_OK) rlog_i(logTAG, "%s: status register has been cleared", _name);
	else rlog_e(logTAG, "Failed to clear status register for sensor [ %s ]: %d %s!", _name, err, esp_err_to_name(err));
	return this->rSensor::setEspError(err, false);
}

SHT3xD_STATUS SHT3xD::readStatusRegister()
{
  SHT3xD_STATUS ret;
	esp_err_t err = readBuffer(SHT3xD_CMD_READ_STATUS, 0, 3);
  if (err == ESP_OK) {
		// BIT0 - Write data checksum status :: '0': checksum of last write transfer was correct / '1': checksum of last write transfer failed
		ret.lastChecksumFailed = _bufData[1] & 0x01;
		// BIT1 - Command status :: '0': last command executed successfully  / '1': last command not processed. It was either invalid, failed the integrated command checksum 
		ret.lastCommandFailed = _bufData[1] & 0x02;
		// BIT2-3 - Reserved
		// BIT4 - System reset detected :: '0': no reset detected since last ‘clear status register’ command / '1': reset detected (hard reset, soft reset command or supply fail)
		ret.systemResetDetected = _bufData[1] & 0x10;
		// BIT5-9 - Reserved
		// BIT10 - T tracking alert :: ‘0’ : no alert / ‘1’ : alert
		ret.alertTemperature = _bufData[0] & 0x04;
		// BIT11 - RH tracking alert :: ‘0’ : no alert / ‘1’ : alert
  	ret.alertHumidity = _bufData[0] & 0x08;
		// BIT12 - Reserved
		// BIT13 - Heater status :: ‘0’ : Heater OFF / ‘1’ : Heater ON
    ret.heaterStatus = _bufData[0] & 0x20;
		// BIT14 - Reserved
		// BIT15 - Alert pending status :: '0': no pending alerts / '1': at least one pending alert
    ret.alertPendingStatus = _bufData[0] & 0x80;
  } else rlog_e(logTAG, "Failed to read status register for sensor [ %s ]: %d %s!", _name, err, esp_err_to_name(err));
  this->rSensor::setEspError(err, false);	
	return ret;
}

/**
 * The SHT3x is equipped with an internal heater, which is meant for plausibility checking only. 
 * The temperature increase achieved by the heater depends on various parameters and lies in the range of a few degrees centigrade. 
 * It can be switched on and off by command. The status is listed in the status register. 
 * After a reset the heater is disabled (default condition).
 * */
sensor_status_t SHT3xD::setHeater(const bool heaterMode)
{
  esp_err_t err = sendCommand(heaterMode ? SHT3xD_CMD_HEATER_ENABLE : SHT3xD_CMD_HEATER_DISABLE);
	if (err == ESP_OK) {
		SHT3xD_STATUS status = readStatusRegister();
		if (status.heaterStatus != heaterMode) {
			rlog_e(logTAG, "Heating mode is not confirmed by sensor [ %s ]", _name);
			this->rSensor::setRawStatus(SENSOR_STATUS_ERROR, true);
			return SENSOR_STATUS_ERROR;
		};
		rlog_i(logTAG, "%s: heater is %s", _name, heaterMode ? RSENSOR_LOG_MSG_HEATER_ON : RSENSOR_LOG_MSG_HEATER_OFF);
	}	else rlog_e(logTAG, "Failed to set heating mode for sensor [ %s ]: %d %s", err, esp_err_to_name(err));
	return this->rSensor::setEspError(err, true);
}

bool SHT3xD::isHeaterEnabled()
{
  SHT3xD_STATUS status = readStatusRegister();
	return status.heaterStatus;
}

/**
 * In this mode one issued measurement command yields a stream of data pairs. 
 * Each data pair consists of one 16 bit temperature and one 16 bit humidity value.
 * Transmission of the measurement data can be initiated through the fetch data command.
 * 
 * The periodic data acquisition mode can be stopped using the break command. 
 * It is recommended to stop the periodic data acquisition prior to sending another command (except Fetch Data command) using the break command. 
 * Upon reception of the break command the sensor will abort the ongoing measurement and enter the single shot mode. This takes 1ms.
 * */
sensor_status_t SHT3xD::startPeriodicMode(const SHT3xD_FREQUENCY frequency, const SHT3xD_REPEATABILITY repeatability)
{
  esp_err_t err = ESP_FAIL;
  if (frequency == SHT3xD_SINGLE) {
		if (_frequency != SHT3xD_SINGLE) {
  		err = sendCommand(SHT3xD_CMD_STOP_PERIODIC);
			ets_delay_us(SHT3xD_DELAY_STOP_PERIODIC);
		} else {
			err = ESP_OK;
		};
	} else {
		switch (repeatability) {
			case SHT3xD_REPEATABILITY_LOW:
				switch (frequency) {
					case SHT3xD_FREQUENCY_HZ5:
						err = sendCommand(SHT3xD_CMD_PERIODIC_HALF_L);
						vTaskDelay(2000 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_1HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_1_L);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_2HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_2_L);
						vTaskDelay(500 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_4HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_4_L);
						vTaskDelay(250 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_10HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_10_L);
						vTaskDelay(100 / portTICK_PERIOD_MS);
						break;
					default:
						err = ESP_FAIL;
						break;
				};
				break;
			case SHT3xD_REPEATABILITY_MEDIUM:
				switch (frequency) {
					case SHT3xD_FREQUENCY_HZ5:
						err = sendCommand(SHT3xD_CMD_PERIODIC_HALF_M);
						vTaskDelay(2000 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_1HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_1_M);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_2HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_2_M);
						vTaskDelay(500 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_4HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_4_M);
						vTaskDelay(250 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_10HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_10_M);
						vTaskDelay(100 / portTICK_PERIOD_MS);
						break;
					default:
						err = ESP_FAIL;
						break;
				};
				break;
			case SHT3xD_REPEATABILITY_HIGH:
				switch (frequency) {
					case SHT3xD_FREQUENCY_HZ5:
						err = sendCommand(SHT3xD_CMD_PERIODIC_HALF_H);
						vTaskDelay(2000 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_1HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_1_H);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_2HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_2_H);
						vTaskDelay(500 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_4HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_4_H);
						vTaskDelay(250 / portTICK_PERIOD_MS);
						break;
					case SHT3xD_FREQUENCY_10HZ:
						err = sendCommand(SHT3xD_CMD_PERIODIC_10_H);
						vTaskDelay(100 / portTICK_PERIOD_MS);
						break;
					default:
						err = ESP_FAIL;
						break;
				};
				break;
		};
	};
	// Check results
	if (err == ESP_OK) {
		_frequency = frequency;
		_repeatability = repeatability;
		rlog_i(logTAG, "%s: set measurement mode: frequency=0x%.2x, repeatability=0x%.2x", _name, frequency, repeatability);
	} else {
		rlog_e(logTAG, "Failed to set measurement mode: %d %s!", err, esp_err_to_name(err));
	};
	return this->rSensor::setEspError(err, true);
}

/**
 * Readout of measurement results for periodic mode
 * After issuing the ART command the sensor will start acquiring data with a frequency of 4Hz.
 * */
sensor_status_t SHT3xD::activateART()
{
	esp_err_t err = sendCommand(SHT3xD_CMD_ART);
	if (err == ESP_OK) rlog_i(logTAG, "%s: accelerated response time activated", _name);
	else rlog_e(logTAG, "Failed to set ART: %d %s!", err, esp_err_to_name(err));
	return this->rSensor::setEspError(err, true);
}

/**
 * Recalculation of values
 * */
value_t SHT3xD::raw2temperature(uint16_t rawValue)
{
	return 175.0 * (float)rawValue / 65535.0 - 45.0;
}


value_t SHT3xD::raw2humidity(uint16_t rawValue)
{
	return 100.0 * (float)rawValue / 65535.0;
}

uint16_t SHT3xD::temperature2raw(value_t value)
{
	return (value + 45.0) / 175.0 * 65535.0;
}

uint16_t SHT3xD::humidity2raw(value_t value)
{
	return value / 100.0 * 65535.0;
}

/**
 * Reading current values from the sensor
 * */
sensor_status_t SHT3xD::readRawDataCustom(const uint16_t command, const uint32_t measure_delay)
{
	// Read temperature and humidity RAW values
	esp_err_t err = readBuffer(command, measure_delay, 6);
  if (err != ESP_OK) {
    rlog_e(logTAG, "Failed to read values from sensor [ %s ]: %d %s!", _name, err, esp_err_to_name(err));
    return this->rSensor::setEspError(err, false);
  };
  // Store values in sensors: first two bytes are the temperature (in most other sensors, the humidity goes first)
  this->rSensorX2::setRawValues(raw2humidity((_bufData[3] << 8) | _bufData[4]), raw2temperature((_bufData[0] << 8) | _bufData[1]));
  return SENSOR_STATUS_OK;
}

sensor_status_t SHT3xD::readRawData()
{
	if (_frequency == SHT3xD_SINGLE) {
		// Single shot mode
		switch (_mode) {
			case SHT3xD_MODE_HOLD:
				switch (_repeatability) {
				case SHT3xD_REPEATABILITY_LOW:
				  return readRawDataCustom(SHT3xD_CMD_CLOCK_STRETCH_L, SHT3xD_DELAY_MEASURE_L);
				case SHT3xD_REPEATABILITY_MEDIUM:
					return readRawDataCustom(SHT3xD_CMD_CLOCK_STRETCH_M, SHT3xD_DELAY_MEASURE_M);
				case SHT3xD_REPEATABILITY_HIGH:
					return readRawDataCustom(SHT3xD_CMD_CLOCK_STRETCH_H, SHT3xD_DELAY_MEASURE_H);
				default:
					return SENSOR_STATUS_NOT_SUPPORTED;
				};
				break;
			case SHT3xD_MODE_NOHOLD:
				switch (_repeatability) {
				case SHT3xD_REPEATABILITY_LOW:
				  return readRawDataCustom(SHT3xD_CMD_POLLING_L, SHT3xD_DELAY_MEASURE_L);
				case SHT3xD_REPEATABILITY_MEDIUM:
					return readRawDataCustom(SHT3xD_CMD_POLLING_M, SHT3xD_DELAY_MEASURE_M);
				case SHT3xD_REPEATABILITY_HIGH:
					return readRawDataCustom(SHT3xD_CMD_POLLING_H, SHT3xD_DELAY_MEASURE_H);
				default:
					return SENSOR_STATUS_NOT_SUPPORTED;
				};
				break;
			default:
			  return SENSOR_STATUS_NOT_SUPPORTED;
		};
	} else {
		// Periodic data acquisition mode
		return readRawDataCustom(SHT3xD_CMD_FETCH_DATA, 0);
	};
}


/**
 * Setting limits for alarm triggering
 * */
sensor_status_t SHT3xD::writeAlertDataCustom(const uint16_t command, const value_t humidity, const value_t temperature)
{
	// Convert to raw
	uint16_t rawTemperature = temperature2raw(temperature);
	uint16_t rawHumidity = humidity2raw(humidity);
	uint16_t rawData = (rawHumidity & 0xFE00) | ((rawTemperature >> 7) & 0x001FF);
	// Prepare data
	uint8_t	bufAlert[3];
  bufAlert[0] = rawData >> 8;
	bufAlert[1] = rawData & 0xFF;
	bufAlert[2] = CRC8(bufAlert[0], bufAlert[1], SHT3xD_CRC_BASE);
	// Send data to device
	_bufCmd[0] = command >> 8;
	_bufCmd[1] = command & 0xFF;
	esp_err_t err = writeI2C(_I2C_num, _I2C_address, _bufCmd, 2, bufAlert, 3, SHT3xD_TIMEOUT);
  if (err == ESP_OK) {
		rlog_i(logTAG, "%s: alert limit 0x%.2x set successfully: humidity=%f, temperature=%f", _name, command, humidity, temperature);
		SHT3xD_STATUS status = readStatusRegister();
		if (status.lastChecksumFailed) {
			rlog_e(logTAG, "Failed to send alert limits for sensor [ %s ]: checksum failed!", _name);
			this->rSensor::setRawStatus(SENSOR_STATUS_CRC_ERROR, true);
			return SENSOR_STATUS_CRC_ERROR;
		};
		if (status.lastCommandFailed) {
			rlog_e(logTAG, "Failed to send alert limits for sensor [ %s ]: command failed!", _name);
			this->rSensor::setRawStatus(SENSOR_STATUS_ERROR, true);
			return SENSOR_STATUS_ERROR;
		};
	}	else rlog_e(logTAG, "Failed to send alert limits for sensor [ %s ]: %d %s!", _name, err, esp_err_to_name(err));
  return this->rSensor::setEspError(err, true);
}

sensor_status_t SHT3xD::setAlertLow(const value_t temperatureSet, const value_t temperatureClear, const value_t humiditySet, const value_t humidityClear) 
{
	sensor_status_t ret = writeAlertDataCustom(SHT3xD_CMD_WRITE_ALR_LIMIT_LS, temperatureSet, humiditySet);
	if (ret == SENSOR_STATUS_OK) {
		ret = writeAlertDataCustom(SHT3xD_CMD_WRITE_ALR_LIMIT_LC, temperatureClear, humidityClear);
	};
	return ret;
}

sensor_status_t SHT3xD::setAlertHigh(const value_t temperatureSet, const value_t temperatureClear, const value_t humiditySet, const value_t humidityClear) 
{
	sensor_status_t ret = writeAlertDataCustom(SHT3xD_CMD_WRITE_ALR_LIMIT_HS, temperatureSet, humiditySet);
	if (ret == SENSOR_STATUS_OK) {
		ret = writeAlertDataCustom(SHT3xD_CMD_WRITE_ALR_LIMIT_HC, temperatureClear, humidityClear);
	};
	return ret;
}

/**
 * Reading alert limits from sensor
 * */
sensor_status_t SHT3xD::readAlertDataCustom(const uint16_t command, value_t *humidity, value_t *temperature)
{
	// Read temperature and humidity RAW values
	esp_err_t err = readBuffer(command, 0, 3);
  if (err == ESP_OK) {
		// Convert to float values
		uint16_t rawData = (_bufData[0] << 8) | _bufData[1];
		*humidity = raw2humidity(rawData & 0xFE00);
		*temperature = raw2temperature(rawData << 7);
  } else rlog_e(logTAG, "Failed to read alert limits for sensor [ %s ]: %d %s!", _name, err, esp_err_to_name(err));
  return this->rSensor::setEspError(err, true);
};

sensor_status_t SHT3xD::readAlertLowSet(const uint16_t command, value_t *humidity, value_t *temperature)
{
	return readAlertDataCustom(SHT3xD_CMD_READ_ALR_LIMIT_LS, humidity, temperature);
}

sensor_status_t SHT3xD::readAlertLowClear(const uint16_t command, value_t *humidity, value_t *temperature)
{
	return readAlertDataCustom(SHT3xD_CMD_READ_ALR_LIMIT_LC, humidity, temperature);
}

sensor_status_t SHT3xD::readAlertHighSet(const uint16_t command, value_t *humidity, value_t *temperature)
{
	return readAlertDataCustom(SHT3xD_CMD_READ_ALR_LIMIT_HS, humidity, temperature);
}

sensor_status_t SHT3xD::readAlertHighClear(const uint16_t command, value_t *humidity, value_t *temperature)
{
	return readAlertDataCustom(SHT3xD_CMD_READ_ALR_LIMIT_HC, humidity, temperature);
}

bool SHT3xD::isAlertPendingStatus()
{
  SHT3xD_STATUS status = readStatusRegister();
	return status.alertPendingStatus;
}

bool SHT3xD::isAlertHumidity()
{
  SHT3xD_STATUS status = readStatusRegister();
	return status.alertHumidity;
}

bool SHT3xD::isAlertTemperature()
{
  SHT3xD_STATUS status = readStatusRegister();
	return status.alertTemperature;
}


