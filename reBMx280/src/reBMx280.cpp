#include "reBMx280.h"
#include "freertos/FreeRTOS.h"
#include "rStrings.h"
#include "reEsp32.h"
#include "reI2C.h"
#include "rLog.h"
#include "string.h"
#include "driver/i2c.h"

#define BMx280_REGISTER_DIG_T1        0x88
#define BMx280_REGISTER_DIG_T2        0x8A
#define BMx280_REGISTER_DIG_T3        0x8C

#define BMx280_REGISTER_DIG_P1        0x8E
#define BMx280_REGISTER_DIG_P2        0x90
#define BMx280_REGISTER_DIG_P3        0x92
#define BMx280_REGISTER_DIG_P4        0x94
#define BMx280_REGISTER_DIG_P5        0x96
#define BMx280_REGISTER_DIG_P6        0x98
#define BMx280_REGISTER_DIG_P7        0x9A
#define BMx280_REGISTER_DIG_P8        0x9C
#define BMx280_REGISTER_DIG_P9        0x9E

#define BMx280_REGISTER_DIG_H1        0xA1
#define BMx280_REGISTER_DIG_H2        0xE1
#define BMx280_REGISTER_DIG_H3        0xE3
#define BMx280_REGISTER_DIG_H4_MSB    0xE4
#define BMx280_REGISTER_DIG_H4_LSB    0xE5
#define BMx280_REGISTER_DIG_H5_MSB    0xE6
#define BMx280_REGISTER_DIG_H6        0xE7

#define BMx280_REGISTER_CHIPID        0xD0
#define BMx280_REGISTER_VERSION       0xD1
#define BMx280_REGISTER_SOFTRESET     0xE0

#define BMx280_REGISTER_CAL26         0xE1 // R calibration stored in 0xE1-0xF0

#define BMx280_REGISTER_CONTROLHUMID  0xF2
#define BMx280_REGISTER_STATUS        0XF3
#define BMx280_REGISTER_CONTROL       0xF4
#define BMx280_REGISTER_CONFIG        0xF5
#define BMx280_REGISTER_ALLDATA       0xF7
#define BMx280_REGISTER_PRESSUREDATA  0xF7
#define BMx280_REGISTER_TEMPDATA      0xFA
#define BMx280_REGISTER_HUMIDDATA     0xFD

#define BMx280_TIMEOUT                1000
#define BMx280_DELAY_SOFT_RESET       10000

BMx280::BMx280():rSensorX3()
{
  _I2C_num = 0;
  _I2C_address = 0;
  _type = BMx_UNKNOWN;
  _mode = MODE_SLEEP;
  _t_sb = STANDBY_1000ms;
  _filter = FILTER_OFF;
  _osrs_p = OSR_OFF;
  _osrs_t = OSR_OFF;
  _osrs_h = OSR_OFF;
}

// Displaying multiple values in one topic
#if CONFIG_SENSOR_DISPLAY_ENABLED

void BMx280::initDisplayMode()
{
  _displayMode = SENSOR_MIXED_ITEMS_23;
  _displayFormat = (char*)CONFIG_FORMAT_MIXED_STRING2;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

#if CONFIG_SENSOR_AS_PLAIN

bool BMx280::publishCustomValues()
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

char* BMx280::jsonCustomValues()
{
  #if CONFIG_SENSOR_DEWPOINT_ENABLE
    if ((_item2) && (_item3)) {
      char * _dew_point = _item2->jsonDataValue(false, calcDewPoint(_item2->getValue().filteredValue, _item3->getValue().filteredValue));
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
bool BMx280::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const BMx_TYPE type, const int numI2C, const uint8_t addrI2C, 
  const BMx_MODE mode, const BMx_STANDBYTIME t_sb, const BMx_IIR_FILTER filter,
  const BMx_OVERSAMPLING osrs_p, const BMx_OVERSAMPLING osrs_t, const BMx_OVERSAMPLING osrs_h,
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const sensor_filter_t filterMode3, const uint16_t filterSize3,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  _type = type;
  // Initialize internal items
  if (this->rSensorX3::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2, filterMode3, filterSize3)) {
    // Start device
    return (initHardware(numI2C, addrI2C) && (setMode(mode, t_sb, filter, osrs_p, osrs_t, osrs_h) == SENSOR_STATUS_OK));
  };
  return false;
}

void BMx280::createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                               const sensor_filter_t filterMode2, const uint16_t filterSize2,
                               const sensor_filter_t filterMode3, const uint16_t filterSize3)
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
  if (_type == BMx_BME280) {
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
  };
}

/**
 *  Register internal parameters
 * */
void BMx280::registerItemsParameters(paramsGroupHandle_t parent_group)
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
}

/**
 * Connecting external previously created items, for example statically declared
 * */
bool BMx280::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  const BMx_TYPE type, const int numI2C, const uint8_t addrI2C, 
  const BMx_MODE mode, const BMx_STANDBYTIME t_sb, const BMx_IIR_FILTER filter,
  const BMx_OVERSAMPLING osrs_p, const BMx_OVERSAMPLING osrs_t, const BMx_OVERSAMPLING osrs_h,
  rSensorItem* item1, rSensorItem* item2, rSensorItem* item3,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  _type = type;
  // Assign items
  this->rSensorX3::setSensorItems(item1, item2, item3);
  // Start device
  return (initHardware(numI2C, addrI2C) && (setMode(mode, t_sb, filter, osrs_p, osrs_t, osrs_h) == SENSOR_STATUS_OK));
}

/**
 * Start device
 * */
bool BMx280::initHardware(const int numI2C, const uint8_t addrI2C)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  // Init calibration data
  memset(&_cal_data, 0, sizeof(_cal_data));
  // Check sensor id
  readSendorId();
  // Soft reset
  sensor_status_t err = softReset();
  if (err != SENSOR_STATUS_OK) return false;
  // Wait calibration data
  while (isReadingCalibration()) { vTaskDelay(1); };
  // Read calibration data
  err = readCalibration();
  if (err != SENSOR_STATUS_OK) return false;
  // Done
  return true;
}

/**
 * Read registers
 * */
esp_err_t BMx280::readRegisterI8(uint8_t reg_addr, int8_t *value)
{
  return readRegisterU8(reg_addr, (uint8_t*)value);
}

esp_err_t BMx280::readRegisterU8(uint8_t reg_addr, uint8_t *value)
{
  return readI2C(_I2C_num, _I2C_address, &reg_addr, 1, value, 1, 0, BMx280_TIMEOUT);
}

esp_err_t BMx280::readRegisterI16(uint8_t reg_addr, int16_t *value)
{
  return readRegisterU16(reg_addr, (uint16_t*)value);
}

esp_err_t BMx280::readRegisterU16(uint8_t reg_addr, uint16_t *value)
{
  uint8_t buf[2] = {0};
  esp_err_t err = readI2C(_I2C_num, _I2C_address, &reg_addr, 1, buf, 2, 0, BMx280_TIMEOUT);
  *value = (buf[1] << 8) | buf[0];
  return err;
}

/**
 * Write to registers
 * */
esp_err_t BMx280::writeRegisterU8(uint8_t reg_addr, uint8_t value)
{
  return writeI2C(_I2C_num, _I2C_address, &reg_addr, 1, &value, 1, BMx280_TIMEOUT);
}

/**
 * Read 8 bytes to buffer
 * */
esp_err_t BMx280::readBuffer(uint8_t reg_addr)
{
  return readI2C(_I2C_num, _I2C_address, &reg_addr, 1, _readBuf, 8, 0, BMx280_TIMEOUT);
}

/**
 * Soft reset
 * */
sensor_status_t BMx280::softReset()
{
  esp_err_t err = writeRegisterU8(BMx280_REGISTER_SOFTRESET, 0xB6);
	if (err == ESP_OK) {
		ets_delay_us(BMx280_DELAY_SOFT_RESET);
		rlog_i(_name, RSENSOR_LOG_MSG_SOFT_RESET, _name);
	}	else {
	  rlog_e(_name, RSENSOR_LOG_MSG_SOFT_RESET_FAILED, err, esp_err_to_name(err));
	};
	return this->rSensor::setEspError(err, true);
}

/**
 * Reading sensor id
 * */
uint8_t BMx280::readSendorId()
{
  uint8_t chip_id = 0;
  // Read chip_id
	esp_err_t err = readRegisterU8(BMx280_REGISTER_CHIPID, &chip_id);
	this->rSensor::setEspError(err, true);
	if (err != ESP_OK) {
    rlog_w(_name, "Failed to read chip id: %d %s!", err, esp_err_to_name(err));
    return chip_id;
	};
  // Check chip_id
  switch (_type) {
    case BMx_BMP280:
      if (chip_id != 0x58) {
        rlog_e(_name, "Chip_id type mismatch: expected 0x58, read 0x%.2X!", chip_id);
      };
      break;
    case BMx_BME280:
      if (chip_id != 0x60) {
        rlog_e(_name, "Chip_id type mismatch: expected 0x60, read 0x%.2X!", chip_id);
      };
      break;
    default:
      break;
  };
  // Read chip_varsion
  uint8_t version = 0;
  err = readRegisterU8(BMx280_REGISTER_VERSION, &version);
  this->rSensor::setEspError(err, true);
  if (err != ESP_OK) {
    rlog_w(_name, "Failed to read version: %d %s!", err, esp_err_to_name(err));
    return chip_id;
	};
  rlog_i(_name, "Chip id: 0x%.2X, version: 0x%.2X", chip_id, version);
	return chip_id;
}

/**
 * Reads the factory-set coefficients
 * */
bool BMx280::isReadingCalibration() 
{
  uint8_t status = 0;
  esp_err_t err = readRegisterU8(BMx280_REGISTER_STATUS, &status);
  if (err != ESP_OK) {
    rlog_w(_name, "Failed to read status register: %d %s!", err, esp_err_to_name(err));
	};
  return (status & (1 << 0)) != 0;
}

sensor_status_t BMx280::readCalibration()
{
  esp_err_t err = readRegisterU16(BMx280_REGISTER_DIG_T1, &_cal_data.dig_T1);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_T2, &_cal_data.dig_T2);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_T3, &_cal_data.dig_T3);
  if (err != ESP_OK) goto exit_error;

  err = readRegisterU16(BMx280_REGISTER_DIG_P1, &_cal_data.dig_P1);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_P2, &_cal_data.dig_P2);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_P3, &_cal_data.dig_P3);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_P4, &_cal_data.dig_P4);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_P5, &_cal_data.dig_P5);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_P6, &_cal_data.dig_P6);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_P7, &_cal_data.dig_P7);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_P8, &_cal_data.dig_P8);
  if (err != ESP_OK) goto exit_error;
  err = readRegisterI16(BMx280_REGISTER_DIG_P9, &_cal_data.dig_P9);
  if (err != ESP_OK) goto exit_error;
  
  if (_type == BMx_BME280) {
    uint8_t h4msb, h4lsb, h5msb;
    err = readRegisterU8(BMx280_REGISTER_DIG_H1, &_cal_data.dig_H1);
    if (err != ESP_OK) goto exit_error;
    err = readRegisterI16(BMx280_REGISTER_DIG_H2, &_cal_data.dig_H2);
    if (err != ESP_OK) goto exit_error;
    err = readRegisterU8(BMx280_REGISTER_DIG_H3, &_cal_data.dig_H3);
    if (err != ESP_OK) goto exit_error;
    err = readRegisterU8(BMx280_REGISTER_DIG_H4_MSB, &h4msb);
    if (err != ESP_OK) goto exit_error;
    err = readRegisterU8(BMx280_REGISTER_DIG_H4_LSB, &h4lsb);
    if (err != ESP_OK) goto exit_error;
    err = readRegisterU8(BMx280_REGISTER_DIG_H5_MSB, &h5msb);
    if (err != ESP_OK) goto exit_error;
    _cal_data.dig_H4 = (h4msb << 4) | (h4lsb & 0x0F);
    _cal_data.dig_H5 = (h5msb << 4) | ((h4lsb >> 4) & 0x0F);
    err = readRegisterI8(BMx280_REGISTER_DIG_H6, &_cal_data.dig_H6);
    if (err != ESP_OK) goto exit_error;
  };

  rlog_d(_name, "Calibration data loaded successfully");
  return SENSOR_STATUS_OK;

  // Emergency exit
exit_error:
  rlog_e(_name, "Failed to read calibration data: %d %s!", err, esp_err_to_name(err));
  this->rSensor::setRawStatus(SENSOR_STATUS_CAL_ERROR, true);
  return SENSOR_STATUS_CAL_ERROR;
}

/**
 * Set measurement mode 
 * */
sensor_status_t BMx280::setMode(const BMx_MODE mode, const BMx_STANDBYTIME t_sb, const BMx_IIR_FILTER filter,
  const BMx_OVERSAMPLING osrs_p, const BMx_OVERSAMPLING osrs_t, const BMx_OVERSAMPLING osrs_h)
{
  uint8_t spi3or4 = 0;
  uint8_t ctrl_hum = osrs_h;
  uint8_t config = (t_sb << 5) | (filter << 2) | spi3or4;
  uint8_t ctrl_meas = (osrs_t << 5) | (osrs_p << 2) | mode;

  // Making sure sensor is in sleep mode before setting configuration as it otherwise may be ignored
  esp_err_t err = writeRegisterU8(BMx280_REGISTER_CONTROL, MODE_SLEEP);
  if (err != ESP_OK) goto exit_error;

  // Set BMx280_REGISTER_CONTROLHUMID (before ctrl_meas!)
  if (_type == BMx_BME280) {
    err = writeRegisterU8(BMx280_REGISTER_CONTROLHUMID, ctrl_hum);
    if (err != ESP_OK) goto exit_error;
    _osrs_h = osrs_h;
  };

  // Set BMx280_REGISTER_CONTROL
  err = writeRegisterU8(BMx280_REGISTER_CONTROL, ctrl_meas);
  if (err != ESP_OK) goto exit_error;
  _mode = mode;
  _osrs_p = osrs_p;
  _osrs_t = osrs_t;

  // Set BMx280_REGISTER_CONFIG
  err = writeRegisterU8(BMx280_REGISTER_CONFIG, config);
  if (err != ESP_OK) goto exit_error;
  _t_sb = t_sb;
  _filter = filter;

  rlog_i(_name, "Measurement mode set: mode=0x%x, filter=0x%x, t_sb=0x%x, osrs_p=0x%x, osrs_t=0x%x, osrs_h=0x%x", mode, filter, t_sb, osrs_p, osrs_t, osrs_h);
  return this->rSensor::setEspError(err, true);
 
  // Emergency exit
exit_error:
  rlog_e(_name, "Failed to set measurement mode: %d %s!", err, esp_err_to_name(err));
  return this->rSensor::setEspError(err, true);
}

/**
 * Reading values from sensor
 * */
bool BMx280::isMeasuring(esp_err_t * error) 
{
  uint8_t status = 0;
  *error = readRegisterU8(BMx280_REGISTER_STATUS, &status);
  return (status & 0x08) != 0;
}

sensor_status_t BMx280::readRawData()
{
  esp_err_t err;
  float valueP, valueT, valueH = 0.0;
  // Sensor disabled
  if (_mode == MODE_SLEEP) {
    rlog_v("name", "Sensor in sleep mode!");
    return this->rSensor::setEspError(ESP_ERR_INVALID_STATE, false);
  };
  // Start forced measurement
  if (_mode == MODE_FORCED) {
    uint8_t ctrl_meas = (_osrs_t << 5) | (_osrs_p << 2) | _mode;
    err = writeRegisterU8(BMx280_REGISTER_CONTROL, ctrl_meas);
    if (err != ESP_OK) {
      rlog_e(_name, "Failed to start forced measurement: %d %s!", err, esp_err_to_name(err));
      return this->rSensor::setEspError(err, false);
    };
  };
  // Wait measuring
  while (isMeasuring(&err)) {
    if (err != ESP_OK) {
      rlog_w(_name, "Failed to read status register: %d %s!", err, esp_err_to_name(err));
      return this->rSensor::setEspError(err, false);
    };
    vTaskDelay(1);
  };
  // Read 8 bytes in buffer
  err = readBuffer(BMx280_REGISTER_ALLDATA);
  if (err != ESP_OK) {
    rlog_e(_name, "Failed to read data: %d %s!", err, esp_err_to_name(err));
    return this->rSensor::setEspError(err, false);
  };
  // Set pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	int32_t adc_P = ((uint32_t)_readBuf[0] << 12) | ((uint32_t)_readBuf[1] << 4) | ((_readBuf[2] >> 4) & 0x0F);
  // Set temperature in C as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
  int32_t adc_T = ((uint32_t)_readBuf[3] << 12) | ((uint32_t)_readBuf[4] << 4) | ((_readBuf[5] >> 4) & 0x0F);
  // Set humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
	int32_t adc_H = ((uint32_t)_readBuf[6] << 8) | ((uint32_t)_readBuf[7]);
  // Calculate temperature - must be done first to get t_fine
  int32_t var1t, var2t, t_fine;
  var1t = ((((adc_T >> 3) - ((int32_t)_cal_data.dig_T1 << 1))) * ((int32_t)_cal_data.dig_T2)) >> 11;
  var2t = (((((adc_T >> 4) - ((int32_t)_cal_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)_cal_data.dig_T1))) >> 12) * ((int32_t)_cal_data.dig_T3)) >> 14;
  t_fine = var1t + var2t;
  valueT = (t_fine * 5 + 128) >> 8;
  valueT = valueT / 100.0;
  // Calculate pressure
  int64_t var1p, var2p, p;
  var1p = ((int64_t)t_fine) - 128000;
  var2p = var1p * var1p * (int64_t)_cal_data.dig_P6;
  var2p = var2p + ((var1p * (int64_t)_cal_data.dig_P5) << 17);
  var2p = var2p + (((int64_t)_cal_data.dig_P4) << 35);
  var1p = ((var1p * var1p * (int64_t)_cal_data.dig_P3) >> 8) + ((var1p * (int64_t)_cal_data.dig_P2) << 12);
  var1p = (((((int64_t)1) << 47) + var1p)) * ((int64_t)_cal_data.dig_P1) >> 33;
  if (var1p == 0) {
    valueP = 0.0; // avoid exception caused by division by zero
  } else {
    p = 1048576 - adc_P;
    p = (((p << 31) - var2p) * 3125) / var1p;
    var1p = (((int64_t)_cal_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2p = (((int64_t)_cal_data.dig_P8) * p) >> 19;
    p = ((p + var1p + var2p) >> 8) + (((int64_t)_cal_data.dig_P7) << 4);
    valueP = (float)p / 256.0;
  };
  // Calculate humidity
  if (_type == BMx_BME280) {
    int32_t v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)_cal_data.dig_H4) << 20) - (((int32_t)_cal_data.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                (((((((v_x1_u32r * ((int32_t)_cal_data.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)_cal_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                ((int32_t)2097152)) * ((int32_t)_cal_data.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)_cal_data.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    valueH = (v_x1_u32r >> 12) / 1024.0;
  };

  this->rSensorX3::setRawValues(valueP, valueT, valueH);
  return SENSOR_STATUS_OK;
}