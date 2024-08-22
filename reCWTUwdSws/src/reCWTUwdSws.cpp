#include "reCWTUwdSws.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "rLog.h"
#include <time.h>
#include "mbcontroller.h"

static const char* logTAG = "USWS";

// Read status registers, read function code: 0x03 
#define FUNCTION_CODE_STATUS_READ     0x03
#define FUNCTION_CODE_STATUS_WRITE    0x06

#define REG_STATUS_COUNT              14
#define REG_STATUS_WIND_SPEED         0x01F4
#define REG_STATUS_WIND_STRENGHT      0x01F5
#define REG_STATUS_WIND_DIRECTION_07  0x01F6
#define REG_STATUS_WIND_DIRECTION_360 0x01F7
#define REG_STATUS_HUMIDITY           0x01F8
#define REG_STATUS_TEMPERATURE        0x01F9
#define REG_STATUS_NOISE              0x01FA
#define REG_STATUS_PM25               0x01FB
#define REG_STATUS_PM10               0x01FC
#define REG_STATUS_PRESSURE           0x01FD
#define REG_STATUS_ILLUMINANCE_HIGH   0x01FE
#define REG_STATUS_ILLUMINANCE_LOW    0x01FF
#define REG_STATUS_ILLUMINANCE_100    0x0200
#define REG_STATUS_RAINFALL           0x0201

#define REG_INDEX_WIND_SPEED          0
#define REG_INDEX_WIND_STRENGHT       1
#define REG_INDEX_WIND_DIRECTION_07   2
#define REG_INDEX_WIND_DIRECTION_360  3
#define REG_INDEX_HUMIDITY            4
#define REG_INDEX_TEMPERATURE         5
#define REG_INDEX_NOISE               6
#define REG_INDEX_PM25                7
#define REG_INDEX_PM10                8
#define REG_INDEX_PRESSURE            9
#define REG_INDEX_ILLUMINANCE_HIGH    10
#define REG_INDEX_ILLUMINANCE_LOW     11
#define REG_INDEX_ILLUMINANCE_100     12
#define REG_INDEX_RAINFALL            13

// Parameters registers, read function code: 0x03, write function code: 0x06
#define REG_PARAM_SLAVE_ID            0x07D0
#define REG_PARAM_BAUD_RATE           0x07D1

// Calibration register: write function code: 0x06
#define REG_CAL_WIND_DIRECTION_OFFSET 0x6000
#define REG_CAL_WIND_SPEED_ZERO       0x6001
#define REG_CAL_RAINFALL_ZERO         0x6002

#define VAL_CAL_WIND_SPEED_ZERO       0xAA
#define VAL_CAL_RAINFALL_ZERO         0x5A

reCWTUwdSws::reCWTUwdSws(uint8_t eventId, 
  void* modbus, const uint8_t address,
  const char* sensorName, const char* topicName, const bool topicLocal, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
:rSensor(eventId, 11, 
  sensorName, topicName, topicLocal, 
  minReadInterval, errorLimit,
  cb_status, cb_publish)
{
  _modbus = modbus;
  _address = address;
}

void reCWTUwdSws::setSensorItems(rSensorItem* itemTemperature, rSensorItem* itemHumidity, 
      rSensorItem* itemPressure, 
      rSensorItem* itemIlluminance, 
      rSensorItem* itemWindSpeed, rSensorItem* itemWindStrength, rSensorItem* itemWindDirection, 
      rSensorItem* itemNoise, 
      rSensorItem* itemPM25, rSensorItem* itemPM10, 
      rSensorItem* itemRainfall)
{
  setSensorItem(0, itemTemperature);
  setSensorItem(1, itemHumidity);
  setSensorItem(2, itemPressure);
  setSensorItem(3, itemIlluminance);
  setSensorItem(4, itemWindSpeed);
  setSensorItem(5, itemWindStrength);
  setSensorItem(6, itemWindDirection);
  setSensorItem(7, itemNoise);
  setSensorItem(8, itemPM25);
  setSensorItem(9, itemPM10);
  setSensorItem(10, itemRainfall);
}

sensor_value_t reCWTUwdSws::getTemperature(const bool readSensor)
{
  return getItemValue(0, readSensor);
}

sensor_value_t reCWTUwdSws::getHumidity(const bool readSensor)
{
  return getItemValue(1, readSensor);
}

sensor_value_t reCWTUwdSws::getPressure(const bool readSensor)
{
  return getItemValue(2, readSensor);
}

sensor_value_t reCWTUwdSws::getIlluminance(const bool readSensor)
{
  return getItemValue(3, readSensor);
}

sensor_value_t reCWTUwdSws::getWindSpeed(const bool readSensor)
{
  return getItemValue(4, readSensor);
}

sensor_value_t reCWTUwdSws::getWindStrength(const bool readSensor)
{
  return getItemValue(5, readSensor);
}

sensor_value_t reCWTUwdSws::getWindDirection(const bool readSensor)
{
  return getItemValue(6, readSensor);
}

sensor_value_t reCWTUwdSws::getNoise(const bool readSensor)
{
  return getItemValue(7, readSensor);
}

sensor_value_t reCWTUwdSws::getPM25(const bool readSensor)
{
  return getItemValue(8, readSensor);
}

sensor_value_t reCWTUwdSws::getPM10(const bool readSensor)
{
  return getItemValue(9, readSensor);
}

sensor_value_t reCWTUwdSws::getRainfall(const bool readSensor)
{
  return getItemValue(10, readSensor);
}

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* reCWTUwdSws::getDisplayValue()
{
  char* ret = nullptr;
  // Temperature
  if (_items[0]) {
    ret = _items[0]->getStringFiltered();
  };
  // Humidity
  if (_items[1]) {
    ret = concat_strings_div(ret, _items[1]->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

// Start device
sensor_status_t reCWTUwdSws::sensorReset()
{
  return SENSOR_STATUS_OK;
};

/**
 * Read data
 * */
sensor_status_t reCWTUwdSws::readRawData()
{
  int16_t buffer[REG_STATUS_COUNT] = {0};
  sensor_status_t ret = SENSOR_STATUS_OK;
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_STATUS_READ,
    .reg_start  = REG_STATUS_WIND_SPEED,
    .reg_size   = REG_STATUS_COUNT
  };
  esp_err_t err = mbc_master_send_request(&_request, (void*)&buffer[0]);
  if (err == ESP_OK) {
    time_t read_time = time(nullptr);
    // 0 - itemTemperature
    if (_items[0] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[0]->setRawValue((float)buffer[REG_INDEX_TEMPERATURE] / 10.0, read_time);
    };
    // 1 - itemHumidity
    if (_items[1] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[1]->setRawValue((float)buffer[REG_INDEX_HUMIDITY] / 10.0, read_time);
    };
    // 2 - itemPressure
    if (_items[2] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[2]->setRawValue((float)buffer[REG_INDEX_PRESSURE] * 100.0, read_time);
    };
    // 3 - itemIlluminance
    if (_items[3] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[3]->setRawValue((float)(buffer[REG_INDEX_ILLUMINANCE_HIGH] << 16 | buffer[REG_INDEX_ILLUMINANCE_LOW]), read_time);
    };
    // 4 - itemWindSpeed
    if (_items[4] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[4]->setRawValue((float)buffer[REG_INDEX_WIND_SPEED] / 10.0, read_time);
    };
    // 5 - itemWindStrength
    if (_items[5] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[5]->setRawValue((float)buffer[REG_INDEX_WIND_STRENGHT], read_time);
    };
    // 6 - itemWindDirection
    if (_items[6] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[6]->setRawValue((float)buffer[REG_INDEX_WIND_DIRECTION_360], read_time);
    };
    // 7 - itemNoise
    if (_items[7] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[7]->setRawValue((float)buffer[REG_INDEX_NOISE] / 10.0, read_time);
    };
    // 8 - itemPM25
    if (_items[8] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[8]->setRawValue((float)buffer[REG_INDEX_PM25], read_time);
    };
    // 9 - itemPM10
    if (_items[9] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[9]->setRawValue((float)buffer[REG_INDEX_PM10], read_time);
    };
    // 10 - itemRainfall
    if (_items[10] && (ret = SENSOR_STATUS_OK)) {
      ret = _items[10]->setRawValue((float)buffer[REG_INDEX_RAINFALL], read_time);
    };
  } else {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };
  return ret;
};

/**
 * Configure
 * */
sensor_status_t reCWTUwdSws::setBaudRate(uint16_t value)
{
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_STATUS_WRITE,
    .reg_start  = REG_PARAM_BAUD_RATE,
    .reg_size   = 1
  };
  return convertEspError(mbc_master_send_request(&_request, (void*)&value));
}

sensor_status_t reCWTUwdSws::setSlaveId(uint16_t value)
{
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_STATUS_WRITE,
    .reg_start  = REG_PARAM_SLAVE_ID,
    .reg_size   = 1
  };
  return convertEspError(mbc_master_send_request(&_request, (void*)&value));
}

sensor_status_t reCWTUwdSws::setWindSpeedZero()
{
  uint16_t value = VAL_CAL_WIND_SPEED_ZERO;
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_STATUS_WRITE,
    .reg_start  = REG_CAL_WIND_SPEED_ZERO,
    .reg_size   = 1
  };
  return convertEspError(mbc_master_send_request(&_request, (void*)&value));
}

sensor_status_t reCWTUwdSws::setRainfallZero()
{
  uint16_t value = VAL_CAL_RAINFALL_ZERO;
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_STATUS_WRITE,
    .reg_start  = REG_CAL_RAINFALL_ZERO,
    .reg_size   = 1
  };
  return convertEspError(mbc_master_send_request(&_request, (void*)&value));
}

