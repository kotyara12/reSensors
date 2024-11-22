#include "reGorynych.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "rLog.h"
#include <time.h>
#include "mbcontroller.h"

static const char* logTAG = "GORYNYCH";

// Read registers, read function code: 0x03 
#define FUNCTION_CODE_READ            0x03
#define FUNCTION_CODE_WRITE           0x06

// "Triple" registers: the real address of the register is REG_XXX + (index - 1)
#define REG_VALUE_INT                 0x0000
#define REG_VALUE_RAW                 0x0003
#define REG_VALUE_INT10               0x0006
#define REG_ERROR_LAST                0x000F
#define REG_ERROR_COUNT               0x0012
#define REG_ERROR_CORRECTION          0x001A

// Single registers
#define REG_FIRMWARE_VERSION          0x000E

// Parameters registers, read function code: 0x03, write function code: 0x06
#define REG_PARAM_SLAVE_ID            0x0015
#define REG_PARAM_BAUD_RATE           0x0016
#define REG_PARAM_EVEN                0x0017
#define REG_PARAM_STOP_BITS           0x0018
#define REG_PARAM_LEDS                0x0019
#define REG_PARAM_READ_INTERVAL       0x001D


reGorynych::reGorynych(uint8_t eventId, 
  void* modbus, const uint8_t address, const uint8_t index,
  const char* sensorName, const char* topicName, const bool topicLocal, 
  const uint32_t minReadInterval, const uint16_t bufferLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
:rSensor(eventId, 11, 
  sensorName, topicName, topicLocal, 
  minReadInterval, bufferLimit,
  cb_status, cb_publish)
{
  _modbus = modbus;
  _address = address;
  _index = index;
}

void reGorynych::setSensorItems(rSensorItem* itemTemperature)
{
  setSensorItem(0, itemTemperature);
}

sensor_value_t reGorynych::getTemperature(const bool readSensor)
{
  return getItemValue(0, readSensor);
}

// Start device
sensor_status_t reGorynych::sensorReset()
{
  return SENSOR_STATUS_OK;
};

/**
 * Read data
 * */
sensor_status_t reGorynych::readRawData()
{
  int16_t buffer = 0;
  uint16_t reg;
  // Read current sensor state
  reg = REG_ERROR_LAST + (_index-1);
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_READ,
    .reg_start  = reg,
    .reg_size   = 1
  };
  esp_err_t err = mbc_master_send_request(&_request, (void*)&buffer);
  if (err == ESP_OK) {
    if (buffer == 0) {
      // Read RAW value
      reg = (uint16_t)REG_VALUE_RAW + (_index-1);
      mb_param_request_t _request = {
        .slave_addr = _address,
        .command    = FUNCTION_CODE_READ,
        .reg_start  = reg,
        .reg_size   = 1
      };
      esp_err_t err = mbc_master_send_request(&_request, (void*)&buffer);
      if (err == ESP_OK) {
        if (_items[0]) {
          return _items[0]->setRawValue((float)buffer * 0.0625, time(nullptr));
        };
      } else {
        rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
        return convertEspError(err);
      };
    } else if ((buffer & 0x03) > 0x00) {
      return SENSOR_STATUS_CONN_ERROR;
    } if ((buffer & 0x08) > 0x00) {
      return SENSOR_STATUS_CRC_ERROR;
    } else {
      return SENSOR_STATUS_ERROR;
    };
  } else {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };
  return SENSOR_STATUS_ERROR;
};

/**
 * Configure
 * */
sensor_status_t reGorynych::setBaudRate(uint16_t value)
{
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_WRITE,
    .reg_start  = REG_PARAM_BAUD_RATE,
    .reg_size   = 1
  };
  return convertEspError(mbc_master_send_request(&_request, (void*)&value));
}

sensor_status_t reGorynych::setSlaveId(uint16_t value)
{
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_WRITE,
    .reg_start  = REG_PARAM_SLAVE_ID,
    .reg_size   = 1
  };
  return convertEspError(mbc_master_send_request(&_request, (void*)&value));
}

sensor_status_t reGorynych::setLeds(bool enabled)
{
  uint16_t value = (uint16_t)enabled;
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_WRITE,
    .reg_start  = REG_PARAM_LEDS,
    .reg_size   = 1
  };
  return convertEspError(mbc_master_send_request(&_request, (void*)&value));
}

sensor_status_t reGorynych::setCorrection(float value)
{
  float value_f = value / 0.0625;
  int16_t value_d = (int16_t)value_f;
  uint16_t reg = REG_ERROR_CORRECTION + (_index-1);
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_WRITE,
    .reg_start  = reg,
    .reg_size   = 1
  };
  return convertEspError(mbc_master_send_request(&_request, (void*)&value_d));
}
