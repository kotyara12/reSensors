#include "reTH485.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "rLog.h"
#include <time.h>
#include "mbcontroller.h"

static const char* logTAG = "TH485";

reTH485::reTH485(uint8_t eventId):rSensorHT(eventId)
{
  _modbus = nullptr;
  _address = 1;
}

// Dynamically creating internal items on the heap
bool reTH485::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
  void* modbus, const uint8_t address, const uint8_t read_cmd, const uint16_t reg_temp, const uint16_t reg_humd,
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
  _command = read_cmd;
  _reg_temp = reg_temp;
  _reg_humd = reg_humd;
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
bool reTH485::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  void* modbus, const uint8_t address, const uint8_t read_cmd, const uint16_t reg_temp, const uint16_t reg_humd,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
  _command = read_cmd;
  _reg_temp = reg_temp;
  _reg_humd = reg_humd;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX2::setSensorItems(item1, item2);
  // Start device
  return sensorStart();
}

// Start device
sensor_status_t reTH485::sensorReset()
{
  return SENSOR_STATUS_OK;
};

/**
 * Update (read / write) modbus register
 * */
esp_err_t reTH485::callModbusRegister(uint8_t cmd, uint16_t reg, uint16_t* value)
{
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = cmd,
    .reg_start  = reg,
    .reg_size   = 1
  };
  return mbc_master_send_request(&_request, (void*)value);
}

/**
 * Read humidity and temperature data
 * */
sensor_status_t reTH485::readRawData()
{
  uint16_t _temp, _humd;
  RE_OK_CHECK(callModbusRegister(_command, _reg_humd, &_humd), return SENSOR_STATUS_CONN_ERROR);
  RE_OK_CHECK(callModbusRegister(_command, _reg_temp, &_temp), return SENSOR_STATUS_CONN_ERROR);
  return setRawValues((float)_humd/10.0, (float)_temp/10.0);
};
