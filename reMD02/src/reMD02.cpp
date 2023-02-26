#include "reMD02.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "rLog.h"
#include <time.h>
#include "mbcontroller.h"

static const char* logTAG = "MD02";

xyMD02::xyMD02(uint8_t eventId):rSensorHT(eventId)
{
  _modbus = nullptr;
  _address = 1;
}

// Dynamically creating internal items on the heap
bool xyMD02::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
  void* modbus, const uint8_t address,
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
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
bool xyMD02::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  void* modbus, const uint8_t address,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX2::setSensorItems(item1, item2);
  // Start device
  return sensorStart();
}

// Start device
sensor_status_t xyMD02::sensorReset()
{
  return SENSOR_STATUS_OK;
};


/**
 * Read humidity and temperature data
 * */
sensor_status_t xyMD02::readRawData()
{
  uint16_t values[2];

  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = 4, // MB_FUNC_READ_INPUT_REGISTER,
    .reg_start  = 0x0001,
    .reg_size   = 2
  };
  esp_err_t err = mbc_master_send_request(&_request, &values[0]);

  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };

  // Store values in sensors
  return setRawValues((float)values[1]/10.0, (float)values[0]/10.0);
};
