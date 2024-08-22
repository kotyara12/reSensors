#include "reTH485.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "rLog.h"
#include <time.h>
#include "mbcontroller.h"

static const char* logTAG = "TH485";

reTH485::reTH485(uint8_t eventId, 
  void* modbus, const uint8_t address, const uint8_t read_cmd, const uint16_t reg_temp, const uint16_t reg_humd,
  const char* sensorName, const char* topicName, const bool topicLocal, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
:rSensorHT(eventId, 
  sensorName, topicName, topicLocal, 
  minReadInterval, errorLimit,
  cb_status, cb_publish)
{
  _modbus = modbus;
  _address = address;
  _command = read_cmd;
  _reg_temp = reg_temp;
  _reg_humd = reg_humd;
}

// Start device
sensor_status_t reTH485::sensorReset()
{
  return SENSOR_STATUS_OK;
};

/**
 * Read humidity and temperature data
 * */
sensor_status_t reTH485::readRawData()
{
  int16_t _data[2] = {0};

  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = _command,
    .reg_start  = (_reg_humd < _reg_temp ? _reg_humd : _reg_temp),
    .reg_size   = 2
  };

  // Read registers
  esp_err_t err = mbc_master_send_request(&_request, (void*)&_data[0]);

  // Check exit code
  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };

  // Store values in sensors
  if (_reg_humd < _reg_temp) {
    return setRawValues((float)_data[0]/10.0, (float)_data[1]/10.0);
  } else {
    return setRawValues((float)_data[1]/10.0, (float)_data[0]/10.0);
  };
};
