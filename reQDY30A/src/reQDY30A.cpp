#include "reQDY30A.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "rLog.h"
#include <time.h>
#include "mbcontroller.h"

static const char* logTAG = "QDY30";

// Registers, read function code: 0x03, write function code: 0x06
#define FUNCTION_CODE_STATUS_READ    0x03
#define FUNCTION_CODE_STATUS_WRITE   0x06

#define REG_STATUS_ADDRESS           0x0000
#define REG_STATUS_BAUDRATE          0x0001
#define REG_STATUS_UNITS             0x0002
#define REG_STATUS_PRECISION         0x0003
#define REG_STATUS_OUTPUT            0x0004
#define REG_STATUS_ZERO              0x0005
#define REG_STATUS_FULL              0x0006

reQDY30A::reQDY30A(uint8_t eventId, 
  void* modbus, const uint8_t address, const qdy_units_t units,
  const char* sensorName, const char* topicName, const bool topicLocal, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
:rSensor(eventId, 1, 
  sensorName, topicName, topicLocal, 
  minReadInterval, errorLimit,
  cb_status, cb_publish)
{
  _modbus = nullptr;
  _address = address;
  _units = units;
}

void reQDY30A::setSensorItems(rSensorItem* itemLevel)
{
  setSensorItem(0, itemLevel);
}

sensor_status_t reQDY30A::sensorReset()
{
  return SENSOR_STATUS_OK;
};

sensor_status_t reQDY30A::readRawData()
{
  int16_t buffer[3] = {0};

  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = FUNCTION_CODE_STATUS_READ,
    .reg_start  = REG_STATUS_UNITS,
    .reg_size   = 3
  };
  esp_err_t err = mbc_master_send_request(&_request, (void*)&buffer[0]);
  // rlog_d(logTAG, "Read data: value = %d, precs = %d, units = %d", buffer[2], buffer[1], buffer[0]);
  
  // Check exit code
  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };

  // Check exit value
  if (buffer[2] < 0) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_BAD_VALUE, _name);
    return SENSOR_STATUS_BAD_DATA;
  };

  value_t level = NAN;
  switch (buffer[1]) {
    case 1:  
      level = (value_t)buffer[2]/10.0;    // 1-###.#
      break;
    case 2:  
      level = (value_t)buffer[2]/100.0;   // 2-##.##
      break;
    case 3:  
      level = (value_t)buffer[2]/1000.0;  // 3-#.###
      break;
    default: 
      level = (value_t)buffer[2];         // 0-####
      break;
  }

  switch (_units) {
    case QUNITS_CM:
      if (buffer[0] == 2) {
        level = level / 10.0;
      };
      break;
    default:
      if (buffer[0] == 1) {
        level = level * 10.0;
      };
      break;
  };

  return setRawValue(0, level); 
};

sensor_value_t reQDY30A::getLevel(const bool readSensor)
{
  return getItemValue(0, readSensor);
}