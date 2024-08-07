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
  void* modbus, const uint8_t address,
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
}

void reQDY30A::setSensorItems(rSensorItem* itemLevel)
{
  setSensorItem(0, itemLevel);
}

sensor_status_t reQDY30A::sensorReset()
{
  // int16_t value = 2;
  // callModbusRegister(FUNCTION_CODE_STATUS_WRITE, REG_STATUS_UNITS, &value);
  // value = 0;
  // callModbusRegister(FUNCTION_CODE_STATUS_WRITE, REG_STATUS_PRECISION, &value);
  return SENSOR_STATUS_OK;
};

esp_err_t reQDY30A::callModbusRegister(uint8_t cmd, uint16_t reg, int16_t* value)
{
  mb_param_request_t _request = {
    .slave_addr = _address,
    .command    = cmd,
    .reg_start  = reg,
    .reg_size   = 1
  };
  return mbc_master_send_request(&_request, (void*)value);
}

sensor_status_t reQDY30A::readRawData()
{
  int16_t value, precs, units = 0;
  esp_err_t err = ESP_OK;

  err = callModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_OUTPUT, &value);
  if (err != ESP_OK) err = callModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_PRECISION, &precs);
  if (err != ESP_OK) err = callModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_UNITS, &units);
  
  // Check exit code
  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };

  // Check exit value
  if (value < 0) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_BAD_VALUE, _name);
    return SENSOR_STATUS_BAD_DATA;
  };

  switch (precs) {
    case 1:  return setRawValue(0, (value_t)value/10.0);    // 1-###.#
    case 2:  return setRawValue(0, (value_t)value/100.0);   // 2-##.##
    case 3:  return setRawValue(0, (value_t)value/1000.0);  // 3-#.###
    default: return setRawValue(0, (value_t)value);         // 0-####
  }
};

sensor_value_t reQDY30A::getLevel(const bool readSensor)
{
  return getItemValue(0, readSensor);
}