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

reQDY30A::reQDY30A(uint8_t eventId):rSensorX1(eventId)
{
  _modbus = nullptr;
  _address = 1;
}

// Dynamically creating internal items on the heap
bool reQDY30A::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
  void* modbus, const uint8_t address, 
  const sensor_filter_t filterMode, const uint16_t filterSize, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX1::initSensorItems(filterMode, filterSize)) {
    // Start device
    return sensorStart();
  };
  return false;
}

// Connecting external previously created items, for example statically declared
bool reQDY30A::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  void* modbus, const uint8_t address, 
  rSensorItem* item, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX1::setSensorItems(item);
  // Start device
  return sensorStart();
}

// Initialization of internal items
void reQDY30A::createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  // Level
  _item = new rSensorItem(this, CONFIG_SENSOR_LEVEL_NAME, 
    filterMode, filterSize,
    CONFIG_FORMAT_FLOAT2_VALUE, CONFIG_FORMAT_FLOAT2_STRING
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    , CONFIG_FORMAT_TIMESTAMP_L 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
    , CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  if (_item) {
    rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item->getName(), getName());
  };
}
    
void reQDY30A::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  // Level
  if (_item) {
    _item->registerParameters(parent_group, CONFIG_SENSOR_LEVEL_KEY, CONFIG_SENSOR_LEVEL_NAME, CONFIG_SENSOR_LEVEL_FRIENDLY);
  };
}

sensor_status_t reQDY30A::sensorReset()
{
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
  int16_t value, precs = 0;
  esp_err_t err = ESP_OK;

  if (_item) {
    err = callModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_OUTPUT, &value);
    if (err != ESP_OK) {
      err = callModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_PRECISION, &precs);
    };
  };

  // Check exit code
  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };

  switch (precs) {
    case 1:  return setRawValues((value_t)value/10.0);    // 1-###.#
    case 2:  return setRawValues((value_t)value/100.0);   // 2-##.##
    case 3:  return setRawValues((value_t)value/1000.0);  // 3-#.###
    default: return setRawValues((value_t)value);         // 0-####
  }
};
