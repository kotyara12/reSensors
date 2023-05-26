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
  void* modbus, const uint8_t address, const QDY30A_UNITS units, const QDY30A_PRECISION prec,
  const sensor_filter_t filterMode, const uint16_t filterSize, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
  _units = units;
  _prec = prec;
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
  void* modbus, const uint8_t address, const QDY30A_UNITS units, const QDY30A_PRECISION prec,
  rSensorItem* item, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
  _units = units;
  _prec = prec;
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

// Start device
sensor_status_t reQDY30A::sensorReset()
{
  return SENSOR_STATUS_OK;
};

/**
 * Read status register
 * */
esp_err_t reQDY30A::readModbusRegister(uint8_t cmd, uint16_t reg, int16_t* value)
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
sensor_status_t reQDY30A::readRawData()
{
  esp_err_t err = ESP_OK;
  int16_t bufL = 0;

  // Level
  if ((_item) && (err == ESP_OK)) {
    err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_OUTPUT, &bufL);
  };

  // Check exit code
  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };

  // Store values in sensors
  return setRawValues((float)bufL);
};
