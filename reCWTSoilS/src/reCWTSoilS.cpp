#include "reCWTSoilS.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "rLog.h"
#include <time.h>
#include "mbcontroller.h"

static const char* logTAG = "CWTS";

// Read status registers, read function code: 0x03 
#define FUNCTION_CODE_STATUS_READ    0x03

#define REG_STATUS_HUMIDITY          0x0000
#define REG_STATUS_TEMPERATURE       0x0001
#define REG_STATUS_CONDUCTIVITY      0x0002
#define REG_STATUS_PH                0x0003
#define REG_STATUS_SALINITY          0x0007
#define REG_STATUS_TDS               0x0008

#define REG_FACTOR_CONDUCTIVITY      0x0022
#define REG_FACTOR_SALINITY          0x0023
#define REG_FACTOR_TDS               0x0024

#define REG_CAL_TEMPERATURE          0x0050
#define REG_CAL_HUMIDITY             0x0051
#define REG_CAL_CONDUCTIVITY         0x0052
#define REG_CAL_PH                   0x0053

// Parameters registers, read function code: 0x03, write function code: 0x06
#define REG_PARAM_SLAVE_ID           0x07D0
#define REG_PARAM_BAUD_RATE          0x07D1

reCWTSoilS::reCWTSoilS(uint8_t eventId):rSensorX4(eventId)
{
  _modbus = nullptr;
  _address = 1;
}

// Dynamically creating internal items on the heap
bool reCWTSoilS::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
  void* modbus, const uint8_t address, const cwt_soil_type_t type,
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const sensor_filter_t filterMode3, const uint16_t filterSize3,
  const sensor_filter_t filterMode4, const uint16_t filterSize4,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
  _type = type;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX4::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2, filterMode3, filterSize3, filterMode4, filterSize4)) {
    // Start device
    return sensorStart();
  };
  return false;
}

// Connecting external previously created items, for example statically declared
bool reCWTSoilS::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  void* modbus, const uint8_t address, const cwt_soil_type_t type,
  rSensorItem* item1, rSensorItem* item2, rSensorItem* item3, rSensorItem* item4,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _modbus = modbus;
  _address = address;
  _type = type;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX4::setSensorItems(item1, item2, item3, item4);
  // Start device
  return sensorStart();
}

// Initialization of internal items
void reCWTSoilS::createSensorItems(
  const sensor_filter_t filterMode1, const uint16_t filterSize1,
  const sensor_filter_t filterMode2, const uint16_t filterSize2, 
  const sensor_filter_t filterMode3, const uint16_t filterSize3, 
  const sensor_filter_t filterMode4, const uint16_t filterSize4)
{
  // Temperature
  if ((_type == CWTS_TH) || (_type == CWTS_THC) || (_type == CWTS_THPH) || (_type == CWTS_THCPH)) {
    _item1 = new rTemperatureItem(this, CONFIG_SENSOR_TEMP_NAME, (unit_temperature_t)CONFIG_FORMAT_TEMP_UNIT,
      filterMode1, filterSize1,
      CONFIG_FORMAT_TEMP_VALUE, CONFIG_FORMAT_TEMP_STRING
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , CONFIG_FORMAT_TIMESTAMP_L 
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
    );
    if (_item1) {
      rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item1->getName(), getName());
    };
  };
  // Humidity (moisture)
  if ((_type != CWTS_PH) && (_type != CWTS_CPH)) {
    _item2 = new rSensorItem(this, CONFIG_SENSOR_MOISTURE_NAME, 
      filterMode2, filterSize2,
      CONFIG_FORMAT_MOISTURE_VALUE, CONFIG_FORMAT_MOISTURE_STRING
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , CONFIG_FORMAT_TIMESTAMP_L 
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
    );
    if (_item2) {
      rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item2->getName(), getName());
    };
  };
  // Conductivity
  if ((_type == CWTS_HC) || (_type == CWTS_CPH) || (_type == CWTS_THC) || (_type == CWTS_THCPH)) {
    _item3 = new rSensorItem(this, CONFIG_SENSOR_CONDUCTIVITY_NAME, 
      filterMode3, filterSize3,
      CONFIG_FORMAT_INTEGER_VALUE, CONFIG_FORMAT_INTEGER_STRING
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , CONFIG_FORMAT_TIMESTAMP_L 
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
    );
    if (_item3) {
      rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item3->getName(), getName());
    };
  };
  // PH
  if ((_type == CWTS_PH) || (_type == CWTS_CPH) || (_type == CWTS_THPH) || (_type == CWTS_THCPH)) {
    _item4 = new rSensorItem(this, CONFIG_SENSOR_PH_NAME, 
      filterMode4, filterSize4,
      CONFIG_FORMAT_FLOAT1_VALUE, CONFIG_FORMAT_FLOAT1_STRING
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , CONFIG_FORMAT_TIMESTAMP_L 
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
    );
    if (_item4) {
      rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item4->getName(), getName());
    };
  };
}
    
void reCWTSoilS::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  // Temperature
  if (_item1) {
    _item1->registerParameters(parent_group, CONFIG_SENSOR_TEMP_KEY, CONFIG_SENSOR_TEMP_NAME, CONFIG_SENSOR_TEMP_FRIENDLY);
  };
  // Humidity (moisture)
  if (_item2) {
    _item2->registerParameters(parent_group, CONFIG_SENSOR_MOISTURE_KEY, CONFIG_SENSOR_MOISTURE_NAME, CONFIG_SENSOR_MOISTURE_FRIENDLY);
  };
  // Conductivity
  if (_item3) {
    _item3->registerParameters(parent_group, CONFIG_SENSOR_CONDUCTIVITY_KEY, CONFIG_SENSOR_CONDUCTIVITY_NAME, CONFIG_SENSOR_CONDUCTIVITY_FRIENDLY);
  };
  // PH
  if (_item4) {
    _item4->registerParameters(parent_group, CONFIG_SENSOR_PH_KEY, CONFIG_SENSOR_PH_NAME, CONFIG_SENSOR_PH_FRIENDLY);
  };
}

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* reCWTSoilS::getDisplayValue()
{
  char* ret = nullptr;
  uint8_t cnt = 0;
  // Humidity (moisture)
  if (_item2) {
    cnt++;
    ret = concat_strings_div(ret, _item2->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  // Temperature
  if ((_item1) && (cnt < 2)) {
    cnt++;
    ret = concat_strings_div(ret, _item1->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  // PH
  if ((_item4) && (cnt < 2)) {
    cnt++;
    ret = concat_strings_div(ret, _item4->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  // Conductivity
  if ((_item3) && (cnt < 2)) {
    cnt++;
    ret = concat_strings_div(ret, _item3->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

// Start device
sensor_status_t reCWTSoilS::sensorReset()
{
  return SENSOR_STATUS_OK;
};

/**
 * Read status register
 * */
esp_err_t reCWTSoilS::readModbusRegister(uint8_t cmd, uint16_t reg, uint16_t* value)
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
sensor_status_t reCWTSoilS::readRawData()
{
  esp_err_t err = ESP_OK;
  uint16_t bufH, bufT, bufC, bufP = 0;

  // Humidity (moisture)
  if ((_item2) && (err == ESP_OK)) {
    err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_HUMIDITY, &bufH);
  };
  // Temperature
  if ((_item1) && (err == ESP_OK)) {
    err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_TEMPERATURE, &bufT);
  };
  // Conductivity
  if ((_item3) && (err == ESP_OK)) {
    err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_CONDUCTIVITY, &bufC);
  };
  // PH
  if ((_item4) && (err == ESP_OK)) {
    err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_PH, &bufP);
  };

  // Check exit code
  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };

  // Store values in sensors
  return setRawValues((float)bufT/10.0, (float)bufH/10.0, (float)bufC, (float)bufP/10.0);
};
