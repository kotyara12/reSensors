#include "reCWTSlsS.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "rLog.h"
#include <time.h>
#include "mbcontroller.h"

static const char* logTAG = "CWTL";

// Read status registers, read function code: 0x03 
#define FUNCTION_CODE_STATUS_READ     0x03

#define REG_STATUS_HUMIDITY           0x0000
#define REG_STATUS_TEMPERATURE        0x0001
#define REG_STATUS_ILLUMINANCE_H      0x0002
#define REG_STATUS_ILLUMINANCE_L      0x0003
#define REG_STATUS_ILLUMINANCE        0x0004

// Parameters registers, read function code: 0x03, write function code: 0x06
#define REG_PARAM_SLAVE_ID            0x07D0
#define REG_PARAM_BAUD_RATE           0x07D1

reCWTSlsS::reCWTSlsS(uint8_t eventId, 
  void* modbus, const uint8_t address,
  const char* sensorName, const char* topicName, const bool topicLocal, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
:rSensor(eventId, 9, 
  sensorName, topicName, topicLocal, 
  minReadInterval, errorLimit,
  cb_status, cb_publish)
{
  _modbus = modbus;
  _address = address;
}

void reCWTSlsS::setSensorItems(rSensorItem* itemTemperature, rSensorItem* itemHumidity, rSensorItem* itemIlluminance)
{
  setSensorItem(0, itemHumidity);
  setSensorItem(1, itemTemperature);
  setSensorItem(2, itemIlluminance);
}

sensor_value_t reCWTSlsS::getHumidity(const bool readSensor)
{
  return getItemValue(0, readSensor);
}

sensor_value_t reCWTSlsS::getTemperature(const bool readSensor)
{
  return getItemValue(1, readSensor);
}

sensor_value_t reCWTSlsS::getIlluminance(const bool readSensor)
{
  return getItemValue(2, readSensor);
}

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* reCWTSlsS::getDisplayValue()
{
  if (_items[0] && _items[1]) {
    return concat_strings_div(_items[1]->getStringFiltered(), _items[0]->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return nullptr;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

// Start device
sensor_status_t reCWTSlsS::sensorReset()
{
  return SENSOR_STATUS_OK;
};

/**
 * Read humidity and temperature data
 * */
sensor_status_t reCWTSlsS::readRawData()
{
  if (_items[0] && _items[1] && _items[2]) {
    int16_t buffer[4] = {0};
    mb_param_request_t _request = {
      .slave_addr = _address,
      .command    = FUNCTION_CODE_STATUS_READ,
      .reg_start  = REG_STATUS_HUMIDITY,
      .reg_size   = 4
    };
    esp_err_t err = mbc_master_send_request(&_request, (void*)&buffer[0]);
    if (err == ESP_OK) {
      time_t now = time(nullptr);
      sensor_status_t ret = _items[0]->setRawValue((float)buffer[0]/10.0, now);
      if (ret != SENSOR_STATUS_OK) return ret;
      ret = _items[1]->setRawValue((float)buffer[1]/10.0, now);
      if (ret != SENSOR_STATUS_OK) return ret;
      uint32_t illumination = (buffer[2] << 16) | buffer[3];
      ret = _items[2]->setRawValue((float)illumination, now);
      return ret;
    } else {
      rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
      return convertEspError(err);
    };
  };
  return SENSOR_STATUS_NO_INIT;
};
