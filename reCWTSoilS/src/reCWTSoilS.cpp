#include "reCWTSoilS.h"
#include "freertos/FreeRTOS.h"
#include "reEsp32.h"
#include "rLog.h"
#include <time.h>
#include "mbcontroller.h"

static const char* logTAG = "CWTS";

// Read status registers, read function code: 0x03 
#define FUNCTION_CODE_STATUS_READ     0x03

#define REG_STATUS_HUMIDITY           0x0000
#define REG_STATUS_TEMPERATURE        0x0001
#define REG_STATUS_CONDUCTIVITY       0x0002
#define REG_STATUS_PH                 0x0003
#define REG_STATUS_NITROGEN           0x0004
#define REG_STATUS_PHOSPHORUS         0x0005
#define REG_STATUS_POTASSIUM          0x0006
#define REG_STATUS_SALINITY           0x0007
#define REG_STATUS_TDS                0x0008

#define REG_FACTOR_CONDUCTIVITY       0x0022
#define REG_FACTOR_SALINITY           0x0023
#define REG_FACTOR_TDS                0x0024

#define REG_CAL_TEMPERATURE           0x0050
#define REG_CAL_HUMIDITY              0x0051
#define REG_CAL_CONDUCTIVITY          0x0052
#define REG_CAL_PH                    0x0053

// Parameters registers, read function code: 0x03, write function code: 0x06
#define REG_PARAM_SLAVE_ID            0x07D0
#define REG_PARAM_BAUD_RATE           0x07D1

reCWTSoilS::reCWTSoilS(uint8_t eventId, 
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

void reCWTSoilS::setSensorItems(rSensorItem* itemHumidity, rSensorItem* itemTemperature, 
  rSensorItem* itemConductivity, rSensorItem* itemPH,
  rSensorItem* itemNitrogenContent, rSensorItem* itemPhosphorusContent, rSensorItem* itemPotassiumContent,
  rSensorItem* itemSalinity, rSensorItem* itemTDS)
{
  setSensorItem(0, itemHumidity);
  setSensorItem(1, itemTemperature);
  setSensorItem(2, itemConductivity);
  setSensorItem(3, itemPH);
  setSensorItem(4, itemNitrogenContent);
  setSensorItem(5, itemPhosphorusContent);
  setSensorItem(6, itemPotassiumContent);
  setSensorItem(7, itemSalinity);
  setSensorItem(8, itemTDS);
}

sensor_value_t reCWTSoilS::getHumidity(const bool readSensor)
{
  return getItemValue(0, readSensor);
}

sensor_value_t reCWTSoilS::getTemperature(const bool readSensor)
{
  return getItemValue(1, readSensor);
}

sensor_value_t reCWTSoilS::getConductivity(const bool readSensor)
{
  return getItemValue(2, readSensor);
}

sensor_value_t reCWTSoilS::getPH(const bool readSensor)
{
  return getItemValue(3, readSensor);
}

sensor_value_t reCWTSoilS::getNitrogenContent(const bool readSensor)
{
  return getItemValue(4, readSensor);
}

sensor_value_t reCWTSoilS::getPhosphorusContent(const bool readSensor)
{
  return getItemValue(5, readSensor);
}

sensor_value_t reCWTSoilS::getPotassiumContent(const bool readSensor)
{
  return getItemValue(6, readSensor);
}

sensor_value_t reCWTSoilS::getSalinity(const bool readSensor)
{
  return getItemValue(7, readSensor);
}

sensor_value_t reCWTSoilS::getTDS(const bool readSensor)
{
  return getItemValue(8, readSensor);
}

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* reCWTSoilS::getDisplayValue()
{
  char* ret = nullptr;
  uint8_t cnt = 0;
  // Humidity (moisture)
  if (_items[0]) {
    cnt++;
    ret = concat_strings_div(ret, _items[0]->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  // Temperature
  if ((_items[1]) && (cnt < 2)) {
    cnt++;
    ret = concat_strings_div(ret, _items[1]->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  // PH
  if ((_items[3]) && (cnt < 2)) {
    cnt++;
    ret = concat_strings_div(ret, _items[3]->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  // Conductivity
  if ((_items[2]) && (cnt < 2)) {
    cnt++;
    ret = concat_strings_div(ret, _items[2]->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
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
esp_err_t reCWTSoilS::readModbusRegister(uint8_t cmd, uint16_t reg, int16_t* value)
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
  int16_t buffer = 0;
  sensor_status_t ret = SENSOR_STATUS_OK;
  esp_err_t err = ESP_OK;
  for (uint8_t i = 0; i < _items_count; i++) {
    if (_items[i]) {
      // Read register
      switch (i) {
        case 0:
          err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_HUMIDITY, &buffer);
          break;
        case 1:
          err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_TEMPERATURE, &buffer);
          break;
        case 2:
          err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_CONDUCTIVITY, &buffer);
          break;
        case 3:
          err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_PH, &buffer);
          break;
        case 4:
          err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_NITROGEN, &buffer);
          break;
        case 5:
          err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_PHOSPHORUS, &buffer);
          break;
        case 6:
          err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_POTASSIUM, &buffer);
          break;
        case 7:
          err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_SALINITY, &buffer);
          break;
        case 8:
          err = readModbusRegister(FUNCTION_CODE_STATUS_READ, REG_STATUS_TDS, &buffer);
          break;
        default:
          err = ESP_ERR_NOT_SUPPORTED;
      };
      // Put data
      if (err == ESP_OK) {
        if ((i == REG_STATUS_HUMIDITY) || (i == REG_STATUS_TEMPERATURE) || (i == REG_STATUS_PH)) {
          ret = _items[i]->setRawValue((float)buffer / 10.0, time(nullptr));
        } else {
          ret = _items[i]->setRawValue((float)buffer, time(nullptr));
        };
      } else {
        rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
        return convertEspError(err);
      };
    };
  };
  return ret;
};
