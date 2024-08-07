#include "reBH1750.h"
#include <math.h>
#include "rLog.h"
#include "rStrings.h"

static const char * logTAG = "BH1750";

#define OPCODE_HIGH        0x0
#define OPCODE_HIGH2       0x1
#define OPCODE_LOW         0x3

#define OPCODE_RESERT      0x07
#define OPCODE_CONT        0x10
#define OPCODE_OT          0x20

#define OPCODE_POWER_DOWN  0x00
#define OPCODE_POWER_ON    0x01
#define OPCODE_MT_HI       0x40
#define OPCODE_MT_LO       0x60

#define BH1750_I2C_TIMEOUT 3000

BH1750::BH1750(uint8_t eventId, 
  const i2c_port_t numI2C, const uint8_t addrI2C, const bh1750_mode_t mode, const bh1750_resolution_t resolution,
  const char* sensorName, const char* topicName, const bool topicLocal, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
:rSensor(eventId, 1,
  sensorName, topicName, topicLocal, 
  minReadInterval, errorLimit,
  cb_status, cb_publish)
{
  _I2C_num = numI2C;
  _I2C_address = addrI2C;
  _mode = mode;
  _resolution = resolution;
}

void BH1750::setSensorItems(rSensorItem* itemIllumination)
{
  setSensorItem(0, itemIllumination);
}

sensor_status_t BH1750::sensorReset()
{
  esp_err_t err = powerOn();
  if (err == ESP_OK) {
    err = setup(_mode, _resolution);
  };
  if (err == ESP_OK) {
    rlog_i(logTAG, RSENSOR_LOG_MSG_RESET, _name);
  };
  return convertEspError(err);
}

esp_err_t BH1750::sendCommand(uint8_t cmd)
{
  return writeI2C(_I2C_num, _I2C_address, &cmd, 1, nullptr, 0, BH1750_I2C_TIMEOUT);
}

esp_err_t BH1750::powerOn()
{
  esp_err_t err = sendCommand(OPCODE_POWER_ON);
  if (err != ESP_OK) {
    rlog_e(logTAG, "Failed to power up for [%s]: %d %s", _name, err, esp_err_to_name(err));
  };
  return err;
}

esp_err_t BH1750::powerOff()
{
  esp_err_t err = sendCommand(OPCODE_POWER_DOWN);
  if (err != ESP_OK) {
    rlog_e(logTAG, "Failed to power down for [%s]: %d %s", _name, err, esp_err_to_name(err));
  };
  return err;
}

esp_err_t BH1750::setup(bh1750_mode_t mode, bh1750_resolution_t resolution)
{
  uint8_t opcode = mode == BH1750_MODE_CONTINUOUS ? OPCODE_CONT : OPCODE_OT;
  switch (resolution) {
    case BH1750_RES_LOW:  opcode |= OPCODE_LOW;   break;
    case BH1750_RES_HIGH: opcode |= OPCODE_HIGH;  break;
    default:              opcode |= OPCODE_HIGH2; break;
  };
  esp_err_t err = sendCommand(opcode);
  if (err == ESP_OK) {
    _meas_time = (_resolution == BH1750_RES_LOW) ? BH1750_TIME_RES_LOW : BH1750_TIME_RES_HIGH;
  } else {
    rlog_e(logTAG, "Failed to setup sensor [%s]: %d %s", _name, err, esp_err_to_name(err));
  };
  return err;
}

esp_err_t BH1750::setMeasurementTime(uint8_t time)
{
  uint8_t data[2];
  data[0] = OPCODE_MT_HI | (time >> 5);
  data[1] = OPCODE_MT_LO | (time & 0x1f);
  esp_err_t err = writeI2C(_I2C_num, _I2C_address, data, 2, nullptr, 0, BH1750_I2C_TIMEOUT);
  if (err == ESP_OK) {
    _meas_time = time;
  } else {
    rlog_e(logTAG, "Failed to set measurment time for [%s]: %d %s", _name, err, esp_err_to_name(err));
  };
  return err;
}

sensor_status_t BH1750::readRawData()
{
  // If single measurment mode, wake up the sensor as it will auto power off after measurement
  if (_mode == BH1750_MODE_ONE_TIME) {
    esp_err_t err = powerOn();
    if (err == ESP_OK) {
      err = setup(_mode, _resolution);
    };
    if (err != ESP_OK) return convertEspError(err);
    // Wait measurment time
    vTaskDelay(pdMS_TO_TICKS(_meas_time));
  };

  // Read data from sensor
  uint8_t data[2];
  esp_err_t err = readI2C(_I2C_num, _I2C_address, nullptr, 0, data, 2, 0, BH1750_I2C_TIMEOUT);
  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };

  // Convert raw data
  uint16_t level = data[0] << 8 | data[1];
  if (_resolution == BH1750_RES_HIGH2) level /= 2;

  // Convert to LUX
  return setRawValue(0, ((value_t)level * 10.0) / 12.0);
}

sensor_value_t BH1750::getIllumination(const bool readSensor)
{
  return getItemValue(0, readSensor);
}