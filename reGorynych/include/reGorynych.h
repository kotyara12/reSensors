/* 
   RU: Драйвер адаптера TE Gorynych (http://romram.ru/gorynych/TE-GORYNYCH_0004.pdf) для ESP32
   EN: TE Gorynych adapter driver (http://romram.ru/gorynych/TE-GORYNYCH_0004.pdf) for ESP32
   --------------------------------------------------------------------------------
   (с) 2024 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_GORYNYCH_H__
#define __RE_GORYNYCH_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>

#ifdef __cplusplus
extern "C" {
#endif

class reGorynych : public rSensor {
  public:
    reGorynych(uint8_t eventId, 
      void* modbus, const uint8_t address, const uint8_t index,
      const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    void setSensorItems(rSensorItem* itemTemperature);

    sensor_status_t sensorReset() override;

    sensor_value_t getTemperature(const bool readSensor);

    sensor_status_t setBaudRate(uint16_t value);
    sensor_status_t setSlaveId(uint16_t value);
    sensor_status_t setLeds(bool enabled);
    sensor_status_t setCorrection(float value);
  protected:
    sensor_status_t readRawData() override;  
  private:
    void*           _modbus = nullptr;
    uint8_t         _address = 1;
    uint8_t         _index = 1;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_GORYNYCH_H__
