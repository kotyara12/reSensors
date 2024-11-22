/* 
   RU: Драйвер датчика QDY30A (подключение через RS485 Modbus RTU) для ESP32
   EN: Sensor driver QDY30A (connection via RS485 Modbus RTU) for ESP32
   --------------------------------------------------------------------------------
   (с) 2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_QDY30A_H__
#define __RE_QDY30A_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  QUNITS_CM = 0,
  QUNITS_MM = 1
} qdy_units_t;

class reQDY30A : public rSensor {
  public:
    reQDY30A(uint8_t eventId, 
      void* modbus, const uint8_t address, const qdy_units_t units,
      const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    void setSensorItems(rSensorItem* itemLevel);
    sensor_status_t sensorReset() override;
    
    sensor_value_t getLevel(const bool readSensor);
  protected:
    sensor_status_t readRawData() override;  
  private:
    void*            _modbus = nullptr;
    uint8_t          _address = 1;
    qdy_units_t      _units = QUNITS_MM;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_QDY30A_H__
