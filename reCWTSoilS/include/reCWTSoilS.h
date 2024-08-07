/* 
   RU: Драйвер датчика CWT-Soil-THCPH-S (подключение через RS485 Modbus RTU) для ESP32
   EN: Sensor driver CWT-Soil-THCPH-S (connection via RS485 Modbus RTU) for ESP32
   --------------------------------------------------------------------------------
   (с) 2022 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_CWTSOILS_H__
#define __RE_CWTSOILS_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>

#ifdef __cplusplus
extern "C" {
#endif

class reCWTSoilS : public rSensor {
  public:
    reCWTSoilS(uint8_t eventId, 
      void* modbus, const uint8_t address,
      const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    void setSensorItems(rSensorItem* itemHumidity, rSensorItem* itemTemperature, 
      rSensorItem* itemConductivity, rSensorItem* itemPH,
      rSensorItem* itemNitrogenContent, rSensorItem* itemPhosphorusContent, rSensorItem* itemPotassiumContent,
      rSensorItem* itemSalinity, rSensorItem* itemTDS);

    sensor_status_t sensorReset() override;

    sensor_value_t getHumidity(const bool readSensor);
    sensor_value_t getTemperature(const bool readSensor);
    sensor_value_t getConductivity(const bool readSensor);
    sensor_value_t getPH(const bool readSensor);
    sensor_value_t getNitrogenContent(const bool readSensor);
    sensor_value_t getPhosphorusContent(const bool readSensor);
    sensor_value_t getPotassiumContent(const bool readSensor);
    sensor_value_t getSalinity(const bool readSensor);
    sensor_value_t getTDS(const bool readSensor);
  protected:
    sensor_status_t readRawData() override;  

    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
  private:
    void*           _modbus = nullptr;
    uint8_t         _address = 1;
    
    esp_err_t readModbusRegister(uint8_t cmd, uint16_t reg, int16_t* value);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_CWTSOILS_H__
