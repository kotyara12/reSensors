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

typedef enum {
  CWTS_H        = 0,  // H :: Humidity (moisture)
  CWTS_PH       = 1,  // PH :: PH
  CWTS_TH       = 2,  // T+H :: Temperature + Humidity (moisture)
  CWTS_HC       = 3,  // H+C :: Humidity (moisture) + Conductivity
  CWTS_CPH      = 4,  // C+PH :: Conductivity + PH
  CWTS_THC      = 5,  // T+H+C :: Temperature + Humidity (moisture) + Conductivity
  CWTS_THPH     = 6,  // T+H+PH :: Temperature + Humidity (moisture) + PH
  CWTS_THCPH    = 7   // T+H+C+PH :: Temperature + Humidity (moisture) + Conductivity + PH
} cwt_soil_type_t;

#ifdef __cplusplus
extern "C" {
#endif

class reCWTSoilS : public rSensorX4 {
  public:
    reCWTSoilS(uint8_t eventId);

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      void* modbus, const uint8_t address, const cwt_soil_type_t type,
      // temperature filter
      const sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, const uint16_t filterSize1 = 0,
      // moisture filter
      const sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, const uint16_t filterSize2 = 0, 
      // conductivity filter
      const sensor_filter_t filterMode3 = SENSOR_FILTER_RAW, const uint16_t filterSize3 = 0, 
      // PH filter
      const sensor_filter_t filterMode4 = SENSOR_FILTER_RAW, const uint16_t filterSize4 = 0, 
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      void* modbus, const uint8_t address, const cwt_soil_type_t type,
      // temperature item
      rSensorItem* item1, 
      // moisture item
      rSensorItem* item2,
      // conductivity item
      rSensorItem* item3,
      // PH item
      rSensorItem* item4,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    sensor_status_t sensorReset() override;
  protected:
    void createSensorItems(
      // temperature filter
      const sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, const uint16_t filterSize1 = 0,
      // moisture filter
      const sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, const uint16_t filterSize2 = 0, 
      // conductivity filter
      const sensor_filter_t filterMode3 = SENSOR_FILTER_RAW, const uint16_t filterSize3 = 0, 
      // PH filter
      const sensor_filter_t filterMode4 = SENSOR_FILTER_RAW, const uint16_t filterSize4 = 0) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    sensor_status_t readRawData() override;  
  private:
    void*           _modbus = nullptr;
    uint8_t         _address = 1;
    cwt_soil_type_t _type = CWTS_H;
    
    esp_err_t readModbusRegister(uint8_t cmd, uint16_t reg, uint16_t* value);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_CWTSOILS_H__
