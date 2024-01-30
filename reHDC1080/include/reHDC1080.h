/* 
   RU: Модуль для получения данных с датчиков HDC1080 из ESP32
   EN: Module for receiving data from sensors HDC1080 from ESP32
   --------------------------------------------------------------------------------
   (с) 2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_HDC1080_H__
#define __RE_HDC1080_H__

#include <stdint.h>
#include <esp_err.h>
#include "driver/i2c.h"
#include "reSensor.h"

#define HDC1080_ADDRESS 0x40

typedef enum {
	hdc1080_hres_8bit  = 0x02,
	hdc1080_hres_11bit = 0x01,
	hdc1080_hres_14bit = 0x00
} hdc1080_humidity_resolution_t;

typedef enum {
	hdc1080_tres_11bit = 0x01,
	hdc1080_tres_14bit = 0x00
} hdc1080_temperature_resolution_t;

typedef union {
	uint8_t raw[10];
	struct {
		uint16_t manufacturerID;
		uint16_t deviceID;
		uint16_t serialFirst;
		uint16_t serialMid;
		uint16_t serialLast;
	};
} hdc1080_serial_t;

#ifdef __cplusplus
extern "C" {
#endif

class HDC1080 : public rSensorHT {
  public:
    HDC1080(uint8_t eventId);

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      const i2c_port_t numI2C, const hdc1080_humidity_resolution_t humidity_resolution, const hdc1080_temperature_resolution_t temperature_resolution,
      // humidity filter
      const sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, const uint16_t filterSize1 = 0, 
      // temperature filter
      const sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, const uint16_t filterSize2 = 0,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      const i2c_port_t numI2C, const hdc1080_humidity_resolution_t humidity_resolution, const hdc1080_temperature_resolution_t temperature_resolution,
      // humidity filter
      rSensorItem* item1, 
      // temperature filter
      rSensorItem* item2,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    // Reset hardware
    sensor_status_t sensorReset() override;
    // Soft reset
    sensor_status_t softReset();
    // Set resolutions
    sensor_status_t setResolutions(const hdc1080_humidity_resolution_t humidity_resolution, const hdc1080_temperature_resolution_t temperature_resolution);
    // Reading serial number
    hdc1080_serial_t readSerialNumber();
    // Built-in heater
    sensor_status_t setHeater(const bool heater_enable);
  protected:
    sensor_status_t readRawData() override;  
  private:
    i2c_port_t                       _I2C_num;
    hdc1080_temperature_resolution_t _tres;
    hdc1080_humidity_resolution_t    _hres;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_HDC1080_H__
