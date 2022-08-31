/* 
  EN: Module for receiving data from BME280 sensors via I2C bus from ESP32. 
      Based on BMP280 Sensor API (https://github.com/BoschSensortec/BMP280_driver). 
  RU: Модуль для получения данных с датчиков BME280 по I2C шине из ESP32. 
      Основан на BMP280 Sensor API (https://github.com/BoschSensortec/BMP280_driver). 
  --------------------------------------------------------------------------------
  (с) 2022 Разживин Александр | Razzhivin Alexander
  kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_BMP280_H__
#define __RE_BMP280_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>
#include "bmp280/bmp280.h"
#include "bmp280/bmp280_defs.h"

#define BMP280_ADDRESS_0X76  BMP280_I2C_ADDR_PRIM
#define BMP280_ADDRESS_0X77  BMP280_I2C_ADDR_SEC

// Power modes (alias for BMP280 existing examples)
typedef enum {
  BMP280_MODE_SLEEP  = BMP280_SLEEP_MODE,        // Sleep mode: no operation, all registers accessible, lowest power, selected after startup 
  BMP280_MODE_FORCED = BMP280_FORCED_MODE,       // Forced mode: perform one measurement, store results and return to sleep mode
  BMP280_MODE_NORMAL = BMP280_NORMAL_MODE        // Normal mode: perpetual cycling of measurements and inactive periods
} BMP280_MODE;  

// ODR/Standby time (alias for BMP280 existing examples)
typedef enum {
  BMP280_STANDBY_590us   = BMP280_ODR_0_5_MS,    // Standby time of 0.5ms
  BMP280_STANDBY_62500us = BMP280_ODR_62_5_MS,   // Standby time of 62.5ms
  BMP280_STANDBY_125ms   = BMP280_ODR_125_MS,    // Standby time of 125ms
  BMP280_STANDBY_250ms   = BMP280_ODR_250_MS,    // Standby time of 250ms
  BMP280_STANDBY_500ms   = BMP280_ODR_500_MS,    // Standby time of 500ms
  BMP280_STANDBY_1000ms  = BMP280_ODR_1000_MS,   // Standby time of 1s
  BMP280_STANDBY_10ms    = BMP280_ODR_2000_MS,   // Standby time of 2s
  BMP280_STANDBY_20ms    = BMP280_ODR_4000_MS    // Standby time of 4s
} BMP280_STANDBYTIME;

// Oversampling setting (alias for BMP280 existing examples)
typedef enum {
  BMP280_OSM_NONE  = BMP280_OS_NONE,              // Switch off measurement
  BMP280_OSM_X1    = BMP280_OS_1X,                // Perform 1 measurement
  BMP280_OSM_X2    = BMP280_OS_2X,                // Perform 2 measurements
  BMP280_OSM_X4    = BMP280_OS_4X,                // Perform 4 measurements
  BMP280_OSM_X8    = BMP280_OS_8X,                // Perform 8 measurements
  BMP280_OSM_X16   = BMP280_OS_16X                // Perform 16 measurements
} BMP280_OVERSAMPLING;

// IIR Filter settings (alias for BMP280 existing examples)
typedef enum {
  BMP280_FLT_NONE   = BMP280_FILTER_OFF,          // Switch off the filter
  BMP280_FLT_2      = BMP280_FILTER_COEFF_2,      // Filter coefficient of 2
  BMP280_FLT_4      = BMP280_FILTER_COEFF_4,      // Filter coefficient of 4
  BMP280_FLT_8      = BMP280_FILTER_COEFF_8,      // Filter coefficient of 8
  BMP280_FLT_16     = BMP280_FILTER_COEFF_16      // Filter coefficient of 16
} BMP280_IIR_FILTER;

class BMP280 : public rSensorX2 {
  public:
    BMP280(uint8_t eventId);
    ~BMP280();

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      const int numI2C, const uint8_t addrI2C, 
      BMP280_MODE mode = BMP280_MODE_FORCED, BMP280_STANDBYTIME odr = BMP280_STANDBY_1000ms, BMP280_IIR_FILTER filter = BMP280_FLT_NONE,
      BMP280_OVERSAMPLING osPress = BMP280_OSM_X1, BMP280_OVERSAMPLING osTemp = BMP280_OSM_X1,
      // pressure filter
      sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, uint16_t filterSize1 = 0, 
      // temperature filter
      sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, uint16_t filterSize2 = 0,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
      // hardware properties
      const int numI2C, const uint8_t addrI2C, 
      BMP280_MODE mode = BMP280_MODE_FORCED, BMP280_STANDBYTIME odr = BMP280_STANDBY_1000ms, BMP280_IIR_FILTER filter = BMP280_FLT_NONE,
      BMP280_OVERSAMPLING osPress = BMP280_OSM_X1, BMP280_OVERSAMPLING osTemp = BMP280_OSM_X1,
      // pressure filter
      rSensorItem* item1 = nullptr, 
      // temperature filter
      rSensorItem* item2 = nullptr,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    // Reset hardware
    sensor_status_t sensorReset() override;
    // Setting parameters
    bool setConfiguration(BMP280_MODE mode = BMP280_MODE_FORCED, 
      BMP280_STANDBYTIME odr = BMP280_STANDBY_1000ms, BMP280_IIR_FILTER filter = BMP280_FLT_NONE,
      BMP280_OVERSAMPLING osPress = BMP280_OSM_X1, BMP280_OVERSAMPLING osTemp = BMP280_OSM_X1);
    bool setOversampling(BMP280_OVERSAMPLING osPress = BMP280_OSM_X1, BMP280_OVERSAMPLING osTemp = BMP280_OSM_X1);
    bool setIIRFilterSize(BMP280_IIR_FILTER filter);
    bool setODR(BMP280_STANDBYTIME odr);
  protected:
    void createSensorItems(
      // pressure value
      const sensor_filter_t filterMode1, const uint16_t filterSize1, 
      // temperature value
      const sensor_filter_t filterMode2, const uint16_t filterSize2) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
    sensor_status_t readRawData() override;  
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
  private:
    int                      _I2C_num;
    uint8_t                  _I2C_address;
    uint8_t                  _meas_wait;
    struct bmp280_dev        _dev;
    struct bmp280_config     _conf;
    BMP280_MODE              _mode = BMP280_MODE_SLEEP;

    uint8_t osr2int(BMP280_OVERSAMPLING osr);
    sensor_status_t checkApiCode(const char* api_name, int8_t rslt);
    sensor_status_t sendConfiguration();
    sensor_status_t sendPowerMode(BMP280_MODE mode);
};

#endif // __RE_BMP280_H__

