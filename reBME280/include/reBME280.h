/* 
  EN: Module for receiving data from BME280 sensors via I2C bus from ESP32. 
      Based on BME280 Sensor API (https://github.com/BoschSensortec/BME280_driver). 
  RU: Модуль для получения данных с датчиков BME280 по I2C шине из ESP32. 
      Основан на BME280 Sensor API (https://github.com/BoschSensortec/BME280_driver). 
  --------------------------------------------------------------------------------
  (с) 2022 Разживин Александр | Razzhivin Alexander
  kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_BME280_H__
#define __RE_BME280_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>
#include "driver/i2c.h"
#include "bme280/bme280.h"
#include "bme280/bme280_defs.h"

#define BME280_ADDRESS_0X76  BME280_I2C_ADDR_PRIM
#define BME280_ADDRESS_0X77  BME280_I2C_ADDR_SEC

// Power modes (alias for BME280 existing examples)
typedef enum {
  BME280_MODE_SLEEP  = BME280_SLEEP_MODE,               // Sleep mode: no operation, all registers accessible, lowest power, selected after startup 
  BME280_MODE_FORCED = BME280_FORCED_MODE,              // Forced mode: perform one measurement, store results and return to sleep mode
  BME280_MODE_NORMAL = BME280_NORMAL_MODE               // Normal mode: perpetual cycling of measurements and inactive periods
} BME280_MODE;  

// ODR/Standby time (alias for BME280 existing examples)
typedef enum {
  BME280_STANDBY_590us   = BME280_STANDBY_TIME_0_5_MS,  // Standby time of 0.5ms
  BME280_STANDBY_62500us = BME280_STANDBY_TIME_62_5_MS, // Standby time of 62.5ms
  BME280_STANDBY_125ms   = BME280_STANDBY_TIME_125_MS,  // Standby time of 125ms
  BME280_STANDBY_250ms   = BME280_STANDBY_TIME_250_MS,  // Standby time of 250ms
  BME280_STANDBY_500ms   = BME280_STANDBY_TIME_500_MS,  // Standby time of 500ms
  BME280_STANDBY_1000ms  = BME280_STANDBY_TIME_1000_MS, // Standby time of 1s
  BME280_STANDBY_10ms    = BME280_STANDBY_TIME_10_MS,   // Standby time of 10ms
  BME280_STANDBY_20ms    = BME280_STANDBY_TIME_20_MS    // Standby time of 20ms
} BME280_STANDBYTIME;

// Oversampling setting (alias for BME280 existing examples)
typedef enum {
  BME280_OSM_NONE  = BME280_NO_OVERSAMPLING,            // Switch off measurement
  BME280_OSM_X1    = BME280_OVERSAMPLING_1X,            // Perform 1 measurement
  BME280_OSM_X2    = BME280_OVERSAMPLING_2X,            // Perform 2 measurements
  BME280_OSM_X4    = BME280_OVERSAMPLING_4X,            // Perform 4 measurements
  BME280_OSM_X8    = BME280_OVERSAMPLING_8X,            // Perform 8 measurements
  BME280_OSM_X16   = BME280_OVERSAMPLING_16X            // Perform 16 measurements
} BME280_OVERSAMPLING;

// IIR Filter settings (alias for BME280 existing examples)
typedef enum {
  BME280_FLT_NONE   = BME280_FILTER_COEFF_OFF,          // Switch off the filter
  BME280_FLT_2      = BME280_FILTER_COEFF_2,            // Filter coefficient of 2
  BME280_FLT_4      = BME280_FILTER_COEFF_4,            // Filter coefficient of 4
  BME280_FLT_8      = BME280_FILTER_COEFF_8,            // Filter coefficient of 8
  BME280_FLT_16     = BME280_FILTER_COEFF_16            // Filter coefficient of 16
} BME280_IIR_FILTER;

class BME280 : public rSensorX3 {
  public:
    BME280(uint8_t eventId);
    ~BME280();

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, 
      BME280_MODE mode = BME280_MODE_FORCED, BME280_STANDBYTIME odr = BME280_STANDBY_1000ms, BME280_IIR_FILTER filter = BME280_FLT_NONE,
      BME280_OVERSAMPLING osPress = BME280_OSM_X1, BME280_OVERSAMPLING osTemp = BME280_OSM_X1, BME280_OVERSAMPLING osHumd = BME280_OSM_X1,
      // pressure filter
      sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, uint16_t filterSize1 = 0, 
      // temperature filter
      sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, uint16_t filterSize2 = 0,
      // humidity filter
      sensor_filter_t filterMode3 = SENSOR_FILTER_RAW, uint16_t filterSize3 = 0,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, 
      BME280_MODE mode = BME280_MODE_FORCED, BME280_STANDBYTIME odr = BME280_STANDBY_1000ms, BME280_IIR_FILTER filter = BME280_FLT_NONE,
      BME280_OVERSAMPLING osPress = BME280_OSM_X1, BME280_OVERSAMPLING osTemp = BME280_OSM_X1, BME280_OVERSAMPLING osHumd = BME280_OSM_X1,
      // pressure filter
      rSensorItem* item1 = nullptr, 
      // temperature filter
      rSensorItem* item2 = nullptr,
      // humidity filter
      rSensorItem* item3 = nullptr,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    // Get I2C parameters
    i2c_port_t getI2CNum();
    uint8_t getI2CAddress();

    // Sensor reset
    sensor_status_t sensorReset() override;

    // Setting parameters
    bool setConfiguration(BME280_MODE mode = BME280_MODE_FORCED, 
      BME280_STANDBYTIME odr = BME280_STANDBY_1000ms, BME280_IIR_FILTER filter = BME280_FLT_NONE,
      BME280_OVERSAMPLING osPress = BME280_OSM_X1, BME280_OVERSAMPLING osTemp = BME280_OSM_X1, BME280_OVERSAMPLING osHumd = BME280_OSM_X1);
    bool setOversampling(BME280_OVERSAMPLING osPress = BME280_OSM_X1, BME280_OVERSAMPLING osTemp = BME280_OSM_X1, BME280_OVERSAMPLING osHumd = BME280_OSM_X1);
    bool setIIRFilterSize(BME280_IIR_FILTER filter);
    bool setODR(BME280_STANDBYTIME odr);
  protected:
    void createSensorItems(
      // pressure value
      const sensor_filter_t filterMode1, const uint16_t filterSize1, 
      // temperature value
      const sensor_filter_t filterMode2, const uint16_t filterSize2,
      // humidity value
      const sensor_filter_t filterMode3, const uint16_t filterSize3) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
    sensor_status_t readRawData() override;  
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishCustomValues() override; 
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonCustomValues() override; 
    #endif // CONFIG_SENSOR_AS_JSON
  private:
    i2c_port_t               _I2C_num;
    uint8_t                  _I2C_address;
    uint8_t                  _meas_wait;
    struct bme280_dev        _dev;
    BME280_MODE              _mode = BME280_MODE_SLEEP;

    uint8_t osr2int(BME280_OVERSAMPLING osr);
    sensor_status_t checkApiCode(const char* api_name, int8_t rslt);
    sensor_status_t sendPowerMode(BME280_MODE mode);
    sensor_status_t sendConfiguration(uint8_t settings_sel);
};

#endif // __RE_BME280_H__

