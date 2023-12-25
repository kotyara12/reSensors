/* 
  EN: Module for receiving data from BME680 and BME688 sensors via I2C bus from ESP32. 
      Based on BME68X Sensor API (https://github.com/BoschSensortec/BME68x-Sensor-API). 
      !!! Air quality data are presented as sensor resistance (not converted)
  RU: Модуль для получения данных с датчиков BME680 и BME688 по I2C шине из ESP32. 
      Основан на BME68X Sensor API (https://github.com/BoschSensortec/BME68x-Sensor-API). 
      !!! Данные по качеству воздуха представлены в виде сопротивления сенсора (без пересчета)
  --------------------------------------------------------------------------------
  (с) 2021 Разживин Александр | Razzhivin Alexander
  kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_BME68x_H__
#define __RE_BME68x_H__

#include <stdint.h>
#include <esp_err.h>
#include <reSensor.h>
#include "driver/i2c.h"
#include "bme68x/bme68x.h"
#include "bme68x/bme68x_defs.h"

#define BME68x_ADDRESS_LOW  BME68X_I2C_ADDR_LOW
#define BME68x_ADDRESS_HIGH BME68X_I2C_ADDR_HIGH

#define BME68x_DEFAULT_GAS_LIMIT_BAD 500
#define BME68x_DEFAULT_GAS_LIMIT_GOOD 50000

// ODR/Standby time (alias for BME680 existing examples)
typedef enum {
  BME68x_STANDBY_590us   = BME68X_ODR_0_59_MS,   // Standby time of 0.59ms
  BME68x_STANDBY_62500us = BME68X_ODR_62_5_MS,   // Standby time of 62.5ms
  BME68x_STANDBY_125ms   = BME68X_ODR_125_MS,    // Standby time of 125ms
  BME68x_STANDBY_250ms   = BME68X_ODR_250_MS,    // Standby time of 250ms
  BME68x_STANDBY_500ms   = BME68X_ODR_500_MS,    // Standby time of 500ms
  BME68x_STANDBY_1000ms  = BME68X_ODR_1000_MS,   // Standby time of 1s
  BME68x_STANDBY_10ms    = BME68X_ODR_10_MS,     // Standby time of 10ms
  BME68x_STANDBY_20ms    = BME68X_ODR_20_MS,     // Standby time of 20ms
  BME68x_STANDBY_NONE    = BME68X_ODR_NONE       // No standby time
} BME68x_STANDBYTIME;

// Oversampling setting (alias for BME680 existing examples)
typedef enum {
  BME68x_OS_OFF   = BME68X_OS_NONE,              // Switch off measurement
  BME68x_OS_X1    = BME68X_OS_1X,                // Perform 1 measurement
  BME68x_OS_X2    = BME68X_OS_2X,                // Perform 2 measurements
  BME68x_OS_X4    = BME68X_OS_4X,                // Perform 4 measurements
  BME68x_OS_X8    = BME68X_OS_8X,                // Perform 8 measurements
  BME68x_OS_X16   = BME68X_OS_16X                // Perform 16 measurements
} BME68x_OVERSAMPLING;

// IIR Filter settings (alias for BME680 existing examples)
typedef enum {
  BME68x_FILTER_OFF    = BME68X_FILTER_OFF,      // Switch off the filter
  BME68x_FILTER_2      = BME68X_FILTER_SIZE_1,   // Filter coefficient of 2
  BME68x_FILTER_4      = BME68X_FILTER_SIZE_3,   // Filter coefficient of 4
  BME68x_FILTER_8      = BME68X_FILTER_SIZE_7,   // Filter coefficient of 8
  BME68x_FILTER_16     = BME68X_FILTER_SIZE_15,  // Filter coefficient of 16
  BME68x_FILTER_32     = BME68X_FILTER_SIZE_31,  // Filter coefficient of 32
  BME68x_FILTER_64     = BME68X_FILTER_SIZE_63,  // Filter coefficient of 64
  BME68x_FILTER_128    = BME68X_FILTER_SIZE_127  // Filter coefficient of 128
} BME68x_IIR_FILTER;

class rBME68xHeaterHandler: public param_handler_t {
  private:
    rSensor *_sensor;
  public:
    rBME68xHeaterHandler(rSensor *sensor);
    void onChange(param_change_mode_t mode) override;
};

class rIAQItem: public rMapItem {
  public:
    rIAQItem(rSensor *sensor, const char* itemName, rSensorItem *humidityItem, rSensorItem *temperatureItem,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString 
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , const char* formatTimestamp
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , const char* formatTimestampValue, const char* formatStringTimeValue
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
      );
    value_t convertValue(const value_t rawValue) override;
  protected:
    rSensorItem *_humidity = nullptr;
    rSensorItem *_temperature = nullptr;
    void registerItemParameters(paramsGroup_t * group) override;
  private:
    float               _hum_ratio = 0.03;  
    limits_autoshift_t  _limits_autoshift = LIM_AUTOSHIFT_OFF;
    paramsEntryHandle_t _prm_hum_ratio = nullptr;
};

class BME68x : public rSensorX4 {
  public:
    BME68x(uint8_t eventId);
    ~BME68x();

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, 
      BME68x_STANDBYTIME odr = BME68x_STANDBY_NONE, BME68x_IIR_FILTER filter = BME68x_FILTER_OFF,
      BME68x_OVERSAMPLING osPress = BME68x_OS_X1, BME68x_OVERSAMPLING osTemp = BME68x_OS_X1, BME68x_OVERSAMPLING osHum = BME68x_OS_X1,
      // pressure filter
      sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, uint16_t filterSize1 = 0, 
      // temperature filter
      sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, uint16_t filterSize2 = 0,
      // humidity filter
      sensor_filter_t filterMode3 = SENSOR_FILTER_RAW, uint16_t filterSize3 = 0,
      // gas resistance filter
      sensor_filter_t filterMode4 = SENSOR_FILTER_RAW, uint16_t filterSize4 = 0,
      // limits
      const uint32_t minReadInterval = BME68X_PERIOD_POLL, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, 
      BME68x_STANDBYTIME odr = BME68x_STANDBY_NONE, BME68x_IIR_FILTER filter = BME68x_FILTER_OFF,
      BME68x_OVERSAMPLING osPress = BME68x_OS_X1, BME68x_OVERSAMPLING osTemp = BME68x_OS_X1, BME68x_OVERSAMPLING osHum = BME68x_OS_X1,
      // pressure filter
      rSensorItem* item1 = nullptr, 
      // temperature filter
      rSensorItem* item2 = nullptr,
      // humidity filter
      rSensorItem* item3 = nullptr,
      // gas resistance filter
      rSensorItem* item4 = nullptr,
      // limits
      const uint32_t minReadInterval = BME68X_PERIOD_POLL, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    // Register internal parameters
    void registerCustomParameters(paramsGroupHandle_t sensor_group) override;

    // Get I2C parameters
    i2c_port_t getI2CNum();
    uint8_t getI2CAddress();

    // Soft reset
    sensor_status_t sensorReset() override;
    sensor_status_t sendHeaterMode();

    // Setting parameters
    bool setConfiguration(BME68x_STANDBYTIME odr = BME68x_STANDBY_NONE, BME68x_IIR_FILTER filter = BME68x_FILTER_OFF,
      BME68x_OVERSAMPLING osPress = BME68x_OS_X1, BME68x_OVERSAMPLING osTemp = BME68x_OS_X1, BME68x_OVERSAMPLING osHum = BME68x_OS_X1);
    bool setOversampling(BME68x_OVERSAMPLING osPress = BME68x_OS_X1, BME68x_OVERSAMPLING osTemp = BME68x_OS_X1, BME68x_OVERSAMPLING osHum = BME68x_OS_X1);
    bool setIIRFilterSize(BME68x_IIR_FILTER filter);
    bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime);
    bool setODR(BME68x_STANDBYTIME odr);
  protected:
    void createSensorItems(
      // pressure value
      const sensor_filter_t filterMode1, const uint16_t filterSize1, 
      // temperature value
      const sensor_filter_t filterMode2, const uint16_t filterSize2,
      // humidity value
      const sensor_filter_t filterMode3, const uint16_t filterSize3,
      // gas resistance value
      const sensor_filter_t filterMode4, const uint16_t filterSize4) override;
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
    struct bme68x_dev        _dev;
    struct bme68x_conf       _conf;
    struct bme68x_heatr_conf _heatr_conf;
    paramsGroupHandle_t      _prm_heater = nullptr;
    paramsEntryHandle_t      _prm_heatr_temp = nullptr;
    paramsEntryHandle_t      _prm_heatr_dur = nullptr;
    rBME68xHeaterHandler*    _heatr_hndl = nullptr;

    uint8_t osr2int(BME68x_OVERSAMPLING osr);
    sensor_status_t checkApiCode(const char* api_name, int8_t rslt);
    sensor_status_t sendConfiguration();    
};

#endif // __RE_BME68x_H__
