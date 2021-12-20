/* 
   RU: Модуль для получения данных с датчиков BMP280/BME280 по I2C шине из ESP32
   EN: Module for receiving data from sensors BMP280/BME280 from ESP32
   --------------------------------------------------------------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_BMX280_H__
#define __RE_BMX280_H__

#include <stdint.h>
#include <esp_err.h>
#include <rSensor.h>

#define BMx_ADDRESS_0 0x76
#define BMx_ADDRESS_1 0x77

typedef enum {
  BMx_UNKNOWN     = 0,
  BMx_BMP280      = 1, // BMP280: temperature, pressure
  BMx_BME280      = 2, // BME280: temperature, humidity, pressure
} BMx_TYPE;

typedef enum {
  STANDBY_500us   = 0,
  STANDBY_62500us = 1,
  STANDBY_125ms   = 2,
  STANDBY_250ms   = 3,
  STANDBY_50ms    = 4,
  STANDBY_1000ms  = 5,
  STANDBY_10ms    = 6,
  STANDBY_20ms    = 7
} BMx_STANDBYTIME;

typedef enum {
  OSR_OFF        = 0, // Skipped (output set to 0x80000)
  OSR_X1         = 1, // Oversampling ×1
  OSR_X2         = 2, // Oversampling ×2
  OSR_X4         = 3, // Oversampling ×4
  OSR_X8         = 4, // Oversampling ×8
  OSR_X16        = 5  // Oversampling ×16
} BMx_OVERSAMPLING;

typedef enum {
  MODE_SLEEP    = 0, // Sleep mode: no operation, all registers accessible, lowest power, selected after startup
  MODE_FORCED   = 1, // Forced mode: perform one measurement, store results and return to sleep mode
  MODE_NORMAL   = 3  // Normal mode: perpetual cycling of measurements and inactive periods.
} BMx_MODE;

typedef enum {
  FILTER_OFF    = 0,
  FILTER_2      = 1,
  FILTER_4      = 2,
  FILTER_8      = 3,
  FILTER_16     = 4
} BMx_IIR_FILTER;

typedef struct {
  uint16_t dig_T1;  ///< temperature compensation value
  int16_t  dig_T2;  ///< temperature compensation value
  int16_t  dig_T3;  ///< temperature compensation value

  uint16_t dig_P1;  ///< pressure compensation value
  int16_t  dig_P2;  ///< pressure compensation value
  int16_t  dig_P3;  ///< pressure compensation value
  int16_t  dig_P4;  ///< pressure compensation value
  int16_t  dig_P5;  ///< pressure compensation value
  int16_t  dig_P6;  ///< pressure compensation value
  int16_t  dig_P7;  ///< pressure compensation value
  int16_t  dig_P8;  ///< pressure compensation value
  int16_t  dig_P9;  ///< pressure compensation value

  uint8_t  dig_H1;  ///< humidity compensation value
  int16_t  dig_H2;  ///< humidity compensation value
  uint8_t  dig_H3;  ///< humidity compensation value
  int16_t  dig_H4;  ///< humidity compensation value
  int16_t  dig_H5;  ///< humidity compensation value
  int8_t   dig_H6;  ///< humidity compensation value
} BMx280_CAL_DATA;

class BMx280 : public rSensorX3 {
  public:
    BMx280();

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      const BMx_TYPE type, const int numI2C, const uint8_t addrI2C, 
      const BMx_MODE mode = MODE_FORCED, const BMx_STANDBYTIME t_sb = STANDBY_500us, const BMx_IIR_FILTER filter = FILTER_OFF,
      const BMx_OVERSAMPLING osrs_p = OSR_X1, const BMx_OVERSAMPLING osrs_t = OSR_X1, const BMx_OVERSAMPLING osrs_h = OSR_X1,
      // pressure filter
      const sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, const uint16_t filterSize1 = 0, 
      // temperature filter
      const sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, const uint16_t filterSize2 = 0,
      // humidity filter
      const sensor_filter_t filterMode3 = SENSOR_FILTER_RAW, const uint16_t filterSize3 = 0,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
      // hardware properties
      const BMx_TYPE type, const int numI2C, const uint8_t addrI2C, 
      const BMx_MODE mode = MODE_FORCED, const BMx_STANDBYTIME t_sb = STANDBY_500us, const BMx_IIR_FILTER filter = FILTER_OFF,
      const BMx_OVERSAMPLING osrs_p = OSR_X1, const BMx_OVERSAMPLING osrs_t = OSR_X1, const BMx_OVERSAMPLING osrs_h = OSR_X1,
      // pressure filter
      rSensorItem* item1 = nullptr, 
      // temperature filter
      rSensorItem* item2 = nullptr,
      // humidity filter
      rSensorItem* item3 = nullptr,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    // Soft reset
    sensor_status_t softReset();
    // Reading sensor id
    uint8_t readSendorId();
    // Set measurement mode
    sensor_status_t setMode(const BMx_MODE mode = MODE_FORCED, const BMx_STANDBYTIME t_sb = STANDBY_500us, const BMx_IIR_FILTER filter = FILTER_OFF,
      const BMx_OVERSAMPLING osrs_p = OSR_X1, const BMx_OVERSAMPLING osrs_t = OSR_X1, const BMx_OVERSAMPLING osrs_h = OSR_X1);
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
    void initDisplayMode() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishCustomValues() override; 
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonCustomValues() override; 
    #endif // CONFIG_SENSOR_AS_JSON
  private:
    int              _I2C_num;
    uint8_t          _I2C_address;
    uint8_t          _readBuf[8];
    BMx_TYPE         _type;
    BMx_MODE         _mode;
    BMx_STANDBYTIME  _t_sb;
    BMx_IIR_FILTER   _filter;
    BMx_OVERSAMPLING _osrs_p;
    BMx_OVERSAMPLING _osrs_t;
    BMx_OVERSAMPLING _osrs_h;
    BMx280_CAL_DATA  _cal_data;

    bool initHardware(const int numI2C, const uint8_t addrI2C);
    esp_err_t readRegisterI8(uint8_t reg_addr, int8_t *value);
    esp_err_t readRegisterU8(uint8_t reg_addr, uint8_t *value);
    esp_err_t readRegisterI16(uint8_t reg_addr, int16_t *value);
    esp_err_t readRegisterU16(uint8_t reg_addr, uint16_t *value);
    esp_err_t readBuffer(uint8_t reg_addr);
    esp_err_t writeRegisterU8(uint8_t reg_addr, uint8_t value);
    bool isReadingCalibration();
    bool isMeasuring(esp_err_t * error);
    sensor_status_t readCalibration();
};

#endif // __RE_BMX280_H__
