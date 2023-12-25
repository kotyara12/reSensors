/* 
   EN: CCS811 sensor driver for ESP-IDF
   RU: Драйвер сенсора CCS811 для ESP-IDF
   --------------------------
   (с) 2022 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_CCS811_H__
#define __RE_CCS811_H__

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "reParams.h"
#include "reSensor.h"
#include "project_config.h"

// CCS811 I2C addresses
#define CCS811_I2C_ADDRESS_1      0x5A
#define CCS811_I2C_ADDRESS_2      0x5B

// CCS811 operation modes
typedef enum {
  CCS811_MODE_IDLE  = 0,          // Idle, low current mode
  CCS811_MODE_1S    = 1,          // Constant Power mode, IAQ values every 1 s
  CCS811_MODE_10S   = 2,          // Pulse Heating mode, IAQ values every 10 s
  CCS811_MODE_60S   = 3,          // Low Power Pulse Heating, IAQ values every 60 s
  CCS811_MODE_250MS = 4           // Constant Power mode, RAW data every 250 ms
} ccs811_mode_t;

class CCS811 : public rSensorX2 {
  public:
    CCS811(uint8_t eventId);
    ~CCS811();

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, const ccs811_mode_t mode = CCS811_MODE_10S, const gpio_num_t wake_pin = GPIO_NUM_NC,
      // IAQ TVOC filter
      sensor_filter_t filterMode1 = SENSOR_FILTER_RAW, uint16_t filterSize1 = 0,
      // eCO2 filter
      sensor_filter_t filterMode2 = SENSOR_FILTER_RAW, uint16_t filterSize2 = 0,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, const ccs811_mode_t mode = CCS811_MODE_10S, const gpio_num_t wake_pin = GPIO_NUM_NC,
      // IAQ TVOC filter
      rSensorItem* item1 = nullptr,
      // eCO2 filter
      rSensorItem* item2 = nullptr,
      // limits
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    // Initialize sensor
    sensor_status_t sensorReset() override;

    // Set mode
    bool setMode(ccs811_mode_t mode);

    // Baseline & environmental data
    bool getBaseline(uint16_t *baseline);
    bool setBaseline(uint16_t baseline);
    bool setEnvironmentalData(float temperature, float humidity);
    bool getNtcResistance(uint32_t r_ref, uint32_t *res);

    // eCO2 thresholds
    bool enableInterrupt(bool enabled);
    bool enableThreshold(bool enabled);
    bool setCO2Thresholds(uint16_t low, uint16_t high, uint8_t hysteresis);
  protected:
    void createSensorItems(
      // IAQ TVOC value
      const sensor_filter_t filterMode1, const uint16_t filterSize1,
      // eCO2 value
      const sensor_filter_t filterMode2, const uint16_t filterSize2) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
    sensor_status_t readRawData() override;  
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
  private:
    i2c_port_t               _I2C_num;
    uint8_t                  _I2C_address;
    ccs811_mode_t            _mode = CCS811_MODE_IDLE;
    gpio_num_t               _wakePin = GPIO_NUM_NC;

    bool wakeUp();
    bool wakeDown();
    esp_err_t readReg(uint8_t reg, uint8_t *data, uint32_t len);
    esp_err_t writeReg(uint8_t reg, uint8_t *data, uint32_t len);
    sensor_status_t checkErrorStatus();
    sensor_status_t sendMode(ccs811_mode_t mode); 
    sensor_status_t sensorResetEx(ccs811_mode_t mode);

};

#endif // __RE_CCS811_H__

