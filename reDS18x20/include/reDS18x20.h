/* 
   RU: Модуль для получения данных с датчиков Dallas DS18x20 из ESP32
   EN: Module for receiving data from sensors Dallas DS18x20 from ESP32
   --------------------------------------------------------------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_DS18x20_H__
#define __RE_DS18x20_H__

#include <stdbool.h>
#include <stdint.h>
#include <driver/gpio.h>
#include "reSensor.h"
#include "onewire.h"

// Model IDs
typedef enum {
  MODEL_UNKNOWN  = 0x00,
  MODEL_DS18S20  = 0x10,  // also DS1820
  MODEL_DS18B20  = 0x28,  // also MAX31820
  MODEL_DS1822   = 0x22,
  MODEL_DS1825   = 0x3B,
  MODEL_DS28EA00 = 0x42
} DS18x20_MODEL;

// Device resolution
typedef enum {
  DS18x20_RESOLUTION_INVALID = -1,  // Invalid resolution
  DS18x20_RESOLUTION_9_BIT   = 9,   // 9-bit resolution, LSB bits 2,1,0 undefined
  DS18x20_RESOLUTION_10_BIT  = 10,  // 10-bit resolution, LSB bits 1,0 undefined
  DS18x20_RESOLUTION_11_BIT  = 11,  // 11-bit resolution, LSB bit 0 undefined
  DS18x20_RESOLUTION_12_BIT  = 12,  // 12-bit resolution (default)
} DS18x20_RESOLUTION;

#ifdef __cplusplus
extern "C" {
#endif

class DS18x20 : public rSensorX1 {
  public:
    DS18x20(uint8_t eventId);
    
    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
      // hardware properties
      gpio_num_t pin, onewire_addr_t address, int8_t index, DS18x20_RESOLUTION resolution, bool saveScratchPad,
      // temperature filter
      const sensor_filter_t filterMode = SENSOR_FILTER_RAW, const uint16_t filterSize = 0,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      gpio_num_t pin, onewire_addr_t address, int8_t index, DS18x20_RESOLUTION resolution, bool saveScratchPad,
      // temperature filter
      rSensorItem* item,
      // limits
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      // callbacks
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    
    sensor_status_t sensorReset() override;
    sensor_status_t getResolution(DS18x20_RESOLUTION *resolution);
    sensor_status_t setResolution(DS18x20_RESOLUTION resolution);
  protected:
    void createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
    sensor_status_t readRawData() override;

    #if CONFIG_SENSOR_AS_JSON
    char* jsonCustomValues() override;
    #endif // CONFIG_SENSOR_AS_JSON
  private:
    typedef uint8_t scratchpad[9];

  	gpio_num_t _pin = GPIO_NUM_NC;          // The GPIO pin connected to the 1-Wire bus
    DS18x20_MODEL _model = MODEL_UNKNOWN;   // Family
    onewire_addr_t _address = ONEWIRE_NONE; // 1-Wire device ROM address (64-bit)
    bool _parasitePower = false;            // Parasite power flag
    bool _saveScratchPad = false;           // Values will be saved from scratchpad to EEPROM on every scratchpad write
    DS18x20_RESOLUTION _resolution = DS18x20_RESOLUTION_INVALID; // Current resulution

    bool addressSelect();
    sensor_status_t readPowerSupply();

    bool validFamily(uint8_t faminly_byte);
    bool validAddress(uint8_t* rom_code);
    bool readROM(uint64_t *rom_code);
    bool scanDevices(uint8_t index);

    sensor_status_t readScratchpad(uint8_t *buffer);
    sensor_status_t writeScratchpad(uint8_t *buffer);
    sensor_status_t saveScratchpad();

    void waitForDuration();
    sensor_status_t waitForDeviceSignal();
    sensor_status_t waitForConversion();

    sensor_status_t startConvert();
    sensor_status_t readTemperature(float * value);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_DS18x20_H__
