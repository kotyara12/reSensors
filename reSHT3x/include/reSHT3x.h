/* 
   RU: Модуль для получения данных с датчиков SHT30D/SHT31D/SHT35D из ESP32
       Основан на библиотеке от ClosedCube Limited (https://github.com/closedcube/ClosedCube_SHT31D_Arduino)
   EN: Module for receiving data from sensors SHT30D/SHT31D/SHT35D from ESP32
       Based on a library from ClosedCube Limited (https://github.com/closedcube/ClosedCube_SHT31D_Arduino)
   --------------------------------------------------------------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_SHT3X_H__
#define __RE_SHT3X_H__

#include <stdint.h>
#include <esp_err.h>
#include "driver/i2c.h"
#include "reSensor.h"

#define SHT3xD_ADDRESS_1 0x44
#define SHT3xD_ADDRESS_2 0x45

typedef enum {
	SHT3xD_MODE_HOLD,   // SHT3xD_MODE_CLOCK_STRETCH,
	SHT3xD_MODE_NOHOLD, // SHT3xD_MODE_POLLING,
} SHT3xD_MODE;

typedef enum {
	SHT3xD_REPEATABILITY_HIGH,
	SHT3xD_REPEATABILITY_MEDIUM,
	SHT3xD_REPEATABILITY_LOW,
} SHT3xD_REPEATABILITY;

typedef enum {
  SHT3xD_SINGLE,                  // Readout of measurement results for single shot mode
	SHT3xD_FREQUENCY_HZ5,           // Periodic data acquisition mode with a frequency of 0.5 Hz
	SHT3xD_FREQUENCY_1HZ,           // Periodic data acquisition mode with a frequency of 1.0 Hz
	SHT3xD_FREQUENCY_2HZ,           // Periodic data acquisition mode with a frequency of 2.0 Hz
	SHT3xD_FREQUENCY_4HZ,           // Periodic data acquisition mode with a frequency of 4.0 Hz
	SHT3xD_FREQUENCY_10HZ           // Periodic data acquisition mode with a frequency of 10 Hz
} SHT3xD_FREQUENCY;

typedef struct {
  bool lastChecksumFailed;        // Write data checksum status :: '0': checksum of last write transfer was correct '1': checksum of last write transfer failed
  bool lastCommandFailed;         // Command status :: '0': last command executed successfully '1': last command not processed. It was either invalid, failed the integrated command checksum 
  bool systemResetDetected;       // System reset detected :: '0': no reset detected since last ‘clear status register’ command '1': reset detected (hard reset, soft reset command or supply fail)
  bool alertTemperature;          // T tracking alert :: ‘0’ : no alert ‘1’ : alert
  bool alertHumidity;             // RH tracking alert :: ‘0’ : no alert ‘1’ : alert
  bool alertPendingStatus;        // Alert pending status :: '0': no pending alerts '1': at least one pending alert
  bool heaterStatus;              // Heater status :: ‘0’ : Heater OFF ‘1’ : Heater ON
} SHT3xD_STATUS;


#ifdef __cplusplus
extern "C" {
#endif

class SHT3xD : public rSensorHT {
  public:
    SHT3xD(uint8_t eventId);

    // Dynamically creating internal items on the heap
    bool initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,
      // hardware properties
      const i2c_port_t numI2C, const uint8_t addrI2C, const SHT3xD_FREQUENCY frequency, const SHT3xD_MODE mode, const SHT3xD_REPEATABILITY repeatability,
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
      const i2c_port_t numI2C, const uint8_t addrI2C, const SHT3xD_FREQUENCY frequency, const SHT3xD_MODE mode, const SHT3xD_REPEATABILITY repeatability,
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
    // Reading serial number
    uint32_t readSerialNumber();
    // Read status register
    sensor_status_t readStatusRegister(SHT3xD_STATUS* status);
    // Clear status register
    sensor_status_t clearStatusRegister();
    // Built-in heater control
    sensor_status_t setHeaterEx(bool heaterMode, bool checkStatus);
    sensor_status_t setHeater(bool heaterMode);
    bool isHeaterEnabled();
    // After issuing the ART command the sensor will start acquiring data with a frequency of 4Hz
    sensor_status_t activateART();
    // Changing the operating mode
    sensor_status_t setMode(const SHT3xD_FREQUENCY frequency, const SHT3xD_MODE mode, const SHT3xD_REPEATABILITY repeatability);
    // Setting limits for alarm triggering
    sensor_status_t setAlertLow(const value_t temperatureSet, const value_t temperatureClear, const value_t humiditySet, const value_t humidityClear);
    sensor_status_t setAlertHigh(const value_t temperatureSet, const value_t temperatureClear, const value_t humiditySet, const value_t humidityClear);
    // Reading alert limits from sensor
    sensor_status_t readAlertLowSet(const uint16_t command, value_t *humidity, value_t *temperature);
    sensor_status_t readAlertLowClear(const uint16_t command, value_t *humidity, value_t *temperature);
    sensor_status_t readAlertHighSet(const uint16_t command, value_t *humidity, value_t *temperature);
    sensor_status_t readAlertHighClear(const uint16_t command, value_t *humidity, value_t *temperature);
    // Reading alert bits
    bool isAlertPendingStatus();
    bool isAlertHumidity();
    bool isAlertTemperature();
  protected:
    sensor_status_t readRawData() override;  
  private:
    i2c_port_t       _I2C_num;
    uint8_t          _I2C_address;
    SHT3xD_FREQUENCY _frequency;
    SHT3xD_MODE      _mode;
    SHT3xD_REPEATABILITY _repeatability;
    bool             _heater;
    uint8_t          _bufCmd[2] = {0};
    uint8_t          _bufData[6] = {0};

    esp_err_t sendCommand(const uint16_t command);
    esp_err_t readBuffer(const uint16_t command, const uint32_t usWaitData, const uint8_t bytes);
    sensor_status_t startPeriodicMode(const SHT3xD_FREQUENCY frequency, const SHT3xD_REPEATABILITY repeatability);
    value_t raw2temperature(uint16_t rawValue);
    value_t raw2humidity(uint16_t rawValue);
    uint16_t temperature2raw(value_t value);
    uint16_t humidity2raw(value_t value);
    sensor_status_t readRawDataCustom(const uint16_t command, const uint32_t measure_delay);
    sensor_status_t readAlertDataCustom(const uint16_t command, value_t *humidity, value_t *temperature);
    sensor_status_t writeAlertDataCustom(const uint16_t command, const value_t humidity, const value_t temperature);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_SHT3X_H__
