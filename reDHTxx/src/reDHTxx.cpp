#include "reDHTxx.h"
#include "reEsp32.h"
#include "rLog.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <time.h>
#include "soc/rtc.h" 

static const char* logTAG = "DHTxx";

#define DHT_CHECK(err, str) \
  if (err != ESP_OK) { \
    rlog_e(logTAG, "%s for sensor [%s]: #%d %s", str, _name, err, esp_err_to_name(err)); \
    this->rSensor::setRawStatus(SENSOR_STATUS_ERROR, false); \
    return SENSOR_STATUS_ERROR; \
  };

#define DHT_TIMEOUT UINT32_MAX

// Constructor
DHTxx::DHTxx():rSensorHT()
{ 
  // 1 millisecond timeout for reading pulses from DHT sensor
  // Note that count is now ignored as the DHT reading algorithm adjusts itself based on the speed of the processor.
  rtc_cpu_freq_config_t conf;
  rtc_clk_cpu_freq_get_config(&conf);
  _maxCycles = conf.freq_mhz * 1000;
}

// Dynamically creating internal items on the heap
bool DHTxx::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  DHTxx_TYPE sensorType, const uint8_t gpioNum, const bool gpioPullup,
  const sensor_filter_t filterMode1, const uint16_t filterSize1, 
  const sensor_filter_t filterMode2, const uint16_t filterSize2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  _sensorType = sensorType;
  _sensorGPIO = (gpio_num_t)gpioNum;
  _gpioPullup = gpioPullup;
  // Initialize internal items
  if (this->rSensorX2::initSensorItems(filterMode1, filterSize1, filterMode2, filterSize2)) {
    // Start device
    return initHardware();
  };
  return false;
}

// Connecting external previously created items, for example statically declared
bool DHTxx::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal, 
  DHTxx_TYPE sensorType, const uint8_t gpioNum, const bool gpioPullup,
  rSensorItem* item1, rSensorItem* item2,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  _sensorType = sensorType;
  _sensorGPIO = (gpio_num_t)gpioNum;
  _gpioPullup = gpioPullup;
  // Assign items
  this->rSensorX2::setSensorItems(item1, item2);
  // Start device
  return initHardware();
}

// Sensor initialization and start
bool DHTxx::initHardware()
{
  // Initialize GPIO
  gpio_pad_select_gpio(_sensorGPIO);
  // Set pullup, if needed
  gpio_pulldown_dis(_sensorGPIO);
  if (_gpioPullup) {
    gpio_pullup_en(_sensorGPIO);
  } else {
    gpio_pullup_dis(_sensorGPIO);
  };
  rlog_d(logTAG, RSENSOR_LOG_MSG_INIT_OK, _name);
  return true;
}

// Wait change level
uint32_t DHTxx::expectPulse(bool level) 
{
  uint32_t count = 0;
  while (gpio_get_level(_sensorGPIO) == level) {
    if (count++ >= _maxCycles) {
      return DHT_TIMEOUT; // Exceeded timeout, fail.
    };
  };
  return count;
};

// Get data from device
sensor_status_t DHTxx::readRawData()
{
  uint8_t data[5] = {0, 0, 0, 0, 0};

  // Send start signal to DHT sensor, pull down ~ 2ms to wake up
  DHT_CHECK(gpio_set_direction(_sensorGPIO, GPIO_MODE_OUTPUT), "Failed to change port mode");
	DHT_CHECK(gpio_set_level(_sensorGPIO, 0), "Failed to send start signal");
  if ((_sensorType == DHT_DHT11) || (getStatus() == SENSOR_STATUS_TIMEOUT)) {
    ets_delay_us(20000);
  } else {
  	ets_delay_us(2000);
  };
  // Pull up ~ 25 μs to query data
	DHT_CHECK(gpio_set_level(_sensorGPIO, 1), "Failed to send start signal");
  ets_delay_us(25);

  uint32_t cycles[80];
  {
    // Go to input mode, set the falling edge interrupt in order to detect the beginning of the data
    DHT_CHECK(gpio_set_direction(_sensorGPIO, GPIO_MODE_INPUT), "Failed to change port mode");
    
    // Turn off interrupts temporarily because the next sections are timing critical and we don't want any interruptions.
    vTaskSuspendAll();

    // Wait sensor response signal: DHT will keep the line low for 80 µs and then high for 80 µs
    if (expectPulse(0) == DHT_TIMEOUT) {
      xTaskResumeAll();
      rlog_e(logTAG, "%s timeout waiting for start signal low pulse!", _name);
      this->rSensor::setRawStatus(SENSOR_STATUS_TIMEOUT, false);
      return SENSOR_STATUS_TIMEOUT;
    };
    if (expectPulse(1) == DHT_TIMEOUT) {
      xTaskResumeAll();
      rlog_e(logTAG, "%s timeout waiting for start signal high pulse!", _name);
      this->rSensor::setRawStatus(SENSOR_STATUS_TIMEOUT, false);
      return SENSOR_STATUS_TIMEOUT;
    };

    // Now read the 40 bits sent by the sensor. Each bit is sent as a 50 µs
    // low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 µs then it's a 0 and if it's ~70 µs then it's a 1. 
    // We measure the cycle count of the initial 50 µs low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed
    // all the pulses are read into a array and then examined in a later step
    for (int i = 0; i < 80; i += 2) {
      cycles[i] = expectPulse(0);
      cycles[i + 1] = expectPulse(1);
    };

    // Timing critical code is now complete.
    xTaskResumeAll();
  };

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i = 0; i < 40; ++i) {
    uint32_t lowCycles = cycles[2 * i];
    uint32_t highCycles = cycles[2 * i + 1];  
    if ((lowCycles == DHT_TIMEOUT) || (highCycles == DHT_TIMEOUT)) {
      rlog_e(logTAG, "%s timeout waiting for pulse %d!", _name, i + 1);
      this->rSensor::setRawStatus(SENSOR_STATUS_TIMEOUT, false);
      return SENSOR_STATUS_TIMEOUT;
    };
    data[i / 8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i / 8] |= 1;
    };
  };

  // Verify if checksum is ok
  if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    rlog_e(logTAG, "Invalid checksum on reading data from sensor [%s]: %.2X, %.2X, %.2X, %.2X, rcvd CRC: %.2X, calc CRC: %.2X!",
      _name, data[0], data[1], data[2], data[3], data[4], ((data[0] + data[1] + data[2] + data[3]) & 0xFF));
    this->rSensor::setRawStatus(SENSOR_STATUS_CRC_ERROR, false);
    return SENSOR_STATUS_CRC_ERROR;
  };

  // Calculate float values
  float humdValue = 0.0;
  float tempValue = 0.0;
  switch (_sensorType) {
    case DHT_DHT11:
      humdValue = data[0] + data[1] * 0.1;
      tempValue = data[2];
      if (data[3] & 0x80) tempValue = -1-tempValue;
      tempValue += (data[3] & 0x0F) * 0.1;
      break;
    case DHT_DHT12:
      humdValue = data[0] + data[1] * 0.1;
      tempValue = data[2] + (data[3] & 0x0F) * 0.1;
      if (data[2] & 0x80) tempValue = -tempValue;
      break;
    default:
      humdValue = ((uint16_t)data[0] << 8 | data[1]) * 0.1;
      tempValue = (((uint16_t)(data[2] & 0x7F)) << 8 | data[3]) * 0.1;
      if (data[2] & 0x80) tempValue = -tempValue;
      break;
  }

  // Store values in sensors
  this->rSensorX2::setRawValues(humdValue, tempValue);
  return SENSOR_STATUS_OK;
}
