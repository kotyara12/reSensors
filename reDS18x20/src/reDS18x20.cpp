#include "reDS18x20.h"
#include <math.h>
#include "rLog.h"
#include "rStrings.h"

static const char * logTAG = "DS18x20";

// OneWire commands
#define DS18x20_READROM            0x33  // Read ROM (family code + serial number + CRC)
#define DS18x20_TEMP_CONVERT       0x44  // Initiate a single temperature conversion
#define DS18x20_SCRATCHPAD_WRITE   0x4E  // Write 3 bytes of data to the device scratchpad at positions 2, 3 and 4
#define DS18x20_SCRATCHPAD_READ    0xBE  // Read 9 bytes of data (including CRC) from the device scratchpad
#define DS18x20_SCRATCHPAD_COPY    0x48  // Copy the contents of the scratchpad to the device EEPROM
#define DS18x20_EEPROM_RECALL      0xB8  // Restore alarm trigger values and configuration data from EEPROM to the scratchpad
#define ds18x20_READ_PWRSUPPLY     0xB4  // Determine if a device is using parasitic power
#define DS18x20_ALARM_SEARCH       0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define SP_TEMP_LSB                0
#define SP_TEMP_MSB                1
#define SP_HIGH_ALARM_TEMP         2
#define SP_LOW_ALARM_TEMP          3
#define SP_CONFIGURATION           4
#define SP_INTERNAL_BYTE           5
#define SP_COUNT_REMAIN            6
#define SP_COUNT_PER_C             7
#define SP_SCRATCHPAD_CRC          8

#define CONVERSION_TIMEOUT_9_BIT   94
#define CONVERSION_TIMEOUT_10_BIT  188
#define CONVERSION_TIMEOUT_11_BIT  375
#define CONVERSION_TIMEOUT_12_BIT  750

static bool _check_resolution(DS18x20_RESOLUTION resolution)
{
  return (resolution >= DS18x20_RESOLUTION_9_BIT) && (resolution <= DS18x20_RESOLUTION_12_BIT);
}

DS18x20::DS18x20(uint8_t eventId):rSensorX1(eventId)
{
  _pin = GPIO_NUM_NC;
  _address = ONEWIRE_NONE;
  _model = MODEL_UNKNOWN;
  _saveScratchPad = true;
  _parasitePower = false;
}

void DS18x20::createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  // Temperature
  _item = new rTemperatureItem(this, CONFIG_SENSOR_TEMP_NAME, (unit_temperature_t)CONFIG_FORMAT_TEMP_UNIT,
    filterMode, filterSize,
    CONFIG_FORMAT_TEMP_VALUE, CONFIG_FORMAT_TEMP_STRING
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    , CONFIG_FORMAT_TIMESTAMP_L
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
    , CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  if (_item) {
    rlog_d(getName(), RSENSOR_LOG_MSG_CREATE_ITEM, _item->getName(), getName());
  };
}

void DS18x20::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  if (_item) {
    _item->registerParameters(parent_group, CONFIG_SENSOR_TEMP_KEY, CONFIG_SENSOR_TEMP_NAME, CONFIG_SENSOR_TEMP_FRIENDLY);
  };
}

// Dynamically creating internal items on the heap
bool DS18x20::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
  gpio_num_t pin, onewire_addr_t address, int8_t index, DS18x20_RESOLUTION resolution, bool saveScratchPad,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _pin = pin;
  _address = address;
  _resolution = resolution;
  _saveScratchPad = saveScratchPad;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX1::initSensorItems(filterMode, filterSize)) {
    // Start device
    if (onewire_reset(_pin)) {
      if (_address == ONEWIRE_NONE) {
        return scanDevices(index) && sensorStart();
      } else {
        return sensorStart();
      };
    } else {
      rlog_e(logTAG, "Failed to reset 1-Wire bus");
    };
  };
  return false;
}

// Connecting external previously created items, for example statically declared
bool DS18x20::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  gpio_num_t pin, onewire_addr_t address, int8_t index, DS18x20_RESOLUTION resolution, bool saveScratchPad,
  rSensorItem* item,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _pin = pin;
  _address = address;
  _resolution = resolution;
  _saveScratchPad = saveScratchPad;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX1::setSensorItems(item);
  // Start device
  if (onewire_reset(_pin)) {
    if (_address == ONEWIRE_NONE) {
      return scanDevices(index) && sensorStart();
    } else {
      _model = (DS18x20_MODEL)_address;
      return sensorStart();
    };
  } else {
    rlog_e(logTAG, "Failed to reset 1-Wire bus");
  };
  return false;
}

sensor_status_t DS18x20::sensorReset()
{
  sensor_status_t rslt = readPowerSupply();
  if (rslt == SENSOR_STATUS_OK) {
    readROM(nullptr);
    rslt = setResolution(_resolution);
  };
  return rslt;
}

sensor_status_t DS18x20::readRawData()
{
  sensor_status_t rslt = SENSOR_STATUS_OK;
  if (!_check_resolution(_resolution)) {
    rslt = getResolution(nullptr);
  };

  if (rslt == SENSOR_STATUS_OK) {
    rslt = startConvert();
    if (rslt == SENSOR_STATUS_OK) {
      rslt = waitForConversion();
      if (rslt == SENSOR_STATUS_OK) {
        float value = NAN;
        rslt = readTemperature(&value);
        if (rslt == SENSOR_STATUS_OK) {
          rslt = setRawValues(value);
        };
      };
    };
  };
  return rslt;
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Device select ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool DS18x20::addressSelect()
{
  bool result = false;
  if (onewire_reset(_pin)) {
    if (_address == ONEWIRE_NONE) {
      // If there's only one device on the bus, we can skip sending the ROM code and instruct it directly
      result = onewire_skip_rom(_pin);
    } else {
      // If there are multiple devices on the bus, a Match ROM command must be issued to address a specific slave
      result = onewire_select(_pin, _address);
      // On Chinese fakes sometimes there are failures
      if (!result) {
        vTaskDelay(1);
        result = onewire_select(_pin, _address);
      };
    };
    if (!result) {
      rlog_e(logTAG, "Sensor [%s]: failed to select sensor", getName());
    };
  } else {
    rlog_e(logTAG, "Sensor [%s]: failed to reset 1-Wire bus on GPIO %d", getName(), _pin);
  };
  return result;
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Device search ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool DS18x20::validFamily(uint8_t faminly_byte) 
{
	switch (faminly_byte) {
    case MODEL_DS18S20:
    case MODEL_DS18B20:
    case MODEL_DS1822:
    case MODEL_DS1825:
    case MODEL_DS28EA00:
      return true;
    default:
      return false;
	};
}

bool DS18x20::validAddress(uint8_t* rom_code) 
{
	return (onewire_crc8(rom_code, 7) == rom_code[7]);
}

bool DS18x20::readROM(uint64_t *rom_code)
{
  if (addressSelect() && onewire_write(_pin, DS18x20_READROM)) {
    uint8_t buf[8] = {0};
    for (int i = 0; i < 8; i++) {
      buf[i] = onewire_read(_pin);
    };
    char data[17] = {0};
    _ui64toa((uint64_t)buf, &data[0], 16);
    rlog_d(logTAG, "Sensor [%s]: read ROM data: %s", getName(), data);
    if (rom_code) {
      *rom_code = (uint64_t)buf;
    };
    return true;
  };
  return false;
}

bool DS18x20::scanDevices(uint8_t index)
{
  uint16_t devicesCount = 0;
  uint16_t ds18Count = 0;

  _model = MODEL_UNKNOWN;
  _address = ONEWIRE_NONE;

  rlog_i(logTAG, "Search devices...");
  onewire_search_t search;
  onewire_search_start(&search);
  onewire_addr_t address = ONEWIRE_NONE;
  while ((address = onewire_search_next(&search, _pin)) != ONEWIRE_NONE) {
    devicesCount++;
    if (validAddress((uint8_t*)&address) && validFamily((uint8_t)address)) {
      ds18Count++;
      char addr[17] = {0};
      _ui64toa(address, &addr[0], 16);
      rlog_i(logTAG, "Found device #%d : %s", ds18Count, addr);
      if ((index == ds18Count) || ((index == 0) && (ds18Count == 1))) {
        _model = (DS18x20_MODEL)address;
        _address = address;
      };
    };
  };
  if (devicesCount == 1) {
    _address = ONEWIRE_NONE;
  };
  rlog_i(logTAG, "Found %d device%s", ds18Count, ds18Count == 1 ? "" : "s");    
  return (ds18Count > 0) && validFamily((uint8_t)_model);
}

#if CONFIG_SENSOR_AS_JSON

char* DS18x20::jsonCustomValues()
{
  char addr[17] = {0};
  _ui64toa(_address, &addr[0], 16);
  return malloc_stringf("\"address\":\"%s\"", addr);
}

#endif // CONFIG_SENSOR_AS_JSON
// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Parasite power ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

sensor_status_t DS18x20::readPowerSupply()
{
  if (addressSelect()) {
    if (onewire_write(_pin, ds18x20_READ_PWRSUPPLY)) {
      int power = onewire_read_bit(_pin);
      if (power >= 0) {
        _parasitePower = power == 0;
        if (_parasitePower) {
          rlog_i(logTAG, "Parasite power detected for GPIO=%d", _pin);
        } else {
          rlog_i(logTAG, "Normal power detected for GPIO=%d", _pin);
        };
        return SENSOR_STATUS_OK;
      };
    };
    rlog_e(logTAG, "Sensor [%s]: failed to read power supply mode", getName());
  };
  return SENSOR_STATUS_CONN_ERROR;
}

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------- ScratchPad ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

sensor_status_t DS18x20::readScratchpad(uint8_t *buffer)
{
  if (addressSelect()) {
    if (onewire_write(_pin, DS18x20_SCRATCHPAD_READ)) {
      uint8_t crc;
      uint8_t expected_crc; 
      for (int i = 0; i < 8; i++)
        buffer[i] = onewire_read(_pin);
      crc = onewire_read(_pin);
      expected_crc = onewire_crc8(buffer, 8);     
      if (crc == expected_crc) {
        return SENSOR_STATUS_OK;
      } else {
        rlog_e(logTAG, "Sensor [%s]: failed to read scratchpad: CRC failed "
          "(temp: %02X %02X, alarm: %02X %02X, config: %02X, reserved: %02X %02X %02X, crc: %02X (expected: %02X))", 
          getName(), buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], crc, expected_crc);
        return SENSOR_STATUS_CONN_ERROR;
      };
    };
    rlog_e(logTAG, "Sensor [%s]: failed to read scratchpad", getName());
  };
  return SENSOR_STATUS_CONN_ERROR;
}

sensor_status_t DS18x20::writeScratchpad(uint8_t *buffer)
{
  if (addressSelect()) {
    if (onewire_write(_pin, DS18x20_SCRATCHPAD_WRITE)) {
      for (int i = 0; i < 3; i++) {
        if (!onewire_write(_pin, buffer[i+SP_HIGH_ALARM_TEMP])) {
          rlog_e(logTAG, "Sensor [%s]: failed to write scratchpad", getName());
          return SENSOR_STATUS_CONN_ERROR;
        };
      };
      // Autosave scratchpad
      if (_saveScratchPad) {
        return saveScratchpad();
      } else {
        return SENSOR_STATUS_OK;
      };
    };
    rlog_e(logTAG, "Sensor [%s]: failed to write scratchpad", getName());
  };
  return SENSOR_STATUS_CONN_ERROR;
}

sensor_status_t DS18x20::saveScratchpad() 
{
  if (addressSelect()) {
    if (onewire_write(_pin, DS18x20_SCRATCHPAD_COPY)) {
      // For parasitic devices, power must be applied within 20us after issuing the convert command
      // Specification: NV Write Cycle Time is typically 2ms, max 10ms
      // Waiting 20ms to allow for sensors that take longer in practice
      if (_parasitePower) onewire_power(_pin);
      vTaskDelay(pdMS_TO_TICKS(20));
      if (_parasitePower) onewire_depower(_pin); 
      return SENSOR_STATUS_OK;
    };
    rlog_e(logTAG, "Sensor [%s]: failed to copy scratchpad", getName());
  };
  return SENSOR_STATUS_CONN_ERROR;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Resolution -----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

sensor_status_t DS18x20::getResolution(DS18x20_RESOLUTION *resolution) 
{
  // DS1820 and DS18S20 have no resolution configuration register
	if (_model == MODEL_DS18S20) {
    _resolution = DS18x20_RESOLUTION_12_BIT;
    if (resolution) *resolution = _resolution;
		return SENSOR_STATUS_NOT_SUPPORTED;
  };

	_resolution = DS18x20_RESOLUTION_INVALID;
  uint8_t scratchpad[9];
  sensor_status_t rslt = readScratchpad(scratchpad);
	if (rslt == SENSOR_STATUS_OK) {
    _resolution = (DS18x20_RESOLUTION)(((scratchpad[SP_CONFIGURATION] >> 5) & 0x03) + (uint8_t)DS18x20_RESOLUTION_9_BIT);
    if (!_check_resolution(_resolution)) {
      _resolution = DS18x20_RESOLUTION_INVALID;
      rlog_e(logTAG, "Sensor [%s]: invalid resolution read from device: 0x%02x", getName(), scratchpad[SP_CONFIGURATION]);
    };
    if (resolution) *resolution = _resolution;
	};
	return rslt;
}

sensor_status_t DS18x20::setResolution(DS18x20_RESOLUTION resolution)
{
  // DS1820 and DS18S20 have no resolution configuration register
  if (_model == MODEL_DS18S20) {
    return SENSOR_STATUS_OK;
  };
  if (!_check_resolution(resolution)) {
    rlog_e(logTAG, "Sensor [%s]: unsupported resolution %d", getName(), resolution);
    return SENSOR_STATUS_NOT_SUPPORTED;
  };

  // Read scratchpad up to and including configuration register
  uint8_t scratchpad[9];
  sensor_status_t rslt = readScratchpad(scratchpad);
  if (rslt == SENSOR_STATUS_OK) {
    // Check current resolution
    _resolution = (DS18x20_RESOLUTION)(((scratchpad[SP_CONFIGURATION] >> 5) & 0x03) + (uint8_t)DS18x20_RESOLUTION_9_BIT);
    if (_check_resolution(_resolution) && (_resolution == resolution)) {
      rlog_i(logTAG, "Sensor [%s]: resolution already set to %d bits", getName(), (int)_resolution);
      return SENSOR_STATUS_OK;
    };
    // Modify configuration register to set resolution
    scratchpad[SP_CONFIGURATION] = ((((uint8_t)resolution - 1) & 0x03) << 5) | 0x1f;
    // Write bytes 2, 3 and 4 of scratchpad
    rslt = writeScratchpad(scratchpad);
    if (rslt == SENSOR_STATUS_OK) {
      // Verify changes
      rslt = getResolution(nullptr);
      if (rslt == SENSOR_STATUS_OK) {
        if (_resolution == resolution) {
          rlog_i(logTAG, "Sensor [%s]: resolution set to %d bits", getName(), (int)_resolution);
          return SENSOR_STATUS_OK;
        } else {
          // Resolution change failed - update the info resolution with the value read from configuration
          rlog_w(logTAG, "Sensor [%s]: resolution consistency lost - refreshed from device: %d", getName(), (int)_resolution);
          return SENSOR_STATUS_ERROR;
        };
      };
    };
  };
  return rslt;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------- Delays --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void DS18x20::waitForDuration()
{
  switch (_resolution) {
    case DS18x20_RESOLUTION_9_BIT:
      vTaskDelay(CONVERSION_TIMEOUT_9_BIT / portTICK_PERIOD_MS);
      break;
    case DS18x20_RESOLUTION_10_BIT:
      vTaskDelay(CONVERSION_TIMEOUT_10_BIT / portTICK_PERIOD_MS);
      break;
    case DS18x20_RESOLUTION_11_BIT:
      vTaskDelay(CONVERSION_TIMEOUT_11_BIT / portTICK_PERIOD_MS);
      break;
    default:
      vTaskDelay(CONVERSION_TIMEOUT_12_BIT / portTICK_PERIOD_MS);
      break;
  };
}

sensor_status_t DS18x20::waitForDeviceSignal()
{
  // wait for conversion to complete - all devices will pull bus low once complete
  TickType_t max_conversion_ticks = CONVERSION_TIMEOUT_12_BIT / portTICK_PERIOD_MS;
  TickType_t start_ticks = xTaskGetTickCount();
  TickType_t duration_ticks = 0;
  uint8_t status = 0;
  do {
    vTaskDelay(1);
    status = onewire_read_bit(_pin);
    duration_ticks = xTaskGetTickCount() - start_ticks;
  } while (status == 0 && duration_ticks < max_conversion_ticks);

  if (duration_ticks >= max_conversion_ticks) {
    rlog_v(logTAG, "Sensor [%s]: conversion timed out", getName());
    return SENSOR_STATUS_CONN_ERROR;
  };

  return SENSOR_STATUS_OK;
}

sensor_status_t DS18x20::waitForConversion()
{
  if (_parasitePower) {
    if (onewire_power(_pin)) {
      waitForDuration();
      onewire_depower(_pin);
      return SENSOR_STATUS_OK;
    };
    return SENSOR_STATUS_CONN_ERROR;
  } else {
    return waitForDeviceSignal();
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Read temperature --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

sensor_status_t DS18x20::startConvert()
{
  if (addressSelect()) {
    // initiate a temperature measurement
    if (onewire_write(_pin, DS18x20_TEMP_CONVERT)) {
      return SENSOR_STATUS_OK;
    };
    rlog_e(logTAG, "Sensor [%s]: failed to initiate a temperature measurement", getName());
  };
  return SENSOR_STATUS_CONN_ERROR;
}

sensor_status_t DS18x20::readTemperature(float * value)
{
  if (addressSelect()) {
    uint8_t scratchpad[9];
    if (readScratchpad(scratchpad) == SENSOR_STATUS_OK) {
      // https://github.com/cpetrich/counterfeit_DS18x20#solution-to-the-85-c-problem
      if (scratchpad[SP_COUNT_REMAIN] == 0x0c && scratchpad[SP_TEMP_MSB] == 0x05 && scratchpad[SP_TEMP_LSB] == 0x50) {
        rlog_e(logTAG, "Sensor [%s]: read power-on value (85.0)", getName());
        return SENSOR_STATUS_ERROR;
      };

      rlog_v(logTAG, "Read data: %02X %02X, resolution %02X (%d)", scratchpad[SP_TEMP_LSB], scratchpad[SP_TEMP_MSB], scratchpad[SP_CONFIGURATION], _resolution);
      _resolution = (DS18x20_RESOLUTION)(((scratchpad[SP_CONFIGURATION] >> 5) & 0x03) + (uint8_t)DS18x20_RESOLUTION_9_BIT);
      int16_t temp_raw = (((int16_t)scratchpad[SP_TEMP_MSB]) << 11) | (((int16_t)scratchpad[SP_TEMP_LSB]) << 3);
      /*
      DS1820 and DS18S20 have a 9-bit temperature register.
      Resolutions greater than 9-bit can be calculated using the data from
      the temperature, and COUNT REMAIN and COUNT PER °C registers in the
      scratchpad.  The resolution of the calculation depends on the model.
      While the COUNT PER °C register is hard-wired to 16 (10h) in a
      DS18S20, it changes with temperature in DS1820.
      After reading the scratchpad, the TEMP_READ value is obtained by
      truncating the 0.5°C bit (bit 0) from the temperature data. The
      extended resolution temperature can then be calculated using the
      following equation:
                                        COUNT_PER_C - COUNT_REMAIN
      TEMPERATURE = TEMP_READ - 0.25 + --------------------------
                                              COUNT_PER_C
      Hagai Shatz simplified this to integer arithmetic for a 12 bits
      value for a DS18S20, and James Cameron added legacy DS1820 support.
      See - http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
      */
      if ((_model == MODEL_DS18S20) && (scratchpad[SP_COUNT_PER_C] != 0)) {
        temp_raw = ((temp_raw & 0xfff0) << 3) - 32 + (((scratchpad[SP_COUNT_PER_C] - scratchpad[SP_COUNT_REMAIN]) << 7) / scratchpad[SP_COUNT_PER_C]);
      };

      if (value) {
        // Convert from raw to Celsius: C = RAW/128
        *value = (float)temp_raw * 0.0078125f; 
      };
      return SENSOR_STATUS_OK;
    };
  };
  return SENSOR_STATUS_CONN_ERROR;
}
