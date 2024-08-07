/* 
   EN: Unified sensor base class used by sensor libraries
   RU: Унифицированный базовый класс сенсора, используемый библиотеками сенсоров
   --------------------------
   (с) 2021-2024 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RESENSOR_H__
#define __RESENSOR_H__

// #include <cstdlib>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>
#include <math.h>
#include <memory.h>
#include "project_config.h"
#include "def_consts.h"
#include "rLog.h"
#include "rStrings.h"
#include "reParams.h"
#include "reEsp32.h"
#include "reEvents.h"
#include "reNvs.h"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Sensors log messages -------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#define RSENSOR_LOG_MSG_INIT                       "Sensor [%s] initialization..."
#define RSENSOR_LOG_MSG_INIT_OK                    "Sensor [%s] initialization completed successfully"
#define RSENSOR_LOG_MSG_NO_INIT                    "Sensor [%s] not initializated"
#define RSENSOR_LOG_MSG_NO_DATA                    "Sensor [%s]: no new data"
#define RSENSOR_LOG_MSG_BAD_VALUE                  "Sensor [%s]: incorrect data"
#define RSENSOR_LOG_MSG_CREATE_ITEM                "Created item \"%s\" for sensor [%s]"
#define RSENSOR_LOG_MSG_CMD_NOT_SUPPORTED          "Command not supported by sensor [%s]"
#define RSENSOR_LOG_MSG_WAKEUP_FAILED              "Failed to wakeup sensor [%s]: %d %s!"
#define RSENSOR_LOG_MSG_CAL_FAILED                 "Failed to load factory calibration data for sensor [%s]"
#define RSENSOR_LOG_MSG_CRC_FAILED                 "Failed to check CRC for sensor [%s]"
#define RSENSOR_LOG_MSG_INIT_FAILED                "Failed to initialize sensor [%s]: %d %s"

#define RSENSOR_LOG_MSG_RESET                      "Sensor [%s] has been reset"
#define RSENSOR_LOG_MSG_RESET_POWER                "Power reset for [%s] sensor"
#define RSENSOR_LOG_MSG_RESET_FAILED               "Soft reset [%s] failed: %d %s"

#define RSENSOR_LOG_MSG_SEND_CONFIG                "Sensor [%s]: send configuration"

#define RSENSOR_LOG_MSG_SET_MODE_HEADER            "Sensor [%s]: set mode 0x%.2x"
#define RSENSOR_LOG_MSG_SET_MODE_OK                "Sensor [%s]: mode 0x%.2x installed"
#define RSENSOR_LOG_MSG_SET_MODE_UNCONFIRMED       "Sensor [%s]: could not set mode to 0x%.2x"
#define RSENSOR_LOG_MSG_SET_MODE_FAILED            "Failed to set mode for sensor [%s]: %d %s"

#define RSENSOR_LOG_MSG_SET_RESOLUTION_HEADER      "Sensor [%s]: set resolution 0x%.2x"
#define RSENSOR_LOG_MSG_SET_RESOLUTION_OK          "Sensor [%s]: resolution 0x%.2x installed"
#define RSENSOR_LOG_MSG_SET_RESOLUTION_FAILED      "Failed to set resolution for sensor [%s]: %d %s"

#define RSENSOR_LOG_MSG_SEND_MEASURMENT            "Failed to send measurment command for sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_READ_HADRWARE_ID           "Failed to read hardware ID from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_READ_HADRWARE_VERSION      "Failed to read hardware version from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_READ_STATUS_FAILED         "Failed to read status from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_READ_MODE_FAILED           "Failed to read mode from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_READ_RESOLUTION_FAILED     "Failed to read resolution from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_SEND_COMMAND_FAILED        "Failed to send command to sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_READ_DATA_FAILED           "Failed to read data from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_READ_HUMD_FAILED           "Failed to read humidity value from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_READ_TEMP_FAILED           "Failed to read temperature value from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_READ_USER_REGISTER_FAILED  "Failed to read user register from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_WRITE_USER_REGISTER_FAILED "Failed to write user register for sensor [%s]: %d %s"

#define RSENSOR_LOG_MSG_HEATER_STATE               "Sensor [%s]: heater is %s"
#define RSENSOR_LOG_MSG_HEATER_ON                  "ON"
#define RSENSOR_LOG_MSG_HEATER_OFF                 "OFF"
#define RSENSOR_LOG_MSG_HEATER_GET_FAILED          "Failed to read heater status for sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_HEATER_SET_FAILED          "Failed to write heater status for sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_HEATER_UNCONFIRMED         "Failed to change heater status for sensor [%s]: UNCONFIRMED"

#define RSENSOR_LOG_MSG_INTERRUPT_STATE            "Sensor [%s]: interrupt is %s"
#define RSENSOR_LOG_MSG_INTERRUPT_ON               "ENABLED"
#define RSENSOR_LOG_MSG_INTERRUPT_OFF              "DISABLED"
#define RSENSOR_LOG_MSG_INTERRUPT_GET_FAILED       "Failed to read interrupt mode from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_INTERRUPT_SET_FAILED       "Failed to write interrupt mode for sensor [%s]: %d %s"

#define RSENSOR_LOG_MSG_THRESHOLD_STATE            "Sensor [%s]: threshold is %s"
#define RSENSOR_LOG_MSG_THRESHOLD_ON               "ENABLED"
#define RSENSOR_LOG_MSG_THRESHOLD_OFF              "DISABLED"
#define RSENSOR_LOG_MSG_THRESHOLD_GET_FAILED       "Failed to read threshold mode from sensor [%s]: %d %s"
#define RSENSOR_LOG_MSG_THRESHOLD_SET_FAILED       "Failed to write threshold mode for sensor [%s]: %d %s"

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------- Check operation result by sensor_status_t --------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// If failed, print msg_failed and exit, returning the status
#define SENSOR_STATUS_CHECK(a, msg_failed) do { \
  sensor_status_t res = a; \
  if (res != SENSOR_STATUS_OK) { \
    rlog_e(logTAG, msg_failed, _name); \
    return res; \
  }; \
} while (0);

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------- Check operation result by esp_err_t -----------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// If failed, print msg_failed and exit, returning the status
#define SENSOR_ERR_CHECK(a, msg_failed) do { \
  esp_err_t err = a; \
  if (err != ESP_OK) { \
    rlog_e(logTAG, msg_failed, _name, err, esp_err_to_name(err)); \
    return convertEspError(err); \
  }; \
} while (0);

// If failed, print msg_failed and exit, returning FALSE
#define SENSOR_ERR_CHECK_BOOL(a, msg_failed) do { \
  esp_err_t err = a; \
  if (err != ESP_OK) { \
    rlog_e(logTAG, msg_failed, _name, err, esp_err_to_name(err)); \
    return false; \
  }; \
} while (0);

#ifdef __cplusplus
extern "C" {
#endif

class rSensor;

typedef enum {
  SENSOR_STATUS_NO_INIT       = 0,   // Sensor not initialized
  SENSOR_STATUS_NO_DATA       = 1,   // Failed to read data
  SENSOR_STATUS_OK            = 2,   // Everything is wonderful
  SENSOR_STATUS_NOT_SUPPORTED = 3,   // Requested operation is not supported
  SENSOR_STATUS_CONN_ERROR    = 4,   // Sensor not connected or communication error
  SENSOR_STATUS_CAL_ERROR     = 5,   // Calibration data not loaded
  SENSOR_STATUS_CRC_ERROR     = 6,   // Checksum error during data transfer
  SENSOR_STATUS_BAD_DATA      = 7,   // Bad values (too different from last measured)
  SENSOR_STATUS_ERROR         = 8    // Any other error
} sensor_status_t;

typedef enum {
  SENSOR_FILTER_RAW           = 0,
  SENSOR_FILTER_AVERAGE       = 1,
  SENSOR_FILTER_MEDIAN        = 2
} sensor_filter_t;

typedef enum {
  SENSOR_MIXED_NONE           = 0,
  SENSOR_MIXED_ITEM_1         = 1,
  SENSOR_MIXED_ITEM_2         = 2,
  SENSOR_MIXED_ITEM_3         = 3,
  SENSOR_MIXED_ITEMS_12       = 4,
  SENSOR_MIXED_ITEMS_13       = 5,
  SENSOR_MIXED_ITEMS_21       = 6,
  SENSOR_MIXED_ITEMS_23       = 7,
  SENSOR_MIXED_ITEMS_31       = 8,
  SENSOR_MIXED_ITEMS_32       = 9,
  SENSOR_MIXED_ITEMS_123      = 10,
  SENSOR_MIXED_ITEMS_213      = 11,
  SENSOR_MIXED_ITEMS_321      = 12,
  SENSOR_MIXED_ITEMS_312      = 13
} sensor_mixed_t;

typedef enum {
  UNIT_TEMP_CELSIUS           = 0,
  UNIT_TEMP_FAHRENHEIT        = 1
} unit_temperature_t;

typedef enum {
  UNIT_PRESSURE_PA            = 0,
  UNIT_PRESSURE_HPA           = 1,
  UNIT_PRESSURE_MMHG          = 2
} unit_pressure_t;

typedef enum { 
  BOUNDS_FIXED                = 0,
  BOUNDS_EXPAND               = 1,
  BOUNDS_EXPAND_MIN           = 2,
  BOUNDS_EXPAND_MAX           = 3,
  BOUNDS_SHIFT_RANGE          = 4
} type_bounds_t;


typedef float value_t;

typedef struct {
  time_t timestamp = 0;
  value_t rawValue = NAN;
  value_t filteredValue = NAN;
} sensor_value_t;

typedef struct {
  #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  bool minValueChanged = false;
  bool maxValueChanged = false;
  #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  sensor_value_t minValue;
  sensor_value_t maxValue;
} sensor_extremums_t;

typedef struct {
  sensor_value_t lastValue;
  sensor_extremums_t extremumsEntirely;
  sensor_extremums_t extremumsWeekly;
  sensor_extremums_t extremumsDaily;
} sensor_data_t;
typedef sensor_data_t * sensor_handle_t;

double calcAbsoluteHumidity(float temp, float humd);
value_t calcDewPoint(value_t tempValue, value_t humidityValue);
value_t calcDewPointSlow(value_t tempValue, value_t humidityValue);

class rSensor;
class rSensorItem;
typedef rSensorItem * rSensorItemHandle;

typedef void (*cb_status_changed_t) (rSensor *sensor, const sensor_status_t oldStatus, const sensor_status_t newStatus);
typedef bool (*cb_publish_data_t) (rSensor *sensor, char* topic, char* payload, const bool free_topic, const bool free_payload);

class rSensorFilterHandler: public param_handler_t {
  private:
    rSensorItem *_item;
  public:
    explicit rSensorFilterHandler(rSensorItem *item);
    void onChange(param_change_mode_t mode) override;
};

class rSensorItem {
  public:
    rSensorItem(rSensor *sensor, const char* itemKey, const char* itemName, const char* itemFriendly,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString);
    virtual ~rSensorItem();

    virtual bool initItem();
    void  setOwner(rSensor *sensor);

    // Properties
    const char* getKey();
    const char* getName();
    const char* getFriendly();

    // Filters
    bool  setFilterMode(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool  doChangeFilterMode();

    // Check and write data
    void  setOffsetValue(float offsetValue);
    void  setValidRange(value_t validMin, value_t validMax);
    void  setRawAndConvertedValue(const value_t rawValue, const value_t convertedValue, const time_t rawTime);
    sensor_status_t setRawValue(const value_t rawValue, const time_t rawTime);
    virtual sensor_status_t checkValue(const value_t rawValue);
    virtual value_t convertValue(const value_t rawValue);
    value_t convertOffsetValue(const value_t rawValue);

    // Read data
    sensor_handle_t getHandle();
    sensor_data_t getValues();
    sensor_value_t getValue();
    char* getStringRaw();
    char* getStringFiltered();

    // Extremums
    sensor_extremums_t getExtremumsEntirely();
    sensor_extremums_t getExtremumsWeekly();
    sensor_extremums_t getExtremumsDaily();
    void resetExtremumsEntirely();
    void resetExtremumsWeekly();
    void resetExtremumsDaily();
    void resetExtremumsTotal();

    // Register internal parameters
    void registerParameters(paramsGroupHandle_t parent_group);

    // Reading data in rSensorItem itself
    virtual sensor_status_t getRawValue(value_t * rawValue);

    // Publishing values
    virtual char* asString(const char* format, const value_t value, bool nan_brackets);
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishDataValue(const char* topic, const char* format, const value_t value);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonDataValue(bool brackets, const char* format, const value_t value);
    #endif // CONFIG_SENSOR_AS_JSON

    // Publishing timestamp
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishTimestamp(const char* topic, sensor_value_t *data);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonTimestamp(sensor_value_t *data);
    #endif // CONFIG_SENSOR_AS_JSON
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE

    // Publishing "timestring" (string value and timestamp)
    char* asStringTimeValue(sensor_value_t *data);
    char* getStringTimeValue();
    #if CONFIG_SENSOR_TIMESTRING_ENABLE
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishStringTimeValue(const char* topic, sensor_value_t *data);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonStringTimeValue(sensor_value_t *data);
    #endif // CONFIG_SENSOR_AS_JSON
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE

    // Publishing raw and filtered values
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishValue(const char* topic, sensor_value_t *data);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonValue(sensor_value_t *data);
    #endif // CONFIG_SENSOR_AS_JSON

    // Publishing part of sensor data
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishPartSensorValue(const char* topic, const char* type, sensor_value_t *data);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonPartSensorValue(const char* type, sensor_value_t *data);
    #endif // CONFIG_SENSOR_AS_JSON

    // Publishing extremes
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishExtremums(const char* topic, sensor_extremums_t *range
      #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
      , const bool minValueChanged, const bool maxValueChanged
      #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
      );
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonExtremums(const char* type, sensor_extremums_t *range);
    #endif // CONFIG_SENSOR_AS_JSON

    // Publishing latest values and extremes
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishValues(const char* topic);
    bool publishNamedValues();
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonValues();
    char* jsonNamedValues();
    #endif // CONFIG_SENSOR_AS_JSON

    // NVS
    void nvsStoreExtremums(const char* nvs_space);
    void nvsRestoreExtremums(const char* nvs_space);
  protected:
    rSensor *_owner = nullptr;
    bool _forcedRawPublish = false;
    virtual void registerItemParameters(paramsGroup_t * group);
  private:
    const char * _itemKey = nullptr;
    const char * _itemName = nullptr;
    const char * _itemFriendly = nullptr;
    const char * _fmtNumeric = nullptr;
    const char * _fmtString = nullptr;
    sensor_data_t _data;
    value_t _limitMin = 0.0;
    value_t _limitMax = 0.0;
    paramsGroup_t * _pgItem = nullptr;
    float _offsetValue = 0.0;
    paramsEntryHandle_t _prm_offsetValue = nullptr;
    float _deltaMax = 0.0;
    paramsEntryHandle_t _prm_deltaMax = nullptr;
    sensor_filter_t _filterMode = SENSOR_FILTER_RAW;
    paramsEntryHandle_t _prm_filterMode = nullptr;
    uint16_t _filterSize = 0;
    paramsEntryHandle_t _prm_filterSize = nullptr;
    uint16_t _filterIndex = 0;
    rSensorFilterHandler *_filterHandler;
    bool _filterInit = false;
    value_t *_filterBuf;
    
    value_t getAverageValue(const value_t rawValue);
    value_t getMedianValue(const value_t rawValue);
    value_t getFilteredValue(const value_t rawValue);
};

class rTemperatureItem: public rSensorItem {
  public:
    rTemperatureItem(rSensor *sensor, const char* itemKey, const char* _itemName, const char* itemFriendly,
      const unit_temperature_t unitValue,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString);
    value_t convertValue(const value_t rawValue) override;
  private:
    unit_temperature_t _units;
};

class rPressureItem: public rSensorItem {
  public:
    rPressureItem(rSensor *sensor, const char* itemKey, const char* itemName, const char* itemFriendly,
      const unit_pressure_t unitValue,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString);
    value_t convertValue(const value_t rawValue) override;
  private:
    unit_pressure_t _units;
};

class rVirtualItem: public rSensorItem {
  public:
    rVirtualItem(rSensor *sensor, const char* itemKey, const char* itemName, const char* itemFriendly,
      const double coefficient,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString);
    value_t convertValue(const value_t rawValue) override;
  protected:
    void registerItemParameters(paramsGroup_t * group) override;
  private:
    double _coefficient = 1.0;
};

class rMapItem: public rSensorItem {
  public:
    rMapItem(rSensor *sensor, const char* itemKey, const char* itemName, const char* itemFriendly,
      const type_bounds_t in_bounds, const value_t in_min, const value_t in_max,
      const value_t out_min, const value_t out_max,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString);

    value_t checkBounds(value_t newValue);
    value_t convertValue(const value_t rawValue) override;
  protected:
    void registerItemParameters(paramsGroup_t * group) override;
  private:
    type_bounds_t _in_bounds; 
    value_t _in_min; 
    value_t _in_max;
    value_t _in_range;
    value_t _out_min; 
    value_t _out_max;
    paramsEntryHandle_t _prm_in_min = nullptr;
    paramsEntryHandle_t _prm_in_max = nullptr;
    paramsEntryHandle_t _prm_in_bounds = nullptr;
    paramsEntryHandle_t _prm_in_range = nullptr;
};

class rSensor {
  public:
    rSensor(uint8_t eventId, const uint8_t items,
      const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    ~rSensor();

    // Reset bus and sensor
    virtual sensor_status_t sensorBusReset();
    virtual sensor_status_t sensorReset() = 0;
    bool sensorStart();

    // Properties
    const char* getName();
    void topicsCreate(bool topicPrimary);
    void topicsFree();
    char* getTopicPub();

    // Set pointer to data storage
    void setSensorItem(const uint8_t index, rSensorItem* item);
    
    // Change filtering mode of one of the internal storages
    bool setFilterMode(const uint8_t index, const sensor_filter_t filterMode, const uint16_t filterSize);

    // Reading data from internal storages
    rSensorItem* getSensorItem(const uint8_t index);
    sensor_handle_t getHandle(const uint8_t index);
    sensor_data_t getItemData(const uint8_t index, const bool readSensor);
    sensor_value_t getItemValue(const uint8_t index, const bool readSensor);
    sensor_extremums_t getItemExtremumsEntirely(const uint8_t index, const bool readSensor);
    sensor_extremums_t getItemExtremumsWeekly(const uint8_t index, const bool readSensor);
    sensor_extremums_t getItemExtremumsDaily(const uint8_t index, const bool readSensor);

    // Reading data from a physical sensor
    sensor_status_t readData();
    sensor_status_t setRawValue(const uint8_t index, const value_t newValue);

    // Register internal parameters
    void registerParameters(paramsGroupHandle_t parent_group, const char * key_name, const char * topic_name, const char * friendly_name);

    // Sensor status
    void setCallbackOnChangeStatus(cb_status_changed_t cb);
    sensor_status_t getStatus();
    const char* statusString(sensor_status_t status);
    const char* getStatusString();

    // Publishing data
    void setCallbackOnPublishData(cb_publish_data_t cb);
    bool publish(char* topic, char* payload, const bool free_payload);
    bool publishData(const bool readSensor);
    
    // Publishing data in JSON
    #if CONFIG_SENSOR_AS_JSON
    virtual char* getJSON();
    #endif // CONFIG_SENSOR_AS_JSON

    // Reset extremums
    void resetExtremumsEntirely();
    void resetExtremumsWeekly();
    void resetExtremumsDaily();
    void resetExtremumsTotal();

    // Store extremums
    void nvsStoreExtremums(const char* nvs_space);
    void nvsRestoreExtremums(const char* nvs_space);
  protected:
    uint8_t             _items_count = 0;
    rSensorItemHandle * _items = nullptr;
    const char *        _name = nullptr;
    const char *        _topicName = nullptr;
    bool                _topicLocal = false;
    char *              _topicPub = nullptr;
    paramsGroup_t *     _pgSensor = nullptr;

    virtual sensor_status_t readRawData() = 0;
    void setRawStatus(sensor_status_t newStatus, bool forced);
    void setErrorStatus(sensor_status_t newStatus, bool forced);
    sensor_status_t convertEspError(const uint32_t error);
    sensor_status_t setEspError(uint32_t error, bool forced);

    // Register parameters of items
    virtual void registerCustomParameters(paramsGroupHandle_t sensor_group); 
    virtual void registerItemsParameters(paramsGroupHandle_t parent_group);

    // Displaying multiple values in one topic
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    virtual char* getDisplayValue();
    virtual char* getDisplayValueStatus();
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED

    // Publishing data in plain text
    #if CONFIG_SENSOR_AS_PLAIN
    virtual bool publishItems();
    virtual bool publishCustomValues();
    #endif // CONFIG_SENSOR_AS_PLAIN

    // Publishing data in JSON
    #if CONFIG_SENSOR_AS_JSON
    virtual char* jsonCustomValues();
    char* jsonDisplayAndCustomValues();
    #endif // CONFIG_SENSOR_AS_JSON
  private:
    uint8_t             _eventId = 0;
    uint32_t            _readInterval;
    int64_t             _readLast;
    sensor_status_t     _lstStatus;
    sensor_status_t     _errStatus;
    uint16_t            _errLimit;
    unsigned long       _errCount;
    cb_status_changed_t _cbOnChangeStatus;
    cb_publish_data_t   _cbOnPublishData;
    void postEventStatus(const sensor_status_t oldStatus, const sensor_status_t newStatus);
};

class rSensorStub: public rSensor {
  public:
    rSensorStub(uint8_t eventId, 
      const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);  
    void setSensorItems(rSensorItem* item);
    // Sensor reset
    sensor_status_t sensorReset() override;
    // Reading data in rSensorItem itself
    virtual sensor_status_t readRawData() override;
    // Puty external value
    sensor_status_t setExtValue(const value_t extValue);
    // Get data from sensor
    sensor_value_t getValue(const bool readSensor);
};

class rSensorHT: public rSensor {
  public:
    rSensorHT(uint8_t eventId, 
      const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 1000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    void setSensorItems(rSensorItem* itemHumidity, rSensorItem* itemTemperature);

    sensor_value_t getTemperature(const bool readSensor);
    sensor_value_t getHumidity(const bool readSensor);
  protected:
    sensor_status_t setRawValues(const value_t newHumidity, const value_t newTemperature);

    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishCustomValues() override; 
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonCustomValues() override; 
    #endif // CONFIG_SENSOR_AS_JSON
};

#ifdef __cplusplus
}
#endif

#endif // __RESENSOR_H__

