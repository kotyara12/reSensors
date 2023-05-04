/* 
   EN: Unified sensor base class used by sensor libraries
   RU: Унифицированный базовый класс сенсора, используемый библиотеками сенсоров
   --------------------------
   (с) 2021-2023 Разживин Александр | Razzhivin Alexander
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
// typedef rSensorItem * rSensorItemHandle;

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
    rSensorItem(rSensor *sensor, const char* itemName,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString 
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , const char* formatTimestamp
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , const char* formatTimestampValue, const char* formatStringTimeValue
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
      );
    virtual ~rSensorItem();

    virtual bool initItem();
    void  setOwner(rSensor *sensor);
    bool  setFilterMode(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool  doChangeFilterMode();
    void  setOffsetValue(float offsetValue);
    void  setValidRange(value_t validMin, value_t validMax);
    void  setRawAndConvertedValue(const value_t rawValue, const value_t convertedValue, const time_t rawTime);
    void  setRawValue(const value_t rawValue, const time_t rawTime);
    virtual sensor_status_t checkValue(const value_t rawValue);
    virtual value_t convertValue(const value_t rawValue);
    value_t convertOffsetValue(const value_t rawValue);
    const char* getName();
    sensor_handle_t getHandle();
    sensor_data_t getValues();
    sensor_value_t getValue();
    char* getStringRaw();
    char* getStringFiltered();
    void resetExtremumsEntirely();
    void resetExtremumsWeekly();
    void resetExtremumsDaily();
    void resetExtremumsTotal();
    sensor_extremums_t getExtremumsEntirely();
    sensor_extremums_t getExtremumsWeekly();
    sensor_extremums_t getExtremumsDaily();

    // Register internal parameters
    void registerParameters(paramsGroupHandle_t parent_group, const char * key_name, const char * topic_name, const char * friendly_name);

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
    bool _forcedRawPublish = false;
    virtual void registerItemParameters(paramsGroup_t * group);
  private:
    rSensor *_owner = nullptr;
    const char * _name = nullptr;
    const char * _fmtNumeric = nullptr;
    const char * _fmtString = nullptr;
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    const char * _fmtTimestamp;
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE
    const char * _fmtTimestampValue;
    const char * _fmtStringTimeValue;
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
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
    rTemperatureItem(rSensor *sensor, const char* itemName, const unit_temperature_t unitValue,
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
  private:
    unit_temperature_t _units;
};

class rPressureItem: public rSensorItem {
  public:
    rPressureItem(rSensor *sensor, const char* itemName, const unit_pressure_t unitValue,
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
  private:
    unit_pressure_t _units;
};

class rMapItem: public rSensorItem {
  public:
    rMapItem(rSensor *sensor, const char* itemName, 
      const type_bounds_t in_bounds, const value_t in_min, const value_t in_max,
      const value_t out_min, const value_t out_max,
      const sensor_filter_t filterMode, const uint16_t filterSize,
      const char* formatNumeric, const char* formatString 
      #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      , const char* formatTimestamp
      #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
      #if CONFIG_SENSOR_TIMESTRING_ENABLE  
      , const char* formatTimestampValue, const char* formatStringTimeValue
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
      );

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
    rSensor(uint8_t eventId);
    ~rSensor();

    // Initialization
    void initProperties(const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    bool sensorStart();
    virtual sensor_status_t sensorReset() = 0;

    // Properties
    const char* getName();
    void topicsCreate(bool topicPrimary);
    void topicsFree();
    char* getTopicPub();

    // Reading data from a physical sensor
    sensor_status_t readData();

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
    virtual char* getJSON() = 0;
    #endif // CONFIG_SENSOR_AS_JSON

    // Reset extremums
    virtual void resetExtremumsEntirely() = 0;
    virtual void resetExtremumsWeekly() = 0;
    virtual void resetExtremumsDaily() = 0;
    virtual void resetExtremumsTotal() = 0;

    // Store extremums
    virtual void nvsStoreExtremums(const char* nvs_space) = 0;
    virtual void nvsRestoreExtremums(const char* nvs_space) = 0;
  protected:
    const char *   _name;
    const char *   _topicName;
    bool           _topicLocal = false;
    char *         _topicPub;
    paramsGroup_t * _pgSensor = nullptr;

    virtual sensor_status_t readRawData() = 0;
    void setRawStatus(sensor_status_t newStatus, bool forced);
    void setErrorStatus(sensor_status_t newStatus, bool forced);
    sensor_status_t convertEspError(const uint32_t error);
    sensor_status_t setEspError(uint32_t error, bool forced);

    // Register parameters of items
    virtual void registerCustomParameters(paramsGroupHandle_t sensor_group); 
    virtual void registerItemsParameters(paramsGroupHandle_t parent_group) = 0;

    // Displaying multiple values in one topic
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    virtual char* getDisplayValue() = 0;
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
    uint8_t         _eventId = 0;
    uint32_t        _readInterval;
    int64_t         _readLast;
    sensor_status_t _lstStatus;
    sensor_status_t _errStatus;
    uint16_t        _errLimit;
    unsigned long   _errCount;
    cb_status_changed_t _cbOnChangeStatus;
    cb_publish_data_t _cbOnPublishData;
    void postEventStatus(const sensor_status_t oldStatus, const sensor_status_t newStatus);
};

class rSensorX1: public rSensor {
  public:
    rSensorX1(uint8_t eventId); 
    ~rSensorX1();
    
    bool setFilterMode(const sensor_filter_t filterMode, const uint16_t filterSize);

    rSensorItem* getSensorItem();
    sensor_handle_t getHandle();
    sensor_data_t getValues(const bool readSensor);
    sensor_value_t getValue(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly(const bool readSensor);
    sensor_extremums_t getExtremumsDaily(const bool readSensor);

    #if CONFIG_SENSOR_AS_JSON
    char*  getJSON() override;
    #endif // CONFIG_SENSOR_AS_JSON

    // Reset extremums
    void resetExtremumsEntirely() override;
    void resetExtremumsWeekly() override;
    void resetExtremumsDaily() override;
    void resetExtremumsTotal() override;

    // Store extremums
    void nvsStoreExtremums(const char* nvs_space) override;
    void nvsRestoreExtremums(const char* nvs_space) override;
  protected:
    rSensorItem *_item;

    void setSensorItems(rSensorItem* item);
    bool initSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize);
    virtual void createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize) = 0;
    sensor_status_t setRawValues(const value_t newValue);
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishItems() override;
    #endif // CONFIG_SENSOR_AS_PLAIN
};

class rSensorStub: public rSensorX1 {
  public:
    rSensorStub(uint8_t eventId);  
    // Connecting external previously created items, for example statically declared
    bool initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
      rSensorItem* item, const uint32_t minReadInterval = 0, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);
    // Sensor reset
    sensor_status_t sensorReset() override;
    // Reading data in rSensorItem itself
    virtual sensor_status_t readRawData() override;
  protected:
    void createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
};

class rSensorX2: public rSensor {
  public:
    rSensorX2(uint8_t eventId);
    ~rSensorX2();

    bool setFilterMode1(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode2(const sensor_filter_t filterMode, const uint16_t filterSize);

    rSensorItem* getSensorItem1();
    rSensorItem* getSensorItem2();
    sensor_handle_t getHandle1();
    sensor_handle_t getHandle2();
    sensor_data_t getValues1(const bool readSensor);
    sensor_data_t getValues2(const bool readSensor);
    sensor_value_t getValue1(const bool readSensor);
    sensor_value_t getValue2(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely1(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely2(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly1(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly2(const bool readSensor);
    sensor_extremums_t getExtremumsDaily1(const bool readSensor);
    sensor_extremums_t getExtremumsDaily2(const bool readSensor);

    #if CONFIG_SENSOR_AS_JSON
    char*  getJSON() override;
    #endif // CONFIG_SENSOR_AS_JSON

    // Reset extremums
    void resetExtremumsEntirely() override;
    void resetExtremumsWeekly() override;
    void resetExtremumsDaily() override;
    void resetExtremumsTotal() override;

    // Store extremums
    void nvsStoreExtremums(const char* nvs_space) override;
    void nvsRestoreExtremums(const char* nvs_space) override;
  protected:
    rSensorItem *_item1;
    rSensorItem *_item2;

    void setSensorItems(rSensorItem* item1, rSensorItem* item2);
    bool initSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                         const sensor_filter_t filterMode2, const uint16_t filterSize2);
    virtual void createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                   const sensor_filter_t filterMode2, const uint16_t filterSize2) = 0;
    
    sensor_status_t setRawValues(const value_t newValue1, const value_t newValue2);
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishItems() override;
    #endif // CONFIG_SENSOR_AS_PLAIN
};

class rSensorHT: public rSensorX2 {
  public:
    rSensorHT(uint8_t eventId);  
  protected:
    void createSensorItems(
      // humidity value
      const sensor_filter_t filterMode1, const uint16_t filterSize1, 
      // temperature value
      const sensor_filter_t filterMode2, const uint16_t filterSize2) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
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

class rSensorX3: public rSensor {
  public:
    rSensorX3(uint8_t eventId);
    ~rSensorX3();

    bool setFilterMode1(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode2(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode3(const sensor_filter_t filterMode, const uint16_t filterSize);

    rSensorItem* getSensorItem1();
    rSensorItem* getSensorItem2();
    rSensorItem* getSensorItem3();
    sensor_handle_t getHandle1();
    sensor_handle_t getHandle2();
    sensor_handle_t getHandle3();
    sensor_data_t getValues1(const bool readSensor);
    sensor_data_t getValues2(const bool readSensor);
    sensor_data_t getValues3(const bool readSensor);
    sensor_value_t getValue1(const bool readSensor);
    sensor_value_t getValue2(const bool readSensor);
    sensor_value_t getValue3(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely1(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely2(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely3(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly1(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly2(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly3(const bool readSensor);
    sensor_extremums_t getExtremumsDaily1(const bool readSensor);
    sensor_extremums_t getExtremumsDaily2(const bool readSensor);
    sensor_extremums_t getExtremumsDaily3(const bool readSensor);

    #if CONFIG_SENSOR_AS_JSON
    char*  getJSON() override;
    #endif // CONFIG_SENSOR_AS_JSON

    // Reset extremums
    void resetExtremumsEntirely() override;
    void resetExtremumsWeekly() override;
    void resetExtremumsDaily() override;
    void resetExtremumsTotal() override;

    // Store extremums
    void nvsStoreExtremums(const char* nvs_space) override;
    void nvsRestoreExtremums(const char* nvs_space) override;
  protected:
    rSensorItem *_item1;
    rSensorItem *_item2;
    rSensorItem *_item3;
    void setSensorItems(rSensorItem* item1, rSensorItem* item2, rSensorItem* item3);
    bool initSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1, 
                         const sensor_filter_t filterMode2, const uint16_t filterSize2,
                         const sensor_filter_t filterMode3, const uint16_t filterSize3);
    virtual void createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                   const sensor_filter_t filterMode2, const uint16_t filterSize2,
                                   const sensor_filter_t filterMode3, const uint16_t filterSize3) = 0;
    sensor_status_t setRawValues(const value_t newValue1, const value_t newValue2, const value_t newValue3);
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishItems() override;
    #endif // CONFIG_SENSOR_AS_PLAIN
};

class rSensorX4: public rSensor {
  public:
    rSensorX4(uint8_t eventId);
    ~rSensorX4();

    bool setFilterMode1(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode2(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode3(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode4(const sensor_filter_t filterMode, const uint16_t filterSize);

    rSensorItem* getSensorItem1();
    rSensorItem* getSensorItem2();
    rSensorItem* getSensorItem3();
    rSensorItem* getSensorItem4();
    sensor_handle_t getHandle1();
    sensor_handle_t getHandle2();
    sensor_handle_t getHandle3();
    sensor_handle_t getHandle4();
    sensor_data_t getValues1(const bool readSensor);
    sensor_data_t getValues2(const bool readSensor);
    sensor_data_t getValues3(const bool readSensor);
    sensor_data_t getValues4(const bool readSensor);
    sensor_value_t getValue1(const bool readSensor);
    sensor_value_t getValue2(const bool readSensor);
    sensor_value_t getValue3(const bool readSensor);
    sensor_value_t getValue4(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely1(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely2(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely3(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely4(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly1(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly2(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly3(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly4(const bool readSensor);
    sensor_extremums_t getExtremumsDaily1(const bool readSensor);
    sensor_extremums_t getExtremumsDaily2(const bool readSensor);
    sensor_extremums_t getExtremumsDaily3(const bool readSensor);
    sensor_extremums_t getExtremumsDaily4(const bool readSensor);

    #if CONFIG_SENSOR_AS_JSON
    char*  getJSON() override;
    #endif // CONFIG_SENSOR_AS_JSON

    // Reset extremums
    void resetExtremumsEntirely() override;
    void resetExtremumsWeekly() override;
    void resetExtremumsDaily() override;
    void resetExtremumsTotal() override;

    // Store extremums
    void nvsStoreExtremums(const char* nvs_space) override;
    void nvsRestoreExtremums(const char* nvs_space) override;
  protected:
    rSensorItem *_item1;
    rSensorItem *_item2;
    rSensorItem *_item3;
    rSensorItem *_item4;
    void setSensorItems(rSensorItem* item1, rSensorItem* item2, rSensorItem* item3, rSensorItem* item4);
    bool initSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1, 
                         const sensor_filter_t filterMode2, const uint16_t filterSize2,
                         const sensor_filter_t filterMode3, const uint16_t filterSize3,
                         const sensor_filter_t filterMode4, const uint16_t filterSize4);
    virtual void createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                   const sensor_filter_t filterMode2, const uint16_t filterSize2,
                                   const sensor_filter_t filterMode3, const uint16_t filterSize3,
                                   const sensor_filter_t filterMode4, const uint16_t filterSize4) = 0;
    sensor_status_t setRawValues(const value_t newValue1, const value_t newValue2, const value_t newValue3, const value_t newValue4);
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishItems() override;
    #endif // CONFIG_SENSOR_AS_PLAIN
};

class rSensorX5: public rSensor {
  public:
    rSensorX5(uint8_t eventId);
    ~rSensorX5();

    bool setFilterMode1(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode2(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode3(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode4(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool setFilterMode5(const sensor_filter_t filterMode, const uint16_t filterSize);

    rSensorItem* getSensorItem1();
    rSensorItem* getSensorItem2();
    rSensorItem* getSensorItem3();
    rSensorItem* getSensorItem4();
    rSensorItem* getSensorItem5();
    sensor_handle_t getHandle1();
    sensor_handle_t getHandle2();
    sensor_handle_t getHandle3();
    sensor_handle_t getHandle4();
    sensor_handle_t getHandle5();
    sensor_data_t getValues1(const bool readSensor);
    sensor_data_t getValues2(const bool readSensor);
    sensor_data_t getValues3(const bool readSensor);
    sensor_data_t getValues4(const bool readSensor);
    sensor_data_t getValues5(const bool readSensor);
    sensor_value_t getValue1(const bool readSensor);
    sensor_value_t getValue2(const bool readSensor);
    sensor_value_t getValue3(const bool readSensor);
    sensor_value_t getValue4(const bool readSensor);
    sensor_value_t getValue5(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely1(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely2(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely3(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely4(const bool readSensor);
    sensor_extremums_t getExtremumsEntirely5(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly1(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly2(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly3(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly4(const bool readSensor);
    sensor_extremums_t getExtremumsWeekly5(const bool readSensor);
    sensor_extremums_t getExtremumsDaily1(const bool readSensor);
    sensor_extremums_t getExtremumsDaily2(const bool readSensor);
    sensor_extremums_t getExtremumsDaily3(const bool readSensor);
    sensor_extremums_t getExtremumsDaily4(const bool readSensor);
    sensor_extremums_t getExtremumsDaily5(const bool readSensor);

    #if CONFIG_SENSOR_AS_JSON
    char*  getJSON() override;
    #endif // CONFIG_SENSOR_AS_JSON

    // Reset extremums
    void resetExtremumsEntirely() override;
    void resetExtremumsWeekly() override;
    void resetExtremumsDaily() override;
    void resetExtremumsTotal() override;

    // Store extremums
    void nvsStoreExtremums(const char* nvs_space) override;
    void nvsRestoreExtremums(const char* nvs_space) override;
  protected:
    rSensorItem *_item1;
    rSensorItem *_item2;
    rSensorItem *_item3;
    rSensorItem *_item4;
    rSensorItem *_item5;
    void setSensorItems(rSensorItem* item1, rSensorItem* item2, rSensorItem* item3, rSensorItem* item4, rSensorItem* item5);
    bool initSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1, 
                         const sensor_filter_t filterMode2, const uint16_t filterSize2,
                         const sensor_filter_t filterMode3, const uint16_t filterSize3,
                         const sensor_filter_t filterMode4, const uint16_t filterSize4,
                         const sensor_filter_t filterMode5, const uint16_t filterSize5);
    virtual void createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                   const sensor_filter_t filterMode2, const uint16_t filterSize2,
                                   const sensor_filter_t filterMode3, const uint16_t filterSize3,
                                   const sensor_filter_t filterMode4, const uint16_t filterSize4,
                                   const sensor_filter_t filterMode5, const uint16_t filterSize5) = 0;
    sensor_status_t setRawValues(const value_t newValue1, const value_t newValue2, const value_t newValue3, const value_t newValue4, const value_t newValue5);
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishItems() override;
    #endif // CONFIG_SENSOR_AS_PLAIN
};

#ifdef __cplusplus
}
#endif

#endif // __RESENSOR_H__

