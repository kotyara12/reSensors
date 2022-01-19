/* 
   EN: A unified base sensor class used by the sensor libraries.
   RU: Унифицированный базовый класс сенсора, используемый библиотеками сенсоров.
   --------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RESENSOR_H__
#define __RESENSOR_H__

#include <sys/types.h>
#include <time.h>
#include <math.h>
#include "project_config.h"
#include "def_consts.h"
#include "reParams.h"
#include "reEvents.h"

#define RSENSOR_LOG_MSG_INIT_OK           "Sensor [%s] initialization completed successfully"
#define RSENSOR_LOG_MSG_CREATE_ITEM       "Created item \"%s\" for sensor [%s]"
#define RSENSOR_LOG_MSG_SOFT_RESET        "Sensor [%s] has been reset!"
#define RSENSOR_LOG_MSG_SOFT_RESET_FAILED "Soft reset failed: %d %s!"
#define RSENSOR_LOG_MSG_HEATER_TMPL       "Heater is %s"
#define RSENSOR_LOG_MSG_HEATER_ON         "ON"
#define RSENSOR_LOG_MSG_HEATER_OFF        "OFF"
#define RSENSOR_LOG_MSG_HEATER_FAILED     "Heating control failed: %d %s!"
#define RSENSOR_LOG_MSG_READ_FAILED       "Failed to read values: %d %s!"
#define RSENSOR_LOG_MSG_READ_DEVICEID     "Failed to read device information: %d %s!"
#define RSENSOR_LOG_MSG_READ_HUMD_FAILED  "Failed to read humidity value: %d %s!"
#define RSENSOR_LOG_MSG_READ_TEMP_FAILED  "Failed to read temperature value: %d %s!"

#ifdef __cplusplus
extern "C" {
#endif

class rSensor;

typedef enum {
  SENSOR_STATUS_NAN           = 0,
  SENSOR_STATUS_OK            = 1,
  SENSOR_STATUS_TIMEOUT       = 2,
  SENSOR_STATUS_CAL_ERROR     = 3,
  SENSOR_STATUS_CRC_ERROR     = 4,
  SENSOR_STATUS_ERROR         = 5,
  SENSOR_STATUS_NOT_SUPPORTED = 6
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
  BOUNDS_AUTOEXPANDING_DIRECT = 1,
  BOUNDS_AUTOEXPANDING_INVERT = 2,
  BOUNDS_SLIDING              = 3
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

    void  setOwner(rSensor *sensor);
    bool  setFilterMode(const sensor_filter_t filterMode, const uint16_t filterSize);
    bool  doChangeFilterMode();
    void  setOffsetValue(float offsetValue);
    void  setRawValue(const value_t rawValue, const time_t rawTime);
    virtual value_t convertValue(const value_t rawValue);
    const char* getName();
    sensor_handle_t getHandle();
    sensor_data_t getValues();
    sensor_value_t getValue();
    char* getStringRaw();
    char* getStringFiltered();
    sensor_extremums_t getExtremumsEntirely();
    sensor_extremums_t getExtremumsWeekly();
    sensor_extremums_t getExtremumsDaily();

    // Register internal parameters
    void registerParameters(paramsGroupHandle_t parent_group, const char * key_name, const char * topic_name, const char * friendly_name);

    // Publishing values
    char* asNumeric(const value_t value);
    char* asString(const value_t value);
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishDataValue(const char* topic, const value_t value);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonDataValue(bool brackets, const value_t value);
    #endif // CONFIG_SENSOR_AS_JSON

    // Publishing timestamp
    char* asTimestamp(const sensor_value_t data);
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishTimestamp(const char* topic, const sensor_value_t data);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonTimestamp(const sensor_value_t data);
    #endif // CONFIG_SENSOR_AS_JSON
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE

    // Publishing "timestring" (string value and timestamp)
    char* asStringTimeValue(const sensor_value_t data);
    #if CONFIG_SENSOR_TIMESTRING_ENABLE
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishStringTimeValue(const char* topic, const sensor_value_t data);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonStringTimeValue(const sensor_value_t data);
    #endif // CONFIG_SENSOR_AS_JSON
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE

    // Publishing raw and filtered values
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishValue(const char* topic, const sensor_value_t data);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonValue(const sensor_value_t data);
    #endif // CONFIG_SENSOR_AS_JSON

    // Publishing part of sensor data
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishPartSensorValue(const char* topic, const char* type, const sensor_value_t data);
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonPartSensorValue(const char* type, const sensor_value_t data);
    #endif // CONFIG_SENSOR_AS_JSON

    // Publishing extremes
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishExtremums(const char* topic, const sensor_extremums_t range
      #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
      , const bool minValueChanged, const bool maxValueChanged
      #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
      );
    #endif // CONFIG_SENSOR_AS_PLAIN
    #if CONFIG_SENSOR_AS_JSON
    char* jsonExtremums(const char* type, const sensor_extremums_t range);
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
  protected:
    bool _forcedRawPublish = false;
    virtual void registerItemParameters(paramsGroup_t * group);
  private:
    rSensor *_owner;
    const char * _name;
    const char * _fmtNumeric;
    const char * _fmtString;
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    const char * _fmtTimestamp;
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE
    const char * _fmtTimestampValue;
    const char * _fmtStringTimeValue;
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
    sensor_data_t _data;
    paramsGroup_t * _pgItem = nullptr;
    float _offsetValue = 0.0;
    paramsEntryHandle_t _prm_offsetValue = nullptr;
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

class rSensor {
  public:
    rSensor();
    ~rSensor();

    // Initialization
    void initProperties(const char* sensorName, const char* topicName, const bool topicLocal, 
      const uint32_t minReadInterval = 2000, const uint16_t errorLimit = 0,
      cb_status_changed_t cb_status = nullptr, cb_publish_data_t cb_publish = nullptr);

    // Properties
    const char* getName();
    void topicsCreate(bool topicPrimary);
    void topicsFree();
    char* getTopicPub();
    void setEventId(uint8_t eventId);

    // Reading data from a physical sensor
    sensor_status_t readRawDataCustom(const uint16_t command, const uint32_t measure_delay);
    sensor_status_t readData();

    // Register internal parameters
    void registerParameters(paramsGroupHandle_t parent_group, const char * key_name, const char * topic_name, const char * friendly_name);

    // Sensor status
    void setCallbackOnChangeStatus(cb_status_changed_t cb);
    sensor_status_t getStatus();
    const char* getStatusString();

    // Displaying multiple values in one topic
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    void setDisplayMode(const sensor_mixed_t displayMode, char* displayFormat);
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED

    // Publishing data
    void setCallbackOnPublishData(cb_publish_data_t cb);
    bool publish(char* topic, char* payload, const bool free_payload);
    bool publishData(const bool readSensor);
    
    // Publishing data in JSON
    #if CONFIG_SENSOR_AS_JSON
    virtual char* getJSON() = 0;
    #endif // CONFIG_SENSOR_AS_JSON
  protected:
    const char *   _name;
    const char *   _topicName;
    bool           _topicLocal = false;
    char *         _topicPub;
    paramsGroup_t * _pgSensor = nullptr;

    // Sensor status
    sensor_status_t _lastStatus;
    virtual sensor_status_t readRawData() = 0;
    void setRawStatus(const sensor_status_t newStatus, const bool forced);
    sensor_status_t setEspError(const uint32_t error, const bool forced);

    // Register parameters of items
    virtual void registerCustomParameters(paramsGroupHandle_t sensor_group); 
    virtual void registerItemsParameters(paramsGroupHandle_t parent_group) = 0;

    // Displaying multiple values in one topic
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    sensor_mixed_t _displayMode;
    char* _displayFormat;
    virtual void  initDisplayMode();
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
    uint8_t        _eventId = 0;
    uint32_t       _readInterval;
    unsigned long  _readLast;
    sensor_status_t _sendStatus;
    uint16_t       _errLimit;
    unsigned long  _errCount;
    cb_status_changed_t _cbOnChangeStatus;
    cb_publish_data_t _cbOnPublishData;
    void postEventStatus(const sensor_status_t oldStatus, const sensor_status_t newStatus);
};

class rSensorX1: public rSensor {
  public:
    rSensorX1(); 
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
  protected:
    rSensorItem *_item;
    void setSensorItems(rSensorItem* item);
    bool initSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize);
    virtual void createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize) = 0;
    void setRawValues(const value_t newValue);
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishItems() override;
    #endif // CONFIG_SENSOR_AS_PLAIN
};

class rSensorX2: public rSensor {
  public:
    rSensorX2();
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
  protected:
    rSensorItem *_item1;
    rSensorItem *_item2;

    void setSensorItems(rSensorItem* item1, rSensorItem* item2);
    bool initSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                         const sensor_filter_t filterMode2, const uint16_t filterSize2);
    virtual void createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                   const sensor_filter_t filterMode2, const uint16_t filterSize2) = 0;
    void setRawValues(const value_t newValue1, const value_t newValue2);
    void setRawValue1(const value_t newValue);
    void setRawValue2(const value_t newValue);
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishItems() override;
    #endif // CONFIG_SENSOR_AS_PLAIN
};

class rSensorHT: public rSensorX2 {
  public:
    rSensorHT();  
  protected:
    void createSensorItems(
      // humidity value
      const sensor_filter_t filterMode1, const uint16_t filterSize1, 
      // temperature value
      const sensor_filter_t filterMode2, const uint16_t filterSize2) override;
    void registerItemsParameters(paramsGroupHandle_t parent_group) override;
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    void initDisplayMode() override;
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
    rSensorX3();
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
    void setRawValues(const value_t newValue1, const value_t newValue2, const value_t newValue3);
    void setRawValue1(const value_t newValue);
    void setRawValue2(const value_t newValue);
    void setRawValue3(const value_t newValue);
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishItems() override;
    #endif // CONFIG_SENSOR_AS_PLAIN
};

class rSensorX4: public rSensor {
  public:
    rSensorX4();
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
    void setRawValues(const value_t newValue1, const value_t newValue2, const value_t newValue3, const value_t newValue4);
    void setRawValue1(const value_t newValue);
    void setRawValue2(const value_t newValue);
    void setRawValue3(const value_t newValue);
    void setRawValue4(const value_t newValue);
    #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* getDisplayValue() override;
    #endif // CONFIG_SENSOR_DISPLAY_ENABLED
    #if CONFIG_SENSOR_AS_PLAIN
    bool publishItems() override;
    #endif // CONFIG_SENSOR_AS_PLAIN
};

class rSensorX5: public rSensor {
  public:
    rSensorX5();
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
    void setRawValues(const value_t newValue1, const value_t newValue2, const value_t newValue3, const value_t newValue4, const value_t newValue5);
    void setRawValue1(const value_t newValue);
    void setRawValue2(const value_t newValue);
    void setRawValue3(const value_t newValue);
    void setRawValue4(const value_t newValue);
    void setRawValue5(const value_t newValue);
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
