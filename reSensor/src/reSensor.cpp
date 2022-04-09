#include "reSensor.h"
#include <cstdlib>
#include <memory.h>
#include <math.h>
#include "rLog.h"
#include "rStrings.h"
#include "esp_task_wdt.h"
#include "def_consts.h"
#ifdef ADRUINO
  #include <Arduino.h>
#else
  #include "reEsp32.h"
#endif // ADRUINO

static const char* logTAG = "SENS";

// =======================================================================================================================
// =======================================================================================================================
// =================================================== meteo functions ===================================================
// =======================================================================================================================
// =======================================================================================================================

value_t calcDewPoint(value_t tempValue, value_t humidityValue) 
{
  double a = 17.271;
  double b = 237.7;
  double c = (a * (double)tempValue) / (b + (double)tempValue) + log((double)humidityValue/100);
  return (value_t)((b * c) / (a - c));
}

value_t calcDewPointSlow(value_t tempValue, value_t humidityValue)
{
  double a0 = (double) 373.15 / (273.15 + (double)tempValue);
  double SUM = (double) -7.90298 * (a0-1.0);
  SUM += 5.02808 * log10(a0);
  SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/a0)))-1) ;
  SUM += 8.1328e-3 * (pow(10,(-3.49149*(a0-1)))-1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM-3) * (double) humidityValue;
  double T = log(VP/0.61078);
  return (value_t)((241.88 * T) / (17.558-T));
}

// =======================================================================================================================
// =======================================================================================================================
// ================================================= Filter mode handler =================================================
// =======================================================================================================================
// =======================================================================================================================

rSensorFilterHandler::rSensorFilterHandler(rSensorItem *item)
{
  _item = item;
}

void rSensorFilterHandler::onChange(param_change_mode_t mode)
{
  if (_item) _item->doChangeFilterMode();
}

// =======================================================================================================================
// =======================================================================================================================
// ===================================================== rSensorItem =====================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensorItem::rSensorItem(rSensor *sensor, const char* itemName,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString 
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , const char* formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , const char* formatTimestampValue, const char* formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
) {
  _owner = sensor;
  _name = itemName;
  _fmtNumeric = formatNumeric;
  _fmtString = formatString;
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  _fmtTimestamp = formatTimestamp;
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  _fmtTimestampValue = formatTimestampValue;
  _fmtStringTimeValue = formatStringTimeValue;
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  _pgItem = nullptr;
  _offsetValue = 0.0;

  _forcedRawPublish = false;
  _filterMode = SENSOR_FILTER_RAW;
  _filterSize = 0;
  _filterIndex = 0;
  _filterInit = false;
  _filterBuf = nullptr;
  _filterHandler = new rSensorFilterHandler(this);
  
  setFilterMode(filterMode, filterSize);
}

// Destructor
rSensorItem::~rSensorItem()
{
  if (_filterHandler) delete _filterHandler;
  if (_filterBuf) free(_filterBuf);
}

bool rSensorItem::initItem()
{
  return true;
}

void rSensorItem::setOwner(rSensor *sensor)
{
  _owner = sensor;
}

bool rSensorItem::setFilterMode(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if ((filterMode != _filterMode) || (_filterSize != filterSize)) {
    // Setting new values
    _filterSize = filterSize;
    _filterMode = filterMode;
    _filterIndex = 0;
    _filterInit = false;
    _filterBuf = nullptr;

    // Allocating memory for an array of filter values
    return doChangeFilterMode();

    // Publish new values
    if (_prm_filterMode) paramsValueStore(_prm_filterMode, false);
    if (_prm_filterSize) paramsValueStore(_prm_filterSize, false);
  };

  return true;
}

bool rSensorItem::doChangeFilterMode()
{
  // Freeing memory if it was previously allocated
  if (_filterBuf) {
    free(_filterBuf);
    _filterBuf = nullptr;
  };
  // Allocating memory for an array of filter values
  if ((_filterMode != SENSOR_FILTER_RAW) && (_filterSize > 0)) {
    _filterInit = false;
    _filterBuf = (value_t*)esp_calloc(_filterSize, sizeof(value_t)); 
    if (_filterBuf == nullptr) {
      _filterSize = 0;
      if (_owner) {
        rlog_e(_owner->getName(), "Failed change data filtering mode for %s: mode=%d, size=%d!", _name, _filterMode, _filterSize);
      } else {
        rlog_e(_name, "Failed change data filtering mode for %s: mode=%d, size=%d!", _name, _filterMode, _filterSize);
      };
      return false;
    };
  };
  if (_owner) {
    rlog_i(_owner->getName(), "Changing data filtering mode for %s: mode=%d, size=%d", _name, _filterMode, _filterSize);
  } else {
    rlog_i(_name, "Changing data filtering mode for %s: mode=%d, size=%d", _name, _filterMode, _filterSize);
  };
  return true;
}

void rSensorItem::setOffsetValue(float offsetValue)
{
  if (offsetValue != _offsetValue) {
    _offsetValue = offsetValue;
    // Publish new values
    if (_prm_offsetValue) paramsValueStore(_prm_offsetValue, false);
  };
}

value_t rSensorItem::convertValue(const value_t rawValue)
{
  return rawValue;
}

sensor_status_t rSensorItem::getRawValue(value_t * rawValue)
{
  return SENSOR_STATUS_NOT_SUPPORTED;
}

void rSensorItem::setRawValue(const value_t rawValue, const time_t rawTime)
{
  _data.lastValue.timestamp = rawTime;
   _data.lastValue.rawValue = rawValue;
   _data.lastValue.filteredValue = getFilteredValue(convertValue(rawValue + _offsetValue));
 
  // Compare the day for the current value and the daily minimum
  struct tm _lastT, _dayT;
  localtime_r(&_data.lastValue.timestamp, &_lastT);
  localtime_r(&_data.extremumsDaily.minValue.timestamp, &_dayT);
  if (_lastT.tm_yday != _dayT.tm_yday) {
    // Reset daily minimum and maximum
    _data.extremumsDaily.minValue.timestamp = 0;
    _data.extremumsDaily.maxValue.timestamp = 0;
    // Reset weekly minimum and maximum
    if (_lastT.tm_wday == CONFIG_FORMAT_FIRST_DAY_OF_WEEK) {
      _data.extremumsWeekly.minValue.timestamp = 0;
      _data.extremumsWeekly.maxValue.timestamp = 0;
    };
  };

  // Determine the absolute minimum and maximum
  if ((_data.lastValue.filteredValue < _data.extremumsEntirely.minValue.filteredValue) || (_data.extremumsEntirely.minValue.timestamp == 0)) {
    _data.extremumsEntirely.minValue = _data.lastValue;
    #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    _data.extremumsEntirely.minValueChanged = true;
    #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  };
  if ((_data.lastValue.filteredValue > _data.extremumsEntirely.maxValue.filteredValue) || (_data.extremumsEntirely.maxValue.timestamp == 0)) {
    _data.extremumsEntirely.maxValue = _data.lastValue;
    #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    _data.extremumsEntirely.maxValueChanged = true;
    #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  };

  // Determine the weekly minimum and maximum
  if ((_data.lastValue.filteredValue < _data.extremumsWeekly.minValue.filteredValue) || (_data.extremumsWeekly.minValue.timestamp == 0)) {
    _data.extremumsWeekly.minValue = _data.lastValue;
    #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    _data.extremumsWeekly.minValueChanged = true;
    #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  };
  if ((_data.lastValue.filteredValue > _data.extremumsWeekly.maxValue.filteredValue) || (_data.extremumsWeekly.maxValue.timestamp == 0)) {
    _data.extremumsWeekly.maxValue = _data.lastValue;
    #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    _data.extremumsWeekly.maxValueChanged = true;
    #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  };

  // Determine the daily minimum and maximum
  if ((_data.lastValue.filteredValue < _data.extremumsDaily.minValue.filteredValue) || (_data.extremumsDaily.minValue.timestamp == 0)) {
    _data.extremumsDaily.minValue = _data.lastValue;
    #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    _data.extremumsDaily.minValueChanged = true;
    #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  };
  if ((_data.lastValue.filteredValue > _data.extremumsDaily.maxValue.filteredValue) || (_data.extremumsDaily.maxValue.timestamp == 0)) {
    _data.extremumsDaily.maxValue = _data.lastValue;
    #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    _data.extremumsDaily.maxValueChanged = true;
    #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  };
}

const char* rSensorItem::getName()
{
  return _name;
}

sensor_handle_t rSensorItem::getHandle()
{
  return &_data;
}

sensor_data_t rSensorItem::getValues()
{
  return _data;
}

sensor_value_t rSensorItem::getValue()
{
  return _data.lastValue;
}

sensor_extremums_t rSensorItem::getExtremumsEntirely()
{
  return _data.extremumsEntirely;
}

sensor_extremums_t rSensorItem::getExtremumsWeekly()
{
  return _data.extremumsWeekly;
}

sensor_extremums_t rSensorItem::getExtremumsDaily()
{
  return _data.extremumsDaily;
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------- Register internal parameters --------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rSensorItem::registerParameters(paramsGroupHandle_t parent_group, const char * key_name, const char * topic_name, const char * friendly_name)
{
  if ((key_name) && (topic_name) && (friendly_name)) {
    if (!_pgItem) {
      _pgItem = paramsRegisterGroup(parent_group, key_name, topic_name, friendly_name);
    };

    if (_pgItem) {
      registerItemParameters(_pgItem);
    } else {
      registerItemParameters(parent_group);
    };
  } else {
    registerItemParameters(parent_group);
  };
}

void rSensorItem::registerItemParameters(paramsGroup_t * group)
{
  _prm_offsetValue = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_OFFSET_KEY, CONFIG_SENSOR_PARAM_OFFSET_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_offsetValue);
  _prm_filterMode = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, _filterHandler, group,
    CONFIG_SENSOR_PARAM_FILTERMODE_KEY, CONFIG_SENSOR_PARAM_FILTERMODE_FRIENDLY,
    CONFIG_MQTT_PARAMS_QOS, &_filterMode);
  if (_prm_filterMode) {
    paramsSetLimitsU8(_prm_filterMode, SENSOR_FILTER_RAW, SENSOR_FILTER_MEDIAN);
  };
  _prm_filterSize = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U16, _filterHandler, group,
    CONFIG_SENSOR_PARAM_FILTERSIZE_KEY, CONFIG_SENSOR_PARAM_FILTERSIZE_FRIENDLY,
    CONFIG_MQTT_PARAMS_QOS, &_filterSize);
  if (_prm_filterSize) {
    paramsSetLimitsU16(_prm_filterSize, 0, CONFIG_SENSOR_PARAM_FILTERSIZE_MAX);
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Publishing values ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

char* rSensorItem::asString(const char* format, const value_t value)
{
  if (isnan(value)) {
    return malloc_stringf("\"%s\"", CONFIG_FORMAT_EMPTY);
  } else {
    return malloc_stringf(format, (float)value);
  };
}

char* rSensorItem::getStringRaw()
{
  return asString(_fmtString, _data.lastValue.rawValue);
}

char* rSensorItem::getStringFiltered()
{
  return asString(_fmtString, _data.lastValue.filteredValue);
}

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishDataValue(const char* topic, const char* format, const value_t value)
{
  bool ret = false;
  if (_owner) {
    // .../%topic%/numeric = 0.00
    char* _topicNum = mqttGetSubTopic(topic, CONFIG_SENSOR_NUMERIC_VALUE);
    ret = (_topicNum) && _owner->publish(_topicNum, asString(format, value), true);
    if (_topicNum) free(_topicNum);
    #if CONFIG_SENSOR_STRING_ENABLE
      // .../%topic%/string = "0.00°С"
      char* _topicStr = mqttGetSubTopic(topic, CONFIG_SENSOR_STRING_VALUE);
      ret = (_topicStr) && _owner->publish(_topicStr, asString(_fmtString, value), true);
      if (_topicStr) free(_topicStr);
    #endif // CONFIG_SENSOR_STRING_ENABLE
    return ret;
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonDataValue(bool brackets, const char* format, const value_t value)
{
  char* ret = nullptr;
  #if CONFIG_SENSOR_STRING_ENABLE
    // {"numeric":0.00,"string":"0.00°С"}
    char* _numeric = asString(format, value);
    if (_numeric) {
      char* _string = asString(_fmtString, value);
      if (_string) {
        if (brackets) {
          ret = malloc_stringf("{\"%s\":%s,\"%s\":\"%s\"}", CONFIG_SENSOR_NUMERIC_VALUE, _numeric, CONFIG_SENSOR_STRING_VALUE, _string);
        } else {
          ret = malloc_stringf("\"%s\":%s,\"%s\":\"%s\"", CONFIG_SENSOR_NUMERIC_VALUE, _numeric, CONFIG_SENSOR_STRING_VALUE, _string);
        };
        free(_string);
      };
      free(_numeric);
    };
  #else
    // 0.00
    ret = asString(format, value);
  #endif // CONFIG_SENSOR_STRING_ENABLE
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Publishing timestamp ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_TIMESTAMP_ENABLE

char* rSensorItem::asTimestamp(const sensor_value_t data)
{
  return malloc_timestr_empty(_fmtTimestamp, data.timestamp);
}

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishTimestamp(const char* topic, const sensor_value_t data)
{
  bool ret = false;
  if (_owner) {
    char* _topicValue = mqttGetSubTopic(topic, CONFIG_SENSOR_TIMESTAMP);
    if (_topicValue) {
      ret = _owner->publish(_topicValue, asTimestamp(data), true);
      free(_topicValue);
    };
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonTimestamp(const sensor_value_t data)
{
  char* ret = nullptr;
  char* _time = asTimestamp(data);
  if (_time) {
    ret = malloc_stringf("\"%s\":\"%s\"", CONFIG_SENSOR_TIMESTAMP, _time);
    free(_time);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

#endif // CONFIG_SENSOR_TIMESTAMP_ENABLE

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------- Publishing "timestring" (string value and timestamp) --------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_TIMESTRING_ENABLE

char* rSensorItem::asStringTimeValue(const sensor_value_t data)
{
  char* ret = nullptr;
  if (isnan(data.filteredValue)) {
    ret = malloc_stringf("%s", CONFIG_FORMAT_EMPTY);
  } else {
    char* _string = asString(_fmtString, data.filteredValue);
    if (_string) {
      char* _time = malloc_timestr_empty(_fmtTimestampValue, data.timestamp);
      if (_time) {
        ret = malloc_stringf(_fmtStringTimeValue, _string, _time);
        free(_time);
      };
      free(_string);
    };
  };
  return ret;
}

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishStringTimeValue(const char* topic, const sensor_value_t data)
{
  bool ret = false;
  if (_owner) {
    char* _topicValue = mqttGetSubTopic(topic, CONFIG_SENSOR_TIMESTRING_VALUE);
    if (_topicValue) {
      ret = _owner->publish(_topicValue, asStringTimeValue(data), true);
      free(_topicValue);
    };
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonStringTimeValue(const sensor_value_t data)
{
  char* ret = nullptr;
  char* _stv = asStringTimeValue(data);
  if (_stv) {
    ret = malloc_stringf("\"%s\":\"%s\"", CONFIG_SENSOR_TIMESTRING_VALUE, _stv);
    free(_stv);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

#endif // CONFIG_SENSOR_TIMESTRING_ENABLE

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------- Publishing raw and filtered values ------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishValue(const char* topic, const sensor_value_t data)
{
  bool ret = false;
  // filtered value
  char* _topicFiltered = mqttGetSubTopic(topic, CONFIG_SENSOR_FILTERED_VALUE);
  if (_topicFiltered) {
    ret = publishDataValue(_topicFiltered, _fmtNumeric, data.filteredValue);
    free(_topicFiltered);
    // raw value
    if (ret) {
      #if (CONFIG_SENSOR_RAW_ENABLE == 1)
        // raw value - always
        char* _topicRaw = mqttGetSubTopic(topic, CONFIG_SENSOR_RAW_VALUE);
        ret = (_topicRaw) && publishDataValue(_topicRaw, "%f", data.rawValue);
        if (_topicRaw) free(_topicRaw);
      #elif (CONFIG_SENSOR_RAW_ENABLE == 2)
        // raw value - only when there is filtration
        if (_forcedRawPublish || (_filterMode != SENSOR_FILTER_RAW) || (_offsetValue != 0.0)) {
          char* _topicRaw = mqttGetSubTopic(topic, CONFIG_SENSOR_RAW_VALUE);
          ret = (_topicRaw) && publishDataValue(_topicRaw, "%f", data.rawValue);
          if (_topicRaw) free(_topicRaw);
        };
      #endif // CONFIG_SENSOR_RAW_ENABLE
    };
  };
  return ret;  
}
#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonValue(const sensor_value_t data)
{
  char* ret = nullptr;
  #if (CONFIG_SENSOR_RAW_ENABLE == 1)
    // {"value":{...},"raw":{...}} - always
    char* _json_raw = jsonDataValue(true, "%f", data.rawValue);
    if (_json_raw) {
      char* _json_flt = jsonDataValue(true, _fmtNumeric, data.filteredValue);
      if (_json_flt) {
        ret = malloc_stringf("\"%s\":%s,\"%s\":%s", CONFIG_SENSOR_FILTERED_VALUE, _json_flt, CONFIG_SENSOR_RAW_VALUE, _json_raw);
        free(_json_flt);
      };
      free(_json_raw);
    };
  #elif (CONFIG_SENSOR_RAW_ENABLE == 2)
    //  {"value":{...},"raw":{...}} - only when there is filtration
    if (_forcedRawPublish || (_filterMode != SENSOR_FILTER_RAW) || (_offsetValue != 0.0)) {
      char* _json_raw = jsonDataValue(true, "%f", data.rawValue);
      if (_json_raw) {
        char* _json_flt = jsonDataValue(true, _fmtNumeric, data.filteredValue);
        if (_json_flt) {
          ret = malloc_stringf("\"%s\":%s,\"%s\":%s", CONFIG_SENSOR_FILTERED_VALUE, _json_flt, CONFIG_SENSOR_RAW_VALUE, _json_raw);
          free(_json_flt);
        };
        free(_json_raw);
      };
    } else {
      // ...
      ret = jsonDataValue(false, _fmtNumeric, data.filteredValue);
    };
  #else
    // ...
    ret = jsonDataValue(false, _fmtNumeric, data.filteredValue);
  #endif // CONFIG_SENSOR_RAW_ENABLE
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------- Publishing part of sensor data ------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishPartSensorValue(const char* topic, const char* type, const sensor_value_t data)
{
  bool ret = false;
  char* _topicData = mqttGetSubTopic(topic, type);
  if (_topicData) {
    ret = publishValue(_topicData, data);
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      if (ret) 
        ret = publishTimestamp(_topicData, data);
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE
      if (ret) 
        ret = publishStringTimeValue(_topicData, data);
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
    free(_topicData);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonPartSensorValue(const char* type, const sensor_value_t data)
{
  char* ret = nullptr;
  char* _json_value = jsonValue(data);
  if (_json_value) {
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      char* _json_time = jsonTimestamp(data);
      if (_json_time) {
        #if CONFIG_SENSOR_TIMESTRING_ENABLE
          // "{"type":{...},"time":"12:45:38 01.02.2021","tsv":"0.00°С 12:45 01.02"}
          char* _json_stv = jsonStringTimeValue(data);
          if (_json_stv) {
            ret = malloc_stringf("\"%s\":{%s,%s,%s}", type, _json_value, _json_time, _json_stv);
            free(_json_stv);
          };
        #else
          // {"type":{...},"time":"12:45:38 01.02.2021"}
          ret = malloc_stringf("\"%s\":{%s,%s}", type, _json_value, _json_time);
        #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
        free(_json_time);
      };
    #else 
      #if CONFIG_SENSOR_TIMESTRING_ENABLE
        // {"type":{...},"tsv":"0.00°С 12:45 01.02"}
        char* _json_stv = jsonStringTimeValue(data);
        if (_json_stv) {
          ret = malloc_stringf("\"%s\":{%s,%s}", type, _json_value, _json_stv);
          free(_json_stv);
        };
      #else
        // "type":{...}
        ret = malloc_stringf("\"%s\":%s", type, _json_value);
      #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    free(_json_value);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Publishing extremes -----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishExtremums(const char* topic, const sensor_extremums_t range
  #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  , const bool minValueChanged, const bool maxValueChanged
  #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
)
{
  #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    return (!minValueChanged || publishPartSensorValue(topic, CONFIG_SENSOR_MINIMAL, range.minValue))
        && (!maxValueChanged || publishPartSensorValue(topic, CONFIG_SENSOR_MAXIMAL, range.maxValue));
  #else
    return publishPartSensorValue(topic, CONFIG_SENSOR_MINIMAL, range.minValue) 
        && publishPartSensorValue(topic, CONFIG_SENSOR_MAXIMAL, range.maxValue);
  #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonExtremums(const char* type, const sensor_extremums_t range)
{
  char* ret = nullptr;
  // "type":{"min":{...},"max":{...}}
  char* _min_value = jsonPartSensorValue(CONFIG_SENSOR_MINIMAL, range.minValue);
  if (_min_value) {
    char* _max_value = jsonPartSensorValue(CONFIG_SENSOR_MAXIMAL, range.maxValue);
    if (_max_value) {
      ret = malloc_stringf("\"%s\":{%s,%s}", type, _min_value, _max_value);
      free(_max_value);
    };
    free(_min_value);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------ Publishing latest values and extremes --------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishValues(const char* topic)
{
  bool ret = publishPartSensorValue(topic, CONFIG_SENSOR_LASTVALUE, _data.lastValue);
  #if CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE || CONFIG_SENSOR_EXTREMUMS_ENTIRELY_ENABLE
    char* _topicExtremums = mqttGetSubTopic(topic, CONFIG_SENSOR_EXTREMUS);
    if (_topicExtremums) {
      #if CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
        if (ret) {
          char* _topicExtremumsDaily = mqttGetSubTopic(_topicExtremums, CONFIG_SENSOR_EXTREMUMS_DAILY);
          ret = (_topicExtremumsDaily) && publishExtremums(_topicExtremumsDaily, _data.extremumsDaily
            #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
            , _data.extremumsDaily.minValueChanged, _data.extremumsDaily.maxValueChanged
            #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
            );
          #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
          if (ret && _data.extremumsDaily.minValueChanged) _data.extremumsDaily.minValueChanged = false;
          if (ret && _data.extremumsDaily.maxValueChanged) _data.extremumsDaily.maxValueChanged = false;
          #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
          if (_topicExtremumsDaily) free(_topicExtremumsDaily);
        };
      #endif // CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
      
      #if CONFIG_SENSOR_EXTREMUMS_WEEKLY_ENABLE
        if (ret) {
          char* _topicExtremumsWeekly = mqttGetSubTopic(_topicExtremums, CONFIG_SENSOR_EXTREMUMS_WEEKLY);
          ret = (_topicExtremumsWeekly) && publishExtremums(_topicExtremumsWeekly, _data.extremumsWeekly
            #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
            , _data.extremumsWeekly.minValueChanged, _data.extremumsWeekly.maxValueChanged
            #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
            );
          #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
          if (ret && _data.extremumsWeekly.minValueChanged) _data.extremumsWeekly.minValueChanged = false;
          if (ret && _data.extremumsWeekly.maxValueChanged) _data.extremumsWeekly.maxValueChanged = false;
          #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
          if (_topicExtremumsWeekly) free(_topicExtremumsWeekly);
        };
      #endif // CONFIG_SENSOR_EXTREMUMS_WEEKLY_ENABLE

      #if CONFIG_SENSOR_EXTREMUMS_ENTIRELY_ENABLE
        if (ret) {
          char* _topicExtremumsEntirely = mqttGetSubTopic(_topicExtremums, CONFIG_SENSOR_EXTREMUMS_ENTIRELY);
          ret = (_topicExtremumsEntirely) && publishExtremums(_topicExtremumsEntirely, _data.extremumsEntirely
            #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
            , _data.extremumsEntirely.minValueChanged, _data.extremumsEntirely.maxValueChanged
            #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
            );
          #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
          if (ret && _data.extremumsEntirely.minValueChanged) _data.extremumsEntirely.minValueChanged = false;
          if (ret && _data.extremumsEntirely.maxValueChanged) _data.extremumsEntirely.maxValueChanged = false;
          #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
          if (_topicExtremumsEntirely) free (_topicExtremumsEntirely);
        };
      #endif // CONFIG_SENSOR_EXTREMUMS_ENTIRELY_ENABLE
      free(_topicExtremums);
    };
  #endif // CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE || CONFIG_SENSOR_EXTREMUMS_ENTIRELY_ENABLE
  return ret;
}

bool rSensorItem::publishNamedValues()
{
  return publishValues(_name);
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonValues()
{
  char* ret = nullptr;
  // {"value":{...},"extremums":{???}}
  char* _last = jsonPartSensorValue(CONFIG_SENSOR_LASTVALUE, _data.lastValue);
  if (_last) {
    #if CONFIG_SENSOR_EXTREMUMS_ENTIRELY_ENABLE
      char* _bnde = jsonExtremums(CONFIG_SENSOR_EXTREMUMS_ENTIRELY, _data.extremumsEntirely);
      if (_bnde) {
        #if CONFIG_SENSOR_EXTREMUMS_WEEKLY_ENABLE
          char* _bndw = jsonExtremums(CONFIG_SENSOR_EXTREMUMS_WEEKLY, _data.extremumsWeekly);
          if (_bndw) {
            #if CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
              // {"value":{...},"extremums":{"daily":{...},"weekly":{...},"entirely":{...}}}
              char* _bndd = jsonExtremums(CONFIG_SENSOR_EXTREMUMS_DAILY, _data.extremumsDaily);
              if (_bndd) {
                ret = malloc_stringf("{%s,\"%s\":{%s,%s,%s}}", _last, CONFIG_SENSOR_EXTREMUS, _bndd, _bndw, _bnde);
                free(_bndd);
              };
            #else
              // {"value":{...},"extremums":{"weekly":{...},"entirely":{...}}}
              ret = malloc_stringf("{%s,\"%s\":{%s,%s}}", _last, CONFIG_SENSOR_EXTREMUS, _bndw, _bnde);
            #endif // CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
            free(_bndw);
          };
        #else
          #if CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
            // {"value":{...},"extremums":{"daily":{...},"entirely":{...}}}
            char* _bndd = jsonExtremums(CONFIG_SENSOR_EXTREMUMS_DAILY, _data.extremumsDaily);
            if (_bndd) {
              ret = malloc_stringf("{%s,\"%s\":{%s,%s}}", _last, CONFIG_SENSOR_EXTREMUS, _bndd, _bnde);
              free(_bndd);
            };
          #else
            // {"value":{...},"extremums":{"entirely":{...}}}
            ret = malloc_stringf("{%s,\"%s\":{%s}}", _last, CONFIG_SENSOR_EXTREMUS, _bnde);
          #endif // CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
        #endif // CONFIG_SENSOR_EXTREMUMS_WEEKLY_ENABLE
        free(_bnde);
      };
    #else
      #if CONFIG_SENSOR_EXTREMUMS_WEEKLY_ENABLE
        char* _bndw = jsonExtremums(CONFIG_SENSOR_EXTREMUMS_WEEKLY, _data.extremumsWeekly);
        if (_bndw) {
          #if CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
            // {"value":{...},"extremums":{"daily":{...},"weekly":{...}}}
            char* _bndd = jsonExtremums(CONFIG_SENSOR_EXTREMUMS_DAILY, _data.extremumsDaily);
            if (_bndd) {
              ret = malloc_stringf("{%s,\"%s\":{%s,%s}}", _last, CONFIG_SENSOR_EXTREMUS, _bndd, _bndw);
              free(_bndd);
            };
          #else
            // {"value":{...},"extremums":{"weekly":{...}}}
            ret = malloc_stringf("{%s,\"%s\":{%s}}", _last, CONFIG_SENSOR_EXTREMUS, _bndw);
          #endif // CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
          free(_bndw);
        };
      #else
        #if CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
          // {"value":{...},"extremums":{"daily":{...}}}
          char* _bndd = jsonExtremums(CONFIG_SENSOR_EXTREMUMS_DAILY, _data.extremumsDaily);
          if (_bndd) {
            ret = malloc_stringf("{%s,\"%s\":{%s}}", _last, CONFIG_SENSOR_EXTREMUS, _bndd);
            free(_bndd);
          };
        #else
          // {{...}}
          ret = malloc_stringf("{%s}", _last);
        #endif // CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
      #endif // CONFIG_SENSOR_EXTREMUMS_WEEKLY_ENABLE
    #endif // CONFIG_SENSOR_EXTREMUMS_ENTIRELY_ENABLE
    free(_last);
  };
  return ret;
}

char* rSensorItem::jsonNamedValues()
{
  char* ret = nullptr;
  char* _values = jsonValues();
  if (_values) {
    ret = malloc_stringf("\"%s\":%s", _name, _values);
    free(_values);
  };
  return ret;
}
#endif // CONFIG_SENSOR_AS_JSON

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------- Filters ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

value_t rSensorItem::getAverageValue(const value_t rawValue)
{
  if (_filterSize > 0) {
    if (!_filterInit) {
      _filterInit = true;
      for (uint16_t i = 0; i < _filterSize; i++) 
        _filterBuf[i] = rawValue;
    };

    _filterBuf[_filterIndex] = rawValue;
    if (++_filterIndex >= _filterSize) _filterIndex = 0;
    
    value_t averageSummary = 0;
    for (uint16_t i = 0; i < _filterSize; i++)
      averageSummary += _filterBuf[i];
    return averageSummary / _filterSize; 
  }
  else return rawValue;
}

value_t rSensorItem::getMedianValue(const value_t rawValue)
{
  if (_filterSize < 3) {
    return rawValue;
  }
  else {
    if (!_filterInit) {
      _filterInit = true;
      for (uint16_t i = 0; i < _filterSize; i++) 
        _filterBuf[i] = rawValue;
    };

    _filterBuf[_filterIndex] = rawValue;

    if ((_filterIndex < _filterSize - 1) && (_filterBuf[_filterIndex] > _filterBuf[_filterIndex + 1])) {
      for (int i = _filterIndex; i < _filterSize - 1; i++) {
        if (_filterBuf[i] > _filterBuf[i + 1]) {
          value_t buff = _filterBuf[i];
          _filterBuf[i] = _filterBuf[i + 1];
          _filterBuf[i + 1] = buff;
        };
      };
    } else {
      if ((_filterIndex > 0) && (_filterBuf[_filterIndex - 1] > _filterBuf[_filterIndex])) {
        for (int i = _filterIndex; i > 0; i--) {
          if (_filterBuf[i] < _filterBuf[i - 1]) {
            value_t buff = _filterBuf[i];
            _filterBuf[i] = _filterBuf[i - 1];
            _filterBuf[i - 1] = buff;
          };
        };
      };
    };
		if (++_filterIndex >= _filterSize) _filterIndex = 0;
    return _filterBuf[_filterSize / 2];
  };
}

value_t rSensorItem::getFilteredValue(const value_t rawValue)
{
  switch (_filterMode) {
    case SENSOR_FILTER_AVERAGE:
      return getAverageValue(rawValue);
      break;

    case SENSOR_FILTER_MEDIAN:
      return getMedianValue(rawValue);
      break;

    default:
      return rawValue;
      break;
  };
}

// =======================================================================================================================
// =======================================================================================================================
// =================================================== rTemperatureItem ==================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rTemperatureItem::rTemperatureItem(rSensor *sensor, const char* itemName, const unit_temperature_t unitValue,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString 
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , const char* formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , const char* formatTimestampValue, const char* formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
// inherited constructor
):rSensorItem(sensor, itemName, filterMode, filterSize, formatNumeric, formatString
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , formatTimestampValue, formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
) {
  _units = unitValue; 
  _forcedRawPublish = true;
};

value_t rTemperatureItem::convertValue(const value_t rawValue)
{
  if (isnan(rawValue)) { return rawValue; };
  switch (_units) {
    case UNIT_TEMP_FAHRENHEIT:
      return 32.0 + 1.8 * rawValue;
    default:
      return rawValue;
  };
}

// =======================================================================================================================
// =======================================================================================================================
// ==================================================== rPressureItem ====================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rPressureItem::rPressureItem(rSensor *sensor, const char* itemName, const unit_pressure_t unitValue,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString 
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , const char* formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , const char* formatTimestampValue, const char* formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
// inherited constructor
):rSensorItem(sensor, itemName, filterMode, filterSize, formatNumeric, formatString
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , formatTimestampValue, formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
) {
  _units = unitValue;
  _forcedRawPublish = true; 
};

value_t rPressureItem::convertValue(const value_t rawValue)
{
  if (isnan(rawValue)) { return rawValue; };
  switch (_units) {
    case UNIT_PRESSURE_HPA:
      return rawValue / 100.0;
    case UNIT_PRESSURE_MMHG:
      return rawValue / 133.3224;
    default:
      return rawValue;
  };
}

// =======================================================================================================================
// =======================================================================================================================
// ======================================================= rSensor =======================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensor::rSensor()
{
  _name = nullptr;
  _topicName = nullptr;
  _topicPub =  nullptr;
  _eventId = 0;
  _readInterval = 0;
  _readLast = 0;
  _errLimit = 0;
  _errCount = 0;
  _lastStatus = SENSOR_STATUS_OK;
  _cbOnChangeStatus = nullptr;
  _cbOnPublishData = nullptr;
}

// Destructor
rSensor::~rSensor()
{
  topicsFree();
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Initialization ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rSensor::initProperties(const char* sensorName, const char* topicName, const bool topicLocal, 
  const uint32_t minReadInterval, const uint16_t errorLimit, 
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _name = sensorName;
  _topicName = topicName;
  _topicLocal = topicLocal;
  _topicPub = nullptr;
  _readInterval = minReadInterval;
  _readLast = 0;
  _errLimit = errorLimit;
  _errCount = 0;
  _pgSensor = nullptr;
  _lastStatus = SENSOR_STATUS_NAN;
  _cbOnChangeStatus = cb_status;
  _cbOnPublishData = cb_publish;
}

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------- Properties ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// Sensor name
const char* rSensor::getName()
{
  return _name;
}

// Mqtt topic
void rSensor::topicsCreate(bool topicPrimary)
{
  if (_topicPub) free(_topicPub);
  _topicPub =  mqttGetTopicDevice1(topicPrimary, _topicLocal, _topicName);
  if (_topicPub ) {
    rlog_i(logTAG, "Generated topic for sensor \"%s\": [ %s ]", _name, _topicPub);
  } else {
    rlog_e(logTAG, "Failed to generate topic for sensor \"%s\"", _name);
  };
}

void rSensor::topicsFree()
{
  if (_topicPub) free(_topicPub);
  _topicPub = nullptr;
  rlog_d(logTAG, "Topic for sensor \"%s\" has been scrapped", _name);
}

char* rSensor::getTopicPub()
{
  return _topicPub;
}

void rSensor::setEventId(uint8_t eventId)
{
  if ((eventId != _eventId) && (eventId < 8)) {
    _eventId = eventId;
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------- Register internal parameters --------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rSensor::registerParameters(paramsGroupHandle_t parent_group, const char * key_name, const char * topic_name, const char * friendly_name)
{
  if (!_pgSensor) {
    _pgSensor = paramsRegisterGroup(parent_group, key_name, topic_name, friendly_name);
  };

  if (_pgSensor) {
    registerCustomParameters(_pgSensor);
    registerItemsParameters(_pgSensor);
  };
}

void rSensor::registerCustomParameters(paramsGroupHandle_t sensor_group)
{
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Sensor status ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rSensor::setCallbackOnChangeStatus(cb_status_changed_t cb)
{
  _cbOnChangeStatus = cb;
}

void rSensor::postEventStatus(const sensor_status_t oldStatus, const sensor_status_t newStatus)
{
  sensor_event_status_t data;
  data.sensor = (void*)this;
  data.sensor_id = _eventId;
  data.old_status = (uint8_t)oldStatus;
  data.new_status = (uint8_t)newStatus;
  eventLoopPost(RE_SENSOR_EVENTS, RE_SENSOR_STATUS_CHANGED, &data, sizeof(data), portMAX_DELAY);
  // rlog_w(_name, "Post event RE_SENSOR_EVENTS :: RE_SENSOR_STATUS_CHANGED for %s (%d). Old status: %d, new status: %d", _name, data.sensor_id, data.old_status, data.new_status);
}

void rSensor::setRawStatus(sensor_status_t newStatus, bool forced)
{
  sensor_status_t prevStatus = _lastStatus;
  if (_lastStatus != newStatus) {
    if (newStatus == SENSOR_STATUS_OK) {
      // Do not send a notification about the status change when the device starts
      bool isFirstOk = (_lastStatus == SENSOR_STATUS_NAN) && (_errCount == 0);
      // Save new status immediately
      _lastStatus = newStatus;
      _errCount = 0;
      if (!isFirstOk) {
        if (_cbOnChangeStatus) {
          _cbOnChangeStatus(this, prevStatus, newStatus);
        };
        postEventStatus(prevStatus, newStatus);
      };
      this->publishData(false);
    } else {
      // Save the new status only if the number of errors exceeds the threshold
      _errCount++;
      if (forced || (_errCount > _errLimit)) {
        _lastStatus = newStatus;
        if (_cbOnChangeStatus) {
          _cbOnChangeStatus(this, _lastStatus, newStatus);
        };
        postEventStatus(prevStatus, newStatus);
        this->publishData(false);
      };
    };
  } else {
    // Count error anyway
    if (newStatus == SENSOR_STATUS_OK) {
      if (_errCount > 0) {
        _errCount = 0;
      };
    } else {
      _errCount++;
    }
  };
}

sensor_status_t rSensor::convertEspError(const uint32_t error)
{
  switch (error) {
    case ESP_OK:              
      return SENSOR_STATUS_OK;
    case ESP_ERR_TIMEOUT:     
      return SENSOR_STATUS_TIMEOUT;
    case ESP_ERR_INVALID_CRC: 
      return SENSOR_STATUS_CRC_ERROR;
    default:                  
      return SENSOR_STATUS_ERROR;
  };
}

sensor_status_t rSensor::setEspError(uint32_t error, bool forced)
{
  sensor_status_t ret = convertEspError(error);
  setRawStatus(ret, forced);
  return ret;
}

sensor_status_t rSensor::getStatus()
{
  return _lastStatus;
}

const char* rSensor::getStatusString()
{
  switch (_lastStatus) {
    case SENSOR_STATUS_NAN:
      return CONFIG_SENSOR_STATUS_NAN;
    case SENSOR_STATUS_OK:
      return CONFIG_SENSOR_STATUS_OK;
    case SENSOR_STATUS_TIMEOUT: 
      return CONFIG_SENSOR_STATUS_TIMEOUT;
    case SENSOR_STATUS_CAL_ERROR: 
      return CONFIG_SENSOR_STATUS_CALIBRATION;
    case SENSOR_STATUS_CRC_ERROR: 
      return CONFIG_SENSOR_STATUS_CRC_ERROR;
    case SENSOR_STATUS_ERROR: 
      return CONFIG_SENSOR_STATUS_ERROR;
    default:
      return CONFIG_SENSOR_STATUS_UNKNOWN;
  }
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------- Displaying multiple values in one topic ---------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* rSensor::getDisplayValueStatus()
{
  #if CONFIG_SENSOR_STATUS_AS_MIXED_ON_ERROR
    if (_lastStatus == SENSOR_STATUS_OK) {
      return getDisplayValue();  
    } else {
      return malloc_string(getStatusString());
    };
  #else
    return getDisplayValue();
  #endif // CONFIG_SENSOR_STATUS_AS_MIXED_ON_ERROR
}
#endif // CONFIG_SENSOR_DISPLAY_ENABLED

// Read data from sensor
sensor_status_t rSensor::readData()
{
  if (millis() >= (_readLast + _readInterval)) {
    _readLast = millis();
    rlog_v(logTAG, "Read data from [ %s ]...", _name);
    return readRawData();
  };
  return _lastStatus;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Publishing sensor data -----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rSensor::setCallbackOnPublishData(cb_publish_data_t cb)
{
  _cbOnPublishData = cb;
}

bool rSensor::publish(char* topic, char* payload, const bool free_payload)
{
  bool ret = false;
  if ((_cbOnPublishData) && (topic) && (payload)) {
    char * _topicPub = mqttGetSubTopic(getTopicPub(), topic);
    if (_topicPub) {
      ret = _cbOnPublishData(this, _topicPub, payload, true, free_payload);
    };
  };
  return ret;
}

bool rSensor::publishData(const bool readSensor)
{
  if (_cbOnPublishData) {
    bool ret = true;
    if (readSensor) readData();

    #if CONFIG_SENSOR_AS_PLAIN
      // Publishing sensor status
      if (ret) 
        ret = publish((char*)CONFIG_SENSOR_STATUS, (char*)getStatusString(), false);
      // Publishing sensor items data
      if (ret) 
        ret = publishItems();
      // Publishing display (mixed) value
      #if CONFIG_SENSOR_DISPLAY_ENABLED
      if (ret && (_displayMode != SENSOR_MIXED_NONE)) 
        ret = publish((char*)CONFIG_SENSOR_DISPLAY, getDisplayValueStatus(), true);
      #endif // CONFIG_SENSOR_DISPLAY_ENABLED
      // Publishing custom value
      if (ret) 
        ret = publishCustomValues();
    #endif // CONFIG_SENSOR_AS_PLAIN

    #if CONFIG_SENSOR_AS_JSON
      if (ret) 
        ret = _cbOnPublishData(this, getTopicPub(), getJSON(), false, true);
    #endif // CONFIG_SENSOR_AS_JSON
    
    // esp_task_wdt_reset();
    // taskYIELD();
    vTaskDelay(1);

    return ret;
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- Publishing data in plain text ------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_AS_PLAIN

// Publishing additional data (for example, calculated data) in descendant classes
bool rSensor::publishCustomValues()
{
  return true;
}

bool rSensor::publishItems()
{
  return true;
}

#endif // CONFIG_SENSOR_AS_PLAIN

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------- Publishing data in JSON -----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_AS_JSON

// Mixing display value and additional data (from descendants of the class)
char* rSensor::jsonCustomValues()
{
  return nullptr;
}

char* rSensor::jsonDisplayAndCustomValues()
{
  char* ret = nullptr;
  #if CONFIG_SENSOR_DISPLAY_ENABLED
    char* _custom = jsonCustomValues();
    char* _display = getDisplayValueStatus();
    if (_custom) {
      if (_display) {
        // Both lines
        ret = malloc_stringf("\"%s\":\"%s\",%s", CONFIG_SENSOR_DISPLAY, _display, _custom);
      } else {
        // Only extra
        ret = malloc_stringf("%s", _custom);
      };
    } else {
      if (_display) {
        // Only display
        ret = malloc_stringf("\"%s\":\"%s\"", CONFIG_SENSOR_DISPLAY, _display);
      };
    };
    if (_display) free(_display);
    if (_custom) free(_custom);
  #else
    ret = jsonCustomValues();
  #endif // CONFIG_SENSOR_DISPLAY_ENABLED
  return ret;
}

#endif //CONFIG_SENSOR_AS_JSON

// =======================================================================================================================
// =======================================================================================================================
// ====================================================== rSensorX1 ======================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensorX1::rSensorX1():rSensor()
{ 
  _item = nullptr;
}

// Destructor
rSensorX1::~rSensorX1()
{
  // We always delete an item, even if it is attached from the outside
  if (_item) delete _item;
}

// Set external items
void rSensorX1::setSensorItems(rSensorItem* item)
{
  _item = item;
  if (_item) {
    _item->setOwner(this);
    _item->initItem();
  };
};

// Initialization of internal items
bool rSensorX1::initSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  // Create items by default if they were not assigned externally
  if (!_item) {
    createSensorItems(filterMode, filterSize);
    if (_item) {
      _item->setOwner(this);
      _item->initItem();
    };
  };
  return true;
}

// Set filter mode
bool rSensorX1::setFilterMode(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item) return _item->setFilterMode(filterMode, filterSize);
  return false;
}

// Writing measured RAW values to internal items
void rSensorX1::setRawValues(const value_t newValue)
{
  if (isnan(newValue)) {
    setRawStatus(SENSOR_STATUS_NAN, false);
  } else {
    time_t timestamp = time(nullptr);
    if (_item) _item->setRawValue(newValue, timestamp);
    setRawStatus(SENSOR_STATUS_OK, true);
  };
}

// Get a pointer to storage
rSensorItem* rSensorX1::getSensorItem()
{
  return _item;
}

sensor_handle_t rSensorX1::getHandle()
{
  if (_item) return _item->getHandle();
  return nullptr;
}

// Get data from storage
sensor_data_t rSensorX1::getValues(const bool readSensor)
{
  if (readSensor) readData();
  if (_item) return _item->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX1::getValue(const bool readSensor)
{
  if (readSensor) readData();
  if (_item) return _item->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX1::getExtremumsEntirely(const bool readSensor)
{
  if (readSensor) readData();
  if (_item) return _item->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX1::getExtremumsWeekly(const bool readSensor)
{
  if (readSensor) readData();
  if (_item) return _item->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX1::getExtremumsDaily(const bool readSensor)
{
  if (readSensor) readData();
  if (_item) return _item->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* rSensorX1::getDisplayValue()
{
  if (_item) {
    return  _item->asStringTimeValue(_item->getValue());
  };
  return nullptr;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorX1::publishItems()
{
  return _item->publishNamedValues();
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorX1::getJSON()
{
  char* ret = nullptr;
  char* _json_values = nullptr;
  if (_item) { _json_values = _item->jsonNamedValues(); };
  // Add mixed content line
  _json_values = concat_strings_div(_json_values, jsonDisplayAndCustomValues(), ",");
  // Generating full JSON
  if (_json_values) {
    #if CONFIG_SENSOR_STATUS_ENABLE
      ret = malloc_stringf("{\"%s\":\"%s\",%s}", CONFIG_SENSOR_STATUS, getStatusString(), _json_values);
    #else
      ret = malloc_stringf("{%s}", _json_values);
    #endif //CONFIG_SENSOR_STATUS_ENABLE
    free(_json_values);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// =======================================================================================================================
// =======================================================================================================================
// ===================================================== rSensorStub =====================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensorStub::rSensorStub():rSensorX1()
{ 
}

// Connecting external previously created items, for example statically declared
bool rSensorStub::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  rSensorItem* item, const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  this->rSensorX1::setSensorItems(item);
  return (_item != nullptr);
}

// Initialization of internal items
void rSensorStub::createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  rlog_e(logTAG, "Only external items can be used for [ %s ] sensor!", _name);
  setRawStatus(SENSOR_STATUS_NOT_SUPPORTED, true);
}

// Registration of parameters
void rSensorStub::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  if (_item) {
    _item->registerParameters(parent_group, nullptr, nullptr, nullptr);
  };
}

sensor_status_t rSensorStub::readRawData()
{
  sensor_status_t status = SENSOR_STATUS_NOT_SUPPORTED;
  if (_item) {
    value_t rawValue;
    status = _item->getRawValue(&rawValue);
    if (status == SENSOR_STATUS_OK) {
      setRawValues(rawValue);
    };
  };
  setRawStatus(status, false);
  return status;
}

// =======================================================================================================================
// =======================================================================================================================
// ====================================================== rSensorX2 ======================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensorX2::rSensorX2():rSensor()
{ 
  _item1 = nullptr;
  _item2 = nullptr;
}

// Destructor
rSensorX2::~rSensorX2()
{
  // We always delete an item, even if it is attached from the outside
  if (_item1) delete _item1;
  if (_item2) delete _item2;
}

// Set external items
void rSensorX2::setSensorItems(rSensorItem* item1, rSensorItem* item2)
{
  _item1 = item1;
  _item2 = item2;
  if (_item1) {
    _item1->setOwner(this);
    _item1->initItem();
  };
  if (_item2) {
    _item2->setOwner(this);
    _item2->initItem();
  };
};

// Initialization of internal items
bool rSensorX2::initSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                const sensor_filter_t filterMode2, const uint16_t filterSize2)
{
  // Create items by default if they were not assigned externally
  if ((!_item1) || (!_item2)) {
    createSensorItems(filterMode1, filterSize1, filterMode2, filterSize2);
    if (_item1) {
      _item1->setOwner(this);
      _item1->initItem();
    };
    if (_item2) {
      _item2->setOwner(this);
      _item2->initItem();
    };
  };
  return true;
}

// Change filter mode
bool rSensorX2::setFilterMode1(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item1) return _item1->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX2::setFilterMode2(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item2) return _item2->setFilterMode(filterMode, filterSize);
  return false;
}

// Writing measured RAW values to internal items
void rSensorX2::setRawValues(const value_t newValue1, const value_t newValue2)
{
  if ((isnan(newValue1) && (_item1)) 
   || (isnan(newValue2) && (_item2))) {
    setRawStatus(SENSOR_STATUS_NAN, true);
  } else {
    time_t timestamp = time(nullptr);
    if (_item1) _item1->setRawValue(newValue1, timestamp);
    if (_item2) _item2->setRawValue(newValue2, timestamp);
    setRawStatus(SENSOR_STATUS_OK, true);
  };
}

void rSensorX2::setRawValue1(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item1) _item1->setRawValue(newValue, timestamp);
}

void rSensorX2::setRawValue2(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item2) _item2->setRawValue(newValue, timestamp);
}

// Get a pointer to storage
rSensorItem* rSensorX2::getSensorItem1()
{
  return _item1;
}

rSensorItem* rSensorX2::getSensorItem2()
{
  return _item2;
}

sensor_handle_t rSensorX2::getHandle1()
{
  if (_item1) return _item1->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX2::getHandle2()
{
  if (_item2) return _item2->getHandle();
  return nullptr;
}

// Get data from storage
sensor_data_t rSensorX2::getValues1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX2::getValues2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX2::getValue1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX2::getValue2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX2::getExtremumsEntirely1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX2::getExtremumsEntirely2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX2::getExtremumsWeekly1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX2::getExtremumsWeekly2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX2::getExtremumsDaily1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX2::getExtremumsDaily2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

// Displaying multiple values in one topic
#if CONFIG_SENSOR_DISPLAY_ENABLED

char* rSensorX2::getDisplayValue()
{
  char* ret = nullptr;
  if (_item1) { 
    ret = _item1->getStringFiltered(); 
  };
  if (_item2) {
    ret = concat_strings_div(ret, _item2->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorX2::publishItems()
{
  return _item1->publishNamedValues() && _item2->publishNamedValues();
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorX2::getJSON()
{
  char* ret = nullptr;
  char* _json_values = nullptr;
  if (_item1) { _json_values = _item1->jsonNamedValues(); };
  if (_item2) { _json_values = concat_strings_div(_json_values, _item2->jsonNamedValues(), ","); };
  // Add mixed content line
  _json_values = concat_strings_div(_json_values, jsonDisplayAndCustomValues(), ",");
  // Generating full JSON
  if (_json_values) {
    #if CONFIG_SENSOR_STATUS_ENABLE
      ret = malloc_stringf("{\"%s\":\"%s\",%s}", CONFIG_SENSOR_STATUS, getStatusString(), _json_values);
    #else
      ret = malloc_stringf("{%s}", _json_values);
    #endif //CONFIG_SENSOR_STATUS_ENABLE
    free(_json_values);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// =======================================================================================================================
// =======================================================================================================================
// ====================================================== rSensorHT ======================================================
// =======================================================================================================================
// =======================================================================================================================

rSensorHT::rSensorHT():rSensorX2() 
{
}

// Initialization of internal items
void rSensorHT::createSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                  const sensor_filter_t filterMode2, const uint16_t filterSize2)
{
  // Humidity
  _item1 = new rSensorItem(this, CONFIG_SENSOR_HUMIDITY_NAME, 
    filterMode1, filterSize1,
    CONFIG_FORMAT_HUMIDITY_VALUE, CONFIG_FORMAT_HUMIDITY_STRING
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    , CONFIG_FORMAT_TIMESTAMP_L
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
    , CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  if (_item1) {
    rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item1->getName(), _name);
  };

  // Temperature
  _item2 = new rTemperatureItem(this, CONFIG_SENSOR_TEMP_NAME, (unit_temperature_t)CONFIG_FORMAT_TEMP_UNIT,
    filterMode2, filterSize2,
    CONFIG_FORMAT_TEMP_VALUE, CONFIG_FORMAT_TEMP_STRING
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    , CONFIG_FORMAT_TIMESTAMP_L 
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
    , CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  if (_item2) {
    rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item2->getName(), _name);
  };
}

// Register internal parameters
void rSensorHT::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  // Humidity
  if (_item1) {
    _item1->registerParameters(parent_group, CONFIG_SENSOR_HUMIDITY_KEY, CONFIG_SENSOR_HUMIDITY_NAME, CONFIG_SENSOR_HUMIDITY_FRIENDLY);
  };
  // Temperature
  if (_item2) {
    _item2->registerParameters(parent_group, CONFIG_SENSOR_TEMP_KEY, CONFIG_SENSOR_TEMP_NAME, CONFIG_SENSOR_TEMP_FRIENDLY);
  };
}

// Displaying multiple values in one topic
#if CONFIG_SENSOR_DISPLAY_ENABLED

char* rSensorHT::getDisplayValue()
{
  char* ret = nullptr;
  if (_item2) { 
    ret = _item2->getStringFiltered(); 
  };
  if (_item1) {
    ret = concat_strings_div(ret, _item1->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorHT::publishCustomValues()
{
  bool ret = rSensor::publishCustomValues();

  #if CONFIG_SENSOR_DEWPOINT_ENABLE
    if ((ret) && (_item1) && (_item2)) {
      ret = _item2->publishDataValue(CONFIG_SENSOR_DEWPOINT, 
        calcDewPoint(_item2->getValue().filteredValue, _item1->getValue().filteredValue));
    };
  #endif // CONFIG_SENSOR_DEWPOINT_ENABLE

  return ret;
} 

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorHT::jsonCustomValues()
{
  #if CONFIG_SENSOR_DEWPOINT_ENABLE
    if ((_item1) && (_item2)) {
      char * _dew_point = _item2->jsonDataValue(true, calcDewPoint(_item2->getValue().filteredValue, _item1->getValue().filteredValue));
      char * ret = malloc_stringf("\"%s\":%s", CONFIG_SENSOR_DEWPOINT, _dew_point);
      if (_dew_point) free(_dew_point);
      return ret;  
    };
  #endif // CONFIG_SENSOR_DEWPOINT_ENABLE
  return nullptr;
}

#endif // CONFIG_SENSOR_AS_JSON

// =======================================================================================================================
// =======================================================================================================================
// ====================================================== rSensorX3 ======================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensorX3::rSensorX3():rSensor()
{ 
  _item1 = nullptr;
  _item2 = nullptr;
  _item3 = nullptr;
}

// Destructor
rSensorX3::~rSensorX3()
{
  // We always delete an item, even if it is attached from the outside
  if (_item1) delete _item1;
  if (_item2) delete _item2;
  if (_item3) delete _item3;
}

// Set external items
void rSensorX3::setSensorItems(rSensorItem* item1, rSensorItem* item2, rSensorItem* item3)
{
  _item1 = item1;
  _item2 = item2;
  _item3 = item3;
  if (_item1) {
    _item1->setOwner(this);
    _item1->initItem();
  };
  if (_item2) {
    _item2->setOwner(this);
    _item2->initItem();
  };
  if (_item3) {
    _item3->setOwner(this);
    _item3->initItem();
  };
};

// Initialization of internal items
bool rSensorX3::initSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                const sensor_filter_t filterMode2, const uint16_t filterSize2,
                                const sensor_filter_t filterMode3, const uint16_t filterSize3)
{
  // Create items by default if they were not assigned externally
  if ((!_item1) || (!_item2) || (!_item3)) {
    createSensorItems(filterMode1, filterSize1, filterMode2, filterSize2, filterMode3, filterSize3);
    if (_item1) {
      _item1->setOwner(this);
      _item1->initItem();
    };
    if (_item2) {
      _item2->setOwner(this);
      _item2->initItem();
    };
    if (_item3) {
      _item3->setOwner(this);
      _item3->initItem();
    };
  };
  return true;
}

// Change filter mode
bool rSensorX3::setFilterMode1(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item1) return _item1->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX3::setFilterMode2(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item2) return _item2->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX3::setFilterMode3(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item3) return _item3->setFilterMode(filterMode, filterSize);
  return false;
}

// Writing measured RAW values to internal items
void rSensorX3::setRawValues(const value_t newValue1, const value_t newValue2, const value_t newValue3)
{
  if ((isnan(newValue1) && (_item1)) 
   || (isnan(newValue2) && (_item2)) 
   || (isnan(newValue3) && (_item3))) {
    setRawStatus(SENSOR_STATUS_NAN, true);
  } else {
    time_t timestamp = time(nullptr);
    if (_item1) _item1->setRawValue(newValue1, timestamp);
    if (_item2) _item2->setRawValue(newValue2, timestamp);
    if (_item3) _item3->setRawValue(newValue3, timestamp);
    setRawStatus(SENSOR_STATUS_OK, true);
  };
}

void rSensorX3::setRawValue1(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item1) _item1->setRawValue(newValue, timestamp);
}

void rSensorX3::setRawValue2(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item2) _item2->setRawValue(newValue, timestamp);
}

void rSensorX3::setRawValue3(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item3) _item3->setRawValue(newValue, timestamp);
}

// Get a pointer to storage
rSensorItem* rSensorX3::getSensorItem1()
{
  return _item1;
}

rSensorItem* rSensorX3::getSensorItem2()
{
  return _item2;
}

rSensorItem* rSensorX3::getSensorItem3()
{
  return _item3;
}

sensor_handle_t rSensorX3::getHandle1()
{
  if (_item1) return _item1->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX3::getHandle2()
{
  if (_item2) return _item2->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX3::getHandle3()
{
  if (_item3) return _item3->getHandle();
  return nullptr;
}

// Get data from storage
sensor_data_t rSensorX3::getValues1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX3::getValues2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX3::getValues3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX3::getValue1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX3::getValue2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX3::getValue3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX3::getExtremumsEntirely1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX3::getExtremumsEntirely2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX3::getExtremumsEntirely3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX3::getExtremumsWeekly1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX3::getExtremumsWeekly2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX3::getExtremumsWeekly3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX3::getExtremumsDaily1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX3::getExtremumsDaily2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX3::getExtremumsDaily3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* rSensorX3::getDisplayValue()
{
  char* ret = nullptr;
  if (_item1) { 
    ret = _item1->getStringFiltered(); 
  };
  if (_item2) {
    ret = concat_strings_div(ret, _item2->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  if (_item3) {
    ret = concat_strings_div(ret, _item3->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorX3::publishItems()
{
  return _item1->publishNamedValues() && _item2->publishNamedValues() && _item3->publishNamedValues();
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorX3::getJSON()
{
  char* ret = nullptr;
  char* _json_values = nullptr;
  if (_item1) { _json_values = _item1->jsonNamedValues(); };
  if (_item2) { _json_values = concat_strings_div(_json_values, _item2->jsonNamedValues(), ","); };
  if (_item3) { _json_values = concat_strings_div(_json_values, _item3->jsonNamedValues(), ","); };
  // Add mixed content line
  _json_values = concat_strings_div(_json_values, jsonDisplayAndCustomValues(), ",");
  // Generating full JSON
  if (_json_values) {
    #if CONFIG_SENSOR_STATUS_ENABLE
      ret = malloc_stringf("{\"%s\":\"%s\",%s}", CONFIG_SENSOR_STATUS, getStatusString(), _json_values);
    #else
      ret = malloc_stringf("{%s}", _json_values);
    #endif //CONFIG_SENSOR_STATUS_ENABLE
    free(_json_values);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// =======================================================================================================================
// =======================================================================================================================
// ====================================================== rSensorX4 ======================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensorX4::rSensorX4():rSensor()
{ 
  _item1 = nullptr;
  _item2 = nullptr;
  _item3 = nullptr;
  _item4 = nullptr;
}

// Destructor
rSensorX4::~rSensorX4()
{
  // We always delete an item, even if it is attached from the outside
  if (_item1) delete _item1;
  if (_item2) delete _item2;
  if (_item3) delete _item3;
  if (_item4) delete _item4;
}

// Set external items
void rSensorX4::setSensorItems(rSensorItem* item1, rSensorItem* item2, rSensorItem* item3, rSensorItem* item4)
{
  _item1 = item1;
  _item2 = item2;
  _item3 = item3;
  _item4 = item4;
  if (_item1) {
    _item1->setOwner(this);
    _item1->initItem();
  };
  if (_item2) {
    _item2->setOwner(this);
    _item2->initItem();
  };
  if (_item3) {
    _item3->setOwner(this);
    _item3->initItem();
  };
  if (_item4) {
    _item4->setOwner(this);
    _item4->initItem();
  };
};

// Initialization of internal items
bool rSensorX4::initSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                const sensor_filter_t filterMode2, const uint16_t filterSize2,
                                const sensor_filter_t filterMode3, const uint16_t filterSize3,
                                const sensor_filter_t filterMode4, const uint16_t filterSize4)
{
  // Create items by default if they were not assigned externally
  if ((!_item1) || (!_item2) || (!_item3) || (!_item4)) {
    createSensorItems(filterMode1, filterSize1, filterMode2, filterSize2, filterMode3, filterSize3, filterMode4, filterSize4);
    if (_item1) {
      _item1->setOwner(this);
      _item1->initItem();
    };
    if (_item2) {
      _item2->setOwner(this);
      _item2->initItem();
    };
    if (_item3) {
      _item3->setOwner(this);
      _item3->initItem();
    };
    if (_item4) {
      _item4->setOwner(this);
      _item4->initItem();
    };
  };
  return true;
}

// Change filter mode
bool rSensorX4::setFilterMode1(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item1) return _item1->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX4::setFilterMode2(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item2) return _item2->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX4::setFilterMode3(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item3) return _item3->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX4::setFilterMode4(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item4) return _item4->setFilterMode(filterMode, filterSize);
  return false;
}

// Writing measured RAW values to internal items
void rSensorX4::setRawValues(const value_t newValue1, const value_t newValue2, const value_t newValue3, const value_t newValue4)
{
  if ((isnan(newValue1) && (_item1))
   || (isnan(newValue2) && (_item2)) 
   || (isnan(newValue3) && (_item3)) 
   || (isnan(newValue4) && (_item4))) {
    setRawStatus(SENSOR_STATUS_NAN, true);
  } else {
    time_t timestamp = time(nullptr);
    if (_item1) _item1->setRawValue(newValue1, timestamp);
    if (_item2) _item2->setRawValue(newValue2, timestamp);
    if (_item3) _item3->setRawValue(newValue3, timestamp);
    if (_item4) _item4->setRawValue(newValue4, timestamp);
    setRawStatus(SENSOR_STATUS_OK, true);
  };
}

void rSensorX4::setRawValue1(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item1) _item1->setRawValue(newValue, timestamp);
}

void rSensorX4::setRawValue2(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item2) _item2->setRawValue(newValue, timestamp);
}

void rSensorX4::setRawValue3(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item3) _item3->setRawValue(newValue, timestamp);
}

void rSensorX4::setRawValue4(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item4) _item4->setRawValue(newValue, timestamp);
}

// Get a pointer to storage
rSensorItem* rSensorX4::getSensorItem1()
{
  return _item1;
}

rSensorItem* rSensorX4::getSensorItem2()
{
  return _item2;
}

rSensorItem* rSensorX4::getSensorItem3()
{
  return _item3;
}

rSensorItem* rSensorX4::getSensorItem4()
{
  return _item4;
}

sensor_handle_t rSensorX4::getHandle1()
{
  if (_item1) return _item1->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX4::getHandle2()
{
  if (_item2) return _item2->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX4::getHandle3()
{
  if (_item3) return _item3->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX4::getHandle4()
{
  if (_item4) return _item4->getHandle();
  return nullptr;
}

// Get data from storage
sensor_data_t rSensorX4::getValues1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX4::getValues2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX4::getValues3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX4::getValues4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX4::getValue1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX4::getValue2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX4::getValue3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX4::getValue4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsEntirely1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsEntirely2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsEntirely3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsEntirely4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsWeekly1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsWeekly2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsWeekly3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsWeekly4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsDaily1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsDaily2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsDaily3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX4::getExtremumsDaily4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* rSensorX4::getDisplayValue()
{
  char* ret = nullptr;
  if (_item1) { 
    ret = _item1->getStringFiltered(); 
  };
  if (_item2) {
    ret = concat_strings_div(ret, _item2->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  if (_item3) {
    ret = concat_strings_div(ret, _item3->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  if (_item4) {
    ret = concat_strings_div(ret, _item4->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorX4::publishItems()
{
  return _item1->publishNamedValues() && _item2->publishNamedValues() && _item3->publishNamedValues() && _item4->publishNamedValues();
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorX4::getJSON()
{
  char* ret = nullptr;
  char* _json_values = nullptr;
  if (_item1) { _json_values = _item1->jsonNamedValues(); };
  if (_item2) { _json_values = concat_strings_div(_json_values, _item2->jsonNamedValues(), ","); };
  if (_item3) { _json_values = concat_strings_div(_json_values, _item3->jsonNamedValues(), ","); };
  if (_item4) { _json_values = concat_strings_div(_json_values, _item4->jsonNamedValues(), ","); };
  // Add mixed content line
  _json_values = concat_strings_div(_json_values, jsonDisplayAndCustomValues(), ",");
  // Generating full JSON
  if (_json_values) {
    #if CONFIG_SENSOR_STATUS_ENABLE
      ret = malloc_stringf("{\"%s\":\"%s\",%s}", CONFIG_SENSOR_STATUS, getStatusString(), _json_values);
    #else
      ret = malloc_stringf("{%s}", _json_values);
    #endif //CONFIG_SENSOR_STATUS_ENABLE
    free(_json_values);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// =======================================================================================================================
// =======================================================================================================================
// ====================================================== rSensorX5 ======================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensorX5::rSensorX5():rSensor()
{ 
  _item1 = nullptr;
  _item2 = nullptr;
  _item3 = nullptr;
  _item4 = nullptr;
  _item5 = nullptr;
}

// Destructor
rSensorX5::~rSensorX5()
{
  // We always delete an item, even if it is attached from the outside
  if (_item1) delete _item1;
  if (_item2) delete _item2;
  if (_item3) delete _item3;
  if (_item4) delete _item4;
  if (_item5) delete _item5;
}

// Set external items
void rSensorX5::setSensorItems(rSensorItem* item1, rSensorItem* item2, rSensorItem* item3, rSensorItem* item4, rSensorItem* item5)
{
  _item1 = item1;
  _item2 = item2;
  _item3 = item3;
  _item4 = item4;
  _item5 = item5;
  if (_item1) {
    _item1->setOwner(this);
    _item1->initItem();
  };
  if (_item2) {
    _item2->setOwner(this);
    _item2->initItem();
  };
  if (_item3) {
    _item3->setOwner(this);
    _item3->initItem();
  };
  if (_item4) {
    _item4->setOwner(this);
    _item4->initItem();
  };
  if (_item5) {
    _item5->setOwner(this);
    _item5->initItem();
  };
};

// Initialization of internal items
bool rSensorX5::initSensorItems(const sensor_filter_t filterMode1, const uint16_t filterSize1,
                                const sensor_filter_t filterMode2, const uint16_t filterSize2,
                                const sensor_filter_t filterMode3, const uint16_t filterSize3,
                                const sensor_filter_t filterMode4, const uint16_t filterSize4,
                                const sensor_filter_t filterMode5, const uint16_t filterSize5)
{
  // Create items by default if they were not assigned externally
  if ((!_item1) || (!_item2) || (!_item3) || (!_item4) || (!_item5)) {
    createSensorItems(filterMode1, filterSize1, filterMode2, filterSize2, filterMode3, filterSize3, filterMode4, filterSize4, filterMode5, filterSize5);
    if (_item1) {
      _item1->setOwner(this);
      _item1->initItem();
    };
    if (_item2) {
      _item2->setOwner(this);
      _item2->initItem();
    };
    if (_item3) {
      _item3->setOwner(this);
      _item3->initItem();
    };
    if (_item4) {
      _item4->setOwner(this);
      _item4->initItem();
    };
    if (_item5) {
      _item5->setOwner(this);
      _item5->initItem();
    };
  };
  return true;
}

// Change filter mode
bool rSensorX5::setFilterMode1(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item1) return _item1->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX5::setFilterMode2(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item2) return _item2->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX5::setFilterMode3(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item3) return _item3->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX5::setFilterMode4(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item4) return _item4->setFilterMode(filterMode, filterSize);
  return false;
}

bool rSensorX5::setFilterMode5(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if (_item5) return _item5->setFilterMode(filterMode, filterSize);
  return false;
}

// Writing measured RAW values to internal items
void rSensorX5::setRawValues(const value_t newValue1, const value_t newValue2, const value_t newValue3, const value_t newValue4, const value_t newValue5)
{
  if ((isnan(newValue1) && (_item1))
   || (isnan(newValue2) && (_item2)) 
   || (isnan(newValue3) && (_item3)) 
   || (isnan(newValue4) && (_item4))
   || (isnan(newValue5) && (_item5))) {
    setRawStatus(SENSOR_STATUS_NAN, true);
  } else {
    time_t timestamp = time(nullptr);
    if (_item1) _item1->setRawValue(newValue1, timestamp);
    if (_item2) _item2->setRawValue(newValue2, timestamp);
    if (_item3) _item3->setRawValue(newValue3, timestamp);
    if (_item4) _item4->setRawValue(newValue4, timestamp);
    if (_item5) _item5->setRawValue(newValue5, timestamp);
    setRawStatus(SENSOR_STATUS_OK, true);
  };
}

void rSensorX5::setRawValue1(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item1) _item1->setRawValue(newValue, timestamp);
}

void rSensorX5::setRawValue2(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item2) _item2->setRawValue(newValue, timestamp);
}

void rSensorX5::setRawValue3(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item3) _item3->setRawValue(newValue, timestamp);
}

void rSensorX5::setRawValue4(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item4) _item4->setRawValue(newValue, timestamp);
}

void rSensorX5::setRawValue5(const value_t newValue)
{
  time_t timestamp = time(nullptr);
  if (_item5) _item5->setRawValue(newValue, timestamp);
}

// Get a pointer to storage
rSensorItem* rSensorX5::getSensorItem1()
{
  return _item1;
}

rSensorItem* rSensorX5::getSensorItem2()
{
  return _item2;
}

rSensorItem* rSensorX5::getSensorItem3()
{
  return _item3;
}

rSensorItem* rSensorX5::getSensorItem4()
{
  return _item4;
}

rSensorItem* rSensorX5::getSensorItem5()
{
  return _item5;
}

sensor_handle_t rSensorX5::getHandle1()
{
  if (_item1) return _item1->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX5::getHandle2()
{
  if (_item2) return _item2->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX5::getHandle3()
{
  if (_item3) return _item3->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX5::getHandle4()
{
  if (_item4) return _item4->getHandle();
  return nullptr;
}

sensor_handle_t rSensorX5::getHandle5()
{
  if (_item5) return _item5->getHandle();
  return nullptr;
}

// Get data from storage
sensor_data_t rSensorX5::getValues1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX5::getValues2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX5::getValues3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX5::getValues4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_data_t rSensorX5::getValues5(const bool readSensor)
{
  if (readSensor) readData();
  if (_item5) return _item5->getValues();
  sensor_data_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX5::getValue1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX5::getValue2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX5::getValue3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX5::getValue4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_value_t rSensorX5::getValue5(const bool readSensor)
{
  if (readSensor) readData();
  if (_item5) return _item5->getValue();
  sensor_value_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsEntirely1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsEntirely2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsEntirely3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsEntirely4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsEntirely5(const bool readSensor)
{
  if (readSensor) readData();
  if (_item5) return _item5->getExtremumsEntirely();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsWeekly1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsWeekly2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsWeekly3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsWeekly4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsWeekly5(const bool readSensor)
{
  if (readSensor) readData();
  if (_item5) return _item5->getExtremumsWeekly();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsDaily1(const bool readSensor)
{
  if (readSensor) readData();
  if (_item1) return _item1->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsDaily2(const bool readSensor)
{
  if (readSensor) readData();
  if (_item2) return _item2->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsDaily3(const bool readSensor)
{
  if (readSensor) readData();
  if (_item3) return _item3->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsDaily4(const bool readSensor)
{
  if (readSensor) readData();
  if (_item4) return _item4->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensorX5::getExtremumsDaily5(const bool readSensor)
{
  if (readSensor) readData();
  if (_item5) return _item5->getExtremumsDaily();
  sensor_extremums_t empty_data;
  return empty_data;
}

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* rSensorX5::getDisplayValue()
{
  char* ret = nullptr;
  if (_item1) { 
    ret = _item1->getStringFiltered(); 
  };
  if (_item2) {
    ret = concat_strings_div(ret, _item2->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  if (_item3) {
    ret = concat_strings_div(ret, _item3->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  if (_item4) {
    ret = concat_strings_div(ret, _item4->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  if (_item5) {
    ret = concat_strings_div(ret, _item5->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorX5::publishItems()
{
  return _item1->publishNamedValues() 
      && _item2->publishNamedValues() 
      && _item3->publishNamedValues() 
      && _item4->publishNamedValues() 
      && _item5->publishNamedValues();
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorX5::getJSON()
{
  char* ret = nullptr;
  char* _json_values = nullptr;
  if (_item1) { _json_values = _item1->jsonNamedValues(); };
  if (_item2) { _json_values = concat_strings_div(_json_values, _item2->jsonNamedValues(), ","); };
  if (_item3) { _json_values = concat_strings_div(_json_values, _item3->jsonNamedValues(), ","); };
  if (_item4) { _json_values = concat_strings_div(_json_values, _item4->jsonNamedValues(), ","); };
  if (_item5) { _json_values = concat_strings_div(_json_values, _item5->jsonNamedValues(), ","); };
  // Add mixed content line
  _json_values = concat_strings_div(_json_values, jsonDisplayAndCustomValues(), ",");
  // Generating full JSON
  if (_json_values) {
    #if CONFIG_SENSOR_STATUS_ENABLE
      ret = malloc_stringf("{\"%s\":\"%s\",%s}", CONFIG_SENSOR_STATUS, getStatusString(), _json_values);
    #else
      ret = malloc_stringf("{%s}", _json_values);
    #endif //CONFIG_SENSOR_STATUS_ENABLE
    free(_json_values);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON
