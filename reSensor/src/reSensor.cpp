#include "reSensor.h"

static const char* logTAG = "SENS";

// =======================================================================================================================
// =======================================================================================================================
// =================================================== Meteo routines ====================================================
// =======================================================================================================================
// =======================================================================================================================

double calcAbsoluteHumidity(float temp, float humd)
{
  // return humd * 10 * ((6.112 * 100.0 * exp((17.67 * temp)/(243.12 + temp)))/(461.52 * (temp + 273.15)));
  if (!isnan(temp) && !isnan(humd)) {
    return 6.112 * exp((17.67 * temp)/(temp + 243.5)) * humd * 2.1674 / (273.15 + temp);
  };
  return NAN;
}

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
rSensorItem::rSensorItem(rSensor *sensor, const char* itemKey, const char* itemName, const char* itemFriendly,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString) 
{
  _owner = sensor;
  _itemKey = itemKey;
  _itemName = itemName;
  _itemFriendly = itemFriendly;
  _fmtNumeric = formatNumeric;
  _fmtString = formatString;
  _pgItem = nullptr;
  _offsetValue = 0.0;
  _deltaMax = 0.0;
  _limitMin = 0.0;
  _limitMax = 0.0;

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
  bool ret = true;

  if ((filterMode != _filterMode) || (_filterSize != filterSize)) {
    // Setting new values
    _filterSize = filterSize;
    _filterMode = filterMode;
    _filterIndex = 0;
    _filterInit = false;
    _filterBuf = nullptr;

    // Allocating memory for an array of filter values
    ret = doChangeFilterMode();

    // Publish new values
    if (ret && _prm_filterMode) paramsValueStore(_prm_filterMode, false);
    if (ret && _prm_filterSize) paramsValueStore(_prm_filterSize, false);
  };

  return ret;
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
        rlog_e(_owner->getName(), "Failed change data filtering mode for %s: mode=%d, size=%d!", _owner->getName(), _filterMode, _filterSize);
      } else {
        rlog_e(this->getName(), "Failed change data filtering mode for %s: mode=%d, size=%d!", this->getName(), _filterMode, _filterSize);
      };
      return false;
    };
  };
  if (_owner) {
    rlog_i(_owner->getName(), "Changing data filtering mode for %s: mode=%d, size=%d", _owner->getName(), _filterMode, _filterSize);
  } else {
    rlog_i(this->getName(), "Changing data filtering mode for %s: mode=%d, size=%d", this->getName(), _filterMode, _filterSize);
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

void rSensorItem::setValidRange(value_t validMin, value_t validMax)
{
  _limitMin = validMin;
  _limitMax = validMax;
}

sensor_status_t rSensorItem::checkValue(const value_t rawValue)
{
  // Check for empty value
  if (isnan(rawValue)) {
    return SENSOR_STATUS_NO_DATA;
  };
  // Check for valid range values, if set
  if ((_limitMin < _limitMax) && ((rawValue < _limitMin) || (rawValue > _limitMax))) {
    return SENSOR_STATUS_BAD_DATA;
  };
  // Check for maximum change from previous measure, if set
  if ((_deltaMax > 0.0) && !isnan(_data.lastValue.rawValue) && (abs(rawValue - _data.lastValue.rawValue) > _deltaMax)) {
    return SENSOR_STATUS_BAD_DATA;
  };
  return SENSOR_STATUS_OK;
}

value_t rSensorItem::convertValue(const value_t rawValue)
{
  return rawValue;
}

value_t rSensorItem::convertOffsetValue(const value_t rawValue)
{
  return convertValue(rawValue + _offsetValue);
}

sensor_status_t rSensorItem::getRawValue(value_t * rawValue)
{
  return SENSOR_STATUS_NOT_SUPPORTED;
}

void rSensorItem::setRawAndConvertedValue(const value_t rawValue, const value_t convertedValue, const time_t rawTime)
{
  _data.lastValue.timestamp = rawTime;
  _data.lastValue.rawValue = rawValue;
  _data.lastValue.filteredValue = getFilteredValue(convertedValue);
 
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
  if (!isnan(_data.lastValue.filteredValue)) {
    if ((_data.lastValue.filteredValue < _data.extremumsEntirely.minValue.filteredValue) 
      || isnan(_data.extremumsEntirely.minValue.filteredValue) 
      || (_data.extremumsEntirely.minValue.timestamp == 0)) {
      _data.extremumsEntirely.minValue = _data.lastValue;
      #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
      _data.extremumsEntirely.minValueChanged = true;
      #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    };
    if ((_data.lastValue.filteredValue > _data.extremumsEntirely.maxValue.filteredValue) 
      || isnan(_data.extremumsEntirely.maxValue.filteredValue) 
      || (_data.extremumsEntirely.maxValue.timestamp == 0)) {
        _data.extremumsEntirely.maxValue = _data.lastValue;
        #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
        _data.extremumsEntirely.maxValueChanged = true;
        #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    };

    // Determine the weekly minimum and maximum
    if ((_data.lastValue.filteredValue < _data.extremumsWeekly.minValue.filteredValue) 
      || isnan(_data.extremumsWeekly.minValue.filteredValue) 
      || (_data.extremumsWeekly.minValue.timestamp == 0)) {
        _data.extremumsWeekly.minValue = _data.lastValue;
        #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
        _data.extremumsWeekly.minValueChanged = true;
        #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    };
    if ((_data.lastValue.filteredValue > _data.extremumsWeekly.maxValue.filteredValue) 
      || isnan(_data.extremumsWeekly.maxValue.filteredValue) 
      || (_data.extremumsWeekly.maxValue.timestamp == 0)) {
        _data.extremumsWeekly.maxValue = _data.lastValue;
        #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
        _data.extremumsWeekly.maxValueChanged = true;
        #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    };

    // Determine the daily minimum and maximum
    if ((_data.lastValue.filteredValue < _data.extremumsDaily.minValue.filteredValue) 
      || isnan(_data.extremumsDaily.minValue.filteredValue) 
      || (_data.extremumsDaily.minValue.timestamp == 0)) {
        _data.extremumsDaily.minValue = _data.lastValue;
        #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
        _data.extremumsDaily.minValueChanged = true;
        #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    };
    if ((_data.lastValue.filteredValue > _data.extremumsDaily.maxValue.filteredValue) 
      || isnan(_data.extremumsDaily.maxValue.filteredValue) 
      || (_data.extremumsDaily.maxValue.timestamp == 0)) {
        _data.extremumsDaily.maxValue = _data.lastValue;
        #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
        _data.extremumsDaily.maxValueChanged = true;
        #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    };
  };
}

sensor_status_t rSensorItem::setRawValue(const value_t rawValue, const time_t rawTime)
{
  value_t cnvValue = convertOffsetValue(rawValue);
  sensor_status_t ret = checkValue(cnvValue);
  if (ret == SENSOR_STATUS_OK) {
    setRawAndConvertedValue(rawValue, (value_t)cnvValue, rawTime);
  };
  return ret;
}

const char* rSensorItem::getKey()
{
  return _itemKey;
}

const char* rSensorItem::getName()
{
  return _itemName;
}

const char* rSensorItem::getFriendly()
{
  return _itemFriendly;
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

void rSensorItem::resetExtremumsEntirely()
{
  _data.extremumsEntirely.minValue.timestamp = 0;
  _data.extremumsEntirely.minValue.rawValue = 0.0;
  _data.extremumsEntirely.minValue.filteredValue = 0.0;
  _data.extremumsEntirely.maxValue.timestamp = 0;
  _data.extremumsEntirely.maxValue.rawValue = 0.0;
  _data.extremumsEntirely.maxValue.filteredValue = 0.0;
  #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    _data.extremumsEntirely.minValueChanged = true;
    _data.extremumsEntirely.maxValueChanged = true;
  #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
};
  

void rSensorItem::resetExtremumsWeekly()
{
  _data.extremumsWeekly.minValue.timestamp = 0;
  _data.extremumsWeekly.minValue.rawValue = 0.0;
  _data.extremumsWeekly.minValue.filteredValue = 0.0;
  _data.extremumsWeekly.maxValue.timestamp = 0;
  _data.extremumsWeekly.maxValue.rawValue = 0.0;
  _data.extremumsWeekly.maxValue.filteredValue = 0.0;
  #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    _data.extremumsWeekly.minValueChanged = true;
    _data.extremumsWeekly.maxValueChanged = true;
  #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
}

void rSensorItem::resetExtremumsDaily()
{
  _data.extremumsDaily.minValue.timestamp = 0;
  _data.extremumsDaily.minValue.rawValue = 0.0;
  _data.extremumsDaily.minValue.filteredValue = 0.0;
  _data.extremumsDaily.maxValue.timestamp = 0;
  _data.extremumsDaily.maxValue.rawValue = 0.0;
  _data.extremumsDaily.maxValue.filteredValue = 0.0;
  #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    _data.extremumsDaily.minValueChanged = true;
    _data.extremumsDaily.maxValueChanged = true;
  #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
}

void rSensorItem::resetExtremumsTotal()
{
  resetExtremumsEntirely();
  resetExtremumsWeekly();
  resetExtremumsDaily();
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------- Register internal parameters --------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rSensorItem::registerParameters(paramsGroupHandle_t parent_group)
{
  if ((_itemKey) && (_itemName) && (_itemFriendly)) {
    if (!_pgItem) {
      _pgItem = paramsRegisterGroup(parent_group, _itemKey, _itemName, _itemFriendly);
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
  _prm_deltaMax = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_DELTAMAX_KEY, CONFIG_SENSOR_PARAM_DELTAMAX_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_deltaMax);
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

char* rSensorItem::asString(const char* format, const value_t value, bool nan_brackets)
{
  if (isnan(value)) {
    return malloc_stringf(nan_brackets ? "\"%s\"" : "%s", CONFIG_FORMAT_EMPTY);
  } else {
    return malloc_stringf(format, (float)value);
  };
}

char* rSensorItem::getStringRaw()
{
  return asString(_fmtString, _data.lastValue.rawValue, false);
}

char* rSensorItem::getStringFiltered()
{
  return asString(_fmtString, _data.lastValue.filteredValue, false);
}

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishDataValue(const char* topic, const char* format, const value_t value)
{
  bool ret = false;
  if (_owner) {
    // .../%topic%/numeric = 0.00
    char* _topicNum = mqttGetSubTopic(topic, CONFIG_SENSOR_NUMERIC_VALUE);
    ret = (_topicNum != nullptr) && _owner->publish(_topicNum, asString(format, value, false), true);
    if (_topicNum != nullptr) free(_topicNum);
    #if CONFIG_SENSOR_STRING_ENABLE
      // .../%topic%/string = "0.00°С"
      char* _topicStr = mqttGetSubTopic(topic, CONFIG_SENSOR_STRING_VALUE);
      ret = (_topicStr != nullptr) && _owner->publish(_topicStr, asString(_fmtString, value, false), true);
      if (_topicStr != nullptr) free(_topicStr);
    #endif // CONFIG_SENSOR_STRING_ENABLE
    return ret;
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonDataValue(bool brackets, const char* format, const value_t value)
{
#if CONFIG_SENSOR_STRING_ENABLE
  // {"numeric":0.00,"string":"0.00°С"}
  char* ret = nullptr;
  char* _numeric = asString(format, value, true);
  char* _string = asString(_fmtString, value, true);
  if ((_numeric != nullptr) && (_string != nullptr)) {
    if (brackets) {
      ret = malloc_stringf("{\"%s\":%s,\"%s\":\"%s\"}", CONFIG_SENSOR_NUMERIC_VALUE, _numeric, CONFIG_SENSOR_STRING_VALUE, _string);
    } else {
      ret = malloc_stringf("\"%s\":%s,\"%s\":\"%s\"", CONFIG_SENSOR_NUMERIC_VALUE, _numeric, CONFIG_SENSOR_STRING_VALUE, _string);
    };
    if (_string != nullptr) free(_string);
    if (_numeric != nullptr) free(_numeric);
  };
  return ret;
#else
  // 0.00
  return asString(format, value, true);
#endif // CONFIG_SENSOR_STRING_ENABLE
}

#endif // CONFIG_SENSOR_AS_JSON

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Publishing timestamp ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_TIMESTAMP_ENABLE

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishTimestamp(const char* topic, sensor_value_t *data)
{
  bool ret = false;
  if (_owner != nullptr) {
    char* _topicValue = mqttGetSubTopic(topic, CONFIG_SENSOR_TIMESTAMP);
    if (_topicValue != nullptr) {
      char _time[CONFIG_FORMAT_STRFTIME_BUFFER_SIZE]};
      time2str_empty(CONFIG_FORMAT_TIMESTAMP_L, &(data->timestamp), &_time[0], sizeof(_time));
      ret = _owner->publish(_topicValue, _time, false);
      free(_topicValue);
    };
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonTimestamp(sensor_value_t *data)
{
  char _time[CONFIG_FORMAT_STRFTIME_BUFFER_SIZE];
  time2str_empty(CONFIG_FORMAT_TIMESTAMP_L, &(data->timestamp), &_time[0], sizeof(_time));
  return malloc_stringf("\"%s\":\"%s\"", CONFIG_SENSOR_TIMESTAMP, _time);
}

#endif // CONFIG_SENSOR_AS_JSON

#endif // CONFIG_SENSOR_TIMESTAMP_ENABLE

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------- Publishing "timestring" (string value and timestamp) --------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_TIMESTRING_ENABLE

char* rSensorItem::asStringTimeValue(sensor_value_t *data)
{
  char* ret = nullptr;
  if (isnan(data->filteredValue)) {
    ret = malloc_stringf("%s", CONFIG_FORMAT_EMPTY);
  } else {
    char* _string = asString(_fmtString, data->filteredValue, false);
    if (_string != nullptr) {
      char _time[CONFIG_FORMAT_STRFTIME_BUFFER_SIZE];
      time2str_empty(CONFIG_FORMAT_TIMESTAMP_S, &(data->timestamp), &_time[0], sizeof(_time));
      ret = malloc_stringf(CONFIG_FORMAT_TSVALUE, _string, _time);
    };
    if (_string != nullptr) free(_string);
  };
  return ret;
}

char* rSensorItem::getStringTimeValue()
{
  return asStringTimeValue(&(_data.lastValue));
}

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishStringTimeValue(const char* topic, sensor_value_t *data)
{
  bool ret = false;
  if (_owner != nullptr) {
    char* _topicValue = mqttGetSubTopic(topic, CONFIG_SENSOR_TIMESTRING_VALUE);
    if (_topicValue != nullptr) {
      ret = _owner->publish(_topicValue, asStringTimeValue(data), true);
      free(_topicValue);
    };
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonStringTimeValue(sensor_value_t *data)
{
  char* ret = nullptr;
  char* _stv = asStringTimeValue(data);
  if (_stv != nullptr) {
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

bool rSensorItem::publishValue(const char* topic, sensor_value_t *data)
{
  bool ret = false;
  // filtered value
  char* _topicFiltered = mqttGetSubTopic(topic, CONFIG_SENSOR_FILTERED_VALUE);
  if (_topicFiltered != nullptr) {
    ret = publishDataValue(_topicFiltered, _fmtNumeric, data->filteredValue);
    free(_topicFiltered);
    // raw value
    if (ret) {
      #if (CONFIG_SENSOR_RAW_ENABLE == 1)
        // raw value - always
        char* _topicRaw = mqttGetSubTopic(topic, CONFIG_SENSOR_RAW_VALUE);
        ret = (_topicRaw != nullptr) && publishDataValue(_topicRaw, "%f", data->rawValue);
        if (_topicRaw != nullptr) free(_topicRaw);
      #elif (CONFIG_SENSOR_RAW_ENABLE == 2)
        // raw value - only when there is filtration
        if (_forcedRawPublish || (_filterMode != SENSOR_FILTER_RAW) || (_offsetValue != 0.0)) {
          char* _topicRaw = mqttGetSubTopic(topic, CONFIG_SENSOR_RAW_VALUE);
          ret = (_topicRaw != nullptr) && publishDataValue(_topicRaw, "%f", data->rawValue);
          if (_topicRaw != nullptr) free(_topicRaw);
        };
      #endif // CONFIG_SENSOR_RAW_ENABLE
    };
  };
  return ret;  
}
#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonValue(sensor_value_t *data)
{
  char* ret = nullptr;
  // Generating JSON for filtered value
  char* _json_flt = jsonDataValue(true, _fmtNumeric, data->filteredValue);
  // Generating JSON for raw value, if enabled
  char* _json_raw = nullptr;
  #if (CONFIG_SENSOR_RAW_ENABLE == 1)
    _json_raw = jsonDataValue(true, "%f", data->rawValue);
  #elif (CONFIG_SENSOR_RAW_ENABLE == 2)
    if (_forcedRawPublish || (_filterMode != SENSOR_FILTER_RAW) || (_offsetValue != 0.0)) {
      _json_raw = jsonDataValue(true, "%f", data->rawValue);
    };
  #endif // CONFIG_SENSOR_RAW_ENABLE
  // If there are two JSONs, we mix them into one and delete the sources. If one - return as is
  if (_json_flt != nullptr) {
    if (_json_raw != nullptr) {
      ret = malloc_stringf("\"%s\":%s,\"%s\":%s", CONFIG_SENSOR_FILTERED_VALUE, _json_flt, CONFIG_SENSOR_RAW_VALUE, _json_raw);
      free(_json_raw);
      free(_json_flt);
    } else {
      ret = _json_flt;
    };
  } else {
    ret = _json_raw;
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------- Publishing part of sensor data ------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishPartSensorValue(const char* topic, const char* type, sensor_value_t *data)
{
  bool ret = false;
  char* _topicData = mqttGetSubTopic(topic, type);
  if (_topicData) {
    ret = publishValue(_topicData, data);
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      if (ret) ret = publishTimestamp(_topicData, data);
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE
      if (ret) ret = publishStringTimeValue(_topicData, data);
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
    free(_topicData);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonPartSensorValue(const char* type, sensor_value_t *data)
{
  char* ret = nullptr;
  char* _json_value = jsonValue(data);
  if (_json_value != nullptr) {
    char* _json_ext = nullptr;
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
      _json_ext = concat_strings_div(_json_ext, jsonTimestamp(data), ",");
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE
      _json_ext = concat_strings_div(_json_ext, jsonStringTimeValue(data), ",");
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
    if (_json_ext == nullptr) {
      // "type":{...}
      ret = malloc_stringf("\"%s\":{%s}", type, _json_value);
    } else {
      // "{"type":{...},"time":"12:45:38 01.02.2021","tsv":"0.00°С 12:45 01.02"}
      ret = malloc_stringf("\"%s\":{%s,%s}", type, _json_value, _json_ext);
      free(_json_ext);
    };
    free(_json_value);
  };
  return ret;
}

#endif // CONFIG_SENSOR_AS_JSON

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Publishing extremes -----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorItem::publishExtremums(const char* topic, sensor_extremums_t *range
  #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
  , const bool minValueChanged, const bool maxValueChanged
  #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
)
{
  #if CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
    return (!minValueChanged || publishPartSensorValue(topic, CONFIG_SENSOR_MINIMAL, &(range->minValue)))
        && (!maxValueChanged || publishPartSensorValue(topic, CONFIG_SENSOR_MAXIMAL, &(range->maxValue)));
  #else
    return publishPartSensorValue(topic, CONFIG_SENSOR_MINIMAL, &(range->minValue)) 
        && publishPartSensorValue(topic, CONFIG_SENSOR_MAXIMAL, &(range->maxValue));
  #endif // CONFIG_SENSOR_EXTREMUMS_OPTIMIZED
}

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorItem::jsonExtremums(const char* type, sensor_extremums_t *range)
{
  char* ret = nullptr;
  // "type":{"min":{...},"max":{...}}
  char* _min_value = jsonPartSensorValue(CONFIG_SENSOR_MINIMAL, &(range->minValue));
  if (_min_value != nullptr) {
    char* _max_value = jsonPartSensorValue(CONFIG_SENSOR_MAXIMAL, &(range->maxValue));
    if (_max_value != nullptr) {
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
          ret = (_topicExtremumsDaily) && publishExtremums(_topicExtremumsDaily, &_data.extremumsDaily
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
          ret = (_topicExtremumsWeekly) && publishExtremums(_topicExtremumsWeekly, &_data.extremumsWeekly
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
          ret = (_topicExtremumsEntirely) && publishExtremums(_topicExtremumsEntirely, &_data.extremumsEntirely
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
  char* _last = jsonPartSensorValue(CONFIG_SENSOR_LASTVALUE, &_data.lastValue);
  if (_last != nullptr) {
    char* _extr = nullptr;
    #if CONFIG_SENSOR_EXTREMUMS_ENTIRELY_ENABLE
      _extr = concat_strings_div(_extr, jsonExtremums(CONFIG_SENSOR_EXTREMUMS_ENTIRELY, &_data.extremumsEntirely), ",");
    #endif // CONFIG_SENSOR_EXTREMUMS_ENTIRELY_ENABLE
    #if CONFIG_SENSOR_EXTREMUMS_WEEKLY_ENABLE
      _extr = concat_strings_div(_extr, jsonExtremums(CONFIG_SENSOR_EXTREMUMS_WEEKLY, &_data.extremumsWeekly), ",");
    #endif // CONFIG_SENSOR_EXTREMUMS_WEEKLY_ENABLE
    #if CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
      _extr = concat_strings_div(_extr, jsonExtremums(CONFIG_SENSOR_EXTREMUMS_DAILY, &_data.extremumsDaily), ",");
    #endif // CONFIG_SENSOR_EXTREMUMS_DAILY_ENABLE
    if (_extr == nullptr) {
      ret = malloc_stringf("{%s}", _last);
    } else {
      ret = malloc_stringf("{%s,\"%s\":{%s}}", _last, CONFIG_SENSOR_EXTREMUS, _extr);
      free(_extr);
    };
    free(_last);
  };
  return ret;
}

char* rSensorItem::jsonNamedValues()
{
  char* ret = nullptr;
  char* _values = jsonValues();
  if (_values) {
    ret = malloc_stringf("\"%s\":%s", _itemName, _values);
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

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- NVS ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rSensorItem::nvsStoreExtremums(const char* nvs_space)
{
  nvs_handle_t nvs_handle;
  if (nvsOpen(nvs_space, NVS_READWRITE, &nvs_handle)) {
    // daily
    nvs_set_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MIN_TIME, _data.extremumsDaily.minValue.timestamp);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MIN_RAW, _data.extremumsDaily.minValue.rawValue);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MIN_FLT, _data.extremumsDaily.minValue.filteredValue);
    nvs_set_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MAX_TIME, _data.extremumsDaily.maxValue.timestamp);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MAX_RAW, _data.extremumsDaily.maxValue.rawValue);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MAX_FLT, _data.extremumsDaily.maxValue.filteredValue);
    // weeky
    nvs_set_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MIN_TIME, _data.extremumsWeekly.minValue.timestamp);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MIN_RAW, _data.extremumsWeekly.minValue.rawValue);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MIN_FLT, _data.extremumsWeekly.minValue.filteredValue);
    nvs_set_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MAX_TIME, _data.extremumsWeekly.maxValue.timestamp);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MAX_RAW, _data.extremumsWeekly.maxValue.rawValue);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MAX_FLT, _data.extremumsWeekly.maxValue.filteredValue);
    // entirely
    nvs_set_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MIN_TIME, _data.extremumsEntirely.minValue.timestamp);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MIN_RAW, _data.extremumsEntirely.minValue.rawValue);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MIN_FLT, _data.extremumsEntirely.minValue.filteredValue);
    nvs_set_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MAX_TIME, _data.extremumsEntirely.maxValue.timestamp);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MAX_RAW, _data.extremumsEntirely.maxValue.rawValue);
    nvs_set_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MAX_FLT, _data.extremumsEntirely.maxValue.filteredValue);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
  };
}

void rSensorItem::nvsRestoreExtremums(const char* nvs_space)
{
  nvs_handle_t nvs_handle;
  if (nvsOpen(nvs_space, NVS_READWRITE, &nvs_handle)) {
    // daily
    nvs_get_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MIN_TIME, &_data.extremumsDaily.minValue.timestamp);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MIN_RAW, &_data.extremumsDaily.minValue.rawValue);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MIN_FLT, &_data.extremumsDaily.minValue.filteredValue);
    nvs_get_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MAX_TIME, &_data.extremumsDaily.maxValue.timestamp);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MAX_RAW, &_data.extremumsDaily.maxValue.rawValue);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_DAY_MAX_FLT, &_data.extremumsDaily.maxValue.filteredValue);
    // weeky
    nvs_get_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MIN_TIME, &_data.extremumsWeekly.minValue.timestamp);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MIN_RAW, &_data.extremumsWeekly.minValue.rawValue);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MIN_FLT, &_data.extremumsWeekly.minValue.filteredValue);
    nvs_get_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MAX_TIME, &_data.extremumsWeekly.maxValue.timestamp);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MAX_RAW, &_data.extremumsWeekly.maxValue.rawValue);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_WEEK_MAX_FLT, &_data.extremumsWeekly.maxValue.filteredValue);
    // entirely
    nvs_get_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MIN_TIME, &_data.extremumsEntirely.minValue.timestamp);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MIN_RAW, &_data.extremumsEntirely.minValue.rawValue);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MIN_FLT, &_data.extremumsEntirely.minValue.filteredValue);
    nvs_get_time(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MAX_TIME, &_data.extremumsEntirely.maxValue.timestamp);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MAX_RAW, &_data.extremumsEntirely.maxValue.rawValue);
    nvs_get_float(nvs_handle, CONFIG_SENSOR_NVS_EXTREMUM_ALL_MAX_FLT, &_data.extremumsEntirely.maxValue.filteredValue);

    nvs_close(nvs_handle);
  };
}

// =======================================================================================================================
// =======================================================================================================================
// =================================================== rTemperatureItem ==================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rTemperatureItem::rTemperatureItem(rSensor *sensor, const char* itemKey, const char* itemName, const char* itemFriendly,
  const unit_temperature_t unitValue,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString)
// inherited constructor
:rSensorItem(sensor, itemKey, itemName, itemFriendly, filterMode, filterSize, formatNumeric, formatString) 
{
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
rPressureItem::rPressureItem(rSensor *sensor, const char* itemKey, const char* itemName, const char* itemFriendly,
  const unit_pressure_t unitValue,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString) 
// inherited constructor
:rSensorItem(sensor, itemKey, itemName, itemFriendly, filterMode, filterSize, formatNumeric, formatString) 
{
  _units = unitValue;
  _forcedRawPublish = true; 
  // Set default valid range: ~700 - 800 mmhg
  // setValidRange(90000, 110000); 
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
// ==================================================== rVirtualItem =====================================================
// =======================================================================================================================
// =======================================================================================================================

rVirtualItem::rVirtualItem(rSensor *sensor, const char* itemKey, const char* itemName, const char* itemFriendly,
  const double coefficient,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString)
// inherited constructor
:rSensorItem(sensor, itemKey, itemName, itemFriendly, filterMode, filterSize, formatNumeric, formatString) 
{
  _coefficient = coefficient;
};

void rVirtualItem::registerItemParameters(paramsGroup_t * group)
{
  rSensorItem::registerItemParameters(group);
  paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_DOUBLE, nullptr, group, 
    CONFIG_SENSOR_PARAM_COEF_KEY, CONFIG_SENSOR_PARAM_COEF_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_coefficient);
}

value_t rVirtualItem::convertValue(const value_t rawValue)
{
  return rawValue * _coefficient;
}

// =======================================================================================================================
// =======================================================================================================================
// ====================================================== rMapItem =======================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rMapItem::rMapItem(rSensor *sensor, const char* itemKey, const char* itemName, const char* itemFriendly,
  const type_bounds_t in_bounds, const value_t in_min, const value_t in_max,
  const value_t out_min, const value_t out_max,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString)
// inherited constructor
:rSensorItem(sensor, itemKey, itemName, itemFriendly, filterMode, filterSize, formatNumeric, formatString)
{
  _forcedRawPublish = true; 

  _in_bounds = in_bounds;
  _in_min = in_min;
  _in_max = in_max;
  _in_range = _in_max - _in_min;
  
  _out_min = out_min;
  _out_max = out_max;
};

void rMapItem::registerItemParameters(paramsGroup_t * group)
{
  rSensorItem::registerItemParameters(group);

  _prm_in_bounds = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, group, 
    CONFIG_SENSOR_PARAM_MAP_BOUNDS_TYPE_KEY, CONFIG_SENSOR_PARAM_MAP_BOUNDS_TYPE_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_in_bounds);
  paramsSetLimitsU8(_prm_in_bounds, BOUNDS_FIXED, BOUNDS_SHIFT_RANGE);

  _prm_in_min = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_MAP_BOUNDS_MIN_KEY, CONFIG_SENSOR_PARAM_MAP_BOUNDS_MIN_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_in_min);

  _prm_in_max = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_MAP_BOUNDS_MAX_KEY, CONFIG_SENSOR_PARAM_MAP_BOUNDS_MAX_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_in_max);

  _prm_in_range = paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_FLOAT, nullptr, group, 
    CONFIG_SENSOR_PARAM_MAP_BOUNDS_RANGE_KEY, CONFIG_SENSOR_PARAM_MAP_BOUNDS_RANGE_FRIENDLY, 
    CONFIG_MQTT_PARAMS_QOS, &_in_range);
}

value_t rMapItem::checkBounds(value_t newValue) 
{
  value_t _value = newValue;

  bool _store_min = false;
  bool _store_max = false;

  // Lower bound check
  if (_in_min < _in_max ? _value < _in_min : _value > _in_min) {
    if ((_in_bounds == BOUNDS_EXPAND) || (_in_bounds == BOUNDS_EXPAND_MIN) || (_in_bounds == BOUNDS_SHIFT_RANGE)) {
      _in_min = _value;
      _store_min = true;
      if (_in_bounds == BOUNDS_SHIFT_RANGE) {
        _in_max = _in_min + _in_range;
        _store_max = true;
      };
    };
  };

  // Upper bound check
  if (_in_min < _in_max ? _value > _in_max : _value < _in_max) {
    if ((_in_bounds == BOUNDS_EXPAND) || (_in_bounds == BOUNDS_EXPAND_MAX) || (_in_bounds == BOUNDS_SHIFT_RANGE)) {
      _in_max = _value;
      _store_max = true;
      if (_in_bounds == BOUNDS_SHIFT_RANGE) {
        _in_min = _in_max - _in_range;
        _store_min = true;
      };
    };
  };

  // Value normalization
  if (_in_min < _in_max) {
    if (_value < _in_min) { _value = _in_min; };
    if (_value > _in_max) { _value = _in_max; };
  } else {
    if (_value > _in_min) { _value = _in_min; };
    if (_value < _in_max) { _value = _in_max; };
  };

  // Store new bounds
  if (_store_min) {
    paramsValueStore(_prm_in_min, false);
    rlog_d(getName(), "New lower range value set: %f", _in_min);
  };
  if (_store_max) {
    paramsValueStore(_prm_in_max, false);
    rlog_d(getName(), "New upper range value set: %f", _in_max);
  };
  if ((_in_bounds != BOUNDS_SHIFT_RANGE) && (_store_min || _store_max)) {
    _in_range = _in_max - _in_min;
    paramsValueStore(_prm_in_range, false);
    rlog_d(getName(), "New range size set: %f", _in_range);
  };

  return _value;
};

value_t rMapItem::convertValue(const value_t rawValue)
{
  if (isnan(rawValue) && ((_in_max - _in_min) != 0.0)) {
    return rawValue; 
  } else { 
    return ((checkBounds(rawValue) - (value_t)_in_min) / ((value_t)_in_max - (value_t)_in_min) * ((value_t)_out_max - (value_t)_out_min) + (value_t)_out_min);
  };
};

// =======================================================================================================================
// =======================================================================================================================
// ======================================================= rSensor =======================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensor::rSensor(uint8_t eventId, const uint8_t items,
  const char* sensorName, const char* topicName, const bool topicLocal, 
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
  _lstStatus = SENSOR_STATUS_NO_INIT;
  _errStatus = SENSOR_STATUS_NO_INIT;
  _cbOnChangeStatus = cb_status;
  _cbOnPublishData = cb_publish;

  _items_count = items;
  _items = new rSensorItemHandle[_items_count];
  for (uint8_t i = 0; i < _items_count; i++) {
    _items[i] = nullptr;
  };
}

// Destructor
rSensor::~rSensor()
{
  topicsFree();
  if (_items) delete[] _items;
}

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------- Properties ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// Sensor name
const char* rSensor::getName()
{
  if (_name != nullptr) {
    return _name;
  } else {
    return "???";
  };
}

// Mqtt topic
void rSensor::topicsCreate(bool topicPrimary)
{
  if (_topicPub) free(_topicPub);
  if (_topicName) _topicPub = mqttGetTopicDevice1(topicPrimary, _topicLocal, _topicName);
  if (_topicPub) {
    rlog_i(logTAG, "Generated topic for sensor \"%s\": [ %s ]", getName(), _topicPub);
  } else {
    rlog_e(logTAG, "Failed to generate topic for sensor \"%s\"", getName());
  };
}

void rSensor::topicsFree()
{
  if (_topicPub) free(_topicPub);
  _topicPub = nullptr;
  rlog_d(logTAG, "Topic for sensor \"%s\" has been scrapped", getName());
}

char* rSensor::getTopicPub()
{
  return _topicPub;
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Data storages ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rSensor::setSensorItem(const uint8_t index, rSensorItem* item)
{
  if ((_items) && (index < _items_count) && (item)) {
    _items[index] = item;
    _items[index]->setOwner(this);
    _items[index]->initItem();
  };
};

// Get a pointer to storage
rSensorItem* rSensor::getSensorItem(const uint8_t index)
{
  if ((_items) && (index < _items_count) && (_items[index])) {
    return _items[index];
  };
  return nullptr;
}

sensor_handle_t rSensor::getHandle(const uint8_t index)
{
  if ((_items) && (index < _items_count) && (_items[index])) {
    _items[index]->getHandle();
  };
  return nullptr;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Change filter mode -------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool rSensor::setFilterMode(const uint8_t index, const sensor_filter_t filterMode, const uint16_t filterSize)
{
  if ((_items) && (index < _items_count) && (_items[index])) {
    return _items[index]->setFilterMode(filterMode, filterSize);
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Get data from storages -----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

sensor_data_t rSensor::getItemData(const uint8_t index, const bool readSensor)
{
  if (readSensor) readData();
  if ((_items) && (index < _items_count) && (_items[index])) {
    return _items[index]->getValues();
  };
  sensor_data_t empty_data;
  return empty_data;
}

sensor_value_t rSensor::getItemValue(const uint8_t index, const bool readSensor)
{
  if (readSensor) readData();
  if ((_items) && (index < _items_count) && (_items[index])) {
    return _items[index]->getValue();
  };
  sensor_value_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensor::getItemExtremumsEntirely(const uint8_t index, const bool readSensor)
{
  if (readSensor) readData();
  if ((_items) && (index < _items_count) && (_items[index])) {
    return _items[index]->getExtremumsEntirely();
  };
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensor::getItemExtremumsWeekly(const uint8_t index, const bool readSensor)
{
  if (readSensor) readData();
  if ((_items) && (index < _items_count) && (_items[index])) {
    return _items[index]->getExtremumsWeekly();
  };
  sensor_extremums_t empty_data;
  return empty_data;
}

sensor_extremums_t rSensor::getItemExtremumsDaily(const uint8_t index, const bool readSensor)
{
  if (readSensor) readData();
  if ((_items) && (index < _items_count) && (_items[index])) {
    return _items[index]->getExtremumsDaily();
  };
  sensor_extremums_t empty_data;
  return empty_data;
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

void rSensor::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  if (_items) {
    for (uint8_t i = 0; i < _items_count; i++) {
      if (_items[i]) {
        _items[i]->registerParameters(parent_group);
      };
    };
  };
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
  sensor_status_t prvStatus = _errStatus;
  _lstStatus = newStatus;
  // Status changed
  if (_errStatus != newStatus) {
    // No init
    if (newStatus == SENSOR_STATUS_NO_INIT) {
      _errStatus = newStatus;
      _errCount = 0;
      if (_cbOnChangeStatus) {
        _cbOnChangeStatus(this, prvStatus, newStatus);
      };
      postEventStatus(prvStatus, newStatus);
    // OK
    } else if (newStatus == SENSOR_STATUS_OK) {
      bool isFirstOk = (_errStatus == SENSOR_STATUS_NO_INIT) && (_errCount == 0);
      _errStatus = newStatus;
      _errCount = 0;
      if (!isFirstOk) {
        if (_cbOnChangeStatus) {
          _cbOnChangeStatus(this, prvStatus, newStatus);
        };
        postEventStatus(prvStatus, newStatus);
        publishData(false);
      };
    // Errors
    } else {
      _errCount++;
      if (forced || (_errCount > _errLimit)) {
        _errStatus = newStatus;
        if (_cbOnChangeStatus) {
          _cbOnChangeStatus(this, prvStatus, newStatus);
        };
        postEventStatus(prvStatus, newStatus);
        publishData(false);
      };
    };
  } else {
    // Count error anyway
    if ((newStatus == SENSOR_STATUS_NO_INIT) || (newStatus == SENSOR_STATUS_OK)) {
      if (_errCount > 0) {
        _errCount = 0;
      };
    } else {
      _errCount++;
    }
  };
}

void rSensor::setErrorStatus(sensor_status_t newStatus, bool forced)
{
  if ((_errStatus < SENSOR_STATUS_CONN_ERROR) && (newStatus >= SENSOR_STATUS_CONN_ERROR)) {
    setRawStatus(newStatus, forced);
  };
}

sensor_status_t rSensor::convertEspError(const uint32_t error)
{
  switch (error) {
    case ESP_OK:              
      return SENSOR_STATUS_OK;
    case ESP_ERR_TIMEOUT:     
      return SENSOR_STATUS_CONN_ERROR;
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
  return _errStatus;
}

const char* rSensor::statusString(sensor_status_t status)
{
  switch (status) {
    case SENSOR_STATUS_NO_INIT:
      return CONFIG_SENSOR_STATUS_NO_INIT;
    case SENSOR_STATUS_NO_DATA:
      return CONFIG_SENSOR_STATUS_NO_DATA;
    case SENSOR_STATUS_OK:
      return CONFIG_SENSOR_STATUS_OK;
    case SENSOR_STATUS_CONN_ERROR: 
      return CONFIG_SENSOR_STATUS_CONNECT;
    case SENSOR_STATUS_CAL_ERROR: 
      return CONFIG_SENSOR_STATUS_CALIBRATION;
    case SENSOR_STATUS_CRC_ERROR: 
      return CONFIG_SENSOR_STATUS_CRC_ERROR;
    case SENSOR_STATUS_BAD_DATA:
      return CONFIG_SENSOR_STATUS_BAD_DATA;
    case SENSOR_STATUS_ERROR: 
      return CONFIG_SENSOR_STATUS_ERROR;
    default:
      return CONFIG_SENSOR_STATUS_UNKNOWN;
  }
}

const char* rSensor::getStatusString()
{ 
  return statusString(_errStatus);
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- Communication with sensor ----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

sensor_status_t rSensor::sensorBusReset()
{
  return SENSOR_STATUS_OK;
}

bool rSensor::sensorStart()
{
  rlog_d(logTAG, RSENSOR_LOG_MSG_INIT, getName());
  sensor_status_t resetStatus = sensorBusReset();
  if (resetStatus == SENSOR_STATUS_OK) {
    resetStatus = sensorReset();
    if (resetStatus == SENSOR_STATUS_OK) {
      rlog_i(logTAG, RSENSOR_LOG_MSG_INIT_OK, getName());
      setRawStatus(resetStatus, true);
      return true;
    };
  };
  setRawStatus(resetStatus, true);
  return false;
}

sensor_status_t rSensor::readData()
{
  // Check if the sensor has been initialized
  if (_errStatus == SENSOR_STATUS_NO_INIT) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_NO_INIT, getName());
    return _errStatus;
  };

  // If the previous operation was completed with an error, reset the sensor
  if (!((_lstStatus == SENSOR_STATUS_OK) || (_lstStatus == SENSOR_STATUS_NO_DATA))) {
    _lstStatus = sensorReset();
    if (_lstStatus != SENSOR_STATUS_OK) {
      setRawStatus(_lstStatus, false);
      return _lstStatus;
    };
  };

  // Check if the sensor reading interval has expired
  if ((_readInterval == 0) || ((esp_timer_get_time() - _readLast) >= (int64_t)(_readInterval * 1000))) {
    _readLast = esp_timer_get_time();
    rlog_v(logTAG, "Read data from [ %s ]...", getName());
    _lstStatus = readRawData();

    // If reading fails, reset sensor and try again
    if (!((_lstStatus == SENSOR_STATUS_OK) || (_lstStatus == SENSOR_STATUS_NO_DATA))) {
      _lstStatus = sensorReset();
      if (_lstStatus == SENSOR_STATUS_OK) {
        _lstStatus = readRawData();
      };
    };

    setRawStatus(_lstStatus, false);
  };

  return _errStatus;
}

// Writing measured RAW values to internal items
sensor_status_t rSensor::setRawValue(const uint8_t index, const value_t newValue)
{
  if ((_items) && (index < _items_count) && (_items[index])) {
    return _items[index]->setRawValue(newValue, time(nullptr));
  } else {
    return SENSOR_STATUS_NO_INIT;
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Extremums ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rSensor::resetExtremumsEntirely()
{
  for (uint8_t i = 0; i < _items_count; i++) {
    if (_items[i]) {
      _items[i]->resetExtremumsEntirely();
    };
  };
}

void rSensor::resetExtremumsWeekly()
{
  for (uint8_t i = 0; i < _items_count; i++) {
    if (_items[i]) {
      _items[i]->resetExtremumsWeekly();
    };
  };
}

void rSensor::resetExtremumsDaily()
{
  for (uint8_t i = 0; i < _items_count; i++) {
    if (_items[i]) {
      _items[i]->resetExtremumsDaily();
    };
  };
}

void rSensor::resetExtremumsTotal()
{
  for (uint8_t i = 0; i < _items_count; i++) {
    if (_items[i]) {
      _items[i]->resetExtremumsTotal();
    };
  };
}

void rSensor::nvsStoreExtremums(const char* nvs_space)
{
  for (uint8_t i = 0; i < _items_count; i++) {
    if (_items[i]) {
      char* nvs_space_item = malloc_stringf(CONFIG_SENSOR_NVS_ITEMS, nvs_space, i + 1);
      if (nvs_space_item) {
        _items[i]->nvsStoreExtremums(nvs_space_item);
        free(nvs_space_item);
      };
    };
  };
}

void rSensor::nvsRestoreExtremums(const char* nvs_space)
{
  for (uint8_t i = 0; i < _items_count; i++) {
    if (_items[i]) {
      char* nvs_space_item = malloc_stringf(CONFIG_SENSOR_NVS_ITEMS, nvs_space, i + 1);
      if (nvs_space_item) {
        _items[i]->nvsRestoreExtremums(nvs_space_item);
        free(nvs_space_item);
      };
    };
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------- Displaying multiple values in one topic ---------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_SENSOR_DISPLAY_ENABLED

char* rSensor::getDisplayValue()
{
  char* ret = nullptr;
  if (_items_count == 1) { 
    if (_items[0]) {
      ret = _items[0]->getStringTimeValue();
    };
  };
  if (_items_count > 1) {
    if (_items[0]) {
      ret = _items[0]->getStringFiltered();
    };
    if (_items[1]) {
      ret = concat_strings_div(ret, _items[1]->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
    };
  };
  return ret;
}

char* rSensor::getDisplayValueStatus()
{
  #if CONFIG_SENSOR_STATUS_AS_MIXED_ON_ERROR
    if (_errStatus == SENSOR_STATUS_OK) {
      return getDisplayValue();  
    } else {
      return malloc_string(getStatusString());
    };
  #else
    return getDisplayValue();
  #endif // CONFIG_SENSOR_STATUS_AS_MIXED_ON_ERROR
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

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
  bool ret = false;
  for (uint8_t i = 0; i < _items_count; i++) {
    if (_items[i]) {
      ret = _items[i]->publishNamedValues();
    };
  };
  return ret;
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

char* rSensor::getJSON()
{
  char* ret = nullptr;
  // Concat items
  char* _json_values = nullptr;
  for (uint8_t i = 0; i < _items_count; i++) {
    if (_items[i]) {
      _json_values = concat_strings_div(_json_values, _items[i]->jsonNamedValues(), ",");
    };
  };
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

#endif //CONFIG_SENSOR_AS_JSON

// =======================================================================================================================
// =======================================================================================================================
// ===================================================== rSensorStub =====================================================
// =======================================================================================================================
// =======================================================================================================================

// Constructor
rSensorStub::rSensorStub(uint8_t eventId, 
  const char* sensorName, const char* topicName, const bool topicLocal, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
:rSensor(eventId, 1, 
  sensorName, topicName,  topicLocal, 
  minReadInterval, errorLimit, 
  cb_status, cb_publish)
{ 
}

void rSensorStub::setSensorItems(rSensorItem* item)
{
  setSensorItem(0, item);
}

// Sensor reset
sensor_status_t rSensorStub::sensorReset()
{
  // Item initialization is called only once in setSensorItems(item); >> _item->initItem();
  if (_items[0] != nullptr) {
    return SENSOR_STATUS_OK;
  } else {
    return SENSOR_STATUS_NO_INIT;
  };
}

sensor_status_t rSensorStub::readRawData()
{
  sensor_status_t ret = SENSOR_STATUS_NOT_SUPPORTED;
  if (_items[0]) {
    value_t rawValue;
    ret = _items[0]->getRawValue(&rawValue);
    if (ret == SENSOR_STATUS_OK) {
      return setRawValue(0, rawValue);
    };
  };
  return ret;
}

sensor_status_t rSensorStub::setExtValue(const value_t extValue)
{
  if (_items[0]) {
    sensor_status_t ret = setRawValue(0, extValue);
    setRawStatus(ret, false);
    return ret;
  };
  return SENSOR_STATUS_NO_INIT;
}

sensor_value_t rSensorStub::getValue(const bool readSensor)
{
  return getItemValue(0, readSensor);
}

// =======================================================================================================================
// =======================================================================================================================
// ====================================================== rSensorHT ======================================================
// =======================================================================================================================
// =======================================================================================================================

rSensorHT::rSensorHT(uint8_t eventId, 
  const char* sensorName, const char* topicName, const bool topicLocal, 
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
:rSensor(eventId, 2, 
  sensorName, topicName,  topicLocal, 
  minReadInterval, errorLimit, 
  cb_status, cb_publish)
{
}

void rSensorHT::setSensorItems(rSensorItem* itemHumidity, rSensorItem* itemTemperature)
{
  setSensorItem(0, itemHumidity);
  setSensorItem(1, itemTemperature);
}

sensor_status_t rSensorHT::setRawValues(const value_t newHumidity, const value_t newTemperature)
{
  sensor_status_t ret = SENSOR_STATUS_NO_INIT;
  if (_items[0] && _items[1]) {
    time_t now = time(nullptr);
    ret = _items[0]->setRawValue(newHumidity, now);
    if (ret == SENSOR_STATUS_OK) {
      ret = _items[1]->setRawValue(newTemperature, now);
    };
  };
  return ret;
}

sensor_value_t rSensorHT::getTemperature(const bool readSensor)
{
  return getItemValue(1, readSensor);
}

sensor_value_t rSensorHT::getHumidity(const bool readSensor)
{
  return getItemValue(0, readSensor);
}

// Displaying multiple values in one topic
#if CONFIG_SENSOR_DISPLAY_ENABLED

char* rSensorHT::getDisplayValue()
{
  char* ret = nullptr;
  if (_items[1]) { 
    ret = _items[1]->getStringFiltered(); 
  };
  if (_items[0]) {
    ret = concat_strings_div(ret, _items[0]->getStringFiltered(), CONFIG_JSON_CHAR_EOL);
  };
  return ret;
}

#endif // CONFIG_SENSOR_DISPLAY_ENABLED

#if CONFIG_SENSOR_AS_PLAIN

bool rSensorHT::publishCustomValues()
{
  bool ret = rSensor::publishCustomValues();

  #if CONFIG_SENSOR_DEWPOINT_ENABLE
    if ((ret) && (_items[0]) && (_items[1])) {
      ret = _items[1]->publishDataValue(CONFIG_SENSOR_DEWPOINT, 
        calcDewPoint(_items[1]->getValue().filteredValue, _items[0]->getValue().filteredValue));
    };
  #endif // CONFIG_SENSOR_DEWPOINT_ENABLE

  return ret;
} 

#endif // CONFIG_SENSOR_AS_PLAIN

#if CONFIG_SENSOR_AS_JSON

char* rSensorHT::jsonCustomValues()
{
  #if CONFIG_SENSOR_DEWPOINT_ENABLE
    if ((_items[0]) && (_items[1])) {
      char * _dew_point = _items[1]->jsonDataValue(true, calcDewPoint(_items[1]->getValue().filteredValue, _items[0]->getValue().filteredValue));
      char * ret = malloc_stringf("\"%s\":%s", CONFIG_SENSOR_DEWPOINT, _dew_point);
      if (_dew_point) free(_dew_point);
      return ret;  
    };
  #endif // CONFIG_SENSOR_DEWPOINT_ENABLE
  return nullptr;
}

#endif // CONFIG_SENSOR_AS_JSON

