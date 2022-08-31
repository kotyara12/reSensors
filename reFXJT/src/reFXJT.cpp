#include "reFXJT.h"
#include "mbcontroller.h"

static const char* logTAG = "FXJT";

/***************************************************************************
const mb_parameter_descriptor_t device_parameters[] = 
  // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
  {0, "Direction", "-", 1, MB_PARAM_HOLDING, 0, 1, 0, PARAM_TYPE_U16, PARAM_SIZE_U16, {0,0,0}, PAR_PERMS_READ};
***************************************************************************/

rDirectionItem::rDirectionItem(rSensor *sensor, const char* itemName,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const char* formatNumeric, const char* formatString 
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , const char* formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , const char* formatTimestampValue, const char* formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
):rSensorItem(sensor, itemName, filterMode, filterSize, formatNumeric, formatString
  #if CONFIG_SENSOR_TIMESTAMP_ENABLE
  , formatTimestamp
  #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
  #if CONFIG_SENSOR_TIMESTRING_ENABLE  
  , formatTimestampValue, formatStringTimeValue
  #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
) {};

char* rDirectionItem::asString(const char* format, const value_t value, bool nan_brackets)
{
  if (isnan(value)) {
    return malloc_stringf(nan_brackets ? "\"%s\"" : "%s", CONFIG_FORMAT_EMPTY);
  } else {
    if (value == 0.0) return malloc_stringf(format, CONFIG_FORMAT_NORTH);
    else if (value == 1.0) return malloc_stringf(format, CONFIG_FORMAT_NORTHEAST);
    else if (value == 2.0) return malloc_stringf(format, CONFIG_FORMAT_EAST);
    else if (value == 3.0) return malloc_stringf(format, CONFIG_FORMAT_SOUTHEAST);
    else if (value == 4.0) return malloc_stringf(format, CONFIG_FORMAT_SOUTH);
    else if (value == 5.0) return malloc_stringf(format, CONFIG_FORMAT_SOUTHWEST);
    else if (value == 6.0) return malloc_stringf(format, CONFIG_FORMAT_WEST);
    else if (value == 7.0) return malloc_stringf(format, CONFIG_FORMAT_NORTHWEST);
    else return malloc_stringf(format, CONFIG_FORMAT_EMPTY);
  };
}

FXJT485::FXJT485(uint8_t eventId):rSensorX1(eventId)
{
  _uartPort = 0;
  _uartRx = GPIO_NUM_NC;
  _uartTx = GPIO_NUM_NC;
}

void FXJT485::createSensorItems(const sensor_filter_t filterMode, const uint16_t filterSize)
{
  // Temperature
  _item = new rDirectionItem(this, CONFIG_SENSOR_WINDD_NAME,
    filterMode, filterSize,
    CONFIG_FORMAT_WINDD_VALUE, CONFIG_FORMAT_WINDD_STRING
    #if CONFIG_SENSOR_TIMESTAMP_ENABLE
    , CONFIG_FORMAT_TIMESTAMP_L
    #endif // CONFIG_SENSOR_TIMESTAMP_ENABLE
    #if CONFIG_SENSOR_TIMESTRING_ENABLE  
    , CONFIG_FORMAT_TIMESTAMP_S, CONFIG_FORMAT_TSVALUE
    #endif // CONFIG_SENSOR_TIMESTRING_ENABLE
  );
  if (_item) {
    rlog_d(_name, RSENSOR_LOG_MSG_CREATE_ITEM, _item->getName(), _name);
  };
}

// Dynamically creating internal items on the heap
bool FXJT485::initIntItems(const char* sensorName, const char* topicName, const bool topicLocal,  
  uart_port_t uartPort, int16_t uartRx, int16_t uartTx,
  const sensor_filter_t filterMode, const uint16_t filterSize,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _uartPort = uartPort;
  _uartRx = uartRx;
  _uartTx = uartTx;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Initialize internal items
  if (this->rSensorX1::initSensorItems(filterMode, filterSize)) {
    // Start device
    return sensorStart();
  };
  return false;
}

// Connecting external previously created items, for example statically declared
bool FXJT485::initExtItems(const char* sensorName, const char* topicName, const bool topicLocal,
  uart_port_t uartPort, int16_t uartRx, int16_t uartTx,
  rSensorItem* item,
  const uint32_t minReadInterval, const uint16_t errorLimit,
  cb_status_changed_t cb_status, cb_publish_data_t cb_publish)
{
  _uartPort = uartPort;
  _uartRx = uartRx;
  _uartTx = uartTx;
  // Initialize properties
  initProperties(sensorName, topicName, topicLocal, minReadInterval, errorLimit, cb_status, cb_publish);
  // Assign items
  this->rSensorX1::setSensorItems(item);
  // Start device
  return sensorStart();
}

void FXJT485::registerItemsParameters(paramsGroupHandle_t parent_group)
{
  if (_item) {
    _item->registerParameters(parent_group, CONFIG_SENSOR_WINDD_KEY, CONFIG_SENSOR_WINDD_NAME, CONFIG_SENSOR_WINDD_FRIENDLY);
  };
}

sensor_status_t FXJT485::sensorReset()
{
  return convertEspError(initModbus());
}

esp_err_t FXJT485::initModbus()
{
  if (_modbus == nullptr) {
    rlog_i(logTAG, "Initialize and start Modbus controller");

    // Init Modbus
    RE_ERROR_CHECK(mbc_master_init(MB_PORT_SERIAL_MASTER, &_modbus));
    // Configure Modbus
    mb_communication_info_t comm;
    memset(&comm, 0, sizeof(comm));
    comm.mode = MB_MODE_RTU;
    comm.slave_addr = 0x01;
    comm.port = _uartPort;
    comm.baudrate = 4800;
    comm.parity = MB_PARITY_NONE;
    RE_ERROR_CHECK(mbc_master_setup((void*)&comm));
    // Set UART pins
    RE_ERROR_CHECK(uart_set_pin(_uartPort, _uartTx, _uartRx, -1, -1));
    // Start Modbus
    RE_ERROR_CHECK(mbc_master_start());
    // Set UART driver mode to Half Duplex
    RE_ERROR_CHECK(uart_set_mode(_uartPort, UART_MODE_RS485_HALF_DUPLEX));
    vTaskDelay(5);
    RE_ERROR_CHECK(mbc_master_set_descriptor(device_parameters, 1));
  };

  return ESP_OK;
}

sensor_status_t FXJT485::readRawData()
{
  uint16_t value;
  mb_param_request_t _request = {
    .slave_addr = 0x01,
    .command    = 0x03, // MB_FUNC_READ_HOLDING_REGISTER
    .reg_start  = 0x0000,
    .reg_size   = 0x0002
  };
  esp_err_t err = mbc_master_send_request(&_request, &value);
  /***************************************************************************
  const mb_parameter_descriptor_t* param_descriptor = nullptr;
  esp_err_t err = mbc_master_get_cid_info(0, &param_descriptor);
  if ((err == ESP_OK) && (param_descriptor != nullptr)) {
    uint8_t type = 0;
    err = mbc_master_get_parameter(0, (char*)param_descriptor->param_key, (uint8_t*)&value, &type);
  };
  ***************************************************************************/
  if (err != ESP_OK) {
    rlog_e(logTAG, RSENSOR_LOG_MSG_READ_DATA_FAILED, _name, err, esp_err_to_name(err));
    return convertEspError(err);
  };
  return setRawValues((value_t)value);
}

