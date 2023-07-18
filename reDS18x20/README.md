# Драйвер датчиков температуры семейства Maxim (Dallas) DS18x20 для ESP32 / ESP-IDF

> If you do not understand this text, please use the English version: <br/>https://github.com/kotyara12/reDS18x20/blob/master/README_EN.md <br/>Sorry for the machine translation into English.

Данная библиотека представляет собой класс для работы с датчиками температуры семейства DS18x20 производства Maxim (ранее Dallas), предназначенный только для [Espressif ESP32 ESP-IDF framework](https://github.com/espressif/esp-idf). В качестве транспорта использована слегка модифицированная версия библиотеки [onewire от UncleRus](https://github.com/UncleRus/esp-idf-lib/tree/master/components/onewire) (добавлена возможность чтения 1 бита для ожидания окончания измерения и проверки режима паразитного питания). 

## Возможности
- Поиск датчиков на шине. Если на шине подключен только один датчик DS18x20, нет необходимости указывать его адрес - драйвер сам найдет его при старте. Даже если датчиков на одной и той же шине несколько, это не мешает определить их так же по индексу.
- Поиск датчиков по указанному адресу. Если Вам известен адрес датчика, можно сразу указать его при инициализации. Это дает уверенность в том, что Вы получите данные с нужного датчика, если их несколько.
- Поддержка паразитного питания. Определяется автоматически при инициализации.

Возможности, реализованные в классе-предке [rSensor](https://github.com/kotyara12/reSensors/tree/master/rSensor):

- Поддержка фильтров значений: среднее и медиана с возможностью указания размера буфера. По умолчанию отключен (RAW).
- Фиксация минимальных и максимальных значений: за сутки, неделю, всё время.
- Генерация данных в виде JSON-строки. В том числе текущие значения, время измерения, минимальные и максимальные значения.
- Публикация данных на MQTT-брокере в простом виде в нескольких топиках или в JSON-формате в одном топике.
- Поддержка сохранения настроек фильтрации в NVS и автоматическая подписка на эти параметры через MQTT.

Для более подробной информации смотрите описание класса [rSensor](https://github.com/kotyara12/reSensors/tree/master/rSensor).

## Использование

**Для создания экземпляра класса просто объявите его**, например так:
```
DS18x20 sensorBoiler;
```
Для конструктора класса параметры не требуются.

**Затем необходимо указать параметры сенсора и создать "хранилище" значений** (class reSensorItem) для хранения и обработки данных температуры. Сделать это можно двумя способами: динамическим (экземпляр хранилища будет создан и размещен в куче) и статическим (вы должны самостоятельно создать экземпляр reSensorItem и передать его методу инициализации).

**Динамическая инициализация**:
```
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
```

**Статическая инициализация**:
```
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
```

#### Параметры
- ```const char* sensorName``` - Условное имя сенсора
- ```const char* topicName``` - _Субтопик_ MQTT для публикации значений. Это только часть полного топика, который по умолчанию выглядит как ```%prefix%/%location%/%device%/%topicName%```, однако это можно изменить с помощью настроек проекта.
- ```bool topicLocal``` - Генерировать локальный топик для публикции на MQTT-брокере. В моих проектах существует возможность генерировать "локальные" и "публичные" топики. Локальные топики не выходят за пределы брокера, расположенного в локальной сети (обычно на роутере), и не доступны извне. Публичные топики перебрасываются локальным брокером с помощью моста на внешний облачный брокер и доступны везде, где есть интернет. По умолчанию false.
- ```gpio_num_t pin``` - номер вывода MCU, к которому подключена 1-Wire шина с датчиком или датчиками.
- ```onewire_addr_t address``` - Адрес датчика, если он вам известен. Если адрес датчика не известен, укажите ```ONEWIRE_NONE```.
- ```int8_t index``` - индекс для поиска датчика на шине. Укажите значение от 1 и выше, если вы хотите найти датчик с незвестным адресом.
- ```DS18x20_RESOLUTION resolution``` - Разрешение датчика. Может принимать значения ```DS18x20_RESOLUTION_9_BIT```, ```DS18x20_RESOLUTION_10_BIT```, ```DS18x20_RESOLUTION_11_BIT``` и ```DS18x20_RESOLUTION_12_BIT```.
- ```bool saveScratchPad``` - сохранять параметры датчика (разрешение и граничные температуры) после их изменения в EEPROM датчика.
- ```uint32_t minReadInterval``` - Минимальный интервал опроса датчика в миллисекундах. Если опрашивать датчик чаще заданного здесь интервала,  то реального чтения датчика происходить не будет, а будет возвращаться ранее считанное значение. Эта функция позволяет избегать саморазогрева сенсоров от слишком частого опроса (некоторые типы сенсоров склонны к этому).
- ```uint16_t errorLimit``` - Количество ошибок подряд, после чего будет изменен статус датчика. Значение больше 1 позволяет "проглатывать" одиночные помехи на шинах и, соответственно, ошибки (обычно в таких случаях бывает ошибка CRC).
- ```cb_status_changed_t cb_status``` - Функция обратного вызова при изменении статуса (ошибка, норма и т.д.). Может использоваться для уведомления пользователя о нештатных ситуациях.
- ```cb_publish_data_t cb_publish``` - Функция обратного вызова для периодической публикции данных на MQTT-брокере.

При инициализации будет автоматически перезапущен и сам датчик, считан тип питания и задано размешение. После этого можно опрашивать датчик и получать с него данные.


**Пример динамической инициализации**

Функция обратного вызова:
```
static bool sensorsPublish(rSensor *sensor, char* topic, char* payload, const bool free_topic, const bool free_payload)
{
  return mqttPublish(topic, payload, 0, true, false, free_topic, free_payload);
}
```
Создание и инициализация сенсора:
```
DS18x20 sensorBoiler;
sensorBoiler.initIntItems("boiler", "boiler", false,
  GPIO_NUM_16, ONEWIRE_NONE, 1, DS18x20_RESOLUTION_12_BIT, true, 
  SENSOR_FILTER_RAW, 0, 0, 0, nullptr, sensorsPublish);
```


### Чтение температуры
Для опроса датчика используйте метод ```readData()```. Считанные с датчика значения будут помещены в хранилище, и их можно будет прочитать с помощью различных методов (```getValue```, ```getExtremums...``` и других). Например так:
```
sensorBoiler.readData();
if (sensorBoiler.getStatus() == SENSOR_STATUS_OK) {
  rlog_i("BOILER", "Values raw: %.2f °С | out: %.2f °С | min: %.2f °С | max: %.2f °С", 
    sensorBoiler.getValue(false).rawValue, 
    sensorBoiler.getValue(false).filteredValue,
    sensorBoiler.getExtremumsDaily(false).minValue.filteredValue,
    sensorBoiler.getExtremumsDaily(false).maxValue.filteredValue);
  sensorBoiler.publishData(false);
};
```


## Известные проблемы
- Пока не реализован механизм установки критических (граничных) температур и чтения тревог.

## Зависмости
- https://github.com/kotyara12/rLog
- https://github.com/kotyara12/reSensors/tree/master/rSensor

## Примечания
Данные замечания относятся к моим библиотекам, размещенным на ресурсе https://github.com/kotyara12?tab=repositories.

- библиотеки, название которых начинается с префикса **re**, предназначены только для ESP32 и ESP-IDF (FreeRTOS)
- библиотеки, название которых начинается с префикса **ra**, предназначены только для ARDUINO
- библиотеки, название которых начинается с префикса **r**, могут быть использованы и для ARDUINO, и для ESP-IDF

Так как я в настроящее время разрабатываю программы в основном для ESP-IDF, основная часть моих библиотек предназначена только для этого фреймворка. Но Вы можете портировать их для другой системы, взяв за основу.
