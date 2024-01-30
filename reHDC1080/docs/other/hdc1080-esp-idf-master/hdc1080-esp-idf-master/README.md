
# Driver for the TI HDC1080 temperature and humidty sensor.

The driver is for the usage with the ESP8266 and [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos). 

It is also working with ESP32 and [ESP-IDF](https://github.com/espressif/esp-idf.git) using a wrapper component for ESP8266 functions, see folder ```components/esp8266_wrapper```, as well as Linux based systems using a wrapper library.

## About the sensor



## Measurement Process

### Sensor modes


## Measurement results


```
float temperature;
float humidity;

...
// get the results and do something with them
if (hdc1080_read(sensor, &temperature, &humidity))
{
    ...
}
...
```


## Usage

First, the hardware configuration has to be established.

### Hardware configurations

Following figure shows the hardware configuration for ESP8266 and ESP32 if no interrupt is used.

```
  +------------------+       +---------+
  | ESP8266 / ESP32  |       | HDC1080 |
  |                  |       |         |
  |    GPIO 14 (SCL) >-------> SCL     |
  |    GPIO 13 (SDA) <-------> SDA     |
  |                  |       |         |
  +------------------+       +---------+
```

### Communication interface settings

Dependent on the hardware configuration, the communication interface settings have to be defined.

```
// define I2C interfaces at which HDC1080 sensors can be connected
#define I2C_BUS       0
#define I2C_SCL_PIN   14
#define I2C_SDA_PIN   13

```

### Main program

Before using the HDC1080 driver, function ```i2c_init``` needs to be called for each I2C interface to setup them.


Once I2C interfaces to be used are initialized, function ```hdc1080_init_sensor``` has to be called for each HDC1080 sensor to initialize the sensor and to check its availability as well as its error state. The parameters specify the I2C bus to which it is connected and its I2C slave address.

```
static hdc1080_sensor_t* sensor;    // pointer to sensor device data structure
...
if ((sensor = hdc1080_init_sensor (I2C_BUS, HDC1080_I2C_ADDRESS)))
{
    ...
}
...
```

Function ```hdc1080_init_sensor``` returns a pointer to the sensor device data structure or NULL in case of error.

If initialization of the sensor was successful, the sensor mode has be set to start periodic measurement. The sensor mode can be changed anytime later.

Finally, a user task that uses the sensor has to be created.

```
xTaskCreate(user_task, "user_task", 256, NULL, 2, 0);
```

**Please note:** To avoid concurrency situations when driver functions are used to access the sensor, for example to read data, the user task must not be created until the sensor configuration is completed.

If new data are fetched **periodically** the implementation of the user task is quite simply and could look like following.

```
void user_task(void *pvParameters)
{
    float temperature;
    float humidity;

    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        // get the results and do something with them
        if (hdc1080_read(sensor, &temperature, &humidity))
            ...
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
    }
}
...
```

The user task simply fetches new data with the same rate as the measurements are performed.

**Please note:** The rate of fetching the measurement results must be not greater than the rate of periodic measurements of the sensor, however, it *should be less* to avoid conflicts caused by the timing tolerance of the sensor.

## Full Example

```
/**
 * Simple example with one sensor connected to I2C bus 0. 
 *
 * Harware configuration:
 *
 *   +-----------------+   +-----------+
 *   | ESP8266 / ESP32 |   | HDC1080   |
 *   |                 |   |           |
 *   |   GPIO 14 (SCL) ----> SCL       |
 *   |   GPIO 13 (SDA) <---> SDA       |
 *   +-----------------+   +-----------+
 */

/* -- use following constants to define the example mode ----------- */

/* -- includes ----------------------------------------------------- */

#include "hdc1080.h"

/* -- platform dependent definitions ------------------------------- */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 2048

#else  // ESP8266 (esp-open-rtos)

// user task stack depth for ESP8266
#define TASK_STACK_DEPTH 256

#endif  // ESP_PLATFORM

// I2C interface defintions for ESP32 and ESP8266
#define I2C_BUS       0
#define I2C_SCL_PIN   14
#define I2C_SDA_PIN   13
#define I2C_FREQ      I2C_FREQ_100K



/* -- user tasks --------------------------------------------------- */

static hdc1080_sensor_t* sensor;



/*
 * In this example, user task fetches the sensor values every seconds.
 */

void user_task_periodic(void *pvParameters)
{
    float temperature;
    float humidity;

    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {

        // get the results and do something with them
        if (hdc1080_read(sensor, &temperature, &humidity))
            printf("%.3f HDC1080 Sensor periodic: temperature %.2f degrees, humidity %.2f\n",
                   (double)sdk_system_get_time()*1e-3, temperature, humidity);

        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
    }
}


/* -- main program ------------------------------------------------- */

void user_init(void)
{
    // Set UART Parameter.
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);

    /** -- MANDATORY PART -- */

    // init all I2C bus interfaces at which HDC1080 sensors are connected
    i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    

    // init the sensor with slave address HDC1080_I2C_ADDR connected I2C_BUS.
    sensor = hdc1080_init_sensor (I2C_BUS, HDC1080_ADDR);

    if (sensor)
    {
        printf("Initialized HDC1080 sensor: manufacurer %x, device id %x\n", hdc1080_get_manufacturer_id(sensor), hdc1080_get_device_id(sensor));
        // create a periodic task that uses the sensor
        xTaskCreate(user_task_periodic, "user_task_periodic", TASK_STACK_DEPTH, NULL, 2, NULL);

    }
    else
        printf("Could not initialize HDC1080 sensor\n");
}

```

## Thanks / Credits
Created the driver with the help of source code from:
https://github.com/gschorcht/ccs811-esp-idf
https://github.com/closedcube/ClosedCube_HDC1080_Arduino
