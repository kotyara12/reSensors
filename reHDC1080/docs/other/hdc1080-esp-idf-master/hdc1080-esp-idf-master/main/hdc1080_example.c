/**
 * Simple example with one sensor connected to I2C bus 0. It demonstrates the
 * different approaches to fetch the data. Either the interrupt *nINT* is used
 * whenever new data are available or exceed defined thresholds or the new
 * data are fetched periodically.
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

        // printf("%.3f HDC1080 Sensor single: temperature %.2f degrees, humidity %.2f\n",
        //            (double)sdk_system_get_time()*1e-3, hdc1080_get_temperature(sensor), hdc1080_get_humidity(sensor));
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
    //hdc1080_set_resolution(sensor, hdc1080_8bit, hdc1080_8bit);

    if (sensor)
    {
        hdc1080_registers_t registers = hdc1080_get_registers(sensor);
        printf("Initialized HDC1080 sensor: manufacurer 0x%x, device id 0x%x\n", hdc1080_get_manufacturer_id(sensor), hdc1080_get_device_id(sensor));
        printf("Config: %x\n", registers.raw);
        registers.acquisition_mode = 1;
        hdc1080_set_registers(sensor, registers);
        // create a periodic task that uses the sensor
        xTaskCreate(user_task_periodic, "user_task_periodic", TASK_STACK_DEPTH, NULL, 2, NULL);

    }
    else
        printf("Could not initialize HDC1080 sensor\n");
}

