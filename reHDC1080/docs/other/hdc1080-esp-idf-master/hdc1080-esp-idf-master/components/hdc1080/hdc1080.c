/*
Arduino Library for Texas Instruments HDC1080 Digital Humidity and Temperature Sensor
Written by nikwest for esp32 idf
inspired by https://github.com/closedcube/ClosedCube_HDC1080_Arduino
---
The MIT License (MIT)
Copyright (c) 2016-2017 ClosedCube Limited
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <stdlib.h>

#include "hdc1080.h"


#define	HDC1080_TEMPERATURE		0x00
#define HDC1080_HUMIDITY		0x01
#define HDC1080_CONFIGURATION	0x02
#define HDC1080_MANUFACTURER_ID 0xFE
#define HDC1080_DEVICE_ID		0xFF
#define HDC1080_SERIAL_ID_FIRST	0xFB
#define HDC1080_SERIAL_ID_MID	0xFC
#define HDC1080_SERIAL_ID_LAST	0xFD

#define RAW2HUM(raw) (((float)raw) * 100/65536)
#define RAW2TEMP(raw) (((float)raw) * 165/65536 - 40)

static bool _hdc1080_is_available (hdc1080_sensor_t* dev);
static uint16_t _hdc1080_read(hdc1080_sensor_t* dev, uint8_t reg);
static bool _hdc1080_read_data(hdc1080_sensor_t* dev, uint8_t reg, uint8_t* data, uint32_t len, TickType_t delay);


hdc1080_sensor_t* hdc1080_init_sensor (uint8_t bus, uint8_t addr) {
    hdc1080_sensor_t* dev;
    
    if ((dev = malloc (sizeof(hdc1080_sensor_t))) == NULL) {
        return NULL;
    }

    // init sensor data structure
    dev->bus  = bus;
    dev->address = addr;

    if (!_hdc1080_is_available(dev)){
        free (dev);
        return NULL;
    }

    hdc1080_set_resolution(dev, hdc1080_14bit, hdc1080_14bit);

    return dev;
}

uint16_t hdc1080_get_manufacturer_id(hdc1080_sensor_t* sensor) {
    return _hdc1080_read(sensor, HDC1080_MANUFACTURER_ID);
}

uint16_t hdc1080_get_device_id(hdc1080_sensor_t* sensor){
    return _hdc1080_read(sensor, HDC1080_DEVICE_ID);
}

hdc1080_registers_t hdc1080_get_registers(hdc1080_sensor_t* sensor) {
    hdc1080_registers_t reg;
	reg.raw = _hdc1080_read(sensor, HDC1080_CONFIGURATION);
	return reg;
}	

void hdc1080_set_registers(hdc1080_sensor_t* dev, hdc1080_registers_t registers) {
    uint8_t reg = HDC1080_CONFIGURATION;
    if(i2c_slave_write(dev->bus, dev->address, &reg, (uint8_t*) &(registers.raw), sizeof(registers.raw))) {
        
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
}

hdc1080_serial_t hdc1080_get_serial(hdc1080_sensor_t* sensor);

void hdc1080_heat_up(hdc1080_sensor_t* sensor, uint8_t seconds);

void hdc1080_set_resolution(hdc1080_sensor_t* sensor, hdc1080_measurement_bitwidth_t humidity, hdc1080_measurement_bitwidth_t temperature) {
    hdc1080_registers_t reg;
	reg.humidity_bitwidth = humidity;
	reg.temperature_bitwidth = temperature;

	hdc1080_set_registers(sensor, reg);
}

float hdc1080_get_temperature(hdc1080_sensor_t* sensor) {
    uint16_t rawT = _hdc1080_read(sensor, HDC1080_TEMPERATURE);
	return RAW2TEMP(rawT);
}

float hdc1080_get_humidity(hdc1080_sensor_t* sensor) {
    uint16_t rawH = _hdc1080_read(sensor, HDC1080_HUMIDITY);
	return RAW2HUM(rawH);
}

bool hdc1080_read(hdc1080_sensor_t* sensor, float* temperature, float* humidity) {
    uint8_t raw[4];
    uint16_t result;
    if(!_hdc1080_read_data(sensor, HDC1080_TEMPERATURE, (uint8_t*) &raw, 4, 20)) {
        return false;
    }

    result = raw[0]<<8 | raw[1];
    *temperature = RAW2TEMP(result);
    result = raw[2]<<8 | raw[3];
    *humidity = RAW2HUM(result);
    return true;
}


// private

static bool _hdc1080_is_available(hdc1080_sensor_t* dev) {
    // TODO check device
    return true;
}

static uint16_t _hdc1080_read(hdc1080_sensor_t* dev, uint8_t reg) {
    uint8_t result[2];
    if(!_hdc1080_read_data(dev, reg, (uint8_t*) &result, 2, 10)) {
        return 0;
    }
    return  (uint16_t) (result[0] <<8) | result[1];
}

static bool _hdc1080_read_data(hdc1080_sensor_t* dev, uint8_t reg, uint8_t* data, uint32_t len, TickType_t delay) {
    if (!dev) return 0;

    if(i2c_slave_write(dev->bus, dev->address, &reg, NULL, 0)) {
        return false;
    }

    vTaskDelay(delay/portTICK_PERIOD_MS);

    if(i2c_slave_read(dev->bus, dev->address, NULL, data, len)) {
        return false;
    }

    return true;
}
