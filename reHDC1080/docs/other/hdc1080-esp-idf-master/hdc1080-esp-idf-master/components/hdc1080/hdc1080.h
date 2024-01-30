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


#ifndef _HDC1080_h

#define _HDC1080_h

#include "hdc1080_platform.h"

#include "stdint.h"
#include "stdbool.h"


#ifdef __cplusplus
extern "C"
{
#endif

#define HDC1080_ADDR 0x40

typedef enum {
	hdc1080_8bit = 0x02,
	hdc1080_11bit = 0x01,
	hdc1080_14bit = 0x00
} hdc1080_measurement_bitwidth_t;

typedef union {
	uint8_t raw[6];
	struct {
		uint16_t serialFirst;
		uint16_t serialMid;
		uint16_t serialLast;
	};
} hdc1080_serial_t;

typedef union {
	uint16_t raw;
	struct {
		uint8_t humidity_bitwidth : 2;
		uint8_t temperature_bitwidth : 1;
		uint8_t battery_status : 1;
		uint8_t acquisition_mode : 1;
		uint8_t heater : 1;
		uint8_t reserved_again : 1;
		uint8_t software_reset : 1;
        uint8_t unused;
	};
} hdc1080_registers_t;

typedef struct {
    uint8_t bus;
    uint8_t address;
} hdc1080_sensor_t;

hdc1080_sensor_t* hdc1080_init_sensor (uint8_t bus, uint8_t addr);

uint16_t hdc1080_get_manufacturer_id(hdc1080_sensor_t* sensor); // 0x5449 ID of Texas Instruments
uint16_t hdc1080_get_device_id(hdc1080_sensor_t* sensor); // 0x1050 ID of the device

hdc1080_registers_t hdc1080_get_registers(hdc1080_sensor_t* sensor);	
void hdc1080_set_registers(hdc1080_sensor_t* sensor, hdc1080_registers_t registers);

hdc1080_serial_t hdc1080_get_serial(hdc1080_sensor_t* sensor);

void hdc1080_heat_up(hdc1080_sensor_t* sensor, uint8_t seconds);

void hdc1080_set_resolution(hdc1080_sensor_t* sensor, hdc1080_measurement_bitwidth_t humidity, hdc1080_measurement_bitwidth_t temperature);

float hdc1080_get_temperature(hdc1080_sensor_t* sensor);
float hdc1080_get_humidity(hdc1080_sensor_t* sensor);
bool hdc1080_read(hdc1080_sensor_t* sensor, float* temperature, float* humidity);

#ifdef __cplusplus
}
#endif 

#endif