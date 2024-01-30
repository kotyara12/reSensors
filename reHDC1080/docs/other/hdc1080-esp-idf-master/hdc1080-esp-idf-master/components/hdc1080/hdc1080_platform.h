/**
 * Platform file: platform specific definitions, includes and functions
 */

#ifndef __HDC1080_PLATFORM_H__
#define __HDC1080_PLATFORM_H__

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// platform specific includes
#include "esp8266_wrapper.h"
#include <errno.h>

#endif // ESP_PLATFORM

#endif // __CCS811_PLATFORM_H__
