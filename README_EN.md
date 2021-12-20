# reSensors

This repository contains drivers for various sensors adapted for the [Espressif ESP32 ESP-IDF framework](https://github.com/espressif/esp-idf). For more details see the README in the sensor subfolders. ***I apologize: not all sensors have descriptions at the moment - there is a sorely lack of time for that.***

**All sensor drivers are descendants of the [rSensor class](https://github.com/kotyara12/reSensors/tree/master/reSensor)**, which provides the following functionality:
- Monitoring the status of the sensor (norm, error, timeout, CRC error) with the ability to notify the user about a change in state through a callback.
- Built-in customizable data filters with the ability to change the buffer size. Currently implemented:
    - SENSOR_FILTER_RAW: no filtering, default
    - SENSOR_FILTER_AVERAGE: average
    - SENSOR_FILTER_MEDIAN: median
- Fixing the minimum and maximum values: for a day, a week, for all the time since the last restart of the device.
- Generation of JSON data packet. This package will include: sensor status, current values, measurement time, minimum and maximum values.
- Publishing data on an MQTT broker (using a callback): in a simple form and in several topics or in JSON format in one topic.
- Support for saving filtering settings in NVS and automatic subscription to these parameters via MQTT.

For more details see the description of the [rSensor class](https://github.com/kotyara12/reSensors/tree/master/reSensor).


## Notes
These comments refer to my libraries hosted on the resource https://github.com/kotyara12?tab=repositories.

- libraries whose name starts with the **re** prefix are intended only for ESP32 and ESP-IDF (FreeRTOS)
- libraries whose name begins with the **ra** prefix are intended only for ARDUINO
- libraries whose name starts with the **r** prefix can be used for both ARDUINO and ESP-IDF

Since I am currently developing programs mainly for ESP-IDF, the bulk of my libraries are intended only for this framework. But you can port them to another system using.


## License
This library is free software; you can redistribute and / or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful, but _WITHOUT ANY WARRANTY_; _not even without the implied warranties of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE_. See the GNU Lesser General Public License for details.