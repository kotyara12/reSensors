# Driver for TE-Gorynych adapter

TE-Gorynych adapter driver (http://romram.ru/gorynych/TE-GORYNYCH_0004.pdf) for ESP32-based devices running FreeRTOS.
The driver serves only one connected sensor! That is, for each DS18B20 connected to the adapter, you should use its own copy of the driver.

