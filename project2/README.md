# EE 418 Project 2

Building on Project 1, for this project we create a FreeRTOS task to manage the DHT22
temperature / humidity sensor. The task handles the necessary initialization, then takes
a measurement every 2 seconds.

We include two functions that return the most recent measured temperature (Fahrenheit) and
relative humidity.

We also include a simple webserver that displays a webpage showing the current temperature
and humidity. The webserver requires that the ESP32 be connected to a WiFi network.

## Implementation details

The interface to the DHT22 sensor is similar to Project 1. However, there are two changes
that must be made:
- Write a task function that will manage the DHT22. This function should call dhtInit() first, and
then in an infinite loop, it should take a measurement every 2 seconds. Note that this is almost identical to the app_main() function from Project 1.
- Write functions to return the most recent measured temperature and relative humidity. These
functions will be used by the webserver to display the current readings

The code for the webserver is supplied to you and does not need any modifications. The process of
connecting to a WiFi access point and running the webserver is no-trivial, but does not involve FreeRTOS for the most part so you are not being asked to develop the code from scratch.

Note that you will need to run menuconfig to specify the WiFi SSID and password (that should only need to be done one time).

## Files supplied to you

- webserver.h / webserver.c contains the necessary functions for connecting to WiFi and running the webserver. There is one public function, webServerInit(), that must be called from app_main() to start the webserver. You do not need to make any changes to webserver.h or webserver.c

- Kconfig.projbuild defines some configuration parameters that can be modified using menuconfig. 

- idf_component.yml is used to specify the mDNS library that is used. mDNS allows us to connect to the webserver using a hostname instead of an IP address