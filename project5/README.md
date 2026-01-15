# EE 418 Project 5

Refining our design from Projects 2 - 4, in this project we use a FreeRTOS mutex to protect the global temperature and humidity variables.

## Implementation details
Modify the getTemperature() and getHumidity() functions to acquire the mutex before accessing the temperature and humidity values (it's ok to use the same mutex for both since they will tend to be accessed together).

We also need to add setter functions to update the values with mutex protection.

## Files supplied to you

- webserver.h / webserver.c contains the necessary functions for connecting to WiFi and running the webserver. There is one public function, webServerInit(), that must be called from app_main() to start the webserver. These files are identical to Projects 2 - 4 , and you do not need to make any changes to webserver.h or webserver.c

- Kconfig.projbuild defines some configuration parameters that can be modified using menuconfig. 

- idf_component.yml is used to specify the mDNS library that is used. mDNS allows us to connect to the webserver using a hostname instead of an IP address

## Files you need to provide
Copy the following files from Project 4 as a starting point (replacing the stub files that are provided)
- main.c
- dht.h / dht.c
