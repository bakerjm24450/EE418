# EE 418 Project 4

Refining our design from Projects 2 and 3, in this project we use a FreeRTOS timer to trigger the temperature and humidity measurements every 2 seconds.

## Implementation details
Replace the 2-second delay in your DHT task with a FreeRTOS timer. Use another queue for synchronization of the timer callback and the task -- the timer callback will send a value to the queue, and the task can wait at the queue rather than using vTaskDelay.

## Files supplied to you

- webserver.h / webserver.c contains the necessary functions for connecting to WiFi and running the webserver. There is one public function, webServerInit(), that must be called from app_main() to start the webserver. These files are identical to Projects 2 and 3, and you do not need to make any changes to webserver.h or webserver.c

- Kconfig.projbuild defines some configuration parameters that can be modified using menuconfig. 

- idf_component.yml is used to specify the mDNS library that is used. mDNS allows us to connect to the webserver using a hostname instead of an IP address

## Files you need to provide
Copy the following files from Project 3 as a starting point (replacing the stub files that are provided)
- main.c
- dht.h / dht.c
