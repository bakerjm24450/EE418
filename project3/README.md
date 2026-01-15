# EE 418 Project 3

Refining our design from Project 2, in this project we use FreeRTOS queues to communicate between the ISR and the DHT task. Note that a queue can be used for both communication of data and for synchronization.

Using queues allows us to eliminate the global variables that were used for holding the data bits and the bit count for communicating with the DHT22 sensor. 

## Implementation details
The bitCount and dataBits variables should now be declared local to the ISR (but as static variables so that the values persist across different invocations of the ISR). One queue will be used to reset the bit count -- to start a new measurement, the DHT task shoudl write 0 to this queue.

A second queue is used to send the received data bits from the ISR to the task after all bits have been received.

## Files supplied to you

- webserver.h / webserver.c contains the necessary functions for connecting to WiFi and running the webserver. There is one public function, webServerInit(), that must be called from app_main() to start the webserver. These files are identical to Project 2, and you do not need to make any changes to webserver.h or webserver.c

- Kconfig.projbuild defines some configuration parameters that can be modified using menuconfig. 

- idf_component.yml is used to specify the mDNS library that is used. mDNS allows us to connect to the webserver using a hostname instead of an IP address

## Files you need to provide
Copy the following files from Project 2 as a starting point (replacing the stub files that are provided)
- main.c
- dht.h / dht.c
