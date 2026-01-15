# EE 418 Project 6

Refining our design from Projects 2 - 5, in this project we use a task notification for synchronization between the timer callback function and the task.

## Implementation details
Replace the queue that is used for synchronization between the timer and the task with a task notification.

## Files supplied to you

- webserver.h / webserver.c contains the necessary functions for connecting to WiFi and running the webserver. There is one public function, webServerInit(), that must be called from app_main() to start the webserver. These files are identical to the eariler projects, and you do not need to make any changes to webserver.h or webserver.c

- Kconfig.projbuild defines some configuration parameters that can be modified using menuconfig. 

- idf_component.yml is used to specify the mDNS library that is used. mDNS allows us to connect to the webserver using a hostname instead of an IP address

## Files you need to provide
Copy the following files from Project 5 as a starting point (replacing the stub files that are provided)
- main.c
- dht.h / dht.c
