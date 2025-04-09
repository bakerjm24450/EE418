# EE 418 Project 7

Building on the previous project, this system is a complete smart thermostat with
logging capabilities. It uses a DHT22 to measure the ambient temperature and humidity,
and displays information on a HX8357 TFT display. It maintains a daily schedule, 
controlling a temperature setpoint across four distinct time periods. It also logs
the temperature readings over the last 24 hours.

The system also runs a simple webserver that displays the current conditions, the
previous 24 hours of data, and allows the user to change the daily schedule.

To configure your system, run menuconfig and update the EE 418 Project Configuration entries as needed. 
In particular, be sure to update
- WiFi SSID
- WiFi password
- mDNS hostname
- DHT22 GPIO pin
- Thermostat GPIO pin

As you add .c files to the project, you need to include them in main/CMakeLists.txt for the
project to build correctly.
