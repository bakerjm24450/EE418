// Interface to the DHT22 temperature and humidity sensor
#pragma once

// task function for controlling DHT sensor
void dhtTask(void *args);

// return latest temperature reading
double dhtGetTemperature();

// return latest humidity reading
double dhtGetHumidity();