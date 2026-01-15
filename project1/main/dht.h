// Interface to the DHT22 temperature and humidity sensor
#pragma once

#include <esp_err.h>

// initialize the sensor and software interface
esp_err_t dhtInit();

// initiate a new measurement
esp_err_t dhtStartMeasurement();

// get the results of a measurement
esp_err_t dhtGetResults(double *temperature, double *humidity);