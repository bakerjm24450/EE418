// Interface to the DHT22 temperature and humidity sensor
#pragma once

// Initialize DHT sensor and start periodic readings
esp_err_t dhtInit();

// return latest temperature reading
double dhtGetTemperature();

// return latest humidity reading
double dhtGetHumidity();