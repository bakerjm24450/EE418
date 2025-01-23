// Interface to the DHT22 temperature and humidity sensor

#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

#include "dht.h"

// for logging messages
static const char* TAG = "DHT";
