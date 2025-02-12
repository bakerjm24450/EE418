// Interface to the DHT22 temperature and humidity sensor

#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

#include "dht.h"

static const char* TAG = "DHT";

static const gpio_num_t dhtPin = CONFIG_DHT22_GPIO;
