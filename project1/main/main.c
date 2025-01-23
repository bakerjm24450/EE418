#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include <esp_log.h>

#include "dht.h"

// for logging messages
static const char* TAG = "Project 1";

void app_main(void)
{
    double humidity;
    double temperature;
    esp_err_t result;

    // initialize dht sensor
    dhtInit();

    for (;;) {
        // delay 2 sec between measurements
        vTaskDelay(pdMS_TO_TICKS(2000));

        // start a measurement
        ESP_LOGD(TAG, "Starting measurement");
        dhtStartMeasurement();

        // wait for completion
        if ((result = dhtGetResults(&temperature, &humidity)) == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %gF, humidity: %g%%", temperature, humidity);
        }
        else if (result == ESP_ERR_TIMEOUT) {
            ESP_LOGI(TAG, "Timeout communicating with DHT22 sensor");
        }
        else {
            ESP_LOGI(TAG, "Checksum mismatch with DHT22 sensor");
        }
    }

}