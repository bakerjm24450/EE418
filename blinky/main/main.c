// Blink an LED

#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <esp_log.h>

// TAG is used for debug output
static const char *TAG = "Blinky";

// LED connected to pin 5
static const gpio_num_t led = GPIO_NUM_5;

void app_main(void)
{
    // configure gpio pin as output
    gpio_set_direction(led, GPIO_MODE_OUTPUT);

    for (;;)
    {
        // turn LED on
        gpio_set_level(led, 1);

        // print a debug log message
        ESP_LOGI(TAG, "LED on");

        // delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));

        // turn LED off and delay
        gpio_set_level(led, 0);
        ESP_LOGI(TAG, "LED off");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}