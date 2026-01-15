#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_check.h>

static const char *TAG = "HW8";

static const gpio_num_t RED_LED_PIN = CONFIG_RED_LED_GPIO;     // Red LED pin number
static const gpio_num_t GREEN_LED_PIN = CONFIG_GREEN_LED_GPIO; // Green LED pin number



void app_main(void)
{
    // FIXME -- add your code here
}