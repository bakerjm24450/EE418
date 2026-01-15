#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_check.h>

static const char* TAG = "HW3";

static const gpio_num_t INPUT_PIN = CONFIG_INPUT_GPIO;   // Input pin number
static const gpio_num_t OUTPUT_PIN = CONFIG_OUTPUT_GPIO;  // Output pin number


void app_main(void)
{
    // FIXME -- Your code goes here
}