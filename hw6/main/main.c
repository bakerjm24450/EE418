#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_check.h>
#include <esp_random.h>
#include <mbedtls/md.h>
#include <string.h>

static const char *TAG = "HW6";

static const gpio_num_t RED_LED_PIN = CONFIG_RED_LED_GPIO;     // Red LED pin number
static const gpio_num_t GREEN_LED_PIN = CONFIG_GREEN_LED_GPIO; // Green LED pin number


// Task to just waste some CPU time
void wasteTimeTask(void *args)
{
    char *key = "Kwyjibo";
    char *payload = "Mmm, doughnuts...";
    uint8_t hmacResult[32];

    for (;;)
    {
        mbedtls_md_context_t ctx;
        mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

        const size_t payloadLength = strlen(payload);
        const size_t keyLength = strlen(key);

        int numreps = 1;
        // int numreps = 10;
        // int numreps = esp_random() % 13 + 3; // random between 3 and 15

        for (int i = 0; i < numreps; i++)
        {
            // Calculate a SHA-256 hash
            mbedtls_md_init(&ctx);
            mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
            mbedtls_md_hmac_starts(&ctx, (const unsigned char *)key, keyLength);
            mbedtls_md_hmac_update(&ctx, (const unsigned char *)payload, payloadLength);
            mbedtls_md_hmac_finish(&ctx, hmacResult);
            mbedtls_md_free(&ctx);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    // FIXME -- add your code here
}