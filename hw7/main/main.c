#include <stdio.h>
#include <string.h>
#include <mbedtls/md.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>

static const char *TAG = "HW7";

// Task to just waste some CPU time
void task(void *args)
{
   int id = (int)args; // get my ID number

   printf("Task %d starting...", id);

   char key[256];
   sprintf(key, "Kwyjibo%d", id);
   char *payload = "Mmm, doughnuts...";
   uint8_t hmacResult[32];

   mbedtls_md_context_t ctx;
   mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
   const size_t payloadLength = strlen(payload);
   const size_t keyLength = strlen(key);

   for (;;)
   {
      // calculate SHA-256 hashn (model a computational load)
      mbedtls_md_init(&ctx);
      mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
      mbedtls_md_hmac_starts(&ctx, (const unsigned char *)key, keyLength);
      mbedtls_md_hmac_update(&ctx, (const unsigned char *)payload, payloadLength);
      mbedtls_md_hmac_finish(&ctx, hmacResult);
      mbedtls_md_free(&ctx);

      // Print out results
      printf("Task %d result = ", id);
      for (int i = 0; i < 32; i++)
      {
         printf("%02x", hmacResult[i]);
      }
      printf("\n");

      // delay to allow other tasks to run
      vTaskDelay(pdMS_TO_TICKS(10));
   }
}

void app_main()
{
   setbuf(stdout, NULL);

   // create 4 tasks
   for (int i = 0; i < 4; i++)
   {
      xTaskCreatePinnedToCore(
          task,
          "Task",
          10000,
          (void *)i, // task ID number
          3,
          NULL,
          1);
   }
}