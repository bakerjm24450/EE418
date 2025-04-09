
#include "display.h"
#include "webserver.h"

#include "esp_log.h"

static const char* TAG = "Project 7";

void app_main(void)
{
    // FIXME -- initialize the other subsystems

    // initialize wifi and webserver
    webServerInit();
    
    // initialize LCD display
    displayInit();
}