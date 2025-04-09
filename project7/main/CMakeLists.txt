idf_component_register(SRCS "main.c" "display.c" "webserver.c" 
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_spi esp_driver_gpio esp_wifi esp_lcd esp_http_server nvs_flash
                    )