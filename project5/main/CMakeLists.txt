idf_component_register(
    SRCS "main.c" "dht.c" "webserver.c"
    INCLUDE_DIRS "."
    REQUIRES esp_timer esp_driver_gpio esp_wifi esp_http_server nvs_flash
)