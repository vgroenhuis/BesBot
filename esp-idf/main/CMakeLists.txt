idf_component_register(SRCS "main.c" "display.c" "wheels.c" INCLUDE_DIRS "." EMBED_TXTFILES "page.html"
                       REQUIRES closer_demo esp_http_server nvs_flash driver esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_i2c u8g2 cjson esp_timer lwip esp_wifi)
