idf_component_register(
    SRCS "main.c" "uart1.c" "wifi.c" "i2c_chipset.c" "adc.c" "bluetooth.c"
    PRIV_REQUIRES esp_wifi nvs_flash esp_netif esp_event driver bt driver esp_timer esp_adc
    INCLUDE_DIRS "."
)
