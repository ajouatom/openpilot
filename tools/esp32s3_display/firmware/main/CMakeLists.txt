idf_component_register(SRCS "usb_bridge.c" "network.c" "provision.c" "display_status.c"
  INCLUDE_DIRS "." REQUIRES usb esp_timer esp_wifi esp_event esp_netif esp_psram nvs_flash
  lwip esp_driver_uart esp_driver_gpio esp_http_server json mbedtls)
