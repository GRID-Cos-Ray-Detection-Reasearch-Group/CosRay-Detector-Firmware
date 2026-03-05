idf_component_register(
    SRCS "Dac.c"
    REQUIRES esp_driver_gpio esp_driver_uart esp_timer log hal esp_hw_support  esp_driver_spi
    INCLUDE_DIRS "."
)
