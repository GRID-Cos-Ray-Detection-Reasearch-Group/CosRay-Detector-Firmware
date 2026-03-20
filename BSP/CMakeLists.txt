idf_component_register(
    SRCS "dataprph.c" "gps_module.c" "gps_interrupt.c" "Dac.c" "Preamplifier.c"
    REQUIRES esp_driver_gpio esp_driver_uart esp_timer log hal esp_hw_support  esp_driver_spi esp_adc freertos
    INCLUDE_DIRS "."
)