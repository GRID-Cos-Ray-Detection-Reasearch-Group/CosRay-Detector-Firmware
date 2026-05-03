idf_component_register(
    SRCS "main.c"
         "typedefs.c"
         "../BSP/bsp.c"
         "../BSP/bleprph.c"
         "../BSP/dataprph.c"
         "../BSP/flashstorage.c"
         "../BSP/gps_module.c"
         "../BSP/gps_interrupt.c"
         "../BSP/Dac.c"
         "../BSP/Preamplifier.c"
         "../BSP/acce.c"
    INCLUDE_DIRS "."
                 "../BSP/"
    REQUIRES esp_driver_gpio esp_driver_uart esp_timer log hal esp_hw_support
             esp_driver_spi esp_adc freertos nvs_flash bt esp_driver_i2c
)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=deprecated-declarations" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=format" APPEND)
