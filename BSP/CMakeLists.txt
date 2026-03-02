# BSP组件的CMakeLists.txt文件

idf_component_register(SRCS "gps_module.c" "gps_interrupt.c"
					PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer log hal esp_hw_support 
                    INCLUDE_DIRS ".")
