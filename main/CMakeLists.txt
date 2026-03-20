idf_component_register(
    SRCS        "main.c"
                "typedefs.c"
                "../BSP/bsp.c"
                "../BSP/bleprph.c"
                "../BSP/dataprph.c"
                "../BSP/flashstorage.c"
    INCLUDE_DIRS "."
                "../BSP/"
)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=deprecated-declarations" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=format" APPEND)