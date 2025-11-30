idf_component_register(
    SRCS        "main.c"
                "typedefs.c"
                "../BSP/bsp.c"
                "../BSP/bleprph.c"
                "../BSP/dataprph.c"
    INCLUDE_DIRS "."
                "../BSP/"
)
