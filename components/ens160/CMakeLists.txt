idf_component_register(
        SRCS ens160.c
        INCLUDE_DIRS .
        REQUIRES i2cdev log esp_idf_lib_helpers
)
