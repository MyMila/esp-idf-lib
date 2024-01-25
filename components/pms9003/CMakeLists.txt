idf_component_register(
        SRCS pms9003.c
        INCLUDE_DIRS .
        REQUIRES log esp_idf_lib_helpers driver
)
