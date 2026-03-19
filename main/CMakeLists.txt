set(srcs "main.cpp")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES uart_protocol common nvs_flash)
