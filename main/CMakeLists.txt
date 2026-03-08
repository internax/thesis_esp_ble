set(srcs "main.cpp")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES ble_nrf_comm uart_protocol common nvs_flash)
