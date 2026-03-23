set(srcs "main.cpp")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES nfc_pn532 uart_protocol ble_nrf_comm common nvs_flash led_indicator)
