idf_component_register(SRCS "pn532.cpp" "nfc_pairing.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES driver common uart_protocol ble_nrf_comm esp_timer)
