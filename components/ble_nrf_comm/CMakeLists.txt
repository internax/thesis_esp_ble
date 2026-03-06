idf_component_register(SRCS "nrf_ble.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES bt common
                    PRIV_REQUIRES esp_timer nvs_flash)
