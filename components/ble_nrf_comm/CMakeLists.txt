idf_component_register(SRCS "nrf_ble.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES bt
                    PRIV_REQUIRES driver esp_timer nvs_flash)
