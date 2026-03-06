idf_component_register(
    SRCS "test_crc8.cpp"
          "test_encode_decode.cpp"
    INCLUDE_DIRS "."
    REQUIRES unity uart_protocol
)
