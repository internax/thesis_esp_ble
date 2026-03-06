cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(COMPONENTS "main uart_protocol common")
set(EXCLUDE_COMPONENTS "ble_nrf_comm")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uart_integration_test)
