# Samostatný testovací projekt — kompiluje POUZE uart_protocol + common + unity.
# ble_nrf_comm a NimBLE se vůbec nezapojí.
#
# Spuštění (z této složky):
#   idf.py build flash monitor
#
# Nebo z kořene projektu přes VSCode task "Test: Host (bez ESP32)".

cmake_minimum_required(VERSION 3.16)

# EXTRA_COMPONENT_DIRS musí ukazovat na NADŘAZENOU složku komponent,
# ne na komponentu samotnou. Z test/ jde ../.. na components/.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

# Pouze tyto komponenty se zkompilují — ble_nrf_comm se vynechá
set(COMPONENTS "main uart_protocol common unity")
set(EXCLUDE_COMPONENTS "ble_nrf_comm")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_uart_protocol)
