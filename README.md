# ESP32-S3 BLE to UART Bridge

## Overview

Firmware for **ESP32-S3 DevKitC-1** built on **ESP-IDF** with the **NimBLE** stack.

The device runs a continuous passive BLE scanner that captures non-connectable advertisements (`ADV_NONCONN_IND`) from sensor nodes (e.g. nRF-based devices), extracts relevant data, and forwards it over UART as binary frames to a second ESP32-S3 acting as a **Matter bridge**.

## Architecture

Two C++ classes and one shared struct communicate through a **FreeRTOS queue**.

```
┌─────────────┐   device_state_t   ┌──────────────┐
│ BleAdvScanner│ ──── queue ──────► │ UartProtocol │
│  (NimBLE)   │                    │  (UART TX)   │
└─────────────┘                    └──────────────┘
```

### `device_state_t` (struct, namespace `ble`)

| Field | Type | Description |
|-------|------|-------------|
| `mac` | `uint8_t[6]` | BLE MAC address (little-endian) |
| `cmd` | `uint8_t` | Command byte |
| `data` | `uint8_t` | Payload byte |

### `BleAdvScanner` (namespace `ble`)

- Receives a `QueueHandle_t` via constructor
- Runs a continuous BLE passive scan (100 % duty cycle)
- On each captured non-connectable advertisement, parses the raw payload into `device_state_t` and pushes it to the queue
- Supports a MAC address **whitelist** (up to 16 devices) — only whitelisted devices are forwarded

### `UartProtocol`

- Receives a `QueueHandle_t`, UART port number and baud rate via constructor
- Dequeues `device_state_t` entries and transmits them as binary frames over UART

### UART Binary Protocol

Each frame is **11 bytes**:

| Field | Size | Value / Description |
|-------|------|---------------------|
| `START` | 1 B | `0xAA` — frame delimiter |
| `LEN`   | 1 B | Payload length in bytes |
| `ADDR`  | 6 B | BLE MAC address |
| `CMD`   | 1 B | Command byte (see table below) |
| `DATA`  | 1 B | Payload byte |
| `CRC8`  | 1 B | CRC-8 checksum over all preceding bytes |

### Command Table

| Command | Byte | Description |
|---------|------|-------------|
| `create_on_off_node` | `0x01` | Create a new Matter on/off node for the given MAC |
| `create_level_node`  | `0x02` | Create a new Matter level-control node for the given MAC |
| `remove_node`        | `0x03` | Remove the Matter node with the given MAC |
| `set_binary`         | `0x04` | Set binary value on the node with the given MAC |
| `set_on_off_node`    | `0x05` | Set on/off state on the node with the given MAC |
| `set_level_node`     | `0x06` | Set level value on the node with the given MAC |

## Project Structure

```
esp_ble_uart/
├── main/
│   ├── main.cpp              # Entry point — creates queue, starts scanner
│   └── CMakeLists.txt
└── components/
    └── ble_nrf_comm/
        ├── nrf_ble.hpp       # BleAdvScanner class & device_state_t
        └── nrf_ble.cpp       # BleAdvScanner implementation
```

## Build & Flash

### Requirements

- [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/)
- ESP32-S3 DevKitC-1
- USB cable

### Build

```bash
idf.py set-target esp32s3
idf.py build
```

### Flash & Monitor

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

> Press `Ctrl+]` to exit the serial monitor.

## Hardware

| Component | Description |
|-----------|-------------|
| ESP32-S3 DevKitC-1 | BLE scanner + UART transmitter (this firmware) |
| ESP32-S3 (second board) | Matter bridge — receives UART frames |

UART connection between the two boards:

| Signal | ESP32-S3 #1 (scanner) | ESP32-S3 #2 (bridge) |
|--------|-----------------------|----------------------|
| TX     | configured UART TX pin | RX pin |
| GND    | GND | GND |
