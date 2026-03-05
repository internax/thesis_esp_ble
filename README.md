# ESP32-S3 BLE Scanner with NFC Pairing

## Overview

Firmware for **ESP32-S3 DevKitC-1** built on **ESP-IDF** with the **NimBLE** stack.

The device manages BLE device pairing via NFC and runs a continuous passive BLE scanner that captures non-connectable advertisements (`ADV_NONCONN_IND`) from paired devices. Data is forwarded over **bidirectional UART** to a second ESP32-S3 acting as a **Matter bridge**.

## Architecture

Three C++ classes and one shared struct communicate through two **FreeRTOS queues** (`tx_queue` and `rx_queue`).

<!-- ```
                        ┌──────────────┐
           pair/unpair  │  NfcPairing  │  pair/unpair
          ┌─────────────┤  (NFC + btn) ├─────────────┐
          │             └──────┬───────┘             │
          ▼                    │ wait_for_ack         ▼
  ┌──────────────┐             │              ┌──────────────┐
  │  BleScanner  │             │              │ UartProtocol │
  │  (NimBLE)    │─tx_queue───►│◄────rx_queue─│  (UART TX/RX)│
  └──────────────┘             │              └──────────────┘
                               ▼
                         Matter bridge
                         (ESP32-S3 #2)
``` -->

``` mermaid

classDiagram
    class device_state_t {
        <<struct>>
        mac: uint8_t[6]
        cmd: uint8_t
        data: uint8_t
    }

    class NfcPairing {
        -ble_scanner: BleScanner*
        -uart: UartProtocol*
        +init()
        +on_pair_button()
        +on_unpair_button()
        -read_mac(): uint8_t[6]
        -rollback(mac)
    }

    class BleScanner {
        -scan_params: ble_scan_params_t
        -tx_queue: QueueHandle_t
        +BleScanner(tx_queue)
        +start()
        +stop()
        +pair(mac): bool
        +unpair(mac): bool
        +on_adv_received()
        -parse(raw_data): device_state_t
    }

    class UartProtocol {
        -uart_port: uart_port_t
        -tx_queue: QueueHandle_t
        -rx_queue: QueueHandle_t
        +UartProtocol(tx_queue, rx_queue, port, baud_rate)
        +send_loop()
        +receive_loop()
        +wait_for_ack(timeout_ms): bool
        -encode(device_state_t): uint8_t[]
        -decode(uint8_t[]): device_state_t
        -calc_crc8(data): uint8_t
    }

    NfcPairing --> BleScanner : pair/unpair
    NfcPairing --> UartProtocol : wait_for_ack
    BleScanner --> device_state_t : creates
    UartProtocol <-- device_state_t : sends/receives

```

### `device_state_t` (struct)

| Field | Type | Description |
|-------|------|-------------|
| `mac` | `uint8_t[6]` | BLE MAC address (little-endian) |
| `cmd` | `uint8_t` | Command byte |
| `data` | `uint8_t` | Payload byte |

### `NfcPairing`

- Holds references to `BleScanner` and `UartProtocol`
- Reads MAC address from an NFC tag
- **Pair button press:** calls `BleScanner.pair(mac)` → if OK, sends `NODE_ADDED` over UART → waits for ACK → on timeout or `ACK_ERR`, rolls back whitelist
- **Unpair button press:** same flow with `BleScanner.unpair(mac)` and `NODE_REMOVED`
- Pairing is **idempotent** — repeated `NODE_ADDED` for the same MAC results in `ACK_OK`

### `BleScanner`

- Receives `tx_queue` via constructor
- Manages a **MAC whitelist** persisted in NVS (survives reboot)
- `pair(mac)` / `unpair(mac)` — adds/removes MAC from whitelist, returns `bool`
- Runs a continuous passive BLE scan (100 % duty cycle)
- On each captured non-connectable advertisement from a whitelisted device, parses the payload into `device_state_t` and pushes it to `tx_queue`

### `UartProtocol`

- Receives `tx_queue`, `rx_queue`, UART port number and baud rate via constructor
- `send_loop` — dequeues `device_state_t` from `tx_queue`, encodes and transmits a binary frame
- `receive_loop` — listens for incoming frames (`ACK_OK` / `ACK_ERR`), decodes and pushes to `rx_queue`
- `wait_for_ack(timeout_ms)` — blocks until ACK is received or timeout expires
- Retry: 2–3 attempts for `ONOFF_STATUS` / `LEVEL_STATUS`; rollback on pairing failure

### Main

- Creates `tx_queue` and `rx_queue`
- Instantiates `NfcPairing`, `BleScanner` and `UartProtocol` with shared queues
- Starts BLE scan and UART loops on separate FreeRTOS tasks

## Project Structure

```
esp_ble_uart/
├── main/
│   ├── main.cpp              # Entry point — creates queues, starts tasks
│   └── CMakeLists.txt
└── components/
    └── ble_nrf_comm/
        ├── nrf_ble.hpp       # BleAdvScanner class & device_state_t
        └── nrf_ble.cpp       # BleAdvScanner implementation
```

## UART Binary Protocol

Each frame is **11 bytes**:

| Field | Size | Value / Description |
|-------|------|---------------------|
| `START` | 1 B | `0xAA` — frame delimiter |
| `LEN`   | 1 B | Payload length in bytes |
| `ADDR`  | 6 B | BLE MAC address |
| `CMD`   | 1 B | Command byte (see table below) |
| `DATA`  | 1 B | Payload byte |
| `CRC8`  | 1 B | CRC-8 checksum over all preceding bytes |

## Command Table

| Command | Byte | Direction | `DATA` field |
|---------|------|-----------|--------------|
| `NODE_ADDED`   | `0x01` | BLE → Matter | `device_type` (1 B) |
| `NODE_REMOVED` | `0x02` | BLE → Matter | empty |
| `ONOFF_STATUS` | `0x10` | BLE → Matter | `bool` (1 B) |
| `LEVEL_STATUS` | `0x11` | BLE → Matter | `uint8` 0–254 (1 B) |
| `ACK_OK`       | `0x80` | Matter → BLE | empty |
| `ACK_ERR`      | `0x81` | Matter → BLE | empty |

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
| ESP32-S3 DevKitC-1 #1 | BLE scanner + NFC pairing + UART TX/RX (this firmware) |
| ESP32-S3 DevKitC-1 #2 | Matter bridge — receives UART frames, sends ACKs |

UART connection between the two boards:

| Signal | ESP32-S3 #1 (scanner) | ESP32-S3 #2 (bridge) |
|--------|-----------------------|----------------------|
| TX     | UART TX pin | RX pin |
| RX     | UART RX pin | TX pin |
| GND    | GND | GND |

## Notes

- MAC whitelist is persisted in **NVS** — survives reboot
- Pairing operations are **idempotent** — safe to repeat
- No reverse control channel — target devices are energy-harvesting switches (BLE only transmits)
