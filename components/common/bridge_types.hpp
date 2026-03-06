#pragma once

#include <cstdint>
#include <cstddef>

namespace bridge
{
    // -------------------------------------------------------------------------
    // Protocol limits
    // -------------------------------------------------------------------------
    static constexpr size_t MAX_DEVICES = 16;   // max paired devices (whitelist capacity)

    // -------------------------------------------------------------------------
    // Shared message struct — exchanged between BleScanner and UartProtocol
    // -------------------------------------------------------------------------
    struct device_state_t
    {
        uint8_t mac[6];   // BLE MAC address (little-endian)
        uint8_t cmd;      // command byte (see CMD_* constants below)
        uint8_t data;     // payload byte
    };

    // -------------------------------------------------------------------------
    // Command bytes — BLE scanner → Matter bridge
    // -------------------------------------------------------------------------
    static constexpr uint8_t CMD_NODE_ADDED   = 0x01;
    static constexpr uint8_t CMD_NODE_REMOVED = 0x02;
    static constexpr uint8_t CMD_ONOFF_STATUS = 0x10;
    static constexpr uint8_t CMD_LEVEL_STATUS = 0x11;

    // -------------------------------------------------------------------------
    // Command bytes — Matter bridge → BLE scanner
    // -------------------------------------------------------------------------
    static constexpr uint8_t CMD_ACK_OK  = 0x80;
    static constexpr uint8_t CMD_ACK_ERR = 0x81;

} // namespace bridge
