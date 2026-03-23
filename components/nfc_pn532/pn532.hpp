#pragma once

#include <cstdint>
#include "driver/uart.h"

namespace nfc
{
    static constexpr uint8_t PN532_PAYLOAD_MAX = 32;

    // -------------------------------------------------------------------------
    // NDEF record result — returned by read_tag()
    // -------------------------------------------------------------------------
    struct nfc_tag_t
    {
        uint8_t record_type;                  // NDEF record type byte, e.g. 'T' (0x54)
        uint8_t payload[PN532_PAYLOAD_MAX];   // raw NDEF record payload
        uint8_t payload_len;
    };

    // -------------------------------------------------------------------------
    // Pn532 — ESP-IDF port of Adafruit_PN532 (HSU/UART interface only)
    // -------------------------------------------------------------------------
    class Pn532
    {
    public:
        Pn532(uart_port_t port, int tx_pin, int rx_pin);

        /** Installs UART driver and calls wakeup() + SAMConfiguration. */
        bool begin();

        /** Retrieves chip firmware version. Call after begin(). */
        bool get_firmware_version(uint8_t &ic, uint8_t &ver, uint8_t &rev);

        /**
         * Polls for an ISO 14443-A tag and reads its first NDEF record payload.
         *
         * @param tag         Populated on success
         * @param timeout_ms  How long to keep polling
         */
        bool read_tag(nfc_tag_t &tag, uint32_t timeout_ms = 2000);

    private:
        uart_port_t port_;
        int         tx_pin_;
        int         rx_pin_;

        // Initialisation helpers
        void wakeup();
        bool sam_config();

        // Tag memory
        bool read_pages(uint8_t tg, uint8_t start_page, uint8_t out[16]);

        // NDEF parser
        static bool parse_ndef(const uint8_t *raw, uint8_t raw_len, nfc_tag_t &tag);

        // Adafruit-ported low-level HSU helpers
        bool send_command_check_ack(uint8_t *cmd, uint8_t cmdlen, uint32_t timeout_ms = 1000);
        void write_command(uint8_t *cmd, uint8_t cmdlen);
        void read_data(uint8_t *buf, uint8_t n);
        bool read_ack();
        bool wait_ready(uint32_t timeout_ms = 1000);
    };

} // namespace nfc
