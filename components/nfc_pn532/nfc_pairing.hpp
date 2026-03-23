#pragma once

#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "pn532.hpp"
#include "nrf_ble.hpp"
#include "uart_protocol.hpp"

namespace nfc
{
    // -------------------------------------------------------------------------
    // Event reported to the caller via callback
    // -------------------------------------------------------------------------
    enum class PairingEvent
    {
        IN_PROGRESS,  // tag read started, waiting for PN532
        SUCCESS,      // pair/unpair completed and acknowledged
        FAILED,       // tag read timeout, parse error, or UART ACK missing
    };

    // -------------------------------------------------------------------------
    // NfcPairing — coordinates NFC-triggered BLE pairing/unpairing
    //
    // Flow (pair):
    //   User holds tag → presses PAIR button
    //   → read_tag() → parse MAC from NDEF Text payload
    //   → BleScanner.pair(mac)
    //   → UartProtocol.send_reliable(NODE_ADDED)
    //   → if no ACK: BleScanner.unpair(mac)  [rollback]
    //   → callback(SUCCESS | FAILED)
    //
    // Flow (unpair) is symmetric with NODE_REMOVED and rollback via re-pair.
    // -------------------------------------------------------------------------
    class NfcPairing
    {
    public:
        /**
         * @param pn532       Initialised Pn532 driver (begin() already called)
         * @param scanner     BleAdvScanner instance
         * @param uart        UartProtocol instance
         * @param btn_pair    GPIO number for PAIR button (active-low, internal pull-up)
         * @param btn_unpair  GPIO number for UNPAIR button (active-low, internal pull-up)
         */
        NfcPairing(Pn532              &pn532,
                   ble::BleAdvScanner &scanner,
                   uart::UartProtocol &uart,
                   gpio_num_t          btn_pair,
                   gpio_num_t          btn_unpair);

        /**
         * Registers GPIO ISRs for both buttons and starts the FreeRTOS task.
         * Call once after all drivers are initialised.
         */
        void start();

        /**
         * Register a callback invoked on every state change.
         * Called from the pairing task — do not block inside the callback.
         */
        void set_event_callback(std::function<void(PairingEvent)> cb);

    private:
        Pn532              &pn532_;
        ble::BleAdvScanner &scanner_;
        uart::UartProtocol &uart_;
        gpio_num_t          btn_pair_;
        gpio_num_t          btn_unpair_;

        QueueHandle_t                      btn_queue_;   // carries BtnEvent
        std::function<void(PairingEvent)>  event_cb_;

        // Internal button event type
        enum class BtnEvent : uint8_t { PAIR, UNPAIR };

        // Per-button ISR context — holds queue handle + which event to send
        struct BtnIsrArg {
            QueueHandle_t queue;
            BtnEvent      event;
            uint32_t      last_ms; // software debounce timestamp
        };
        BtnIsrArg pair_arg_;
        BtnIsrArg unpair_arg_;

        // GPIO setup helper
        void configure_button(gpio_num_t pin, BtnIsrArg *arg);

        // FreeRTOS task
        static void pairing_task(void *arg);
        void        pairing_loop();

        // Pairing/unpairing flows
        bool do_pair(const uint8_t mac[6], uint8_t device_type);
        bool do_unpair(const uint8_t mac[6]);

        // NDEF Text payload → MAC (little-endian) and device type
        // Payload format: [lang_len(1B)][lang...][ASCII "XX:XX:XX:XX:XX:XX,TT"]
        // Tag stores MAC big-endian; stored in bridge as little-endian.
        static bool parse_tag_data(const nfc_tag_t &tag, uint8_t mac[6], uint8_t &device_type);

        // ISR handler shared by both buttons
        static void IRAM_ATTR btn_isr_handler(void *arg);

        // Notify callback safely (no-op if not set)
        void notify(PairingEvent e);
    };

} // namespace nfc
