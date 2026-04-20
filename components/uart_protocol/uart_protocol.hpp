#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "bridge_types.hpp"

namespace uart
{
    // -------------------------------------------------------------------------
    // Frame constants
    // -------------------------------------------------------------------------
    static constexpr uint8_t FRAME_START = 0xAA;
    static constexpr size_t  FRAME_SIZE  = 11;  // START + LEN + ADDR(6) + CMD + DATA + CRC8

    // -------------------------------------------------------------------------
    // UartProtocol
    // -------------------------------------------------------------------------
    class UartProtocol
    {
    public:
        /**
         * Configures and installs the UART driver.
         *
         * @param tx_queue  Queue of bridge::device_state_t to send
         * @param rx_queue  Queue of bridge::device_state_t for received frames
         * @param port      UART port number (e.g. UART_NUM_1)
         * @param baud_rate Baud rate (e.g. 115200)
         * @param tx_pin    GPIO number for UART TX
         * @param rx_pin    GPIO number for UART RX
         */
        UartProtocol(QueueHandle_t rx_queue,
                     uart_port_t port, int baud_rate, int tx_pin, int rx_pin);

        /** Starts receive_loop as a FreeRTOS task. */
        void start();

        /**
         * Sends a frame, waits for ACK, retries on failure.
         * Blocks the caller until ACK_OK is received or all attempts are exhausted.
         * Safe to call from any task — protected by internal TX mutex.
         *
         * @param state       Frame to send
         * @param retries     Total number of attempts (1 = send once, no retry)
         * @param timeout_ms  ACK wait time per attempt in milliseconds
         * @return true   if ACK_OK was received within any attempt
         * @return false  after all attempts failed (timeout or ACK_ERR)
         */
        bool send_reliable(const bridge::device_state_t &state, int retries, uint32_t timeout_ms);

        /**
         * Blocks until an ACK frame is received or the timeout expires.
         * Useful when the caller manages send/retry logic itself.
         *
         * @param timeout_ms  Maximum wait time in milliseconds
         * @return true   on CMD_ACK_OK
         * @return false  on CMD_ACK_ERR or timeout
         */
        bool wait_for_ack(uint32_t timeout_ms);

    private:
        QueueHandle_t     rx_queue_;
        uart_port_t       port_;
        SemaphoreHandle_t tx_mutex_;          // serialises concurrent UART writes
        SemaphoreHandle_t send_reliable_mutex_; // ensures only one send_reliable at a time

        static void receive_task(void *arg);

        void receive_loop();

        static void    encode(const bridge::device_state_t &state, uint8_t out[FRAME_SIZE]);
        static bool    decode(const uint8_t frame[FRAME_SIZE], bridge::device_state_t &out);
        static uint8_t calc_crc8(const uint8_t *data, size_t len);
    };

} // namespace uart
