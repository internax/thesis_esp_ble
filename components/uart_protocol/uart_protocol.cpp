#include "uart_protocol.hpp"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "UartProtocol";

namespace uart
{
    // -------------------------------------------------------------------------
    // Constructor — configures and installs the UART driver
    // -------------------------------------------------------------------------

    UartProtocol::UartProtocol(QueueHandle_t tx_queue, QueueHandle_t rx_queue,
                               uart_port_t port, int baud_rate, int tx_pin, int rx_pin)
        : tx_queue_(tx_queue), rx_queue_(rx_queue), port_(port)
    {
        const uart_config_t cfg = {
            .baud_rate  = baud_rate,
            .data_bits  = UART_DATA_8_BITS,
            .parity     = UART_PARITY_DISABLE,
            .stop_bits  = UART_STOP_BITS_1,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_ERROR_CHECK(uart_param_config(port_, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(port_, tx_pin, rx_pin,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        // RX buffer: room for several frames; no TX ring buffer (synchronous writes)
        ESP_ERROR_CHECK(uart_driver_install(port_, FRAME_SIZE * 4, 0, 0, nullptr, 0));
        tx_mutex_ = xSemaphoreCreateMutex();
    }

    // -------------------------------------------------------------------------
    // Public
    // -------------------------------------------------------------------------

    void UartProtocol::start()
    {
        xTaskCreate(send_task,    "uart_tx", 2048, this, 5, nullptr);
        xTaskCreate(receive_task, "uart_rx", 2048, this, 5, nullptr);
    }

    bool UartProtocol::send_reliable(const bridge::device_state_t &state, int retries, uint32_t timeout_ms)
    {
        uint8_t frame[FRAME_SIZE];
        encode(state, frame);

        for (int attempt = 1; attempt <= retries; attempt++) {
            xSemaphoreTake(tx_mutex_, portMAX_DELAY);
            uart_write_bytes(port_, frame, FRAME_SIZE);
            xSemaphoreGive(tx_mutex_);

            if (wait_for_ack(timeout_ms)) {
                return true;
            }
            ESP_LOGW(TAG, "send_reliable: attempt %d/%d — no ACK_OK", attempt, retries);
        }
        return false;
    }

    bool UartProtocol::wait_for_ack(uint32_t timeout_ms)
    {
        bridge::device_state_t state;
        if (xQueueReceive(rx_queue_, &state, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            return state.cmd == bridge::CMD_ACK_OK;
        }
        ESP_LOGW(TAG, "ACK timeout (%lu ms)", timeout_ms);
        return false;
    }

    // -------------------------------------------------------------------------
    // Static task entry points
    // -------------------------------------------------------------------------

    void UartProtocol::send_task(void *arg)
    {
        static_cast<UartProtocol *>(arg)->send_loop();
    }

    void UartProtocol::receive_task(void *arg)
    {
        static_cast<UartProtocol *>(arg)->receive_loop();
    }

    // -------------------------------------------------------------------------
    // send_loop — dequeues device_state_t entries and sends encoded binary frames
    // -------------------------------------------------------------------------

    void UartProtocol::send_loop()
    {
        bridge::device_state_t state;
        uint8_t frame[FRAME_SIZE];

        while (true) {
            if (xQueueReceive(tx_queue_, &state, portMAX_DELAY) == pdTRUE) {
                encode(state, frame);
                xSemaphoreTake(tx_mutex_, portMAX_DELAY);
                int written = uart_write_bytes(port_, frame, FRAME_SIZE);
                xSemaphoreGive(tx_mutex_);
                if (written != static_cast<int>(FRAME_SIZE)) {
                    ESP_LOGE(TAG, "UART write error: wrote %d of %zu bytes", written, FRAME_SIZE);
                } else {
                    ESP_LOGD(TAG, "TX cmd=0x%02X mac=%02X:%02X:%02X:%02X:%02X:%02X",
                             state.cmd,
                             state.mac[5], state.mac[4], state.mac[3],
                             state.mac[2], state.mac[1], state.mac[0]);
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // receive_loop — reads incoming frames byte by byte, decodes and queues them
    // -------------------------------------------------------------------------

    void UartProtocol::receive_loop()
    {
        uint8_t frame[FRAME_SIZE];

        while (true) {
            // Sync: scan for START byte
            uint8_t b;
            while (uart_read_bytes(port_, &b, 1, portMAX_DELAY) > 0) {
                if (b == FRAME_START) break;
            }
            frame[0] = FRAME_START;

            // Read remaining 10 bytes; 100 ms is enough at any reasonable baud rate
            int n = uart_read_bytes(port_, frame + 1, FRAME_SIZE - 1, pdMS_TO_TICKS(100));
            if (n != static_cast<int>(FRAME_SIZE - 1)) {
                ESP_LOGW(TAG, "RX timeout: got %d bytes after START", n);
                continue;
            }

            bridge::device_state_t state;
            if (decode(frame, state)) {
                ESP_LOGD(TAG, "RX cmd=0x%02X", state.cmd);
                xQueueSend(rx_queue_, &state, 0);
            } else {
                ESP_LOGW(TAG, "RX CRC error — frame discarded");
            }
        }
    }

    // -------------------------------------------------------------------------
    // Frame codec
    // -------------------------------------------------------------------------

    void UartProtocol::encode(const bridge::device_state_t &s, uint8_t out[FRAME_SIZE])
    {
        out[0] = FRAME_START;
        out[1] = 8;            // payload length: ADDR(6) + CMD(1) + DATA(1)
        memcpy(&out[2], s.mac, 6);
        out[8]  = s.cmd;
        out[9]  = s.data;
        out[10] = calc_crc8(out, 10);   // CRC covers bytes 0..9
    }

    bool UartProtocol::decode(const uint8_t frame[FRAME_SIZE], bridge::device_state_t &out)
    {
        if (frame[0] != FRAME_START)           return false;
        if (calc_crc8(frame, 10) != frame[10]) return false;

        memcpy(out.mac, &frame[2], 6);
        out.cmd  = frame[8];
        out.data = frame[9];
        return true;
    }

    // CRC-8/SMBUS — polynomial 0x07, init 0x00, no reflection
    uint8_t UartProtocol::calc_crc8(const uint8_t *data, size_t len)
    {
        uint8_t crc = 0x00;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int bit = 0; bit < 8; bit++) {
                crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
            }
        }
        return crc;
    }

} // namespace uart
