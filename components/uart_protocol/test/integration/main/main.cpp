/**
 * UART integration test — sender / receiver
 *
 * Wiring:
 *   Board A TX (GPIO 16) --> Board B RX (GPIO 17)
 *   Board A RX (GPIO 17) --> Board B TX (GPIO 16)
 *   GND <--> GND
 *
 * Flash sender:   (leave #define UART_ROLE_SENDER)
 * Flash receiver: comment out #define UART_ROLE_SENDER
 */

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "uart_protocol.hpp"
#include "bridge_types.hpp"

static const char *TAG = "UART_TEST";

#define UART_PORT   UART_NUM_1
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define BAUD_RATE   115200

// Comment out on the receiver board
// #define UART_ROLE_SENDER
#define UART_ROLE_RECEIVER

// =============================================================================
// SENDER — sends a frame every 2s, logs whether ACK arrived
// =============================================================================
#ifdef UART_ROLE_SENDER

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Role: SENDER");

    QueueHandle_t tx_queue = xQueueCreate(4, sizeof(bridge::device_state_t));
    QueueHandle_t rx_queue = xQueueCreate(4, sizeof(bridge::device_state_t));

    static uart::UartProtocol uart(tx_queue, rx_queue,
                                   UART_PORT, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    uart.start();

    bridge::device_state_t state = {};
    state.mac[0] = 0xAA; state.mac[1] = 0xBB; state.mac[2] = 0xCC;
    state.mac[3] = 0xDD; state.mac[4] = 0xEE; state.mac[5] = 0xFF;
    state.cmd  = bridge::CMD_ONOFF_STATUS;
    state.data = 0x01;

    while (true) {
        ESP_LOGI(TAG, "Sending frame...");
        bool ok = uart.send_reliable(state, 3, 500);
        if (ok) {
            ESP_LOGI(TAG, "ACK OK");
        } else {
            ESP_LOGW(TAG, "No ACK after 3 attempts");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// =============================================================================
// RECEIVER — waits for a frame, sends ACK back
// =============================================================================
#else

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Role: RECEIVER");

    QueueHandle_t tx_queue = xQueueCreate(4, sizeof(bridge::device_state_t));
    QueueHandle_t rx_queue = xQueueCreate(4, sizeof(bridge::device_state_t));

    static uart::UartProtocol uart(tx_queue, rx_queue,
                                   UART_PORT, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    uart.start();

    bridge::device_state_t state;
    bridge::device_state_t ack = {};
    ack.cmd = bridge::CMD_ACK_OK;

    while (true) {
        if (xQueueReceive(rx_queue, &state, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Received cmd=0x%02X data=0x%02X  MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                     state.cmd, state.data,
                     state.mac[5], state.mac[4], state.mac[3],
                     state.mac[2], state.mac[1], state.mac[0]);

            memcpy(ack.mac, state.mac, 6);
            xQueueSend(tx_queue, &ack, 0);
            ESP_LOGI(TAG, "ACK sent");
        }
    }
}

#endif
