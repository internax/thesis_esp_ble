/*
 * Test: přidání on/off zařízení do Matter přes UART (CMD_NODE_ADDED)
 */

#include <cstring>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "uart_protocol.hpp"
#include "bridge_types.hpp"

static const char *TAG = "bridge";

static constexpr uart_port_t UART_PORT   = UART_NUM_1;
static constexpr int         UART_TX_PIN = 16;
static constexpr int         UART_RX_PIN = 17;
static constexpr int         UART_BAUD   = 115200;

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    QueueHandle_t uart_tx = xQueueCreate(4, sizeof(bridge::device_state_t));
    QueueHandle_t uart_rx = xQueueCreate(4, sizeof(bridge::device_state_t));
    static uart::UartProtocol uart_proto(uart_tx, uart_rx,
                                         UART_PORT, UART_BAUD, UART_TX_PIN, UART_RX_PIN);
    uart_proto.start();

    static const uint8_t DEVICE_MAC[6] = { 0xca, 0xe8, 0x92, 0x96, 0x77, 0x32 };

    bridge::device_state_t msg = {};
    memcpy(msg.mac, DEVICE_MAC, 6);
    msg.cmd  = bridge::CMD_NODE_ADDED;
    msg.data = 0x01;  // 0x01 = on/off device

    bool ok = uart_proto.send_reliable(msg, 3, 500);
    if (ok) {
        ESP_LOGI(TAG, "NODE_ADDED OK — %02X:%02X:%02X:%02X:%02X:%02X",
                 DEVICE_MAC[5], DEVICE_MAC[4], DEVICE_MAC[3],
                 DEVICE_MAC[2], DEVICE_MAC[1], DEVICE_MAC[0]);
    } else {
        ESP_LOGE(TAG, "NODE_ADDED FAILED");
    }

    vTaskDelay(pdMS_TO_TICKS(10000));

    // Simulace spínače: 3x zapnout → počkat 5s → vypnout
    bridge::device_state_t onoff = {};
    memcpy(onoff.mac, DEVICE_MAC, 6);
    onoff.cmd = bridge::CMD_ONOFF_STATUS;

    for (int i = 0; i < 3; i++) {
        onoff.data = 0x01;
        ok = uart_proto.send_reliable(onoff, 3, 500);
        ESP_LOGI(TAG, "[%d/3] ON  → %s", i + 1, ok ? "OK" : "FAILED");

        vTaskDelay(pdMS_TO_TICKS(5000));

        onoff.data = 0x00;
        ok = uart_proto.send_reliable(onoff, 3, 500);
        ESP_LOGI(TAG, "[%d/3] OFF → %s", i + 1, ok ? "OK" : "FAILED");

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    // Odebrání zařízení z Matter
    msg.cmd  = bridge::CMD_NODE_REMOVED;
    msg.data = 0x00;

    ok = uart_proto.send_reliable(msg, 3, 500);
    ESP_LOGI(TAG, "NODE_REMOVED → %s", ok ? "OK" : "FAILED");
}

// =============================================================================
// BLE → UART bridge (předchozí kód)
// =============================================================================
//
// #include "nrf_ble.hpp"
//
// static constexpr int      UART_RETRIES    = 2;
// static constexpr uint32_t UART_TIMEOUT_MS = 50;
//
// struct bridge_task_arg_t {
//     QueueHandle_t         ble_queue;
//     uart::UartProtocol   *uart_proto;
// };
//
// struct dedup_entry_t {
//     uint8_t mac[6];
//     uint8_t last_data;
//     bool    valid;
// };
//
// static dedup_entry_t *dedup_find(dedup_entry_t *table, size_t count, const uint8_t mac[6])
// {
//     for (size_t i = 0; i < count; i++) {
//         if (table[i].valid && memcmp(table[i].mac, mac, 6) == 0) {
//             return &table[i];
//         }
//     }
//     return nullptr;
// }
//
// static void bridge_task(void *arg)
// {
//     auto *ctx = static_cast<bridge_task_arg_t *>(arg);
//     bridge::device_state_t state;
//     dedup_entry_t dedup_table[bridge::MAX_DEVICES] = {};
//
//     while (true) {
//         if (xQueueReceive(ctx->ble_queue, &state, portMAX_DELAY) != pdTRUE) continue;
//
//         dedup_entry_t *entry = dedup_find(dedup_table, bridge::MAX_DEVICES, state.mac);
//         if (entry == nullptr) {
//             for (auto &e : dedup_table) {
//                 if (!e.valid) {
//                     memcpy(e.mac, state.mac, 6);
//                     e.last_data = state.data;
//                     e.valid     = true;
//                     entry = &e;
//                     break;
//                 }
//             }
//         } else if (entry->last_data == state.data) {
//             continue;
//         } else {
//             entry->last_data = state.data;
//         }
//
//         bool ok = ctx->uart_proto->send_reliable(state, UART_RETRIES, UART_TIMEOUT_MS);
//         if (!ok) {
//             ESP_LOGW(TAG, "UART send failed (MAC %02X:%02X:%02X:%02X:%02X:%02X)",
//                      state.mac[5], state.mac[4], state.mac[3],
//                      state.mac[2], state.mac[1], state.mac[0]);
//         }
//     }
// }
//
// extern "C" void app_main(void)
// {
//     ...
//     QueueHandle_t ble_queue = xQueueCreate(4, sizeof(bridge::device_state_t));
//     static ble::BleAdvScanner scanner(ble_queue);
//     static const uint8_t MAC_1[6] = { 0xca, 0xe8, 0x92, 0x96, 0x77, 0x32 };
//     scanner.pair(MAC_1);
//     static bridge_task_arg_t ctx = { ble_queue, &uart_proto };
//     xTaskCreate(bridge_task, "bridge", 4096, &ctx, 6, nullptr);
//     scanner.start();
// }
