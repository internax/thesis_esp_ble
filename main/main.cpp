/*
 * BLE → UART Matter bridge
 *
 * Pipeline:
 *   BleAdvScanner ──(ble_queue)──► bridge_task ──► UartProtocol ──► Matter controller
 *
 * bridge_task čte BLE události a spolehlivě je přeposílá přes UART.
 * Veškerá logika stmívání/časování je řešena na straně Matter controlleru.
 */

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "nrf_ble.hpp"
#include "uart_protocol.hpp"
#include "bridge_types.hpp"

static const char *TAG = "bridge";

// UART konfigurace — přizpůsob GPIO číslům hardwaru
static constexpr uart_port_t UART_PORT   = UART_NUM_1;
static constexpr int         UART_TX_PIN = 16;
static constexpr int         UART_RX_PIN = 17;
static constexpr int         UART_BAUD   = 115200;

// send_reliable parametry
static constexpr int      UART_RETRIES    = 2;
static constexpr uint32_t UART_TIMEOUT_MS = 50;

struct bridge_task_arg_t {
    QueueHandle_t         ble_queue;
    uart::UartProtocol   *uart_proto;
};

// Dedup tabulka — jeden záznam na spárované zařízení
struct dedup_entry_t {
    uint8_t mac[6];
    uint8_t last_data;
    bool    valid;
};

static dedup_entry_t *dedup_find(dedup_entry_t *table, size_t count, const uint8_t mac[6])
{
    for (size_t i = 0; i < count; i++) {
        if (table[i].valid && memcmp(table[i].mac, mac, 6) == 0) {
            return &table[i];
        }
    }
    return nullptr;
}

static void bridge_task(void *arg)
{
    auto *ctx = static_cast<bridge_task_arg_t *>(arg);
    bridge::device_state_t state;

    dedup_entry_t dedup_table[bridge::MAX_DEVICES] = {};

    while (true) {
        if (xQueueReceive(ctx->ble_queue, &state, portMAX_DELAY) != pdTRUE) continue;

        // Dedup: přeskočíme pakety se stejným data jako naposledy odeslaný pro daný MAC
        dedup_entry_t *entry = dedup_find(dedup_table, bridge::MAX_DEVICES, state.mac);
        if (entry == nullptr) {
            // Nové zařízení — přidej do tabulky
            for (auto &e : dedup_table) {
                if (!e.valid) {
                    memcpy(e.mac, state.mac, 6);
                    e.last_data = state.data;
                    e.valid     = true;
                    entry = &e;
                    break;
                }
            }
        } else if (entry->last_data == state.data) {
            continue;  // duplikát, přeskočíme
        } else {
            entry->last_data = state.data;
        }

        bool ok = ctx->uart_proto->send_reliable(state, UART_RETRIES, UART_TIMEOUT_MS);
        if (!ok) {
            ESP_LOGW(TAG, "UART send failed (MAC %02X:%02X:%02X:%02X:%02X:%02X cmd=0x%02X)",
                     state.mac[5], state.mac[4], state.mac[3],
                     state.mac[2], state.mac[1], state.mac[0],
                     state.cmd);
        }
    }
}

extern "C" void app_main(void)
{
    // NVS — vyžadováno BLE stackem
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // UART
    QueueHandle_t uart_tx = xQueueCreate(4, sizeof(bridge::device_state_t));
    QueueHandle_t uart_rx = xQueueCreate(4, sizeof(bridge::device_state_t));
    static uart::UartProtocol uart_proto(uart_tx, uart_rx,
                                         UART_PORT, UART_BAUD, UART_TX_PIN, UART_RX_PIN);
    uart_proto.start();

    // BLE scanner
    QueueHandle_t ble_queue = xQueueCreate(4, sizeof(bridge::device_state_t));
    static ble::BleAdvScanner scanner(ble_queue);

    static const uint8_t MAC_1[6] = { 0xca, 0xe8, 0x92, 0x96, 0x77, 0x32 };
    scanner.pair(MAC_1);

    // Bridge task — vyšší priorita než UART tasky (5) pro minimální latenci
    static bridge_task_arg_t ctx = { ble_queue, &uart_proto };
    xTaskCreate(bridge_task, "bridge", 4096, &ctx, 6, nullptr);

    scanner.start();
}
