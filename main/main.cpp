// =============================================================================
// NFC pairing — párování a odpárování BLE zařízení přes NFC
// =============================================================================

#include <cstring>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "pn532.hpp"
#include "nfc_pairing.hpp"
#include "nrf_ble.hpp"
#include "uart_protocol.hpp"
#include "bridge_types.hpp"
#include "led_indicator.hpp"

static const char *TAG = "main";

// NFC
static constexpr uart_port_t NFC_UART_PORT = UART_NUM_2;
static constexpr int         NFC_TX_PIN    = 15;
static constexpr int         NFC_RX_PIN    = 7;

// UART bridge
static constexpr uart_port_t UART_PORT   = UART_NUM_1;
static constexpr int         UART_TX_PIN = 12;
static constexpr int         UART_RX_PIN = 13;
static constexpr int         UART_BAUD   = 115200;

// Tlačítka
static constexpr gpio_num_t BTN_PAIR         = GPIO_NUM_21;
static constexpr gpio_num_t BTN_UNPAIR       = GPIO_NUM_14;
static constexpr gpio_num_t BTN_FACTORY_RESET = GPIO_NUM_11;
static constexpr uint32_t   FACTORY_RESET_HOLD_MS = 5000;

// LED (WS2812B) — GPIO 48 je vestavěná RGB LED na ESP32-S3 DevKitC-1
static constexpr gpio_num_t LED_PIN = GPIO_NUM_10;

struct factory_reset_arg_t {
    ble::BleAdvScanner *scanner;
    uart::UartProtocol *uart;
};

static void factory_reset_task(void *arg)
{
    auto *ctx = static_cast<factory_reset_arg_t *>(arg);

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BTN_FACTORY_RESET),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    while (true) {
        if (gpio_get_level(BTN_FACTORY_RESET) == 0) {
            uint32_t held_ms = 0;
            while (gpio_get_level(BTN_FACTORY_RESET) == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
                held_ms += 50;
                if (held_ms >= FACTORY_RESET_HOLD_MS) {
                    ESP_LOGW("factory_reset", "Factory reset triggered");

                    // 1. Smazat lokální whitelist
                    ctx->scanner->clear_all();

                    // 2. Informovat Matter bridge (bez čekání na ACK — restartuje se)
                    bridge::device_state_t msg = {};
                    msg.cmd = bridge::CMD_FACTORY_RESET;
                    ctx->uart->send_reliable(msg, 1, 200);

                    // 3. Restart BLE scanneru
                    esp_restart();
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // UART fronty
    QueueHandle_t uart_tx = xQueueCreate(4, sizeof(bridge::device_state_t));
    QueueHandle_t uart_rx = xQueueCreate(4, sizeof(bridge::device_state_t));

    // Komponenty
    static nfc::Pn532          pn532(NFC_UART_PORT, NFC_TX_PIN, NFC_RX_PIN);
    static ble::BleAdvScanner  scanner(uart_tx);
    static uart::UartProtocol  uart_proto(uart_tx, uart_rx,
                                          UART_PORT, UART_BAUD, UART_TX_PIN, UART_RX_PIN);
    static nfc::NfcPairing     nfc_pairing(pn532, scanner, uart_proto,
                                           BTN_PAIR, BTN_UNPAIR);
    static led::LedIndicator   led_indicator(LED_PIN);

    // Inicializace
    if (!pn532.begin()) {
        ESP_LOGE(TAG, "PN532 init failed");
        return;
    }

    scanner.load_from_nvs();
    uart_proto.start();
    scanner.start();
    led_indicator.start();

    static factory_reset_arg_t fr_arg = { &scanner, &uart_proto };
    xTaskCreate(factory_reset_task, "fr_button", 4096, &fr_arg, 3, nullptr);

    nfc_pairing.set_event_callback([&led_indicator](nfc::PairingEvent e) {
        switch (e) {
            case nfc::PairingEvent::IN_PROGRESS:
                led_indicator.show(led::LedState::IN_PROGRESS); break;
            case nfc::PairingEvent::SUCCESS:
                led_indicator.show(led::LedState::SUCCESS);     break;
            case nfc::PairingEvent::FAILED:
                led_indicator.show(led::LedState::FAILED);      break;
        }
    });

    nfc_pairing.start();

    // Hlavní task nemá co dělat — vše běží v samostatných FreeRTOS taskách
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =============================================================================
// Test: PN532 NFC čtečka — ověření komunikace
// =============================================================================
//
// #include <cstring>
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "pn532.hpp"
//
// static const char *TAG = "main";
// static constexpr uart_port_t NFC_UART_PORT = UART_NUM_2;
// static constexpr int         NFC_TX_PIN    = 37;
// static constexpr int         NFC_RX_PIN    = 36;
//
// extern "C" void app_main(void)
// {
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);
//
//     static nfc::Pn532 pn532(NFC_UART_PORT, NFC_TX_PIN, NFC_RX_PIN);
//     if (!pn532.begin()) {
//         ESP_LOGE(TAG, "PN532 init failed — zkontroluj zapojení a rozhraní (HSU)");
//         return;
//     }
//     uint8_t ic, ver, rev;
//     if (pn532.get_firmware_version(ic, ver, rev))
//         ESP_LOGI(TAG, "PN532 OK — IC=0x%02X firmware v%d.%d", ic, ver, rev);
//     else
//         ESP_LOGW(TAG, "Firmware version read failed");
//
//     ESP_LOGI(TAG, "Přilož NFC tag...");
//     while (true) {
//         nfc::nfc_tag_t tag = {};
//         if (pn532.read_tag(tag, 2000)) {
//             ESP_LOGI(TAG, "Tag nalezen! type=0x%02X payload_len=%d", tag.record_type, tag.payload_len);
//             ESP_LOG_BUFFER_HEX(TAG, tag.payload, tag.payload_len);
//             char str[nfc::PN532_PAYLOAD_MAX + 1] = {};
//             memcpy(str, tag.payload, tag.payload_len);
//             ESP_LOGI(TAG, "Payload text: \"%s\"", str);
//         }
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }

// =============================================================================
// Test: přidání on/off zařízení do Matter přes UART (CMD_NODE_ADDED)
// =============================================================================
//
// #include <cstring>
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/queue.h"
// #include "uart_protocol.hpp"
// #include "bridge_types.hpp"
//
// static const char *TAG = "bridge";
// static constexpr uart_port_t UART_PORT   = UART_NUM_1;
// static constexpr int         UART_TX_PIN = 16;
// static constexpr int         UART_RX_PIN = 17;
// static constexpr int         UART_BAUD   = 115200;
//
// extern "C" void app_main(void)
// {
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);
//
//     QueueHandle_t uart_tx = xQueueCreate(4, sizeof(bridge::device_state_t));
//     QueueHandle_t uart_rx = xQueueCreate(4, sizeof(bridge::device_state_t));
//     static uart::UartProtocol uart_proto(uart_tx, uart_rx,
//                                          UART_PORT, UART_BAUD, UART_TX_PIN, UART_RX_PIN);
//     uart_proto.start();
//
//     static const uint8_t DEVICE_MAC[6] = { 0xca, 0xe8, 0x92, 0x96, 0x77, 0x32 };
//
//     bridge::device_state_t msg = {};
//     memcpy(msg.mac, DEVICE_MAC, 6);
//     msg.cmd  = bridge::CMD_NODE_ADDED;
//     msg.data = 0x01;  // 0x01 = on/off device
//
//     bool ok = uart_proto.send_reliable(msg, 3, 500);
//     if (ok) {
//         ESP_LOGI(TAG, "NODE_ADDED OK — %02X:%02X:%02X:%02X:%02X:%02X",
//                  DEVICE_MAC[5], DEVICE_MAC[4], DEVICE_MAC[3],
//                  DEVICE_MAC[2], DEVICE_MAC[1], DEVICE_MAC[0]);
//     } else {
//         ESP_LOGE(TAG, "NODE_ADDED FAILED");
//     }
//
//     vTaskDelay(pdMS_TO_TICKS(10000));
//
//     // Simulace spínače: 3x zapnout → počkat 5s → vypnout
//     bridge::device_state_t onoff = {};
//     memcpy(onoff.mac, DEVICE_MAC, 6);
//     onoff.cmd = bridge::CMD_ONOFF_STATUS;
//
//     for (int i = 0; i < 3; i++) {
//         onoff.data = 0x01;
//         ok = uart_proto.send_reliable(onoff, 3, 500);
//         ESP_LOGI(TAG, "[%d/3] ON  → %s", i + 1, ok ? "OK" : "FAILED");
//
//         vTaskDelay(pdMS_TO_TICKS(5000));
//
//         onoff.data = 0x00;
//         ok = uart_proto.send_reliable(onoff, 3, 500);
//         ESP_LOGI(TAG, "[%d/3] OFF → %s", i + 1, ok ? "OK" : "FAILED");
//
//         vTaskDelay(pdMS_TO_TICKS(5000));
//     }
//
//     // Odebrání zařízení z Matter
//     msg.cmd  = bridge::CMD_NODE_REMOVED;
//     msg.data = 0x00;
//
//     ok = uart_proto.send_reliable(msg, 3, 500);
//     ESP_LOGI(TAG, "NODE_REMOVED → %s", ok ? "OK" : "FAILED");
// }

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
