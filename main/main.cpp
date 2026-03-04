/*
 * BLE ADV_NONCONN_IND Scanner for ESP32-S3
 * Passively scans for non-connectable BLE advertisements from nRF sensor nodes.
 * Designed for use as a Matter bridge: scan duty cycle leaves radio time for WiFi.
 */

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "BLE_SCANNER";

// /* Target nRF MAC address (little-endian) */
// /* Example: if MAC is AA:BB:CC:DD:EE:FF, set { 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA } */
// static const uint8_t TARGET_ADDR[6] = { 0xCA, 0xE8, 0x92, 0x96, 0x77, 0x32 };

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nrf_ble.hpp"

static void queue_reader_task(void *arg)
{
    QueueHandle_t queue = static_cast<QueueHandle_t>(arg);
    ble::device_state_t state;

    while (true) {
        if (xQueueReceive(queue, &state, portMAX_DELAY)) {
            ESP_LOGI(TAG, "MAC: %02X:%02X:%02X:%02X:%02X:%02X  cmd=0x%02X  data=0x%02X",
                     state.mac[5], state.mac[4], state.mac[3],
                     state.mac[2], state.mac[1], state.mac[0],
                     state.cmd, state.data);
        }
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

    QueueHandle_t queue = xQueueCreate(10, sizeof(ble::device_state_t));

    static ble::BleAdvScanner scanner(queue);

    xTaskCreate(queue_reader_task, "queue_reader", 2048, queue, 5, nullptr);

    scanner.start();
}
