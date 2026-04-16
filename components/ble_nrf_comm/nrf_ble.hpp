#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
/* Shared protocol types */
#include "bridge_types.hpp"

namespace ble
{
    class BleAdvScanner
    {
    public:
        BleAdvScanner(QueueHandle_t queue);
        void start();

        // Whitelist — called by NfcPairing
        bool pair(const uint8_t mac[6], uint8_t device_type);
        bool unpair(const uint8_t mac[6]);
        void load_from_nvs();
        void clear_all();   // factory reset — wipes RAM whitelist + NVS

    private:
        QueueHandle_t          queue_;
        static BleAdvScanner  *instance_;

        // Whitelist
        struct whitelist_entry_t {
            uint8_t mac[6];
            uint8_t device_type;
            uint8_t last_data;
            bool    valid;
        };
        whitelist_entry_t      whitelist_[bridge::MAX_DEVICES] = {};
        size_t                 whitelist_count_ = 0;
        SemaphoreHandle_t      whitelist_mutex_;

        bool is_whitelisted(const uint8_t mac[6]);
        whitelist_entry_t *find_entry(const uint8_t mac[6]);
        void save_to_nvs();

        // Static C-compatible callbacks passed to NimBLE
        static void host_task(void *param);
        static void on_sync_cb();
        static void on_reset_cb(int reason);
        static int  on_gap_event_cb(struct ble_gap_event *event, void *arg);

        // Instance methods with actual logic
        void on_sync();
        void on_reset(int reason);
        int  on_gap_event(struct ble_gap_event *event);
        void start_scan();
        bool parse_adv(const struct ble_gap_disc_desc *disc, bridge::device_state_t &out);
    };
}
