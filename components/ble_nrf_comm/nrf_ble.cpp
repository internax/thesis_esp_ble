#include "nrf_ble.hpp"
#include "services/gap/ble_svc_gap.h"
#include <cstring>

extern "C" void ble_store_config_init(void);

static const char *TAG = "BleAdvScanner";

namespace ble
{
    BleAdvScanner *BleAdvScanner::instance_ = nullptr;

    // -------------------------------------------------------------------------
    // Public
    // -------------------------------------------------------------------------

    BleAdvScanner::BleAdvScanner(QueueHandle_t queue)
        : queue_(queue), whitelist_count_(0)
    {
        assert(instance_ == nullptr && "BleAdvScanner instance can exist only once!");
        instance_ = this;
        whitelist_mutex_ = xSemaphoreCreateMutex();
    }

    void BleAdvScanner::start()
    {
        esp_err_t ret = nimble_port_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init nimble; rc=%d", ret);
            return;
        }

        ble_hs_cfg.reset_cb        = on_reset_cb;
        ble_hs_cfg.sync_cb         = on_sync_cb;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

        ble_svc_gap_device_name_set("ble-scanner");
        ble_store_config_init();

        nimble_port_freertos_init(host_task);
    }

    bool BleAdvScanner::pair(const uint8_t mac[6])
    {
        xSemaphoreTake(whitelist_mutex_, portMAX_DELAY);
        // Idempotent: already in whitelist is OK
        for (size_t i = 0; i < whitelist_count_; i++) {
            if (memcmp(whitelist_[i], mac, 6) == 0) {
                xSemaphoreGive(whitelist_mutex_);
                return true;
            }
        }
        bool ok = false;
        if (whitelist_count_ < bridge::MAX_DEVICES) {
            memcpy(whitelist_[whitelist_count_++], mac, 6);
            ok = true;
        }
        xSemaphoreGive(whitelist_mutex_);
        return ok;
    }

    bool BleAdvScanner::unpair(const uint8_t mac[6])
    {
        xSemaphoreTake(whitelist_mutex_, portMAX_DELAY);
        bool ok = false;
        for (size_t i = 0; i < whitelist_count_; i++) {
            if (memcmp(whitelist_[i], mac, 6) == 0) {
                // Replace with last element and shrink
                whitelist_count_--;
                memcpy(whitelist_[i], whitelist_[whitelist_count_], 6);
                ok = true;
                break;
            }
        }
        xSemaphoreGive(whitelist_mutex_);
        return ok;
    }

    bool BleAdvScanner::is_whitelisted(const uint8_t mac[6])
    {
        xSemaphoreTake(whitelist_mutex_, portMAX_DELAY);
        if (whitelist_count_ == 0) {
            xSemaphoreGive(whitelist_mutex_);
            return true;  // prázdný whitelist = pustíme vše (discovery mode)
        }
        for (size_t i = 0; i < whitelist_count_; i++) {
            if (memcmp(whitelist_[i], mac, 6) == 0) {
                xSemaphoreGive(whitelist_mutex_);
                return true;
            }
        }
        xSemaphoreGive(whitelist_mutex_);
        return false;
    }

    // -------------------------------------------------------------------------
    // Statické callbacky — pouze přeposílají volání na instanci
    // -------------------------------------------------------------------------

    void BleAdvScanner::host_task(void *param)
    {
        ESP_LOGI(TAG, "BLE Host Task Started");
        nimble_port_run();              // blokuje dokud BLE stack běží
        nimble_port_freertos_deinit();
    }

    void BleAdvScanner::on_sync_cb()
    {
        instance_->on_sync();
    }

    void BleAdvScanner::on_reset_cb(int reason)
    {
        instance_->on_reset(reason);
    }

    int BleAdvScanner::on_gap_event_cb(struct ble_gap_event *event, void *arg)
    {
        return instance_->on_gap_event(event);
    }

    // -------------------------------------------------------------------------
    // Instanční metody — skutečná logika
    // -------------------------------------------------------------------------

    void BleAdvScanner::on_sync()
    {
        int rc = ble_hs_util_ensure_addr(0);
        assert(rc == 0);
        start_scan();
    }

    void BleAdvScanner::on_reset(int reason)
    {
        ESP_LOGE(TAG, "BLE host reset; reason=%d", reason);
    }

    void BleAdvScanner::start_scan()
    {
        uint8_t own_addr_type;
        struct ble_gap_disc_params disc_params = {};

        int rc = ble_hs_id_infer_auto(0, &own_addr_type);
        if (rc != 0) {
            ESP_LOGE(TAG, "Error determining address type; rc=%d", rc);
            return;
        }

        disc_params.passive           = 1;    // pasivní sken, nevysíláme scan request
        disc_params.filter_duplicates = 0;    // chceme každý paket
        disc_params.itvl              = BLE_GAP_SCAN_ITVL_MS(100);
        disc_params.window            = BLE_GAP_SCAN_WIN_MS(100);

        rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                          on_gap_event_cb, nullptr);
        if (rc != 0) {
            ESP_LOGE(TAG, "Error starting scan; rc=%d", rc);
        } else {
            ESP_LOGI(TAG, "Scanning for ADV_NONCONN_IND...");
        }
    }

    int BleAdvScanner::on_gap_event(struct ble_gap_event *event)
    {
        if (event->type != BLE_GAP_EVENT_DISC) {
            return 0;
        }

        const struct ble_gap_disc_desc *disc = &event->disc;

        if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND) {
            return 0;
        }

        if (!is_whitelisted(disc->addr.val)) {
            return 0;
        }

        bridge::device_state_t state;
        if (parse_adv(disc, state)) {
            xQueueSend(queue_, &state, 0);
        }

        return 0;
    }

    bool BleAdvScanner::parse_adv(const struct ble_gap_disc_desc *disc, bridge::device_state_t &out)
    {
        memcpy(out.mac, disc->addr.val, 6);

        const uint8_t *data = disc->data;
        uint8_t        len  = disc->length_data;
        int            offset = 0;

        while (offset < len) {
            uint8_t ad_len  = data[offset];
            if (ad_len == 0 || offset + ad_len >= len) break;

            uint8_t        ad_type     = data[offset + 1];
            const uint8_t *ad_data     = &data[offset + 2];
            uint8_t        ad_data_len = ad_len - 1;

            // Manufacturer Specific Data: [company_id: 2B][data: 1B]
            if (ad_type == 0xFF && ad_data_len >= 3) {
                out.cmd  = 0;
                out.data = ad_data[2];
                return true;
            }

            offset += ad_len + 1;
        }

        return false;  // žádná manufacturer data nenalezena
    }
}
