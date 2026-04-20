#include "nrf_ble.hpp"
#include "services/gap/ble_svc_gap.h"
#include "nvs.h"
#include <cstring>

extern "C" void ble_store_config_init(void);

static const char *TAG = "BleAdvScanner";

namespace ble
{
    BleAdvScanner *BleAdvScanner::instance_ = nullptr;

    // -------------------------------------------------------------------------
    // Public
    // -------------------------------------------------------------------------

    BleAdvScanner::BleAdvScanner(uart::UartProtocol &uart)
        : uart_(uart), status_queue_(nullptr), whitelist_count_(0)
    {
        assert(instance_ == nullptr && "BleAdvScanner instance can exist only once!");
        instance_ = this;
        whitelist_mutex_ = xSemaphoreCreateMutex();
        status_queue_    = xQueueCreate(4, sizeof(bridge::device_state_t));
    }

    void BleAdvScanner::load_from_nvs()
    {
        nvs_handle_t handle;
        if (nvs_open("ble_wl", NVS_READONLY, &handle) != ESP_OK) {
            ESP_LOGI(TAG, "NVS: whitelist not found, starting empty");
            return;
        }

        uint8_t count = 0;
        nvs_get_u8(handle, "count", &count);

        if (count > bridge::MAX_DEVICES) count = bridge::MAX_DEVICES;

        if (count > 0) {
            uint8_t macs[bridge::MAX_DEVICES][6];
            uint8_t types[bridge::MAX_DEVICES] = {};
            size_t  size = count * 6;
            if (nvs_get_blob(handle, "macs", macs, &size) == ESP_OK) {
                size_t tsize = count;
                nvs_get_blob(handle, "types", types, &tsize);  // best-effort, default 0 if missing
                xSemaphoreTake(whitelist_mutex_, portMAX_DELAY);
                for (uint8_t i = 0; i < count; i++) {
                    memcpy(whitelist_[i].mac, macs[i], 6);
                    whitelist_[i].device_type = types[i];
                    whitelist_[i].last_data   = 0xFF;
                    whitelist_[i].valid       = true;
                }
                whitelist_count_ = count;
                xSemaphoreGive(whitelist_mutex_);
                ESP_LOGI(TAG, "NVS: loaded %d device(s) from whitelist", count);
            }
        }

        nvs_close(handle);
    }

    void BleAdvScanner::save_to_nvs()
    {
        nvs_handle_t handle;
        if (nvs_open("ble_wl", NVS_READWRITE, &handle) != ESP_OK) {
            ESP_LOGE(TAG, "NVS: failed to open for writing");
            return;
        }

        uint8_t macs[bridge::MAX_DEVICES][6];
        uint8_t types[bridge::MAX_DEVICES];
        for (size_t i = 0; i < whitelist_count_; i++) {
            memcpy(macs[i], whitelist_[i].mac, 6);
            types[i] = whitelist_[i].device_type;
        }

        nvs_set_u8(handle, "count", (uint8_t)whitelist_count_);
        nvs_set_blob(handle, "macs", macs, whitelist_count_ * 6);
        nvs_set_blob(handle, "types", types, whitelist_count_);
        nvs_commit(handle);
        nvs_close(handle);

        ESP_LOGI(TAG, "NVS: saved %d device(s) to whitelist", whitelist_count_);
    }

    void BleAdvScanner::clear_all()
    {
        xSemaphoreTake(whitelist_mutex_, portMAX_DELAY);
        memset(whitelist_, 0, sizeof(whitelist_));
        whitelist_count_ = 0;
        xSemaphoreGive(whitelist_mutex_);

        nvs_handle_t handle;
        if (nvs_open("ble_wl", NVS_READWRITE, &handle) == ESP_OK) {
            nvs_erase_all(handle);
            nvs_commit(handle);
            nvs_close(handle);
        }

        ESP_LOGW(TAG, "Whitelist cleared (factory reset)");
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
        xTaskCreate(status_task, "ble_status_tx", 4096, this, 4, nullptr);
    }

    bool BleAdvScanner::pair(const uint8_t mac[6], uint8_t device_type)
    {
        xSemaphoreTake(whitelist_mutex_, portMAX_DELAY);
        // Idempotent: already in whitelist is OK
        for (size_t i = 0; i < whitelist_count_; i++) {
            if (memcmp(whitelist_[i].mac, mac, 6) == 0) {
                xSemaphoreGive(whitelist_mutex_);
                return true;
            }
        }
        bool ok = false;
        if (whitelist_count_ < bridge::MAX_DEVICES) {
            memcpy(whitelist_[whitelist_count_].mac, mac, 6);
            whitelist_[whitelist_count_].device_type = device_type;
            whitelist_[whitelist_count_].last_data   = 0xFF;  // neplatná hodnota → první paket vždy projde
            whitelist_[whitelist_count_].valid        = true;
            whitelist_count_++;
            ok = true;
        }
        xSemaphoreGive(whitelist_mutex_);
        if (ok) save_to_nvs();
        return ok;
    }

    bool BleAdvScanner::unpair(const uint8_t mac[6])
    {
        xSemaphoreTake(whitelist_mutex_, portMAX_DELAY);
        bool ok = false;
        for (size_t i = 0; i < whitelist_count_; i++) {
            if (memcmp(whitelist_[i].mac, mac, 6) == 0) {
                whitelist_count_--;
                whitelist_[i] = whitelist_[whitelist_count_];  // přepíš mazaný posledním
                whitelist_[whitelist_count_].valid = false;
                ok = true;
                break;
            }
        }
        xSemaphoreGive(whitelist_mutex_);
        if (ok) save_to_nvs();
        return ok;
    }

    bool BleAdvScanner::is_whitelisted(const uint8_t mac[6])
    {
        xSemaphoreTake(whitelist_mutex_, portMAX_DELAY);
        if (whitelist_count_ == 0) {
            xSemaphoreGive(whitelist_mutex_);
            return false;
        }
        for (size_t i = 0; i < whitelist_count_; i++) {
            if (memcmp(whitelist_[i].mac, mac, 6) == 0) {
                xSemaphoreGive(whitelist_mutex_);
                return true;
            }
        }
        xSemaphoreGive(whitelist_mutex_);
        return false;
    }

    // Volá se pod zamčeným mutexem — vrátí pointer na záznam nebo nullptr
    BleAdvScanner::whitelist_entry_t *BleAdvScanner::find_entry(const uint8_t mac[6])
    {
        for (size_t i = 0; i < whitelist_count_; i++) {
            if (memcmp(whitelist_[i].mac, mac, 6) == 0) {
                return &whitelist_[i];
            }
        }
        return nullptr;
    }

    // -------------------------------------------------------------------------
    // Statické callbacky — pouze přeposílají volání na instanci
    // -------------------------------------------------------------------------

    void BleAdvScanner::status_task(void *arg)
    {
        static_cast<BleAdvScanner *>(arg)->status_loop();
    }

    void BleAdvScanner::status_loop()
    {
        static constexpr int      RETRIES    = 3;
        static constexpr uint32_t TIMEOUT_MS = 500;

        bridge::device_state_t state;
        while (true) {
            if (xQueueReceive(status_queue_, &state, portMAX_DELAY) == pdTRUE) {
                if (!uart_.send_reliable(state, RETRIES, TIMEOUT_MS)) {
                    ESP_LOGW(TAG, "STATUS send failed after %d attempts", RETRIES);
                }
            }
        }
    }

    void BleAdvScanner::host_task(void *param)
    {
        ESP_LOGI(TAG, "BLE Host Task Started");
        nimble_port_run();              // spuštění ble tasku, blokuje dokud BLE stack běží
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

    void BleAdvScanner::on_sync() // ověření mac přiřazení mac adresy a spuštění scanu
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

        int rc = ble_hs_id_infer_auto(0, &own_addr_type); // podíváme se jaké adresy jsou k dispozici, preferujeme adresu zařízení a uložíme dostupný typ
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

        bridge::device_state_t state;
        if (!parse_adv(disc, state)) {
            return 0;
        }

        xSemaphoreTake(whitelist_mutex_, portMAX_DELAY);
        whitelist_entry_t *entry = find_entry(disc->addr.val);
        if (!entry) {
            xSemaphoreGive(whitelist_mutex_);
            return 0;
        }
        bool changed = (entry->last_data != state.data);
        if (changed) {
            entry->last_data = state.data;
        }
        xSemaphoreGive(whitelist_mutex_);

        if((entry->device_type == 0x03)&&(state.data))
        {
            state.data = 0xEE;

            if (changed) {
            ESP_LOGI(TAG, "BLE RX MAC=%02X:%02X:%02X:%02X:%02X:%02X polarita=%s", state.mac[5], state.mac[4], state.mac[3],
                     state.mac[2], state.mac[1], state.mac[0],state.data ? "ON" : "OFF");
            xQueueSend(status_queue_, &state, 0);
        }

        }

        else if((entry->device_type == 0x01)||(entry->device_type == 0x02))
        {
            if (changed) 
            {
                ESP_LOGI(TAG, "BLE RX MAC=%02X:%02X:%02X:%02X:%02X:%02X polarita=%s", state.mac[5], state.mac[4], state.mac[3],
                         state.mac[2], state.mac[1], state.mac[0],state.data ? "ON" : "OFF");
                xQueueSend(status_queue_, &state, 0);
            }
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
                out.cmd  = bridge::CMD_ONOFF_STATUS;
                out.data = ad_data[2];
                return true;
            }

            offset += ad_len + 1;
        }

        return false;  // žádná manufacturer data nenalezena
    }
}
