#include "nfc_pairing.hpp"

#include <cstring>
#include <cstdio>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "bridge_types.hpp"

static const char *TAG = "nfc_pairing";

// Timeout waiting for user to present a tag after button press
static constexpr uint32_t TAG_READ_TIMEOUT_MS = 5000;

// Hardware debounce threshold (ms)
static constexpr uint32_t DEBOUNCE_MS = 50;

// UART send parameters
static constexpr int      UART_RETRIES    = 3;
static constexpr uint32_t UART_TIMEOUT_MS = 500;

namespace nfc
{

NfcPairing::NfcPairing(Pn532              &pn532,
                       ble::BleAdvScanner &scanner,
                       uart::UartProtocol &uart,
                       gpio_num_t          btn_pair,
                       gpio_num_t          btn_unpair)
    : pn532_(pn532),
      scanner_(scanner),
      uart_(uart),
      btn_pair_(btn_pair),
      btn_unpair_(btn_unpair),
      btn_queue_(nullptr),
      pair_arg_  {nullptr, BtnEvent::PAIR,   0},
      unpair_arg_{nullptr, BtnEvent::UNPAIR, 0}
{}

void NfcPairing::set_event_callback(std::function<void(PairingEvent)> cb)
{
    event_cb_ = cb;
}

void NfcPairing::start()
{
    // Queue size 1 — a second button press while pairing is in progress is dropped
    btn_queue_ = xQueueCreate(1, sizeof(BtnEvent));
    pair_arg_.queue   = btn_queue_;
    unpair_arg_.queue = btn_queue_;

    gpio_install_isr_service(0);
    configure_button(btn_pair_,   &pair_arg_);
    configure_button(btn_unpair_, &unpair_arg_);

    xTaskCreate(pairing_task, "nfc_pair", 4096, this, 5, nullptr);
}

// =============================================================================
// Private — GPIO setup
// =============================================================================

void NfcPairing::configure_button(gpio_num_t pin, BtnIsrArg *arg)
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode         = GPIO_MODE_INPUT;
    cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_NEGEDGE; // active-low button
    gpio_config(&cfg);
    gpio_isr_handler_add(pin, btn_isr_handler, arg);
}

// =============================================================================
// Private — ISR
// =============================================================================

void IRAM_ATTR NfcPairing::btn_isr_handler(void *arg)
{
    BtnIsrArg *ctx = static_cast<BtnIsrArg *>(arg);

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (now_ms - ctx->last_ms < DEBOUNCE_MS) return;
    ctx->last_ms = now_ms;

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(ctx->queue, &ctx->event, &woken);
    portYIELD_FROM_ISR(woken);
}

// =============================================================================
// Private — FreeRTOS task
// =============================================================================

void NfcPairing::pairing_task(void *arg)
{
    static_cast<NfcPairing *>(arg)->pairing_loop();
}

void NfcPairing::pairing_loop()
{
    while (true) {
        BtnEvent evt;
        xQueueReceive(btn_queue_, &evt, portMAX_DELAY);

        notify(PairingEvent::IN_PROGRESS);

        nfc_tag_t tag = {};
        if (!pn532_.read_tag(tag, TAG_READ_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "Tag read timeout");
            notify(PairingEvent::FAILED);
            continue;
        }

        uint8_t mac[6] = {};
        uint8_t device_type = 0;
        if (!parse_tag_data(tag, mac, device_type)) {
            ESP_LOGW(TAG, "NDEF MAC parse failed");
            notify(PairingEvent::FAILED);
            continue;
        }

        ESP_LOGI(TAG, "%s MAC %02X:%02X:%02X:%02X:%02X:%02X",
                 evt == BtnEvent::PAIR ? "PAIR" : "UNPAIR",
                 mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

        bool ok = (evt == BtnEvent::PAIR) ? do_pair(mac, device_type) : do_unpair(mac);
        notify(ok ? PairingEvent::SUCCESS : PairingEvent::FAILED);
    }
}

// =============================================================================
// Private — pair / unpair flows
// =============================================================================

bool NfcPairing::do_pair(const uint8_t mac[6], uint8_t device_type)
{
    if (!scanner_.pair(mac, device_type)) {
        ESP_LOGW(TAG, "BleScanner::pair failed (whitelist full?)");
        return false;
    }

    bridge::device_state_t msg = {};
    memcpy(msg.mac, mac, 6);
    msg.cmd  = bridge::CMD_NODE_ADDED;
    msg.data = device_type;

    if (!uart_.send_reliable(msg, UART_RETRIES, UART_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "NODE_ADDED UART failed — rolling back whitelist");
        scanner_.unpair(mac);
        return false;
    }

    return true;
}

bool NfcPairing::do_unpair(const uint8_t mac[6])
{
    if (!scanner_.unpair(mac)) {
        ESP_LOGW(TAG, "BleScanner::unpair failed (not in whitelist?)");
        return false;
    }

    bridge::device_state_t msg = {};
    memcpy(msg.mac, mac, 6);
    msg.cmd  = bridge::CMD_NODE_REMOVED;
    msg.data = 0x00;

    if (!uart_.send_reliable(msg, UART_RETRIES, UART_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "NODE_REMOVED UART failed — rolling back whitelist");
        scanner_.pair(mac, 0);
        return false;
    }

    return true;
}

// =============================================================================
// Private — NDEF Text record → MAC
// =============================================================================

bool NfcPairing::parse_tag_data(const nfc_tag_t &tag, uint8_t mac[6], uint8_t &device_type)
{
    if (tag.record_type != 'T' || tag.payload_len < 9) return false;

    // Payload: [lang_len(1B)] [lang...] [ASCII "XX:XX:XX:XX:XX:XX,T"]
    uint8_t lang_len = tag.payload[0];
    int     offset   = 1 + lang_len;
    if (offset + 17 > tag.payload_len) return false;

    // Parse 6 hex pairs, big-endian display order
    uint8_t parsed[6];
    const uint8_t *s = tag.payload + offset;
    for (int i = 0; i < 6; i++) {
        unsigned val = 0;
        if (sscanf(reinterpret_cast<const char *>(s + i * 3), "%2x", &val) != 1) return false;
        parsed[i] = (uint8_t)val;
    }

    // Reverse: tag is big-endian ("32:77:..."), bridge stores little-endian
    for (int i = 0; i < 6; i++) mac[i] = parsed[5 - i];

    // Device type za čárkou: "XX:XX:XX:XX:XX:XX,T" (1 hex digit min)
    if (offset + 19 > tag.payload_len || s[17] != ',') return false;
    unsigned dt = 0;
    if (sscanf(reinterpret_cast<const char *>(s + 18), "%x", &dt) != 1) return false;
    device_type = static_cast<uint8_t>(dt);

    return true;
}

// =============================================================================
// Private — helpers
// =============================================================================

void NfcPairing::notify(PairingEvent e)
{
    if (event_cb_) event_cb_(e);
}

} // namespace nfc
