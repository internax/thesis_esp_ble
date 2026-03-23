/*
 * PN532 driver — ESP-IDF port of Adafruit_PN532 (HSU/UART path only).
 *
 * Original: https://github.com/adafruit/Adafruit-PN532
 * License : BSD
 *
 * Changes vs. original:
 *   - Arduino HardwareSerial  → ESP-IDF uart_read_bytes / uart_write_bytes
 *   - delay()                 → vTaskDelay(pdMS_TO_TICKS())
 *   - available()             → uart_get_buffered_data_len()
 */

#include "pn532.hpp"

#include <cstring>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pn532";

// Frame constants (same as Adafruit)
static constexpr uint8_t PN532_PREAMBLE    = 0x00;
static constexpr uint8_t PN532_STARTCODE1  = 0x00;
static constexpr uint8_t PN532_STARTCODE2  = 0xFF;
static constexpr uint8_t PN532_POSTAMBLE   = 0x00;
static constexpr uint8_t PN532_HOSTTOPN532 = 0xD4;
static constexpr uint8_t PN532_PN532TOHOST = 0xD5;

static constexpr uint8_t CMD_GETFIRMWAREVERSION  = 0x02;
static constexpr uint8_t CMD_SAMCONFIGURATION    = 0x14;
static constexpr uint8_t CMD_INLISTPASSIVETARGET = 0x4A;
static constexpr uint8_t CMD_INDATAEXCHANGE      = 0x40;
static constexpr uint8_t MIFARE_CMD_READ         = 0x30;

static constexpr int  UART_RX_BUF   = 256;
static constexpr int  READY_TIMEOUT = 1000; // ms — waitready default

static const uint8_t PN532_ACK[6]  = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

// Shared packet buffer (mirrors pn532_packetbuffer in Adafruit)
static constexpr size_t PACKBUF_SIZE = 64;
static uint8_t pn532_buf[PACKBUF_SIZE];

namespace nfc
{

Pn532::Pn532(uart_port_t port, int tx_pin, int rx_pin)
    : port_(port), tx_pin_(tx_pin), rx_pin_(rx_pin) {}

// =============================================================================
// Public API
// =============================================================================

bool Pn532::begin()
{
    uart_config_t cfg = {};
    cfg.baud_rate  = 115200;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_param_config(port_, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port_, tx_pin_, rx_pin_,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(port_, UART_RX_BUF, 0, 0, nullptr, 0));

    // Retry smyčka: PN532 při souběžném bootu s ESP32 potřebuje čas.
    // Zkusíme každých 200 ms, max 25× (= 5 s).
    for (int attempt = 0; attempt < 25; attempt++) {
        uart_flush_input(port_);
        wakeup();
        if (sam_config()) {
            ESP_LOGI(TAG, "PN532 ready (attempt %d)", attempt + 1);
            return true;
        }
        ESP_LOGW(TAG, "SAMConfig attempt %d failed, retrying...", attempt + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGE(TAG, "PN532 did not respond after 5 s");
    return false;
}

void Pn532::wakeup()
{
    const uint8_t w[3] = {0x55, 0x00, 0x00};
    uart_write_bytes(port_, w, sizeof(w));
    vTaskDelay(pdMS_TO_TICKS(2));
}

bool Pn532::sam_config()
{
    pn532_buf[0] = CMD_SAMCONFIGURATION;
    pn532_buf[1] = 0x01; // Normal mode
    pn532_buf[2] = 0x14; // timeout
    pn532_buf[3] = 0x01; // IRQ pin enabled

    if (!send_command_check_ack(pn532_buf, 4)) return false;

    read_data(pn532_buf, 9);
    return (pn532_buf[6] == 0x15);
}

bool Pn532::get_firmware_version(uint8_t &ic, uint8_t &ver, uint8_t &rev)
{
    static const uint8_t expected[6] = {0x00, 0x00, 0xFF, 0x06, 0xFA, 0xD5};

    pn532_buf[0] = CMD_GETFIRMWAREVERSION;
    if (!send_command_check_ack(pn532_buf, 1)) return false;

    read_data(pn532_buf, 13);
    if (memcmp(pn532_buf, expected, 6) != 0) return false;

    ic  = pn532_buf[7];
    ver = pn532_buf[8];
    rev = pn532_buf[9];
    ESP_LOGI(TAG, "Firmware: IC=0x%02X v%d.%d", ic, ver, rev);
    return true;
}

bool Pn532::read_tag(nfc_tag_t &tag, uint32_t timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        pn532_buf[0] = CMD_INLISTPASSIVETARGET;
        pn532_buf[1] = 1;    // max 1 target
        pn532_buf[2] = 0x00; // ISO14443A @ 106 kbps

        if (!send_command_check_ack(pn532_buf, 3, 1000)) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        read_data(pn532_buf, 20);

        // pn532_buf[7] = NbTg
        if (pn532_buf[7] != 1) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // pn532_buf[8]  = Tg
        // pn532_buf[9..10] = ATQA
        // pn532_buf[11] = SAK
        // pn532_buf[12] = UID length
        // pn532_buf[13..] = UID
        uint8_t uid_len = pn532_buf[12];
        ESP_LOGD(TAG, "Tag found, UID len=%d", uid_len);

        // Read NDEF from pages 4–11 (2 × READ = 32 bytes)
        uint8_t raw[32];
        uint8_t page4[16], page8[16];
        if (!read_pages(1, 4, page4) || !read_pages(1, 8, page8)) {
            ESP_LOGW(TAG, "Page read failed");
            return false;
        }
        memcpy(raw,      page4, 16);
        memcpy(raw + 16, page8, 16);

        return parse_ndef(raw, sizeof(raw), tag);
    }

    return false;
}

// =============================================================================
// Private — tag memory
// =============================================================================

bool Pn532::read_pages(uint8_t tg, uint8_t start_page, uint8_t out[16])
{
    pn532_buf[0] = CMD_INDATAEXCHANGE;
    pn532_buf[1] = tg;
    pn532_buf[2] = MIFARE_CMD_READ;
    pn532_buf[3] = start_page;

    if (!send_command_check_ack(pn532_buf, 4)) return false;

    read_data(pn532_buf, 26);
    if (pn532_buf[7] != 0x00) {
        ESP_LOGW(TAG, "read_pages status=0x%02X", pn532_buf[7]);
        return false;
    }
    memcpy(out, pn532_buf + 8, 16);
    return true;
}

// =============================================================================
// Private — NDEF parser (unchanged)
// =============================================================================

bool Pn532::parse_ndef(const uint8_t *raw, uint8_t raw_len, nfc_tag_t &tag)
{
    int i = 0;
    while (i < raw_len) {
        uint8_t tlv_type = raw[i++];
        if (tlv_type == 0xFE) break;
        if (tlv_type == 0x00) continue;
        if (i >= raw_len) return false;

        uint8_t tlv_len = raw[i++];
        if (tlv_type != 0x03) { i += tlv_len; continue; }

        // NDEF Message TLV
        if (i + 2 > raw_len) return false;
        uint8_t flags    = raw[i++];
        uint8_t type_len = raw[i++];
        bool    sr       = (flags & 0x10) != 0;
        bool    il       = (flags & 0x08) != 0;

        uint8_t pay_len;
        if (sr) {
            if (i >= raw_len) return false;
            pay_len = raw[i++];
        } else {
            if (i + 4 > raw_len) return false;
            pay_len = raw[i + 3];
            i += 4;
        }

        if (il) {
            if (i >= raw_len) return false;
            i += 1 + raw[i];
        }

        if (i + type_len > raw_len) return false;
        tag.record_type = (type_len > 0) ? raw[i] : 0;
        i += type_len;

        if (pay_len > PN532_PAYLOAD_MAX || i + pay_len > raw_len) return false;
        memcpy(tag.payload, raw + i, pay_len);
        tag.payload_len = pay_len;
        return true;
    }
    return false;
}

// =============================================================================
// Private — Adafruit-ported low-level HSU communication
// =============================================================================

// Wait until at least 1 byte is available in UART RX buffer
bool Pn532::wait_ready(uint32_t timeout_ms)
{
    size_t available = 0;
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        uart_get_buffered_data_len(port_, &available);
        if (available > 0) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed += 10;
    }
    ESP_LOGW(TAG, "wait_ready timeout (%lu ms)", (unsigned long)timeout_ms);
    return false;
}

// Read exactly n bytes from UART
void Pn532::read_data(uint8_t *buf, uint8_t n)
{
    int got = uart_read_bytes(port_, buf, n, pdMS_TO_TICKS(500));
    if (got != n) {
        ESP_LOGW(TAG, "read_data: wanted %d, got %d", n, got);
    }
}

// Read ACK frame {0x00 0x00 0xFF 0x00 0xFF 0x00}
bool Pn532::read_ack()
{
    uint8_t ack[6];
    read_data(ack, 6);
    return (memcmp(ack, PN532_ACK, 6) == 0);
}

// Build and send a PN532 HSU frame, then wait for ACK
bool Pn532::send_command_check_ack(uint8_t *cmd, uint8_t cmdlen, uint32_t timeout_ms)
{
    write_command(cmd, cmdlen);

    if (!wait_ready(timeout_ms)) return false;
    if (!read_ack()) return false;
    if (!wait_ready(timeout_ms)) return false;

    return true;
}

// Encode and write a PN532 command frame to UART
void Pn532::write_command(uint8_t *cmd, uint8_t cmdlen)
{
    uint8_t LEN = cmdlen + 1; // TFI + cmd bytes
    uint8_t packet[8 + cmdlen];

    packet[0] = PN532_PREAMBLE;
    packet[1] = PN532_STARTCODE1;
    packet[2] = PN532_STARTCODE2;
    packet[3] = LEN;
    packet[4] = (uint8_t)(~LEN + 1); // LCS
    packet[5] = PN532_HOSTTOPN532;

    uint8_t sum = 0;
    for (uint8_t i = 0; i < cmdlen; i++) {
        packet[6 + i] = cmd[i];
        sum += cmd[i];
    }
    packet[6 + cmdlen] = (uint8_t)(~(PN532_HOSTTOPN532 + sum) + 1); // DCS
    packet[7 + cmdlen] = PN532_POSTAMBLE;

    uart_flush_input(port_);
    uart_write_bytes(port_, packet, 8 + cmdlen);
}

} // namespace nfc
