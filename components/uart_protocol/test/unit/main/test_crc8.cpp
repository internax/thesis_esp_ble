/**
 * Unit tests for uart::UartProtocol — calc_crc8, encode, decode
 *
 * Framework: Unity (part of ESP-IDF)
 *
 * Run on device:
 *   idf.py build flash monitor   (from components/uart_protocol/test/)
 */

#include "unity.h"

// Trick: makes private and protected members accessible for this translation unit.
// Works because private/public are compiler-only checks — no difference in memory.
// Use exclusively in test files.
#define private public
#include "uart_protocol.hpp"
#undef private

#include <cstring>

using namespace uart;

// =============================================================================
// TEST GROUP 1 — calc_crc8 isolated tests
// =============================================================================
//
// How these tests work:
//   TEST_CASE("description", "[tag]") defines one test case.
//   TEST_ASSERT_EQUAL_UINT8(expected, actual) compares two values;
//   if they differ, the test fails and Unity prints the difference.
//
// Why these expected values?
//   calc_crc8 implements CRC-8/SMBUS (poly=0x07, init=0x00, no reflection).
//   Values below are verified by manual calculation and the standard table.
// =============================================================================

// Empty input → CRC must stay at init value 0x00
TEST_CASE("CRC-8 empty input returns 0x00", "[crc8]")
{
    // No data passed (len=0), the loop never executes
    uint8_t data[] = {};
    TEST_ASSERT_EQUAL_UINT8(0x00, UartProtocol::calc_crc8(data, 0));
}

// Single zero byte: crc ^= 0x00 → crc=0x00, all shifts stay 0x00
TEST_CASE("CRC-8 single byte 0x00 returns 0x00", "[crc8]")
{
    uint8_t data[] = {0x00};
    TEST_ASSERT_EQUAL_UINT8(0x00, UartProtocol::calc_crc8(data, 1));
}

// Single byte 0x01: after 7 left shifts the bit reaches MSB → XOR poly 0x07 → 0x07
TEST_CASE("CRC-8 single byte 0x01 returns 0x07", "[crc8]")
{
    uint8_t data[] = {0x01};
    TEST_ASSERT_EQUAL_UINT8(0x07, UartProtocol::calc_crc8(data, 1));
}

// Single byte 0xFF: verified step-by-step by hand → 0xF3
TEST_CASE("CRC-8 single byte 0xFF returns 0xF3", "[crc8]")
{
    uint8_t data[] = {0xFF};
    TEST_ASSERT_EQUAL_UINT8(0xF3, UartProtocol::calc_crc8(data, 1));
}

// Standard CRC-8/SMBUS check value:
//   input "123456789" (ASCII bytes) → 0xF4
// Published in Gregory Cook's CRC catalogue.
// If this test passes, the implementation is CRC-8/SMBUS-compliant.
TEST_CASE("CRC-8 SMBUS check value '123456789' is 0xF4", "[crc8]")
{
    const uint8_t data[] = {0x31, 0x32, 0x33, 0x34, 0x35,
                             0x36, 0x37, 0x38, 0x39};
    TEST_ASSERT_EQUAL_UINT8(0xF4, UartProtocol::calc_crc8(data, sizeof(data)));
}

// CRC must be sensitive to data changes — different data, different CRC
TEST_CASE("CRC-8 distinguishes different data", "[crc8]")
{
    uint8_t frame_a[] = {0xAA, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x10, 0x00};
    uint8_t frame_b[] = {0xAA, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x10, 0x01};

    uint8_t crc_a = UartProtocol::calc_crc8(frame_a, sizeof(frame_a));
    uint8_t crc_b = UartProtocol::calc_crc8(frame_b, sizeof(frame_b));

    // Last byte differs by 1 → CRC must differ
    TEST_ASSERT_NOT_EQUAL(crc_a, crc_b);
}

// =============================================================================
// TEST GROUP 2 — encode + decode integration tests
//
// These tests exercise the functions indirectly: encode builds a full frame
// including CRC, decode reads it back and verifies the CRC. If calc_crc8
// is broken, decode returns false and the tests fail.
// =============================================================================

TEST_CASE("encode-decode round-trip preserves data", "[encode][decode]")
{
    bridge::device_state_t original;
    original.mac[0] = 0xAA; original.mac[1] = 0xBB; original.mac[2] = 0xCC;
    original.mac[3] = 0xDD; original.mac[4] = 0xEE; original.mac[5] = 0xFF;
    original.cmd  = bridge::CMD_ONOFF_STATUS;
    original.data = 0x01;

    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(original, frame);

    bridge::device_state_t decoded;
    bool ok = UartProtocol::decode(frame, decoded);

    TEST_ASSERT_TRUE(ok);                                          // CRC must match
    TEST_ASSERT_EQUAL_UINT8(original.cmd,  decoded.cmd);
    TEST_ASSERT_EQUAL_UINT8(original.data, decoded.data);
    TEST_ASSERT_EQUAL_MEMORY(original.mac, decoded.mac, 6);        // compare 6 MAC bytes
}

TEST_CASE("encode writes correct START byte", "[encode]")
{
    bridge::device_state_t state = {};
    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(state, frame);

    // Byte 0 must always be FRAME_START = 0xAA
    TEST_ASSERT_EQUAL_UINT8(FRAME_START, frame[0]);
}

TEST_CASE("decode rejects frame with corrupted CRC", "[decode]")
{
    bridge::device_state_t state;
    memset(state.mac, 0x11, 6);
    state.cmd  = bridge::CMD_NODE_ADDED;
    state.data = 0x00;

    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(state, frame);

    // Deliberately corrupt the CRC byte (index 10)
    frame[10] ^= 0xFF;

    bridge::device_state_t out;
    TEST_ASSERT_FALSE(UartProtocol::decode(frame, out));
}

TEST_CASE("decode rejects frame with missing START byte", "[decode]")
{
    bridge::device_state_t state = {};
    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(state, frame);

    frame[0] = 0x00;   // overwrite START byte

    bridge::device_state_t out;
    TEST_ASSERT_FALSE(UartProtocol::decode(frame, out));
}

// =============================================================================
// Entry point — called by ESP-IDF after system startup
// =============================================================================
extern "C" void app_main(void)
{
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
}
