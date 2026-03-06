#include "unity.h"

#define private public
#include "uart_protocol.hpp"
#undef private

#include <cstring>

using namespace uart;

TEST_CASE("encode writes START byte 0xAA at index 0", "[encode]")
{
    bridge::device_state_t state = {};
    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(state, frame);

    TEST_ASSERT_EQUAL_UINT8(0xAA, frame[0]);
}

TEST_CASE("encode writes cmd and data at correct positions", "[encode]")
{
    bridge::device_state_t state = {};
    state.cmd  = 0x10;
    state.data = 0x42;

    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(state, frame);

    TEST_ASSERT_EQUAL_UINT8(0x10, frame[8]);   // cmd  at index 8
    TEST_ASSERT_EQUAL_UINT8(0x42, frame[9]);   // data at index 9
}

TEST_CASE("encode writes MAC at indices 2-7", "[encode]")
{
    bridge::device_state_t state = {};
    state.mac[0] = 0x11; state.mac[1] = 0x22; state.mac[2] = 0x33;
    state.mac[3] = 0x44; state.mac[4] = 0x55; state.mac[5] = 0x66;

    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(state, frame);

    TEST_ASSERT_EQUAL_MEMORY(state.mac, &frame[2], 6);
}

TEST_CASE("decode round-trip preserves all fields", "[decode]")
{
    bridge::device_state_t original = {};
    original.mac[0] = 0xAA; original.mac[1] = 0xBB; original.mac[2] = 0xCC;
    original.mac[3] = 0xDD; original.mac[4] = 0xEE; original.mac[5] = 0xFF;
    original.cmd  = bridge::CMD_ONOFF_STATUS;
    original.data = 0x01;

    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(original, frame);

    bridge::device_state_t decoded = {};
    TEST_ASSERT_TRUE(UartProtocol::decode(frame, decoded));
    TEST_ASSERT_EQUAL_UINT8(original.cmd,  decoded.cmd);
    TEST_ASSERT_EQUAL_UINT8(original.data, decoded.data);
    TEST_ASSERT_EQUAL_MEMORY(original.mac, decoded.mac, 6);
}

TEST_CASE("decode rejects corrupted CRC", "[decode]")
{
    bridge::device_state_t state = {};
    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(state, frame);
    frame[10] ^= 0xFF;

    bridge::device_state_t out = {};
    TEST_ASSERT_FALSE(UartProtocol::decode(frame, out));
}

TEST_CASE("decode rejects wrong START byte", "[decode]")
{
    bridge::device_state_t state = {};
    uint8_t frame[FRAME_SIZE];
    UartProtocol::encode(state, frame);
    frame[0] = 0x00;

    bridge::device_state_t out = {};
    TEST_ASSERT_FALSE(UartProtocol::decode(frame, out));
}
