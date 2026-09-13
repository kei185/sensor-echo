#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "checksum.h"
#include "lidar/parser.h"
#include "protocol.h"
#include "tx/frame.h"

/*
 * Host build inputs: src/protocol_translate.c, src/lidar/parser.c,
 * src/tx/header.c, src/tx/frame.c, libcrc's crc8.c, and this file.
 * Add src and lib/libcrc-2.0/include to the compiler include paths.
 */

/* Host stand-ins preserve the parser's byte and little-endian read semantics. */
int8_t read_byte(const int8_t* ptr) { return *ptr; }

uint32_t dec_little_endian(const int8_t* ptr, uint8_t len)
{
        uint32_t value = 0u;
        for (uint8_t i = 0u; i < len; ++i)
                value |= (uint32_t)(uint8_t)read_byte(ptr + i) << (i * 8u);
        return value;
}

static void assert_system_frame(
        const uint8_t* frame, size_t size, const char* message, uint32_t timestamp)
{
        const size_t payload_len = strlen(message);
        assert(size == TX_FRAME_HEADER_SIZE + payload_len);
        assert(frame[0] == 0xaau && frame[1] == 0x55u);
        assert(frame[2] == (uint8_t)(payload_len >> 8u));
        assert(frame[3] == (uint8_t)payload_len);
        assert(frame[4] == crc_8(frame + 2u, 2u));
        assert(frame[5] == FRAME_TYPE_SYS);
        assert(frame[6] == (uint8_t)(timestamp >> 24u));
        assert(frame[7] == (uint8_t)(timestamp >> 16u));
        assert(frame[8] == (uint8_t)(timestamp >> 8u));
        assert(frame[9] == (uint8_t)timestamp);
        assert(memcmp(frame + TX_FRAME_HEADER_SIZE, message, payload_len) == 0);
}

static void test_nominal_and_fault_status(void)
{
        int8_t response[] =
                {(int8_t)0xa5, 0x5a, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00};
        uint8_t frame[256] = {0};

        size_t size =
                translate(response, sizeof(response), frame, sizeof(frame), 0x12345678u);
        assert_system_frame(
                frame,
                size,
                "[SENSOR-ECHO] LiDAR STATUS: OK | code=0x00 | sensor=OK encoder=OK "
                "wireless=OK feedback=OK laser=OK data=OK\r\n",
                0x12345678u);

        uint8_t exact[256];
        memset(exact, 0xa5, sizeof(exact));
        assert(translate(response, sizeof(response), exact, size, 0x12345678u) == size);
        assert(memcmp(exact, frame, size) == 0);
        assert(exact[size] == 0xa5u); // No NUL byte is required after the payload.

        response[7] = 0x09;
        size        = translate(response, sizeof(response), frame, sizeof(frame), 0u);
        assert_system_frame(
                frame,
                size,
                "[SENSOR-ECHO] LiDAR STATUS: FAULT | code=0x09 | sensor=FAULT "
                "encoder=OK wireless=OK feedback=FAULT laser=OK data=OK\r\n",
                0u);

        response[7] = (int8_t)0x80;
        size        = translate(response, sizeof(response), frame, sizeof(frame), 0u);
        assert_system_frame(
                frame,
                size,
                "[SENSOR-ECHO] LiDAR STATUS: FAULT | code=0x80 | sensor=OK encoder=OK "
                "wireless=OK feedback=OK laser=OK data=OK\r\n",
                0u);
}

static void test_device_info(void)
{
        int8_t response[27] =
                {(int8_t)0xa5, 0x5a, 0x14, 0x00, 0x00, 0x00, 0x04, (int8_t)151, 2, 7, 3};
        const uint8_t serial[] = {0x00,
                                  0x11,
                                  0x22,
                                  0x33,
                                  0x44,
                                  0x55,
                                  0x66,
                                  0x77,
                                  0x88,
                                  0x99,
                                  0xaa,
                                  0xbb,
                                  0xcc,
                                  0xdd,
                                  0xee,
                                  0xff};
        memcpy(response + 11u, serial, sizeof(serial));

        uint8_t frame[256] = {0};
        size_t  size =
                translate(response, sizeof(response), frame, sizeof(frame), 0x89abcdefu);
        assert_system_frame(
                frame,
                size,
                "[SENSOR-ECHO] LiDAR DEVICE IDENTIFIED | model=151 firmware=2.7 "
                "hardware=3 serial=00112233445566778899AABBCCDDEEFF\r\n",
                0x89abcdefu);

        uint8_t exact[256];
        memset(exact, 0xa5, sizeof(exact));
        assert(translate(response, sizeof(response), exact, size, 0x89abcdefu) == size);
        assert(memcmp(exact, frame, size) == 0);
        assert(exact[size] == 0xa5u);
}

static void assert_rejected(const int8_t* response, size_t response_len, size_t capacity)
{
        uint8_t frame[256];
        memset(frame, 0xa5, sizeof(frame));
        assert(translate(response, response_len, frame, capacity, 0u) == 0u);
        for (size_t i = 0; i < sizeof(frame); ++i)
                assert(frame[i] == 0xa5u);
}

static void test_invalid_responses(void)
{
        int8_t response[] =
                {(int8_t)0xa5, 0x5a, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00};

        assert_rejected(response, 6u, 256u);              // Missing type code.
        assert_rejected(response, 9u, 256u);              // Incomplete content.
        assert_rejected(response, sizeof(response), 20u); // No room for message.
        response[0] = 0x00;
        assert_rejected(response, sizeof(response), 256u); // Wrong start sign.
        response[0] = (int8_t)0xa5;
        response[5] = 0x40;
        assert_rejected(response, sizeof(response), 256u); // Continuous mode.
        response[5] = 0x00;
        response[2] = 0x02;
        assert_rejected(response, sizeof(response), 256u); // Wrong content length.
        response[2] = 0x03;
        response[6] = (int8_t)0x81;
        assert_rejected(response, sizeof(response), 256u); // Scan deferred.
        assert(translate(NULL, 0u, NULL, 0u, 0u) == 0u);
}

static void test_meta_preserves_30_bit_length(void)
{
        const int8_t descriptor[] = {(int8_t)0xa5, 0x5a, 0x03, 0x00, 0x00, 0x3f, 0x06};
        ParserMeta   meta         = {0};
        assert(read_meta(descriptor, sizeof(descriptor), &meta) == &meta);
        assert(meta.res_len == 0x3f000003u);
        assert(meta.res_mode == SYS_RES_MODE_SINGLE);
        assert(meta.type_code == SYS_TYPE_CODE_HEALTH);
}

int main(void)
{
        test_nominal_and_fault_status();
        test_device_info();
        test_invalid_responses();
        test_meta_preserves_30_bit_length();
        return 0;
}
