#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "tx/header.h"
#include "checksum.h"

/*
 * These tests follow the caller's order of operations: write the payload at
 * buffer + TX_FRAME_HEADER_SIZE, then ask tx_frame_write_header() to fill the
 * reserved bytes before it. The expected byte arrays are complete wire frames,
 * so a failed comparison identifies a wrong field order or overwritten payload.
 *
 * Host build inputs are src/tx/frame.c, src/tx/header.c, libcrc's crc8.c, and
 * this file. Add src and lib/libcrc-2.0/include to the compiler include paths.
 */

/** Confirm that the selected libcrc CRC-8 matches the documented check value. */
static void test_libcrc_check_value(void)
{
        const uint8_t input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        assert(crc_8(input, sizeof(input)) == 0xa2u);
}

/**
 * Confirm that a pre-filled three-byte LiDAR payload is unchanged, the header
 * matches the specified wire order, and the return value is the DMA byte count.
 */
static void test_complete_frame(void)
{
        // Reserve ten bytes for the header; place the payload after that space.
        uint8_t frame[TX_FRAME_HEADER_SIZE + 3u] = {0};
        frame[TX_FRAME_HEADER_SIZE]              = 0x01u;
        frame[TX_FRAME_HEADER_SIZE + 1u]         = 0x02u;
        frame[TX_FRAME_HEADER_SIZE + 2u]         = 0x03u;

        size_t size = tx_frame_write_header(
                frame,
                sizeof(frame),
                3u,
                FRAME_TYPE_LIDAR,
                0x12345678u);
        // Length 0x0003 has libcrc CRC 0x53. The CRC covers those two bytes only.
        const uint8_t expected[] = {0xaau,
                                    0x55u,
                                    0x00u,
                                    0x03u,
                                    0x53u,
                                    0x01u,
                                    0x12u,
                                    0x34u,
                                    0x56u,
                                    0x78u,
                                    0x01u,
                                    0x02u,
                                    0x03u};

        // The DMA must send the ten-byte header and all three payload bytes.
        assert(size == sizeof(expected));
        // This also checks that header generation did not modify the payload.
        assert(memcmp(frame, expected, sizeof(expected)) == 0);
}

/**
 * Confirm that a length above 255 is encoded high byte first. The timestamp
 * and the last payload byte also guard against byte swaps or a short transfer.
 */
static void test_multibyte_length(void)
{
        // A 258-byte payload gives a nonzero high byte in the length field.
        uint8_t frame[TX_FRAME_HEADER_SIZE + 258u] = {0};
        for (size_t i = 0; i < 258u; ++i)
                frame[TX_FRAME_HEADER_SIZE + i] = (uint8_t)i;

        size_t size = tx_frame_write_header(
                frame,
                sizeof(frame),
                258u,
                FRAME_TYPE_ENC,
                0x89abcdefu);

        assert(size == sizeof(frame));
        // 258 decimal is 0x0102, so the wire bytes must be 0x01 then 0x02.
        assert(frame[2] == 0x01u);
        assert(frame[3] == 0x02u);
        // libcrc over the two length bytes 0x01 0x02 returns 0x96.
        assert(frame[4] == 0x96u);
        // The final timestamp byte is 0xEF in big-endian wire order.
        assert(frame[9] == 0xefu);
        // Both ends of the already-written payload must remain in place.
        assert(frame[TX_FRAME_HEADER_SIZE] == 0x00u);
        assert(frame[sizeof(frame) - 1u] == 0x01u);
}

/** Confirm that a zero-length payload produces a valid header-only frame. */
static void test_empty_payload(void)
{
        uint8_t frame[TX_FRAME_HEADER_SIZE] = {0};
        size_t size = tx_frame_write_header(frame, sizeof(frame), 0u, FRAME_TYPE_SYS, 0u);

        // No payload bytes are sent, but the complete ten-byte header is sent.
        assert(size == sizeof(frame));
        // CRC over 0x00 0x00 with an initial CRC of zero is zero.
        assert(frame[4] == 0x00u);
}

/**
 * Confirm that payload contents do not affect the current length-only CRC.
 * This is intentional until the documented full-frame CRC follow-up is done.
 */
static void test_crc_uses_length_only(void)
{
        uint8_t first[TX_FRAME_HEADER_SIZE + 2u]  = {0};
        uint8_t second[TX_FRAME_HEADER_SIZE + 2u] = {0};
        first[TX_FRAME_HEADER_SIZE]               = 0x11u;
        second[TX_FRAME_HEADER_SIZE]              = 0xeeu;

        assert(tx_frame_write_header(
                       first,
                       sizeof(first),
                       2u,
                       FRAME_TYPE_LIDAR,
                       0x01020304u) == sizeof(first));
        assert(tx_frame_write_header(
                       second,
                       sizeof(second),
                       2u,
                       FRAME_TYPE_LIDAR,
                       0x01020304u) == sizeof(second));

        // Different payload bytes must leave the CRC equal when lengths match.
        assert(first[TX_FRAME_HEADER_SIZE] != second[TX_FRAME_HEADER_SIZE]);
        assert(first[4] == second[4]);
}

/**
 * Confirm that invalid calls return zero before writing any header bytes.
 * The 0xA5 fill pattern makes an accidental partial header easy to detect.
 */
static void test_invalid_arguments_leave_buffer_unchanged(void)
{
        uint8_t frame[TX_FRAME_HEADER_SIZE + 1u];
        uint8_t original[sizeof(frame)];
        memset(frame, 0xa5, sizeof(frame));
        memcpy(original, frame, sizeof(frame));

        // No destination buffer is available.
        assert(tx_frame_write_header(NULL, sizeof(frame), 1u, FRAME_TYPE_LIDAR, 0u) ==
               0u);
        // The stated capacity cannot hold even the fixed header.
        assert(tx_frame_write_header(
                       frame,
                       TX_FRAME_HEADER_SIZE - 1u,
                       0u,
                       FRAME_TYPE_LIDAR,
                       0u) == 0u);
        // The header fits, but two payload bytes do not fit after it.
        assert(tx_frame_write_header(frame, sizeof(frame), 2u, FRAME_TYPE_LIDAR, 0u) ==
               0u);
        // Type 4 is outside the four frame types defined by the protocol.
        assert(tx_frame_write_header(frame, sizeof(frame), 1u, (FrameType)4, 0u) == 0u);
        // All failed calls must leave the original buffer bytes untouched.
        assert(memcmp(frame, original, sizeof(frame)) == 0);
}

int main(void)
{
        test_libcrc_check_value();
        test_complete_frame();
        test_multibyte_length();
        test_empty_payload();
        test_crc_uses_length_only();
        test_invalid_arguments_leave_buffer_unchanged();
        return 0;
}
