#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "tx/header.h"

static void test_complete_frame(void)
{
        uint8_t frame[TX_FRAME_HEADER_SIZE + 3u] = {0};
        frame[TX_FRAME_HEADER_SIZE]              = 0x01u;
        frame[TX_FRAME_HEADER_SIZE + 1u]         = 0x02u;
        frame[TX_FRAME_HEADER_SIZE + 2u]         = 0x03u;

        size_t size = tx_frame_write_header(
                frame,
                sizeof(frame),
                3u,
                TX_FRAME_TYPE_LIDAR,
                0x12345678u);
        const uint8_t expected[] = {0xaau,
                                    0x55u,
                                    0x00u,
                                    0x03u,
                                    0x4eu,
                                    0x01u,
                                    0x12u,
                                    0x34u,
                                    0x56u,
                                    0x78u,
                                    0x01u,
                                    0x02u,
                                    0x03u};

        assert(size == sizeof(expected));
        assert(memcmp(frame, expected, sizeof(expected)) == 0);
}

static void test_multibyte_length(void)
{
        uint8_t frame[TX_FRAME_HEADER_SIZE + 258u] = {0};
        for (size_t i = 0; i < 258u; ++i)
                frame[TX_FRAME_HEADER_SIZE + i] = (uint8_t)i;

        size_t size = tx_frame_write_header(
                frame,
                sizeof(frame),
                258u,
                TX_FRAME_TYPE_ENCODER,
                0x89abcdefu);

        assert(size == sizeof(frame));
        assert(frame[2] == 0x01u);
        assert(frame[3] == 0x02u);
        assert(frame[4] == 0xdcu);
        assert(frame[9] == 0xefu);
        assert(frame[TX_FRAME_HEADER_SIZE] == 0x00u);
        assert(frame[sizeof(frame) - 1u] == 0x01u);
}

static void test_empty_payload(void)
{
        uint8_t frame[TX_FRAME_HEADER_SIZE] = {0};
        size_t  size =
                tx_frame_write_header(frame, sizeof(frame), 0u, TX_FRAME_TYPE_SYSTEM, 0u);

        assert(size == sizeof(frame));
        assert(frame[4] == 0xbeu);
}

static void test_invalid_arguments_leave_buffer_unchanged(void)
{
        uint8_t frame[TX_FRAME_HEADER_SIZE + 1u];
        uint8_t original[sizeof(frame)];
        memset(frame, 0xa5, sizeof(frame));
        memcpy(original, frame, sizeof(frame));

        assert(tx_frame_write_header(NULL, sizeof(frame), 1u, TX_FRAME_TYPE_LIDAR, 0u) ==
               0u);
        assert(tx_frame_write_header(
                       frame,
                       TX_FRAME_HEADER_SIZE - 1u,
                       0u,
                       TX_FRAME_TYPE_LIDAR,
                       0u) == 0u);
        assert(tx_frame_write_header(frame, sizeof(frame), 2u, TX_FRAME_TYPE_LIDAR, 0u) ==
               0u);
        assert(tx_frame_write_header(frame, sizeof(frame), 1u, (TxFrameType)4, 0u) == 0u);
        assert(memcmp(frame, original, sizeof(frame)) == 0);
}

int main(void)
{
        test_complete_frame();
        test_multibyte_length();
        test_empty_payload();
        test_invalid_arguments_leave_buffer_unchanged();
        return 0;
}
