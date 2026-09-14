#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "experiments/lidar_rate_probe/scan_meter.h"
#include "checksum.h"
#include "lidar/sys.h"
#include "tx/frame.h"
#include "tx/scan.h"

// Other parser functions still use the normal DMA byte reader. A complete
// scan packet must not need it; fail the test if that changes.
int8_t read_byte(const int8_t* buf)
{
        (void)buf;
        abort();
}

uint32_t dec_little_endian(const int8_t* buf, uint8_t len)
{
        (void)buf;
        (void)len;
        abort();
}

static void test_two_points(void)
{
        // 0 and 90 degrees in Q6; distances 100 and 200.
        const uint8_t packet[] = {
                0xaau,
                0x55u,
                0u,
                2u,
                1u,
                0u,
                1u,
                0x2du,
                0u,
                0u,
                0u,
                0x90u,
                0x01u,
                0u,
                0x20u,
                0x03u,
        };
        LidarScanMeter meter;
        lidar_scan_meter_reset(&meter);
        for (size_t i = 0u; i < sizeof(packet); ++i)
                assert(lidar_scan_meter_feed(&meter, packet[i]) ==
                       (i == sizeof(packet) - 1u));
        assert(meter.packet_length == sizeof(packet));

        uint8_t tx[TX_FRAME_HEADER_SIZE + 2u * 4u];
        memset(tx, 0xcc, sizeof(tx));

        assert(tx_scan_frame_write(
                       meter.packet,
                       meter.packet_length,
                       tx,
                       sizeof(tx),
                       0x12345678u) == sizeof(tx));
        const uint8_t expected_payload[] = {100u, 0u, 0u, 0u, 200u, 0u, 0x80u, 0x16u};
        assert(memcmp(tx + TX_FRAME_HEADER_SIZE,
                      expected_payload,
                      sizeof(expected_payload)) == 0);
        assert(tx[0] == 0xaau && tx[1] == 0x55u);
        assert(tx[2] == 0u && tx[3] == sizeof(expected_payload));
        assert(tx[4] == crc_8(tx + 2u, 2u));
        assert(tx[5] == FRAME_TYPE_LIDAR);
        assert(tx[6] == 0x12u && tx[7] == 0x34u && tx[8] == 0x56u && tx[9] == 0x78u);

        // Invalid or incomplete packets must not make a PC frame.
        assert(tx_scan_frame_write(packet, sizeof(packet) - 1u, tx, sizeof(tx), 0u) ==
               0u);
        assert(tx_scan_frame_write(packet, sizeof(packet), tx, sizeof(tx) - 1u, 0u) ==
               0u);
        uint8_t bad_packet[sizeof(packet)];
        memcpy(bad_packet, packet, sizeof(packet));
        bad_packet[0] = 0u;
        assert(tx_scan_frame_write(bad_packet, sizeof(bad_packet), tx, sizeof(tx), 0u) ==
               0u);
        memcpy(bad_packet, packet, sizeof(packet));
        bad_packet[4] = 0u; // FSA check bit must be set.
        assert(tx_scan_frame_write(bad_packet, sizeof(bad_packet), tx, sizeof(tx), 0u) ==
               0u);
        memcpy(bad_packet, packet, sizeof(packet));
        bad_packet[2] = 1u; // A lap-start packet must have one point.
        assert(tx_scan_frame_write(bad_packet, sizeof(bad_packet), tx, sizeof(tx), 0u) ==
               0u);
}

static void test_angle_wrap(void)
{
        // Three points from 350 through 0 to 10 degrees.
        const uint8_t packet[] = {
                0xaau, 0x55u, 0u, 3u, 1u, 0xafu, 1u, 5u, 0u, 0u,
                0u,    4u,    0u, 0u, 4u, 0u,    0u, 4u, 0u,
        };
        uint8_t tx[TX_FRAME_HEADER_SIZE + 3u * 4u];
        assert(tx_scan_frame_write(packet, sizeof(packet), tx, sizeof(tx), 0u) ==
               sizeof(tx));
        assert(tx[TX_FRAME_HEADER_SIZE + 2u] == 0x80u);
        assert(tx[TX_FRAME_HEADER_SIZE + 3u] == 0x57u); // 350 * 64.
        assert(tx[TX_FRAME_HEADER_SIZE + 6u] == 0u);
        assert(tx[TX_FRAME_HEADER_SIZE + 7u] == 0u);
        assert(tx[TX_FRAME_HEADER_SIZE + 10u] == 0x80u);
        assert(tx[TX_FRAME_HEADER_SIZE + 11u] == 0x02u); // 10 * 64.
}

static void test_largest_packet(void)
{
        uint8_t packet[SYS_PACKET_SCAN_FIXED_SIZE + 255u * SYS_PACKET_POINT_DATA_SIZE] = {
                0xaau,
                0x55u,
                0u,
                255u,
                1u,
                0u,
                1u,
                0u,
                0u,
                0u,
        };
        for (size_t i = 0u; i < 255u; ++i)
                packet[SYS_PACKET_SCAN_FIXED_SIZE + i * SYS_PACKET_POINT_DATA_SIZE + 1u] =
                        4u; // Distance 1.

        uint8_t tx[TX_FRAME_HEADER_SIZE + 255u * 4u];
        assert(tx_scan_frame_write(packet, sizeof(packet), tx, sizeof(tx), 0u) ==
               sizeof(tx));
        assert(tx[2] == 0x03u && tx[3] == 0xfcu); // 1020 payload bytes.
        assert(tx[TX_FRAME_HEADER_SIZE] == 1u);
        assert(tx[sizeof(tx) - 1u] == 0u);
}

int main(void)
{
        test_two_points();
        test_angle_wrap();
        test_largest_packet();
        return 0;
}
