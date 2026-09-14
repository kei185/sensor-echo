#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include "experiments/lidar_rate_probe/scan_meter.h"

static void feed_packet(LidarScanMeter* meter, uint8_t ct, uint8_t lsn)
{
        // Build just enough of a scan packet to exercise the byte counter.
        const uint8_t header[] = {0xaau, 0x55u, ct, lsn};
        for (size_t i = 0u; i < sizeof(header); ++i)
                assert(!lidar_scan_meter_feed(meter, header[i]));
        // Put AA 55 inside the body to prove it is skipped as point data,
        // not mistaken for the start of another packet.
        for (uint16_t i = 0u; i < 6u + 3u * lsn; ++i) {
                bool complete =
                        lidar_scan_meter_feed(meter, i == 0u ? 0xaau : i == 1u ? 0x55u : 0u);
                assert(complete == (i == 5u + 3u * lsn));
        }
        assert(meter->packet_length == SYS_PACKET_SCAN_FIXED_SIZE +
                                               lsn * SYS_PACKET_POINT_DATA_SIZE);
        assert(meter->packet[0] == 0xaau && meter->packet[1] == 0x55u);
        assert(meter->packet[SYS_PACKET_SCAN_FIXED_SIZE] == 0u);
}

int main(void)
{
        LidarScanMeter meter;
        lidar_scan_meter_reset(&meter);

        // The first two-point packet belongs to a partial lap and is excluded
        // from the lap average. The next two complete laps have 4 and 3 points.
        feed_packet(&meter, 0u, 2u);
        feed_packet(&meter, 1u, 1u);
        feed_packet(&meter, 0u, 3u);
        feed_packet(&meter, 1u, 1u);
        feed_packet(&meter, 0u, 2u);
        feed_packet(&meter, 1u, 1u);

        assert(meter.bytes == 10u + 3u * 2u + 10u + 3u * 1u + 10u + 3u * 3u + 10u +
                                      3u * 1u + 10u + 3u * 2u + 10u + 3u * 1u);
        assert(meter.packets == 6u);
        assert(meter.points == 10u);
        assert(meter.complete_laps == 2u);
        assert(meter.complete_lap_points == 7u);
        assert(meter.min_lap_points == 3u);
        assert(meter.max_lap_points == 4u);
        assert(meter.current_lap_points == 1u);

        // A zero-point packet is malformed and must not change packet totals.
        lidar_scan_meter_feed(&meter, 0xaau);
        lidar_scan_meter_feed(&meter, 0x55u);
        lidar_scan_meter_feed(&meter, 1u);
        lidar_scan_meter_feed(&meter, 0u);
        assert(meter.malformed_packets == 1u);
        assert(meter.packets == 6u);

        // LSN uses one byte, so 255 is the largest possible point count.
        lidar_scan_meter_reset(&meter);
        feed_packet(&meter, 0u, 255u);
        assert(meter.bytes == 10u + 3u * 255u);
        assert(meter.packets == 1u);
        assert(meter.points == 255u);

        return 0;
}
