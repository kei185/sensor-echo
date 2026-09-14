#include "experiments/lidar_rate_probe/scan_meter.h"

#include <string.h>

enum
{
        SEEK_FIRST_HEADER_BYTE,
        SEEK_SECOND_HEADER_BYTE,
        READ_CT,
        READ_LSN,
        SKIP_PACKET_BODY
};

void lidar_scan_meter_reset(LidarScanMeter* meter) { memset(meter, 0, sizeof(*meter)); }

static void complete_packet(LidarScanMeter* meter)
{
        ++meter->packets;
        meter->points += meter->lsn;

        if ((meter->ct & 1u) != 0u) {
                if (meter->have_lap) {
                        if (meter->complete_laps == 0u ||
                            meter->current_lap_points < meter->min_lap_points)
                                meter->min_lap_points = meter->current_lap_points;
                        if (meter->current_lap_points > meter->max_lap_points)
                                meter->max_lap_points = meter->current_lap_points;
                        meter->complete_lap_points += meter->current_lap_points;
                        ++meter->complete_laps;
                }
                meter->current_lap_points = 0u;
                meter->have_lap           = true;
        }

        if (meter->have_lap)
                meter->current_lap_points += meter->lsn;
        meter->state = SEEK_FIRST_HEADER_BYTE;
}

void lidar_scan_meter_feed(LidarScanMeter* meter, uint8_t byte)
{
        ++meter->bytes;

        switch (meter->state) {
                case SEEK_FIRST_HEADER_BYTE:
                        if (byte == 0xaau)
                                meter->state = SEEK_SECOND_HEADER_BYTE;
                        break;

                case SEEK_SECOND_HEADER_BYTE:
                        if (byte == 0x55u)
                                meter->state = READ_CT;
                        else if (byte != 0xaau)
                                meter->state = SEEK_FIRST_HEADER_BYTE;
                        break;

                case READ_CT:
                        meter->ct    = byte;
                        meter->state = READ_LSN;
                        break;

                case READ_LSN:
                        if (byte == 0u || ((meter->ct & 1u) != 0u && byte != 1u)) {
                                ++meter->malformed_packets;
                                meter->state = SEEK_FIRST_HEADER_BYTE;
                                break;
                        }
                        meter->lsn = byte;
                        // FSA, LSA, CS: six bytes, followed by three bytes per point.
                        meter->remaining = (uint16_t)(6u + 3u * byte);
                        meter->state     = SKIP_PACKET_BODY;
                        break;

                case SKIP_PACKET_BODY:
                        if (--meter->remaining == 0u)
                                complete_packet(meter);
                        break;

                default:
                        meter->state = SEEK_FIRST_HEADER_BYTE;
                        break;
        }
}
