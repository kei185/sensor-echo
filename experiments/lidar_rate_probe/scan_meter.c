#include "experiments/lidar_rate_probe/scan_meter.h"

#include <string.h>

// A scan packet is AA 55, CT, LSN, FSA, LSA, CS, then LSN point records.
// FSA, LSA, and CS use six bytes together; each point record uses three bytes.
// CT and LSN drive the counters. We also save the whole packet so the probe
// can measure the real PC-frame converter after the last byte arrives.
// This prototype does not verify the packet's XOR check code.
// The lap-start packet may have a CRC byte before AA 55; it still counts as
// received traffic, but it is not part of the scan packet below.
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
        // Count points only after every byte of the packet has arrived.
        ++meter->packets;
        meter->points += meter->lsn;

        if ((meter->ct & 1u) != 0u) {
                // The first marker opens a lap. Each later marker closes the
                // previous lap and starts another one. This excludes partial
                // laps at the beginning and end of the five-second sample.
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

        // The start packet's single point belongs to the new lap.
        if (meter->have_lap)
                meter->current_lap_points += meter->lsn;
        meter->state = SEEK_FIRST_HEADER_BYTE;
}

bool lidar_scan_meter_feed(LidarScanMeter* meter, uint8_t byte)
{
        // Bytes/s includes all UART traffic, even a partial packet at the
        // start or end of the sample. Packet and point counts do not.
        ++meter->bytes;

        switch (meter->state) {
                case SEEK_FIRST_HEADER_BYTE:
                        // Look for the first byte of the scan packet marker.
                        if (byte == 0xaau) {
                                meter->packet[0]     = byte;
                                meter->packet_length = 1u;
                                meter->state         = SEEK_SECOND_HEADER_BYTE;
                        }
                        break;

                case SEEK_SECOND_HEADER_BYTE:
                        // A second AA might itself start a new AA 55 marker.
                        if (byte == 0x55u) {
                                meter->packet[1]     = byte;
                                meter->packet_length = 2u;
                                meter->state         = READ_CT;
                        } else if (byte != 0xaau) {
                                meter->packet_length = 0u;
                                meter->state         = SEEK_FIRST_HEADER_BYTE;
                        }
                        break;

                case READ_CT:
                        meter->packet[meter->packet_length++] = byte;
                        meter->ct                             = byte;
                        meter->state                          = READ_LSN;
                        break;

                case READ_LSN:
                        // A packet must contain points. The start-of-lap
                        // packet contains exactly one point in this format.
                        if (byte == 0u || ((meter->ct & 1u) != 0u && byte != 1u)) {
                                ++meter->malformed_packets;
                                meter->packet_length = 0u;
                                meter->state         = SEEK_FIRST_HEADER_BYTE;
                                break;
                        }
                        meter->packet[meter->packet_length++] = byte;
                        meter->lsn                            = byte;
                        // Skip six fixed bytes, then three bytes per point.
                        // Do not search for AA 55 inside this body: point data
                        // can contain those bytes without starting a packet.
                        meter->remaining = (uint16_t)(6u + 3u * byte);
                        meter->state     = SKIP_PACKET_BODY;
                        break;

                case SKIP_PACKET_BODY:
                        meter->packet[meter->packet_length++] = byte;
                        if (--meter->remaining == 0u) {
                                complete_packet(meter);
                                return true;
                        }
                        break;

                default:
                        meter->packet_length = 0u;
                        meter->state         = SEEK_FIRST_HEADER_BYTE;
                        break;
        }
        return false;
}
