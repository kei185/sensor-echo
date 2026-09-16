#ifndef LIDAR_RATE_PROBE_SCAN_METER_H
#define LIDAR_RATE_PROBE_SCAN_METER_H

#include <stdbool.h>
#include <stdint.h>
#include "lidar/sys.h"

#define LIDAR_SCAN_PACKET_MAX_SIZE                                                       \
        (SYS_PACKET_SCAN_FIXED_SIZE + 255u * SYS_PACKET_POINT_DATA_SIZE)

typedef struct
{
        // Results to report after the measurement window.
        uint32_t bytes;               // Every received byte, including headers.
        uint32_t points;              // Points in fully received scan packets.
        uint32_t packets;             // Fully received scan packets.
        uint32_t complete_laps;       // Intervals between two lap-start markers.
        uint32_t complete_lap_points; // Points in those complete laps only.
        uint32_t min_lap_points;      // Smallest complete lap.
        uint32_t max_lap_points;      // Largest complete lap.
        uint32_t malformed_packets;   // Packets rejected by basic header checks.

        // State kept between calls because a DMA poll may end mid-packet.
        uint32_t current_lap_points; // Points since the last lap-start marker.
        uint16_t remaining;          // Bytes left in the current packet body.
        uint8_t  state;              // Current step of the packet reader.
        uint8_t  ct;                 // Packet's CT byte (bit 0 marks a new lap).
        uint8_t  lsn;                // Packet's point count.
        bool     have_lap;           // True after the first lap-start marker.
        uint16_t packet_length;      // Bytes saved in packet below.
        uint8_t  packet[LIDAR_SCAN_PACKET_MAX_SIZE]; // Last complete or partial packet.
} LidarScanMeter;

// Reset both the reported counters and the partial-packet reader.
void lidar_scan_meter_reset(LidarScanMeter* meter);
// Feed one UART byte. Return true only when packet contains a complete scan packet.
bool lidar_scan_meter_feed(LidarScanMeter* meter, uint8_t byte);

#endif
