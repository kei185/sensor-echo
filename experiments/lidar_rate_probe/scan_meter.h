#ifndef LIDAR_RATE_PROBE_SCAN_METER_H
#define LIDAR_RATE_PROBE_SCAN_METER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
        uint32_t bytes;
        uint32_t points;
        uint32_t packets;
        uint32_t complete_laps;
        uint32_t complete_lap_points;
        uint32_t min_lap_points;
        uint32_t max_lap_points;
        uint32_t malformed_packets;

        uint32_t current_lap_points;
        uint16_t remaining;
        uint8_t  state;
        uint8_t  ct;
        uint8_t  lsn;
        bool     have_lap;
} LidarScanMeter;

void lidar_scan_meter_reset(LidarScanMeter* meter);
void lidar_scan_meter_feed(LidarScanMeter* meter, uint8_t byte);

#endif
