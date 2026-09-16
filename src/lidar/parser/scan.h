#ifndef LIDAR_PARSER_SCAN_H
#define LIDAR_PARSER_SCAN_H

#include <stdint.h>

typedef struct ScannedPoint
{
        uint16_t dist;
        uint16_t angle; // nonnegative angle in Q6 degrees (degrees * 64)
} ParserScannedPoint;

// Valid Q6 angles are 0..23039; UINT16_MAX cannot be a decoded angle.
#define PARSER_SCAN_ANGLE_INVALID_Q6 UINT16_MAX

typedef struct
{
        uint16_t start_angle; // decoded sensor angle in Q6 degrees
        uint16_t end_angle;   // decoded sensor angle in Q6 degrees
        uint8_t  data_num;    // one-byte LSN field
} ParserScanMeta;

uint32_t read_scan_frame(ParserScannedPoint*);

#endif /* LIDAR_PARSER_SCAN_H */
