#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H
#include <stdbool.h>
#include <stddef.h>
#include "lidar/sys.h"

typedef struct RxFrameMeta
{
        uint16_t    res_len;
        SysResMode  res_mode;
        SysTypeCode type_code;
} ParserMeta;

typedef struct RxFrameHealth
{
        uint8_t health;
        bool    sensor_abnormal;
        bool    encoder_abnormal;
        bool    wireless_power_abnormal;
        bool    laser_feedback_abnormal;
        bool    laser_drive_abnormal;
        bool    lidar_data_abnormal;
} ParserHealth;

typedef struct RxFrameDeviceInfo
{
        uint8_t model;
        uint8_t firmware_major;
        uint8_t firmware_minor;
        uint8_t hardware_version;
        // Binary bytes in wire order; not a C string.
        uint8_t serial_number[SYS_PACKET_DEVICE_SERIAL_SIZE];
} ParserDeviceInfo;

typedef struct ScannedPoint
{
        uint16_t dist;
        uint16_t angle; // nonnegative angle in Q6 degrees (degrees * 64)
} ParserScannedPoint;

// Valid Q6 angles are 0..23039; UINT16_MAX cannot be a decoded angle.
#define PARSER_SCAN_ANGLE_INVALID_Q6 UINT16_MAX

typedef struct
{
        uint16_t       start_angle; // decoded sensor angle in Q6 degrees
        uint16_t       end_angle;   // decoded sensor angle in Q6 degrees
        uint8_t        data_num;    // one-byte LSN field
        const uint8_t* data_frame_head;
} ParserScanMeta;

ParserMeta* read_meta(int8_t*, uint32_t, ParserMeta*);
bool        read_health_frame(const int8_t*, ParserHealth*);
// Write four little-endian payload bytes per point from one AA 55 packet.
// Return the point count, or zero if input is invalid or output is too small.
size_t read_scan_frame(const uint8_t*, size_t, uint8_t*, size_t);
bool   read_device_info_frame(const int8_t*, ParserDeviceInfo*);

#endif /* LIDAR_PARSER_H */
