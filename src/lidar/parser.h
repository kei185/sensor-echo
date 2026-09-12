#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H
#include <stdbool.h>
#include "lidar/sys.h"

typedef struct RxFrameMeta
{
        uint16_t    res_len;
        SysResMode  res_mode;
        SysTypeCode type_code;
} ParserMeta;

ParserMeta* read_meta(int8_t*, uint32_t, ParserMeta*);

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

bool health_parse(ParserHealth*);

/**
 * Parse a complete single-response frame starting at buf (including its 7-byte
 * descriptor). The bytes must already be received; these functions do not wait for UART
 * or DMA. Return true on successful parsing, including a healthy status of zero. Return
 * false for an incomplete or mismatched frame, leaving the output unchanged.
 */
bool read_health_frame(const uint8_t* buf, uint32_t len, ParserHealth* health);

typedef struct RxFrameDeviceInfo
{
        uint8_t model;
        uint8_t firmware_major;
        uint8_t firmware_minor;
        uint8_t hardware_version;
        // Binary bytes in wire order; not a C string.
        uint8_t serial_number[SYS_PACKET_DEVICE_SERIAL_SIZE];
} ParserDeviceInfo;

bool read_device_info_frame(const uint8_t* buf, uint32_t len, ParserDeviceInfo* info);

typedef struct ScannedPoint
{
        uint16_t dist;
        int16_t  angle;
} ParserScannedPoint;

typedef struct
{
        // Preserve 1/64-degree endpoints until all point angles are interpolated.
        uint16_t       start_angle_q6;
        uint16_t       end_angle_q6;
        uint32_t       data_num;
        const uint8_t* data_frame_head;
} ParserScanMeta;

extern const ParserScanMeta* const PARSER_SCAN_META;

/**
 * Parse one complete scan packet starting at its AA 55 header. The LiDAR angle
 * increases clockwise from the zero direction in datasheet section 2.7. The
 * output angle is an integer degree in [-179, 180], and distance is in millimetres.
 * Return the number of points, or zero for an invalid/truncated packet or insufficient
 * output capacity (measured in ParserScannedPoint elements). No output is written on
 * failure. The CS field is skipped, not checked.
 */
uint32_t read_scan_frame(
        const uint8_t* buf, uint32_t len, ParserScannedPoint* points, uint32_t capacity);

#endif /* LIDAR_PARSER_H */
