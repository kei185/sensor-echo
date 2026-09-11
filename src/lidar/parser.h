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
        uint8_t serial_number[SYS_DEVICE_SERIAL_SIZE]; // Binary bytes, not a C string.
} ParserDeviceInfo;

bool read_device_info_frame(const uint8_t* buf, uint32_t len, ParserDeviceInfo* info);

typedef struct ScannedPoint
{
        uint16_t dist;
        int16_t  angle;
} ParserScannedPoint;

typedef struct
{
        int8_t   start_angle;
        int8_t   end_angle;
        uint32_t data_num;
        int8_t*  data_frame_head;
} ParserScanMeta;

extern const ParserScanMeta* const PARSER_SCAN_META;

// uint32_t read_scan_frame(int8_t*, ParserScannedPoint*z);

#endif /* LIDAR_PARSER_H */
