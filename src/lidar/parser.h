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
