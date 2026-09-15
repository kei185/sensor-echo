#ifndef LIDAR_PARSER_DEVICE_INFO_H
#define LIDAR_PARSER_DEVICE_INFO_H

#include <stdbool.h>
#include <stdint.h>
#include "lidar/sys.h"

typedef struct RxFrameDeviceInfo
{
        uint8_t model;
        uint8_t firmware_major;
        uint8_t firmware_minor;
        uint8_t hardware_version;
        // Binary bytes in wire order; not a C string.
        uint8_t serial_number[SYS_PACKET_DEVICE_SERIAL_SIZE];
} ParserDeviceInfo;

bool read_device_info_frame(const int8_t*, ParserDeviceInfo*);

#endif /* LIDAR_PARSER_DEVICE_INFO_H */
