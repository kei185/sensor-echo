#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "lidar/parser.h"

extern bool rx_dma;
extern bool tx_dma;

void loop(void);

/** Write one parsed LiDAR reply as a PC system frame; return its total byte count. */
size_t translate(
        SysTypeCode             type,
        const ParserHealth*     health,
        const ParserDeviceInfo* device_info,
        uint8_t*                to,
        size_t                  to_capacity,
        uint32_t                timestamp);

#endif
