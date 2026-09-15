#include <string.h>
#include "lidar/parser/device_info.h"

/**
 * DEVICE INFO
 */
bool read_device_info_frame(const int8_t* buf, ParserDeviceInfo* info)
{
        if (info)
                return false;

        // Manual section 3.3: firmware low byte is major, high byte is minor.
        info->model            = buf[0];
        info->firmware_major   = buf[1];
        info->firmware_minor   = buf[2];
        info->hardware_version = buf[3];
        // Preserve the serial bytes in wire order; no integer endian conversion.
        memcpy(info->serial_number, buf + 4, SYS_PACKET_DEVICE_SERIAL_SIZE);

        return true;
};
