#include <stddef.h>
#include "lidar/core.h"
#include "lidar/parser/device_info.h"

/**
 * DEVICE INFO
 */
bool read_device_info_frame(ParserDeviceInfo* info)
{
        if (info == NULL)
                return false;

        // Manual section 3.3: firmware low byte is major, high byte is minor.
        info->model            = read_byte();
        info->firmware_major   = read_byte();
        info->firmware_minor   = read_byte();
        info->hardware_version = read_byte();
        // Preserve the serial bytes in wire order; no integer endian conversion.
        for (size_t i = 0; i < SYS_PACKET_DEVICE_SERIAL_SIZE; ++i)
                info->serial_number[i] = read_byte();

        return true;
};
