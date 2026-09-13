#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "lidar/parser.h"
#include "lidar/sys.h"
#include "protocol.h"
#include "tx/header.h"

static size_t
format_health_message(char* message, size_t capacity, const ParserHealth* health)
{
        int written = snprintf(
                message,
                capacity,
                "[SENSOR-ECHO] LiDAR STATUS: %s | code=0x%02X | sensor=%s encoder=%s "
                "wireless=%s feedback=%s laser=%s data=%s",
                health->health == 0u ? "OK" : "FAULT",
                health->health,
                health->sensor_abnormal ? "FAULT" : "OK",
                health->encoder_abnormal ? "FAULT" : "OK",
                health->wireless_power_abnormal ? "FAULT" : "OK",
                health->laser_feedback_abnormal ? "FAULT" : "OK",
                health->laser_drive_abnormal ? "FAULT" : "OK",
                health->lidar_data_abnormal ? "FAULT" : "OK");

        if (written < 0)
                return 0u;

        size_t length = (size_t)written + 2u;
        if (message == NULL)
                return length;
        if (length > capacity)
                return 0u;

        message[written]      = '\r';
        message[written + 1u] = '\n';
        return length;
}

static size_t
format_device_message(char* message, size_t capacity, const ParserDeviceInfo* info)
{
        int written = snprintf(
                message,
                capacity,
                "[SENSOR-ECHO] LiDAR DEVICE IDENTIFIED | model=%u firmware=%u.%u "
                "hardware=%u serial=",
                info->model,
                info->firmware_major,
                info->firmware_minor,
                info->hardware_version);
        if (written < 0)
                return 0u;

        size_t total_length = (size_t)written + SYS_PACKET_DEVICE_SERIAL_SIZE * 2u + 2u;
        if (message == NULL)
                return total_length;
        if (total_length > capacity)
                return 0u;

        size_t            length = (size_t)written;
        static const char hex[]  = "0123456789ABCDEF";
        for (size_t i = 0; i < SYS_PACKET_DEVICE_SERIAL_SIZE; ++i) {
                message[length++] = hex[info->serial_number[i] >> 4];
                message[length++] = hex[info->serial_number[i] & 0x0fu];
        }
        message[length++] = '\r';
        message[length++] = '\n';
        return length;
}

size_t translate(
        const int8_t* from,
        size_t        from_len,
        uint8_t*      to,
        size_t        to_capacity,
        uint32_t      timestamp)
{
        if (from == NULL || to == NULL || from_len < SYS_PACKET_META_SIZE ||
            to_capacity < TX_FRAME_HEADER_SIZE)
                return 0u;

        ParserMeta meta = {0};
        // A translation starts at the descriptor, never at a later sign in the RX buffer.
        if (read_meta(from, SYS_PACKET_META_SIZE, &meta) == NULL ||
            meta.res_mode != SYS_RES_MODE_SINGLE ||
            meta.res_len > from_len - SYS_PACKET_META_SIZE)
                return 0u;

        const int8_t*    content     = from + SYS_PACKET_META_SIZE;
        ParserHealth     health      = {0};
        ParserDeviceInfo info        = {0};
        size_t           message_len = 0u;

        switch (meta.type_code) {
                case SYS_TYPE_CODE_HEALTH: {
                        if (meta.res_len != SYS_PACKET_HEALTH_CONTENT_SIZE)
                                return 0u;
                        if (!read_health_frame(content, &health))
                                return 0u;
                        message_len = format_health_message(NULL, 0u, &health);
                        break;
                }
                case SYS_TYPE_CODE_DEVICE_INFO: {
                        if (meta.res_len != SYS_PACKET_DEVICE_INFO_CONTENT_SIZE)
                                return 0u;
                        if (!read_device_info_frame(content, &info))
                                return 0u;
                        message_len = format_device_message(NULL, 0u, &info);
                        break;
                }
                default:
                        return 0u;
        }

        if (message_len == 0u || message_len > UINT16_MAX ||
            message_len > to_capacity - TX_FRAME_HEADER_SIZE)
                return 0u;

        // Write the system message directly after the reserved header space.
        char*  payload          = (char*)(to + TX_FRAME_HEADER_SIZE);
        size_t payload_capacity = to_capacity - TX_FRAME_HEADER_SIZE;
        size_t written =
                meta.type_code == SYS_TYPE_CODE_HEALTH
                        ? format_health_message(payload, payload_capacity, &health)
                        : format_device_message(payload, payload_capacity, &info);
        if (written != message_len)
                return 0u;

        return tx_frame_write_header(
                to,
                to_capacity,
                (uint16_t)message_len,
                FRAME_TYPE_SYS,
                timestamp);
}
