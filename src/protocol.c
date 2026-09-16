#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "arbiter.h"
#include "protocol.h"
#include "tx/frame.h"
#include "tx/header.h"
#include "lidar/sys.h"
#include "lidar/core.h"
#include "lidar/parser/meta.h"
#include "lidar/parser/health.h"
#include "lidar/parser/device_info.h"
#include "lidar/parser/scan.h"

static const char DEVICE_INFO_MESSAGE_FORMAT[] =
        "[SENSOR-ECHO] LiDAR DEVICE: model=%u firmware=%u.%u hardware=%u "
        "serial=%s\r\n";
static const char HEALTH_MESSAGE_FORMAT[] =
        "[SENSOR-ECHO] LiDAR STATUS: %s | code=0x%02X\r\n";

bool imu_arrived;
bool enc_arrived;
void loop()
{
        HAL_UART_Transmit(&huart2, (uint8_t*)"DEVICE INITIALIZING...\r\n", 22, 100);

        init_arbiter();

        while (1) {
                // if (scan_stop_requested)
                // stop scan and send ack

                // if (imu_arrived)
                //  translate_imu();

                // if (enc_arrived)
                //  translate_enc();

                // translate(int8_t *to)
        }
}

static size_t translate_device_info(int8_t* to, ParserDeviceInfo* info)
{
        if (to == NULL || info == NULL)
                return 0u;

        // Map each 4-bit value to one hexadecimal character.
        static const char hex[] = "0123456789ABCDEF";
        // Each serial byte needs two characters; reserve one more for NUL.
        char serial[SYS_PACKET_DEVICE_SERIAL_SIZE * 2u + 1u];
        for (size_t i = 0u; i < SYS_PACKET_DEVICE_SERIAL_SIZE; ++i) {
                // The upper 4 bits become the first character.
                serial[i * 2u] = hex[info->serial_number[i] >> 4u];
                // The lower 4 bits become the second character.
                serial[i * 2u + 1u] = hex[info->serial_number[i] & 0x0fu];
        }
        // Terminate the string so snprintf can read it with %s.
        serial[SYS_PACKET_DEVICE_SERIAL_SIZE * 2u] = '\0';

        const size_t capacity = CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE;

        int written = snprintf(
                (char*)to,
                capacity,
                DEVICE_INFO_MESSAGE_FORMAT,
                (unsigned)info->model,
                (unsigned)info->firmware_major,
                (unsigned)info->firmware_minor,
                (unsigned)info->hardware_version,
                serial);

        return written < 0 || (size_t)written >= capacity ? 0u : (size_t)written;
}

static size_t translate_health(int8_t* to, ParserHealth* health)
{
        if (to == NULL || health == NULL)
                return 0u;

        const size_t capacity = CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE;

        int written = snprintf(
                (char*)to,
                capacity,
                HEALTH_MESSAGE_FORMAT,
                health->health > 0u ? "FAULT" : "OK",
                (unsigned)health->health);

        return written < 0 || (size_t)written >= capacity ? 0u : (size_t)written;
}

/*
 * @param to points to the head of the payload field
 */
static size_t translate_frame_content(int8_t* to, SysTypeCode type)
{

        ParserDeviceInfo info;
        ParserHealth     health;

        switch (type) {
                case SYS_TYPE_CODE_DEVICE_INFO:
                        info = (ParserDeviceInfo){0};
                        if (!read_device_info_frame(&info))
                                return 0u;
                        return translate_device_info(to, &info);

                case SYS_TYPE_CODE_HEALTH:
                        health = (ParserHealth){0};
                        if (!read_health_frame(&health))
                                return 0u;
                        return translate_health(to, &health);

                case SYS_TYPE_CODE_SCAN:
                        return (size_t)read_scan_frame((ParserScannedPoint*)to) *
                               sizeof(ParserScannedPoint);

                default:
                        return 0u;
        }
}

/**
 * @param to points to the beginning of a TX frame
 * @return complete TX frame size, or zero if translation fails
 */
size_t translate(int8_t* to)
{
        if (to == NULL)
                return 0u;

        ParserMeta meta = {0};
        if (read_meta(&meta) == NULL)
                return 0u;

        int8_t* writer_payload_head = to + TX_FRAME_HEADER_SIZE;

        size_t payload_length =
                translate_frame_content(writer_payload_head, meta.type_code);

        if (payload_length == 0u || payload_length > UINT16_MAX)
                return 0u;

        FrameType frame_type =
                meta.type_code == SYS_TYPE_CODE_SCAN ? FRAME_TYPE_LIDAR : FRAME_TYPE_SYS;

        return tx_frame_write_header(
                (uint8_t*)to,
                CORE_TX_BUF_SIZE,
                (uint16_t)payload_length,
                frame_type,
                HAL_GetTick());
}
