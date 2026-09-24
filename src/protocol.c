#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "main.h"

#include "arbiter.h"
#include "protocol.h"
#include "startup.h"
#include "stm32f4xx_hal_gpio.h"
#include "tx/frame.h"
#include "tx/header.h"
#include "lidar/sys.h"
#include "lidar/core.h"
#include "lidar/parser/meta.h"
#include "lidar/parser/health.h"
#include "lidar/parser/device_info.h"
#include "lidar/parser/scan.h"

static const char DEVICE_INFO_MESSAGE_FORMAT[] =
        "LiDAR DEVICE: model=%u firmware=%u.%u hardware=%u serial=%s";
static const char HEALTH_MESSAGE_FORMAT[] = "LiDAR STATUS: %s | code=0x%02X";

static size_t translate_scan_frame(uint8_t*);

// bool imu_arrived = false;
// bool enc_arrived = false;
void loop()
{
        if (!run_startup_sequence())
                Error_Handler();

        // send start scan command
        HAL_StatusTypeDef scan_start_status =
                HAL_UART_Transmit(&huart4, MSG[MSG_TYPE_SCAN], MSG_SIZE, 100);

        if (scan_start_status != HAL_OK) {
                LD2_GPIO_Port->ODR ^= LD2_Pin;
                Error_Handler();
        }

        // wait for meta frame for scan
        ParserMeta meta = {0};
        read_meta(&meta);
        if (meta.res_mode != SYS_RES_MODE_CONTINUOUS ||
            meta.type_code != SYS_TYPE_CODE_SCAN) {
                HAL_GPIO_WritePin(
                        INITIAL_HANDSHAKE_FAILED_GPIO_Port,
                        INITIAL_HANDSHAKE_FAILED_Pin,
                        1);

                Error_Handler();
        }

        SCANNING_GPIO_Port->ODR ^= SCANNING_Pin;

        while (1) {
                // Retry a queued TX frame if a previous DMA start was busy.
                try_dispatch_tx();

                // if (scan_stop_requested)
                // stop scan and send ack

                // if (imu_arrived)
                //  translate_imu();

                // if (enc_arrived)
                //  translate_enc();

                TxBufSlot* tbs = get_empty_buf();
                if (tbs == NULL)
                        continue;

                if (is_lapped()) {
                        reset_read_idx();
                        continue;
                }

                size_t len = translate_scan_frame(tbs->_buf);
                if (len == 0u || is_lapped()) {
                        reset_read_idx();
                        continue;
                }

                push_full_slot(tbs, (uint32_t)len);
                try_dispatch_tx();
        }
}

static size_t translate_device_info(char* to, const ParserDeviceInfo* info)
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
                to,
                capacity,
                DEVICE_INFO_MESSAGE_FORMAT,
                (unsigned)info->model,
                (unsigned)info->firmware_major,
                (unsigned)info->firmware_minor,
                (unsigned)info->hardware_version,
                serial);

        return written < 0 || (size_t)written >= capacity ? 0u : (size_t)written;
}

static size_t translate_health(char* to, const ParserHealth* health)
{
        if (to == NULL || health == NULL)
                return 0u;

        const size_t capacity = CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE;

        int written = snprintf(
                to,
                capacity,
                HEALTH_MESSAGE_FORMAT,
                health->health > 0u ? "FAULT" : "OK",
                (unsigned)health->health);

        return written < 0 || (size_t)written >= capacity ? 0u : (size_t)written;
}

/*
 * @param to points to the head of the payload field
 */
static size_t translate_system_frame_content(uint8_t* to, SysTypeCode type)
{

        ParserDeviceInfo info;
        ParserHealth     health;

        switch (type) {
                case SYS_TYPE_CODE_DEVICE_INFO:
                        info = (ParserDeviceInfo){0};
                        if (!read_device_info_frame(&info))
                                return 0u;
                        return translate_device_info((char*)to, &info);

                case SYS_TYPE_CODE_HEALTH:
                        health = (ParserHealth){0};
                        if (!read_health_frame(&health))
                                return 0u;
                        return translate_health((char*)to, &health);

                        // case SYS_TYPE_CODE_SCAN:
                        //         return (size_t)read_scan_frame((ParserScannedPoint*)to)
                        //         *
                        //                sizeof(ParserScannedPoint);

                default:
                        return 0u;
        }
}

static size_t translate_scan_frame(uint8_t* to)
{
        // number of points
        size_t payload_length =
                (size_t)read_scan_frame((ParserScannedPoint*)(to + TX_FRAME_HEADER_SIZE));

        // size of total points
        payload_length *= sizeof(ParserScannedPoint);

        if (payload_length == 0u || payload_length > UINT16_MAX)
                return 0u;

        return tx_frame_write_header(
                to,
                CORE_TX_BUF_SIZE,
                (uint16_t)payload_length,
                FRAME_TYPE_LIDAR,
                HAL_GetTick());
}

/**
 * @param to points to the beginning of a TX frame
 * @return complete TX frame size, or zero if translation fails
 */
size_t translate_single(uint8_t* to)
{
        ParserMeta meta = {0};
        if (read_meta(&meta) == NULL)
                return 0u;

        uint8_t* writer_payload_head = to + TX_FRAME_HEADER_SIZE;

        size_t payload_length =
                translate_system_frame_content(writer_payload_head, meta.type_code);

        if (payload_length == 0u || payload_length > UINT16_MAX)
                return 0u;

        FrameType frame_type;
        switch (meta.type_code) {
                case SYS_TYPE_CODE_DEVICE_INFO:
                        frame_type = FRAME_TYPE_DEVICE_INFO;
                        break;
                case SYS_TYPE_CODE_HEALTH:
                        frame_type = FRAME_TYPE_HEALTH_STATUS;
                        break;
                // case SYS_TYPE_CODE_SCAN:
                //         frame_type = FRAME_TYPE_LIDAR;
                //         break;
                default:
                        return 0u;
        }

        return tx_frame_write_header(
                to,
                CORE_TX_BUF_SIZE,
                (uint16_t)payload_length,
                frame_type,
                HAL_GetTick());
}
