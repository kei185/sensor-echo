
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "protocol.h"
#include "tx/frame.h"
#include "lidar/sys.h"
#include "stm32f4xx_hal_uart.h"
#include "lidar/core.h"
#include "lidar/parser.h"

bool lidar_rx_dma_done = 0;

static const char DEVICE_INFO_MESSAGE_FORMAT[] =
        "[SENSOR-ECHO] LiDAR DEVICE: model=%u firmware=%u.%u hardware=%u "
        "serial=%s\r\n";
static const char HEALTH_MESSAGE_FORMAT[] =
        "[SENSOR-ECHO] LiDAR STATUS: %s | code=0x%02X\r\n";

void initialize();
void arbitrate(RxBuf rx_buf, TxBuf tx_buf);

void loop()
{
        HAL_UART_Transmit(&huart2, (uint8_t*)"DEVICE INITIALIZING...\r\n", 22, 100);

        initialize();

        while (1) {
                if (lidar_rx_dma_done == true) {
                        HAL_UART_Transmit(&huart2, (uint8_t*)"RX DMA trap\r\n", 24, 100);
                        lidar_rx_dma_done = false;
                }
        }
}

void initialize()
{

        HAL_UART_Transmit(&huart4, MSG[MSG_TYPE_RX_HEALTH], MSG_SIZE, 100);
        // TODO revcieve message and parse then send it

        HAL_UART_Transmit(&huart4, MSG[MSG_TYPE_RX_SYS_INFO], MSG_SIZE, 100);
        // TODO revcieve message and parse then send it

        uint8_t rx_buf[COMMAND_SIZE];
        HAL_UART_Receive(&huart2, rx_buf, COMMAND_SIZE, 100);

        // TODO set dma it and start scan
}

size_t translate_device_info(int8_t* to, ParserDeviceInfo* info)
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
        int          written  = snprintf(
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

size_t translate_health(int8_t* to, ParserHealth* health)
{
        if (to == NULL || health == NULL)
                return 0u;

        const size_t capacity = CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE;
        int          written  = snprintf(
                (char*)to,
                capacity,
                HEALTH_MESSAGE_FORMAT,
                health->health > 0u ? "FAULT" : "OK",
                (unsigned)health->health);
        return written < 0 || (size_t)written >= capacity ? 0u : (size_t)written;
}

/**
 * @param from points to the head of buffer
 * @param to points to the first field of the payload
 * @return number of bytes written into to-buffer
 */
size_t translate(int8_t* from, int8_t* to)
{
        if (from == NULL || to == NULL)
                return 0u;

        int8_t* parser_head = from;

        // parse frame header
        ParserMeta meta = {0};
        if (read_meta(from, CORE_RX_BUF_SIZE, &meta) == NULL)
                return 0u;

        // increment pointer
        parser_head += SYS_PACKET_META_SIZE;

        ParserDeviceInfo info;
        ParserHealth     health;

        // prase frame content
        switch (meta.type_code) {
                case SYS_TYPE_CODE_DEVICE_INFO:
                        info = (ParserDeviceInfo){0};
                        if (!read_device_info_frame(parser_head, &info))
                                return 0u;
                        return translate_device_info(to, &info);

                case SYS_TYPE_CODE_HEALTH:
                        health = (ParserHealth){0};
                        if (!read_health_frame(parser_head, &health))
                                return 0u;
                        return translate_health(to, &health);

                case SYS_TYPE_CODE_SCAN:
                        return (size_t)read_scan_frame(
                                       parser_head,
                                       (ParserScannedPoint*)to) *
                               sizeof(ParserScannedPoint);

                default:
                        return 0u;
        }
}
