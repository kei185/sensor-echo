
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "protocol.h"
#include "tx/frame.h"
#include "tx/header.h"
#include "lidar/sys.h"
#include "stm32f4xx_hal_uart.h"
#include "lidar/core.h"
#include "lidar/parser.h"

bool lidar_rx_dma_done = 0;

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

static size_t write_health_message(const ParserHealth* health, char* to, size_t capacity)
{
        if (health == NULL)
                return 0u;

        const char* status = health->health > 0u ? "FAULT" : "OK";
        const char* format = "[SENSOR-ECHO] LiDAR STATUS: %s | code=0x%02X";
        int length = snprintf(to, capacity, format, status, (unsigned)health->health);
        if (length < 0 || (size_t)length + 2u > capacity)
                return 0u;

        to[length++] = '\r';
        to[length++] = '\n';
        return (size_t)length;
}

static void serial_to_hex(const ParserDeviceInfo* info, char serial[33])
{
        static const char hex[] = "0123456789ABCDEF";
        for (size_t i = 0u; i < SYS_PACKET_DEVICE_SERIAL_SIZE; ++i) {
                serial[i * 2u]      = hex[info->serial_number[i] >> 4u];
                serial[i * 2u + 1u] = hex[info->serial_number[i] & 0x0fu];
        }
        serial[32] = '\0';
}

static size_t
write_device_message(const ParserDeviceInfo* info, char* to, size_t capacity)
{
        if (info == NULL)
                return 0u;

        char serial[33];
        serial_to_hex(info, serial);
        const char* format = "[SENSOR-ECHO] LiDAR DEVICE: model=%u firmware=%u.%u "
                             "hardware=%u serial=%s";
        int         length = snprintf(
                to,
                capacity,
                format,
                (unsigned)info->model,
                (unsigned)info->firmware_major,
                (unsigned)info->firmware_minor,
                (unsigned)info->hardware_version,
                serial);
        if (length < 0 || (size_t)length + 2u > capacity)
                return 0u;

        to[length++] = '\r';
        to[length++] = '\n';
        return (size_t)length;
}

size_t translate(
        SysTypeCode             type,
        const ParserHealth*     health,
        const ParserDeviceInfo* device_info,
        uint8_t*                to,
        size_t                  to_capacity,
        uint32_t                timestamp)
{
        if (to == NULL || to_capacity < TX_FRAME_HEADER_SIZE)
                return 0u;

        char*  payload          = (char*)(to + TX_FRAME_HEADER_SIZE);
        size_t payload_capacity = to_capacity - TX_FRAME_HEADER_SIZE;
        size_t payload_len      = 0u;

        switch (type) {
                case SYS_TYPE_CODE_HEALTH:
                        payload_len =
                                write_health_message(health, payload, payload_capacity);
                        break;
                case SYS_TYPE_CODE_DEVICE_INFO:
                        payload_len = write_device_message(
                                device_info,
                                payload,
                                payload_capacity);
                        break;
                default:
                        return 0u;
        }

        if (payload_len == 0u || payload_len > UINT16_MAX)
                return 0u;
        return tx_frame_write_header(
                to,
                to_capacity,
                (uint16_t)payload_len,
                FRAME_TYPE_SYS,
                timestamp);
}
