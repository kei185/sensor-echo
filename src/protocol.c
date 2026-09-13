
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "tx/frame.h"
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

void translate_device_info(int8_t* to, ParserDeviceInfo* info)
{ // todo
}

void translate_health(int8_t* to, ParserHealth* health)
{ // todo
}

/**
 * @param from points to the head of buffer
 * @param to points to the first field of the payload
 * @return number of bytes written into to-buffer
 */
bool translate(int8_t* from, int8_t* to)
{
        int8_t* parser_head = from;

        // parse frame header
        ParserMeta meta = {0};
        if (read_meta(to, CORE_RX_BUF_SIZE, &meta))
                return false;

        // increment pointer
        parser_head += SYS_PACKET_HEADER_SIZE;

        ParserDeviceInfo info;
        ParserHealth     health;

        // prase frame content
        switch (meta.type_code) {
                case SYS_TYPE_CODE_DEVICE_INFO:
                        info = (ParserDeviceInfo){0};
                        read_device_info_frame(parser_head, &info);
                        translate_device_info(to, &info);
                        break;

                case SYS_TYPE_CODE_HEALTH:
                        health = (ParserHealth){0};
                        read_health_frame(parser_head, &health);
                        translate_health(to, &health);
                        break;

                case SYS_TYPE_CODE_SCAN:
                        read_scan_frame(parser_head, (ParserScannedPoint*)to);
                        break;

                default:
                        return false;
        }

        return true;
}
