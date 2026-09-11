
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "tx/frame.h"
#include "lidar/sys.h"
#include "stm32f4xx_hal_uart.h"

bool lidar_rx_dma_done = 0;

COMMAND which_cmd(uint8_t* buf);
void    initialize();

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
        COMMAND cmd = which_cmd(rx_buf);

        // TODO set dma it and start scan
}

// TODO
COMMAND which_cmd(uint8_t* buf) {}
