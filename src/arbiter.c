
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include "lidar/core.h"
#include "arbiter.h"

static volatile bool lidar_rx_failed;

/**
 * let full buffer to be sent by DMA if exists
 */
void try_dispatch_tx(void)
{
        // The HAL TX state owns the queue head until transmission completes.
        if (huart2.gState != HAL_UART_STATE_READY)
                return;

        TxBufSlot* target = get_full_buf();
        while (target != NULL &&
               (target->length == 0u || target->length > CORE_TX_BUF_SIZE)) {
                release(target);
                target = get_full_buf();
        }

        if (target == NULL)
                return;

        HAL_UART_Transmit_DMA(&huart2, target->_buf, (uint16_t)target->length);
}

void dma_receive_complete_handler(void) { increment_lap(); }

void dma_receive_error_handler(void) { lidar_rx_failed = true; }

bool recover_lidar_rx(void)
{
        if (!lidar_rx_failed)
                return false;

        lidar_rx_failed = false;
        if (HAL_UART_AbortReceive(&huart4) != HAL_OK) {
                lidar_rx_failed = true;
                return true;
        }
        setup_nonblocking_rx();
        if (HAL_UART_Receive_DMA(&huart4, RX_BUF->_buf, CORE_RX_BUF_SIZE) != HAL_OK)
                lidar_rx_failed = true;
        return true;
}
// ErrorCallbackはHALのDMA停止完了後に呼ばれる。再開失敗時は次のloopで再試行する。

void uart_transmit_complete_handler(void)
{
        // HAL marks USART2 ready before invoking the completion callback.
        TxBufSlot* target = get_full_buf();

        if (target != NULL)
                release(target);

        try_dispatch_tx();
}
