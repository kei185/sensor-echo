
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include "lidar/core.h"
#include "arbiter.h"

/**
 * let full buffer to be sent by DMA if exists
 */
void try_dispatch_tx(void)
{
        // The HAL TX state owns the queue head until transmission completes.
        if (huart2.gState != HAL_UART_STATE_READY)
                return;

        TxBufSlot* target = get_full_buf();

        if (target == NULL)
                return;

        HAL_UART_Transmit_DMA(&huart2, target->_buf, (uint16_t)target->length);
}

void dma_receive_complete_handler(void) { increment_lap(); }

void uart_transmit_complete_handler(void)
{
        // HAL marks USART2 ready before invoking the completion callback.
        TxBufSlot* target = get_full_buf();

        if (target != NULL)
                release(target);

        try_dispatch_tx();
}
