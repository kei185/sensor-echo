
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include "lidar/sys.h"
#include "lidar/core.h"
#include "tx/frame.h"
#include "arbiter.h"

static bool is_valid_tx_frame(const TxBufSlot* slot)
{
        return slot->length > 0u && slot->length <= CORE_TX_BUF_SIZE;
}

void try_dispatch_tx(void)
{
        // The HAL TX state owns the queue head until transmission completes.
        if (huart2.gState != HAL_UART_STATE_READY)
                return;

        TxBufSlot* target = get_full_buf();
        // Discard invalid slots at the queue head until a valid frame is found.
        while (target != NULL && !is_valid_tx_frame(target)) {
                release(target);
                target = get_full_buf();
        }

        if (target == NULL)
                return;

        HAL_UART_Transmit_DMA(
                &huart2,
                (const uint8_t*)target->_buf,
                (uint16_t)target->length);
}

void dma_receive_complete_handler(void) { increment_lap(); }

void uart_transmit_complete_handler(void)
{
        // HAL marks USART2 ready before invoking the completion callback.
        TxBufSlot* completed = get_full_buf();
        if (completed != NULL)
                release(completed);

        try_dispatch_tx();
}

void init_arbiter(void)
{
        HAL_UART_Transmit(&huart4, MSG[MSG_TYPE_RX_HEALTH], MSG_SIZE, 100);
        // TODO revcieve message and parse then send it

        HAL_UART_Transmit(&huart4, MSG[MSG_TYPE_RX_SYS_INFO], MSG_SIZE, 100);
        // TODO revcieve message and parse then send it

        uint8_t rx_buf[COMMAND_SIZE];
        HAL_UART_Receive(&huart2, rx_buf, COMMAND_SIZE, 100);

        // TODO set dma it and start scan
}
