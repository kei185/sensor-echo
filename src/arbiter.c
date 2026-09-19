
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include "lidar/sys.h"
#include "tx/frame.h"
#include "arbiter.h"

static Arbiter _ARBITER = {0};

const Arbiter* const ARBITER = &_ARBITER;

static bool is_valid_tx_frame(const TxBufSlot* slot)
{
        return slot->length > 0u && slot->length <= CORE_TX_BUF_SIZE;
}

void arbitrate(void)
{
        if (_ARBITER.transmitting != NULL)
                return;

        TxBufSlot* target = get_full_buf();
        while (target != NULL && !is_valid_tx_frame(target)) {
                release(target);
                target = get_full_buf();
        }

        if (target == NULL)
                return;

        // Publish ownership before enabling DMA so the callback sees its slot.
        _ARBITER.transmitting = target;
        if (HAL_UART_Transmit_DMA(
                    &huart2,
                    (const uint8_t*)target->_buf,
                    (uint16_t)target->length) != HAL_OK)
                // Keep the slot queued so a later arbitrate() call can retry it.
                _ARBITER.transmitting = NULL;
}

void reset_arbiter(void) { _ARBITER = (Arbiter){0}; }

void record_skipped_rx_read(void)
{
        if (_ARBITER.skipped_rx_reads < UINT32_MAX)
                ++_ARBITER.skipped_rx_reads;
}

void dma_receive_complete_handler(void) { increment_lap(); }

void uart_transmit_complete_handler(void)
{
        TxBufSlot* completed = _ARBITER.transmitting;
        if (completed == NULL)
                return;

        _ARBITER.transmitting = NULL;
        release(completed);
        arbitrate();
}

void uart_transmit_error_handler(void)
{
        TxBufSlot* failed = _ARBITER.transmitting;
        if (failed == NULL)
                return;

        _ARBITER.transmitting = NULL;
        release(failed);
        arbitrate();
}

void init_arbiter(void)
{
        reset_arbiter();

        HAL_UART_Transmit(&huart4, MSG[MSG_TYPE_RX_HEALTH], MSG_SIZE, 100);
        // TODO revcieve message and parse then send it

        HAL_UART_Transmit(&huart4, MSG[MSG_TYPE_RX_SYS_INFO], MSG_SIZE, 100);
        // TODO revcieve message and parse then send it

        uint8_t rx_buf[COMMAND_SIZE];
        HAL_UART_Receive(&huart2, rx_buf, COMMAND_SIZE, 100);

        // TODO set dma it and start scan
}
