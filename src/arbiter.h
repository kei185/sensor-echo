#ifndef ARBITER_H
#define ARBITER_H

#include <stdbool.h>
#include <stdint.h>
#include "lidar/core.h"

typedef struct
{
        TxBufSlot* volatile transmitting;
        volatile uint32_t rx_wraps;
        volatile uint32_t tx_started;
        volatile uint32_t tx_completed;
        volatile uint32_t tx_start_failures;
        volatile uint32_t tx_transfer_failures;
        volatile uint32_t invalid_tx_frames;
} Arbiter;

extern const Arbiter* const ARBITER;

void init_arbiter(void);
void reset_arbiter(void);
void arbitrate(void);
void dma_receive_complete_handler(void);
void uart_transmit_complete_handler(void);
void uart_transmit_error_handler(void);

#endif
