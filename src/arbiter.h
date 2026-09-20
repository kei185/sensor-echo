#ifndef ARBITER_H
#define ARBITER_H

void try_dispatch_tx(void);
void dma_receive_complete_handler(void);
void uart_transmit_complete_handler(void);

#endif
