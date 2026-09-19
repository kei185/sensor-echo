#ifndef ARBITER_H
#define ARBITER_H

void init_arbiter(void);
void arbitrate(void);
void dma_receive_complete_handler(void);
void uart_transmit_complete_handler(void);

#endif
