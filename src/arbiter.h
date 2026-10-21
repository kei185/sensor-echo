#ifndef ARBITER_H
#define ARBITER_H

#include <stdbool.h>

void try_dispatch_tx(void);
void dma_receive_complete_handler(void);
void dma_receive_error_handler(void);
bool recover_lidar_rx(void);
// 復旧を試みた場合はtrueを返すので、解析中のフレームは破棄する。
void uart_transmit_complete_handler(void);

#endif
