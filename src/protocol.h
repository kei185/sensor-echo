#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern bool rx_dma;
extern bool tx_dma;

void loop(void);
/* `to` needs CORE_TX_BUF_SIZE bytes. Return the complete PC frame size,
 * or zero if translation fails. */
size_t translate(int8_t* from, int8_t* to);

#endif
