#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "lidar/parser.h"

extern bool rx_dma;
extern bool tx_dma;

void loop(void);
/* Return payload bytes written, excluding any C-string terminator; zero on failure. */
size_t translate(int8_t* from, int8_t* to);

#endif
