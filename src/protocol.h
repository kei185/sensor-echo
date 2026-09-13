#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern bool rx_dma;
extern bool tx_dma;

void loop(void);

/**
 * Translate one complete LiDAR status or device-info reply at from into a PC
 * system frame at to. Returns the complete frame size for transmission, or
 * zero for an incomplete, unsupported, or invalid reply. The caller supplies
 * the number of received bytes, TX capacity, and timestamp.
 */
size_t translate(
        const int8_t* from,
        size_t        from_len,
        uint8_t*      to,
        size_t        to_capacity,
        uint32_t      timestamp);

#endif
