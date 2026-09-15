#ifndef TX_SCAN_H
#define TX_SCAN_H

#include <stddef.h>
#include <stdint.h>

// Build one PC frame from one complete LiDAR AA 55 scan packet.
// The caller may reuse tx_buf after this function returns.
// Return the PC frame size, or zero if the input or output does not fit.
size_t tx_scan_frame_write(
        const uint8_t* packet,
        size_t         packet_len,
        uint8_t*       tx_buf,
        size_t         tx_capacity,
        uint32_t       timestamp);

#endif
