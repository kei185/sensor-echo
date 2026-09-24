#ifndef TX_HEADER_H
#define TX_HEADER_H

#include <stddef.h>
#include <stdint.h>
#include "tx/frame.h"

/**
 * Write the header after payload_len bytes have been placed at
 * tx_buf + TX_FRAME_HEADER_SIZE. Payload length and timestamp are encoded
 * little-endian. The CRC currently covers only the two payload-length bytes.
 * Returns the complete frame size, or zero if the buffer, capacity, or type is
 * invalid. On failure, tx_buf is unchanged.
 */
size_t tx_frame_write_header(
        uint8_t*  tx_buf,
        size_t    tx_buf_capacity,
        uint16_t  payload_len,
        FrameType type,
        uint32_t  timestamp);

#endif /* TX_HEADER_H */
