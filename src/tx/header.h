#ifndef TX_HEADER_H
#define TX_HEADER_H

#include <stddef.h>
#include <stdint.h>

#define TX_FRAME_HEADER_SIZE 10u

typedef enum
{
        TX_FRAME_TYPE_SYSTEM  = 0x00,
        TX_FRAME_TYPE_LIDAR   = 0x01,
        TX_FRAME_TYPE_IMU     = 0x02,
        TX_FRAME_TYPE_ENCODER = 0x03,
} TxFrameType;

/**
 * Write the header after payload_len bytes have been placed at
 * tx_buf + TX_FRAME_HEADER_SIZE. Returns the complete frame size, or zero if
 * the buffer, capacity, or type is invalid. On failure, tx_buf is unchanged.
 */
size_t tx_frame_write_header(
        uint8_t*    tx_buf,
        size_t      tx_buf_capacity,
        uint16_t    payload_len,
        TxFrameType type,
        uint32_t    timestamp);

#endif /* TX_HEADER_H */
