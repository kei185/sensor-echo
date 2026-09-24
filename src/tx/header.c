#include "tx/header.h"
#include "checksum.h"

#define TX_FRAME_CRC_OFFSET 4u

// TODO このインターフェース微妙
size_t tx_frame_write_header(
        uint8_t*  tx_buf,
        size_t    tx_buf_capacity,
        uint16_t  payload_len,
        FrameType type,
        uint32_t  timestamp)
{
        if (tx_buf == NULL || tx_buf_capacity < TX_FRAME_HEADER_SIZE ||
            payload_len > tx_buf_capacity - TX_FRAME_HEADER_SIZE)
                return 0u;

        const size_t frame_size = TX_FRAME_HEADER_SIZE + payload_len;

        tx_buf[0]                   = START_OF_FRAME[0];
        tx_buf[1]                   = START_OF_FRAME[1];
        tx_buf[2]                   = (uint8_t)payload_len;
        tx_buf[3]                   = (uint8_t)(payload_len >> 8);
        tx_buf[TX_FRAME_CRC_OFFSET] = 0u;
        tx_buf[5]                   = (uint8_t)type;
        tx_buf[6]                   = (uint8_t)timestamp;
        tx_buf[7]                   = (uint8_t)(timestamp >> 8);
        tx_buf[8]                   = (uint8_t)(timestamp >> 16);
        tx_buf[9]                   = (uint8_t)(timestamp >> 24);

        // NOTE: Extend coverage to the whole frame if the scan-time budget allows it.
        // uint8_t crc = crc_8(tx_buf, TX_FRAME_CRC_OFFSET);
        // for (size_t i = TX_FRAME_CRC_OFFSET + 1u; i < frame_size; ++i)
        //         crc = update_crc_8(crc, tx_buf[i]);
        // tx_buf[TX_FRAME_CRC_OFFSET] = crc;
        tx_buf[TX_FRAME_CRC_OFFSET] = crc_8(tx_buf + 2u, 2u);

        return frame_size;
}
