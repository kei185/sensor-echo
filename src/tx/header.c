#include "tx/header.h"

#define TX_FRAME_CRC_OFFSET 4u
#define TX_FRAME_CRC_POLY   0x07u

static uint8_t crc8_update(uint8_t crc, uint8_t byte)
{
        for (uint8_t bit = 0; bit < 8u; ++bit) {
                uint8_t feedback = ((crc >> 7) ^ (byte >> 7)) & 1u;
                crc              = (uint8_t)(crc << 1);
                if (feedback != 0u)
                        crc ^= TX_FRAME_CRC_POLY;
                byte <<= 1;
        }

        return crc;
}

size_t tx_frame_write_header(
        uint8_t*    tx_buf,
        size_t      tx_buf_capacity,
        uint16_t    payload_len,
        TxFrameType type,
        uint32_t    timestamp)
{
        if (tx_buf == NULL || tx_buf_capacity < TX_FRAME_HEADER_SIZE ||
            payload_len > tx_buf_capacity - TX_FRAME_HEADER_SIZE ||
            type > TX_FRAME_TYPE_ENCODER)
                return 0u;

        const size_t frame_size = TX_FRAME_HEADER_SIZE + payload_len;

        tx_buf[0]                   = 0xaau;
        tx_buf[1]                   = 0x55u;
        tx_buf[2]                   = (uint8_t)(payload_len >> 8);
        tx_buf[3]                   = (uint8_t)payload_len;
        tx_buf[TX_FRAME_CRC_OFFSET] = 0u;
        tx_buf[5]                   = (uint8_t)type;
        tx_buf[6]                   = (uint8_t)(timestamp >> 24);
        tx_buf[7]                   = (uint8_t)(timestamp >> 16);
        tx_buf[8]                   = (uint8_t)(timestamp >> 8);
        tx_buf[9]                   = (uint8_t)timestamp;

        uint8_t crc = 0u;
        for (size_t i = 0; i < frame_size; ++i) {
                if (i != TX_FRAME_CRC_OFFSET)
                        crc = crc8_update(crc, tx_buf[i]);
        }
        tx_buf[TX_FRAME_CRC_OFFSET] = crc;

        return frame_size;
}
