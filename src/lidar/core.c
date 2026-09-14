#include <stdint.h>
#include <stdbool.h>
#include <cmsis_gcc.h>
#include "lidar/core.h"

static int8_t rx_storage[CORE_RX_BUF_SIZE * CORE_RX_BUF_NUM];
static RxBuf  _RX_BUF = {
        .ready_buf_idx = -1,
        ._bufs =
                {
                        [0] = &rx_storage[0],
                        [1] = &rx_storage[CORE_RX_BUF_SIZE],
                },
};
const RxBuf* const RX_BUF = &_RX_BUF;

/**
 * @return buffer safe to read out of multiple buffer
 */
int8_t* cur_buf() { return RX_BUF->_bufs[RX_BUF->ready_buf_idx]; }

/**
 * @brief check if reading l items from buffer is safe
 * or if that is within read-ready rx buffer
 */
bool is_safe_read(const int8_t* buf, uint32_t l)
{
        return cur_buf() <= buf + l && buf + l <= cur_buf() + CORE_RX_BUF_SIZE;
}

// TODO **の引数取ってincrementしたほうがいい？
/**
 * @brief read a byte from the buffer if buffer is safe to read,
 * otherwise wait for buffer to become safe.
 * @return a buffer item of a byte
 */
int8_t read_byte(const int8_t* buf)
{
        while (!is_safe_read(buf, 1))
                __WFI();

        return *buf;
}

/**
 * @brief decode little endian to Byte filed to number reading with specified length
 *
 */
uint32_t dec_little_endian(const int8_t* buf, const uint8_t len)
{
        uint32_t ret = 0;

        for (uint8_t i = 0; i < len; ++i) {
                // Preserve the wire byte before widening the signed read_byte result.
                ret |= (uint32_t)(uint8_t)read_byte(buf) << (i * 8);
                buf++;
        }

        return ret;
}

static int8_t tx_storage[CORE_TX_BUF_SIZE * CORE_TX_BUF_NUM];
static TxBuf  _TX_BUF = {
        ._bufs =
                {
                        [0] = {.full         = false,
                               .transmitting = false,
                               ._buf         = &tx_storage[0]},
                        [1] = {.full         = false,
                               .transmitting = false,
                               ._buf         = &tx_storage[CORE_TX_BUF_SIZE]},
                        [2] = {.full         = false,
                               .transmitting = false,
                               ._buf         = &tx_storage[2u * CORE_TX_BUF_SIZE]},
                },
};
const TxBuf* const TX_BUF = &_TX_BUF;
