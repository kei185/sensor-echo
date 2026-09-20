
#include <stdint.h>
#include <stdbool.h>
#include <cmsis_gcc.h>
#include <stdlib.h>

#include "stm32f446xx.h"
#include "lidar/core.h"

/*
 * is_safe_read() requires:
 *
 *     write_idx - (read_idx + 1) > 1
 *
 * which is equivalent to:
 *
 *     read_idx < write_idx - 2
 *
 * A blocking reply has no moving DMA write head, so expose a virtual write
 * position two bytes past the received data. This makes the last real byte
 * readable while the virtual guard bytes remain unreadable.
 */
#define BLOCKING_READ_GUARD_SIZE 2u

static int8_t rx_storage[CORE_RX_BUF_SIZE];

static RxBuf _RX_BUF = {
        .lap          = 0,
        .remain_bytes = &DMA1_Stream2->NDTR,
        ._buf         = rx_storage,
        .read_idx     = 0,
};
const RxBuf* const RX_BUF = &_RX_BUF;

static volatile uint32_t blocking_remain_bytes = CORE_RX_BUF_SIZE;

static inline uint32_t next_idx(void)
{
        uint32_t n = _RX_BUF.read_idx + 1;

        if (_RX_BUF.read_idx + 1 >= CORE_RX_BUF_SIZE)
                return 0;

        return n;
};

static inline void increment(void)
{
        if (++_RX_BUF.read_idx >= CORE_RX_BUF_SIZE)
                _RX_BUF.read_idx = 0;
}

static inline uint32_t write_idx(void)
{
        return CORE_RX_BUF_SIZE - *_RX_BUF.remain_bytes;
}

bool is_lapped()

{
        return (_RX_BUF.lap >= 1 && write_idx() - _RX_BUF.read_idx > 0) ||
               _RX_BUF.lap > 1;
}

void reset_read_idx()
{
        _RX_BUF.read_idx = write_idx();
        _RX_BUF.lap      = 0;
}

/**
 * called dma complete IRQ
 */
void increment_lap() { _RX_BUF.lap++; }

/**
 * @brief check if reading l items from buffer is safe
 * or if that is within read-ready rx buffer
 */
bool is_safe_read()
{
        int32_t diff = (write_idx() - next_idx());

        if (abs(diff) <= 1)
                return false;

        return true;
}

/**
 * @brief read a byte from the buffer if buffer is safe to read,
 * otherwise wait for buffer to become safe.
 * and increments the internal index;
 *
 * @return a buffer item of a byte
 */
int8_t read_byte()
{
        while (!is_safe_read())
                ;

        int8_t byte = _RX_BUF._buf[_RX_BUF.read_idx];

        increment();

        if (_RX_BUF.read_idx == 0)
                _RX_BUF.lap--;

        return byte;
}

/**
 * @brief decode little endian to Byte filed to number reading with specified length
 *
 */
uint32_t dec_little_endian(const uint8_t len)
{
        uint32_t ret = 0;

        for (uint8_t i = 0; i < len; ++i)
                // Preserve the wire byte before widening the signed read_byte result.
                ret |= (uint32_t)(uint8_t)read_byte() << (i * 8);

        return ret;
}

void setup_blocking_rx(size_t length)
{
        _RX_BUF.read_idx = 0u;
        _RX_BUF.lap      = 0u;
        blocking_remain_bytes =
                CORE_RX_BUF_SIZE - (uint32_t)length - BLOCKING_READ_GUARD_SIZE;
        _RX_BUF.remain_bytes = &blocking_remain_bytes;
}

void setup_nonblocking_rx(void)
{
        _RX_BUF.read_idx     = 0u;
        _RX_BUF.lap          = 0u;
        _RX_BUF.remain_bytes = &DMA1_Stream2->NDTR;
}

static TxBuf _TX_BUF = {
        .writer_head = 0,
        .reader_head = 0,
        .slots =
                {
                        [0] = {.full = false, ._buf = {0}},
                        [1] = {.full = false, ._buf = {0}},
                        [2] = {.full = false, ._buf = {0}},
                        [3] = {.full = false, ._buf = {0}},
                        [4] = {.full = false, ._buf = {0}},
                },
};

const TxBuf* const TX_BUF = &_TX_BUF;

static inline bool full(void) { return _TX_BUF.slots[_TX_BUF.writer_head].full; }

// reader release buf setting it empty when transmission done
void release(TxBufSlot* slot)
{
        slot->length = 0;
        __DMB();
        slot->full = false;

        // let head point to the next read target
        if (++_TX_BUF.reader_head >= CORE_TX_BUF_NUM)
                _TX_BUF.reader_head = 0;
}

// reader gets a buf to transmit
TxBufSlot* get_full_buf(void)
{
        TxBufSlot* reader_slot = &_TX_BUF.slots[_TX_BUF.reader_head];

        if (reader_slot->full)
                return reader_slot;

        return NULL;
}

// writer gets an available buf
TxBufSlot* get_empty_buf(void)
{
        if (full())
                return NULL;

        TxBufSlot* writer_slot = &_TX_BUF.slots[_TX_BUF.writer_head];

        return writer_slot;
}

void push_full_slot(TxBufSlot* slot, uint32_t len)
{
        slot->length = len;
        __DMB();
        slot->full = true;

        // let head point to the next write target
        if (++_TX_BUF.writer_head >= CORE_TX_BUF_NUM)
                _TX_BUF.writer_head = 0;
}
