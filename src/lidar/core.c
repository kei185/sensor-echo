
#include <stdint.h>
#include <stdbool.h>
#include <cmsis_gcc.h>

#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "lidar/core.h"

// TODO このファイルデカい

#define BLOCKING_READ_GUARD_SIZE 2u
// blocking受信でも末尾まで読めるよう、受信済み範囲の後ろに仮想の2バイトを置く。

// データが途切れた待ちは短く切り、フレーム全体にも上限を設ける。
// 最大775バイトのscanは230400 bpsで約34msかかるため、全体には40msを許す。
#define LIDAR_BYTE_TIMEOUT_MS  2u
#define LIDAR_FRAME_TIMEOUT_MS 40u

static uint32_t read_started_at;
static bool     read_timed_out;

static uint8_t rx_storage[CORE_RX_BUF_SIZE];

static RxBuf _RX_BUF = {
        .lap          = 0,
        .remain_bytes = &DMA1_Stream2->NDTR,
        ._buf         = rx_storage,
        .read_idx     = 0,
};
const RxBuf* const RX_BUF = &_RX_BUF;

static volatile uint32_t blocking_remain_bytes = CORE_RX_BUF_SIZE;

static inline void increment(void)
{
        if (++_RX_BUF.read_idx >= (int32_t)CORE_RX_BUF_SIZE)
                _RX_BUF.read_idx = 0;
}

static inline int32_t write_idx(void)
{
        const int32_t write = (int32_t)CORE_RX_BUF_SIZE - (int32_t)*_RX_BUF.remain_bytes;

        // NDTR reaches zero at the ring end, so this calculation briefly
        // produces 4096. DMA writes the next byte at index 0, so return 0.
        return write == (int32_t)CORE_RX_BUF_SIZE ? 0 : write;
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
        uint32_t available =
                (uint32_t)(write_idx() - _RX_BUF.read_idx + (int32_t)CORE_RX_BUF_SIZE) %
                CORE_RX_BUF_SIZE;
        return available > BLOCKING_READ_GUARD_SIZE;
}

void start_read_timeout(void)
{
        read_started_at = HAL_GetTick();
        read_timed_out  = false;
}

bool has_read_timed_out(void)
{
        // unsignedの差分で、tickが一周した場合も経過時間を求める。
        if ((uint32_t)(HAL_GetTick() - read_started_at) >= LIDAR_FRAME_TIMEOUT_MS)
                read_timed_out = true;

        return read_timed_out;
}

/**
 * @brief read a byte from the buffer if buffer is safe to read,
 * otherwise wait for buffer to become safe.
 * and increments the internal index;
 *
 * @return a buffer item of a byte
 */
uint8_t read_byte(void)
{
        const uint32_t wait_started_at = HAL_GetTick();
        while (!is_safe_read()) {
                if (has_read_timed_out() ||
                    (uint32_t)(HAL_GetTick() - wait_started_at) >= LIDAR_BYTE_TIMEOUT_MS) {
                        read_timed_out = true;
                        return 0u;
                }
        }

        // 失敗後は残りを読み進めず、parser側でフレーム全体を破棄する。
        if (has_read_timed_out())
                return 0u;

        uint8_t byte = _RX_BUF._buf[_RX_BUF.read_idx];

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
                ret |= (uint32_t)read_byte() << (i * 8);

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
// increments read_head
void release(TxBufSlot* slot)
{
        slot->length = 0;
        __DMB();
        slot->full = false;

        // let head point to the next read target
        if (++_TX_BUF.reader_head >= CORE_TX_BUF_NUM)
                _TX_BUF.reader_head = 0;
}

// dispatcher gets a buf to transmit
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

// increments write_head
void push_full_slot(TxBufSlot* slot, uint32_t len)
{
        slot->length = len;
        __DMB();
        slot->full = true;

        // let head point to the next write target
        if (++_TX_BUF.writer_head >= CORE_TX_BUF_NUM)
                _TX_BUF.writer_head = 0;
}
