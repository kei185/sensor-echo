#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lidar/core.h"
#include "stm32f446xx.h"

DMA_Stream_TypeDef mock_dma1_stream2;

static void set_dma_write_index(int32_t index)
{
        assert(index >= 0);
        assert(index < (int32_t)CORE_RX_BUF_SIZE);
        mock_dma1_stream2.NDTR = CORE_RX_BUF_SIZE - (uint32_t)index;
}

static void reset_reader_at(int32_t index)
{
        setup_nonblocking_rx();
        set_dma_write_index(index);
        reset_read_idx();
        assert(RX_BUF->read_idx == index);
}

static void overrun_detection_uses_signed_ring_index_differences(void)
{
        // 1周後でもDMA位置がreaderより手前なら、まだ追いついていない。
        reset_reader_at(3500);
        increment_lap();
        set_dma_write_index(100);
        assert(!is_lapped());

        // 1周したDMAがreaderより先へ進むと追い越しになる。
        reset_reader_at(100);
        increment_lap();
        set_dma_write_index(101);
        assert(is_lapped());
}

static void dma_terminal_count_is_normalized_to_the_ring_start(void)
{
        reset_reader_at(123);

        // NDTR == 0はreload直前のring終端なので、次の位置0として扱う。
        mock_dma1_stream2.NDTR = 0u;
        reset_read_idx();
        assert(RX_BUF->read_idx == 0);
}

static void queue_wraps_after_five_slots_and_preserves_fifo_order(void)
{
        TxBufSlot* slots[CORE_TX_BUF_NUM];

        // 5本を順番に埋めると、書き込み位置は先頭へ一周する。
        for (uint32_t i = 0u; i < CORE_TX_BUF_NUM; ++i) {
                slots[i] = get_empty_buf();
                assert(slots[i] == &TX_BUF->slots[i]);
                push_full_slot(slots[i], 100u + i);
        }

        // 一周後の先頭は未送信なので、6本目は確保できない。
        assert(get_empty_buf() == NULL);

        // 先頭を解放すれば、その場所だけを次の書き込みに再利用できる。
        assert(get_full_buf() == slots[0]);
        release(slots[0]);
        assert(get_empty_buf() == slots[0]);
        push_full_slot(slots[0], 200u);
        assert(get_empty_buf() == NULL);

        // 読み手は古い4本を先に送り、最後に再利用した先頭を送る。
        for (uint32_t i = 1u; i < CORE_TX_BUF_NUM; ++i) {
                assert(get_full_buf() == slots[i]);
                release(slots[i]);
        }
        assert(get_full_buf() == slots[0]);
        assert(get_full_buf()->length == 200u);
        release(slots[0]);

        // 全slotを解放するとqueueはemptyになり、次の一周を開始できる。
        assert(get_full_buf() == NULL);
        assert(get_empty_buf() == slots[1]);
}

int main(void)
{
        static_assert(CORE_TX_BUF_NUM == 5u, "TX queue must have five slots");
        static_assert(
                sizeof(TxBuf) == 6764u,
                "Update the documented target memory estimate");

        overrun_detection_uses_signed_ring_index_differences();
        dma_terminal_count_is_normalized_to_the_ring_start();
        queue_wraps_after_five_slots_and_preserves_fifo_order();
        return 0;
}
