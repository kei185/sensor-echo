#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lidar/core.h"
#include "stm32f446xx.h"

DMA_Stream_TypeDef mock_dma1_stream2;

static uint32_t mock_tick;
static uint32_t mock_tick_step;
static uint32_t mock_dma_ready_at;
static uint32_t mock_dma_ready_size = 3u;

uint32_t HAL_GetTick(void)
{
        uint32_t tick = mock_tick;
        mock_tick += mock_tick_step;
        if (mock_dma_ready_at != 0u && tick >= mock_dma_ready_at) {
                mock_dma1_stream2.NDTR = CORE_RX_BUF_SIZE - mock_dma_ready_size;
                mock_dma_ready_at = 0u;
        }
        return tick;
}

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

static void empty_receive_times_out_without_advancing_the_reader(void)
{
        reset_reader_at(0);
        mock_tick      = 100u;
        mock_tick_step = 1u;
        start_read_timeout();

        // DMAが進まなくても短時間で戻り、未受信の場所は消費しない。
        assert(read_byte() == 0u);
        assert(has_read_timed_out());
        assert(RX_BUF->read_idx == 0);
        assert(mock_tick >= 120u && mock_tick < 130u);
        mock_tick_step = 0u;

        // 後からデータが来ても、失敗したフレームの続きとしては読まない。
        RX_BUF->_buf[0] = 0u;
        set_dma_write_index(3);
        assert(read_byte() == 0u);
        assert(RX_BUF->read_idx == 0);

        // 次の解析では失敗状態を解除し、正常な0バイトも読み取れる。
        start_read_timeout();
        assert(read_byte() == 0u);
        assert(!has_read_timed_out());
        assert(RX_BUF->read_idx == 1);
}

static void byte_wait_can_exceed_two_ms_within_the_frame_deadline(void)
{
        reset_reader_at(0);
        RX_BUF->_buf[0] = 0xa5u;
        mock_tick         = 0u;
        mock_tick_step    = 1u;
        mock_dma_ready_at = 10u;
        start_read_timeout();

        assert(read_byte() == 0xa5u);
        assert(!has_read_timed_out());
        assert(RX_BUF->read_idx == 1);
        assert(mock_tick >= 10u && mock_tick < 40u);

        RX_BUF->_buf[1] = 0x5au;
        mock_dma_ready_at   = mock_tick + 10u;
        mock_dma_ready_size = 4u;
        assert(read_byte() == 0x5au);
        assert(!has_read_timed_out());
        assert(RX_BUF->read_idx == 2);
        assert(mock_tick >= 20u && mock_tick < 40u);
        mock_dma_ready_size = 3u;
        mock_tick_step = 0u;
}

static void frame_deadline_can_expire_before_the_byte_wait_limit(void)
{
        reset_reader_at(0);
        mock_tick      = 0u;
        mock_tick_step = 0u;
        start_read_timeout();

        mock_tick      = 25u;
        mock_tick_step = 1u;
        assert(read_byte() == 0u);
        assert(has_read_timed_out());
        assert(RX_BUF->read_idx == 0);
        assert(mock_tick >= 40u && mock_tick < 45u);
        mock_tick_step = 0u;
}

static void frame_deadline_is_not_extended_by_successful_reads(void)
{
        reset_reader_at(0);
        set_dma_write_index(100);
        mock_tick      = 0u;
        mock_tick_step = 0u;
        start_read_timeout();

        // データが途切れず読めても、解析開始から40msで打ち切る。
        for (uint32_t elapsed = 0u; elapsed < 40u; ++elapsed) {
                mock_tick = elapsed;
                (void)read_byte();
                assert(!has_read_timed_out());
        }
        mock_tick = 40u;
        assert(read_byte() == 0u);
        assert(has_read_timed_out());
        assert(RX_BUF->read_idx == 40);
}

static void empty_ring_end_times_out_without_underflowing_the_lap(void)
{
        reset_reader_at(CORE_RX_BUF_SIZE - 1u);
        mock_tick      = 0u;
        mock_tick_step = 1u;
        start_read_timeout();
        assert(!is_safe_read());
        assert(read_byte() == 0u);
        assert(has_read_timed_out());
        assert(RX_BUF->read_idx == CORE_RX_BUF_SIZE - 1u);
        assert(RX_BUF->lap == 0u);
        mock_tick_step = 0u;

        set_dma_write_index(2);
        increment_lap();
        start_read_timeout();
        assert(is_safe_read());
        (void)read_byte();
        assert(RX_BUF->read_idx == 0);
        assert(RX_BUF->lap == 0u);
        assert(!is_safe_read());
}

static void timeouts_work_across_tick_wraparound(void)
{
        reset_reader_at(0);
        mock_tick      = UINT32_MAX - 1u;
        mock_tick_step = 1u;
        start_read_timeout();

        // バイトの待ち時間がtickの一周をまたいでも20msで戻ってくる。
        assert(read_byte() == 0u);
        assert(has_read_timed_out());
        assert(RX_BUF->read_idx == 0);
        assert(mock_tick < 30u);

        mock_tick_step = 0u;
        mock_tick      = UINT32_MAX - 10u;
        start_read_timeout();
        mock_tick += 39u;
        assert(!has_read_timed_out());
        mock_tick += 1u;
        assert(has_read_timed_out());
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
        empty_receive_times_out_without_advancing_the_reader();
        empty_ring_end_times_out_without_underflowing_the_lap();
        byte_wait_can_exceed_two_ms_within_the_frame_deadline();
        frame_deadline_can_expire_before_the_byte_wait_limit();
        frame_deadline_is_not_extended_by_successful_reads();
        timeouts_work_across_tick_wraparound();
        queue_wraps_after_five_slots_and_preserves_fifo_order();
        return 0;
}
