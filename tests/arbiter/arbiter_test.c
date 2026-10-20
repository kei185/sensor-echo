#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "arbiter.h"
#include "lidar/core.h"
#include "stm32f4xx_hal_uart.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

static uint8_t           rx_storage[CORE_RX_BUF_SIZE];
static RxBuf             rx_buf = {._buf = rx_storage};
const RxBuf* const       RX_BUF = &rx_buf;
static uint32_t          rx_reset_calls;
static uint32_t          rx_start_calls;
static uint32_t          rx_abort_calls;
static HAL_StatusTypeDef rx_start_result;
static HAL_StatusTypeDef rx_abort_result;

static TxBufSlot fake_slots[CORE_TX_BUF_NUM];
static uint8_t   fake_reader_head;
static uint32_t  release_calls;
static uint32_t  lap_increment_calls;

static HAL_StatusTypeDef dma_start_result;
static uint32_t          dma_start_calls;
static const uint8_t*    dma_start_data;
static uint16_t          dma_start_length;

static void reset_test_state(void)
{
        memset(fake_slots, 0, sizeof(fake_slots));
        fake_reader_head    = 0u;
        release_calls       = 0u;
        lap_increment_calls = 0u;
        dma_start_result    = HAL_OK;
        dma_start_calls     = 0u;
        dma_start_data      = NULL;
        dma_start_length    = 0u;
        huart2.gState       = HAL_UART_STATE_READY;
        rx_reset_calls      = 0u;
        rx_start_calls      = 0u;
        rx_abort_calls      = 0u;
        rx_start_result     = HAL_OK;
        rx_abort_result     = HAL_OK;
}

static void queue_frame(uint8_t index, uint32_t length, uint8_t marker)
{
        fake_slots[index].full    = true;
        fake_slots[index].length  = length;
        fake_slots[index]._buf[0] = marker;
}

TxBufSlot* get_full_buf(void)
{
        TxBufSlot* slot = &fake_slots[fake_reader_head];
        return slot->full ? slot : NULL;
}

void release(TxBufSlot* slot)
{
        assert(slot == &fake_slots[fake_reader_head]);
        slot->length = 0u;
        slot->full   = false;
        ++release_calls;
        if (++fake_reader_head >= CORE_TX_BUF_NUM)
                fake_reader_head = 0u;
}

void increment_lap(void) { ++lap_increment_calls; }

void setup_nonblocking_rx(void)
{
        ++rx_reset_calls;
        rx_buf.read_idx = 0;
        rx_buf.lap      = 0u;
}

HAL_StatusTypeDef
HAL_UART_Receive_DMA(UART_HandleTypeDef* uart, uint8_t* data, uint16_t length)
{
        assert(uart == &huart4);
        assert(data == rx_storage);
        assert(length == CORE_RX_BUF_SIZE);
        ++rx_start_calls;
        return rx_start_result;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef* uart)
{
        assert(uart == &huart4);
        ++rx_abort_calls;
        return rx_abort_result;
}

HAL_StatusTypeDef
HAL_UART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length)
{
        assert(huart == &huart2);
        if (huart->gState != HAL_UART_STATE_READY)
                return HAL_BUSY;

        ++dma_start_calls;
        dma_start_data   = data;
        dma_start_length = length;
        if (dma_start_result == HAL_OK)
                huart->gState = HAL_UART_STATE_BUSY_TX;
        return dma_start_result;
}

static void try_dispatch_tx_starts_the_oldest_queued_frame(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 24u, 0x31u);

        // 実行
        try_dispatch_tx();

        // 検証
        assert(dma_start_calls == 1u);
        assert(dma_start_data == fake_slots[0]._buf);
        assert(dma_start_length == 24u);
        assert(fake_slots[0].full);
}

static void try_dispatch_tx_does_not_start_a_second_concurrent_transfer(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 12u, 0x41u);
        queue_frame(1u, 13u, 0x42u);
        try_dispatch_tx();

        // 実行
        try_dispatch_tx();

        // 検証
        assert(dma_start_calls == 1u);
        assert(release_calls == 0u);
}

static void transmit_completion_releases_the_slot_and_starts_the_next_frame(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 12u, 0x51u);
        queue_frame(1u, 13u, 0x52u);
        try_dispatch_tx();
        huart2.gState = HAL_UART_STATE_READY;

        // 実行
        uart_transmit_complete_handler();

        // 検証
        assert(release_calls == 1u);
        assert(!fake_slots[0].full);
        assert(dma_start_calls == 2u);
        assert(dma_start_data == fake_slots[1]._buf);
        assert(dma_start_length == 13u);
}

static void dma_start_failure_keeps_the_frame_queued_for_retry(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 32u, 0x61u);
        dma_start_result = HAL_BUSY;

        // 実行
        try_dispatch_tx();

        // 検証
        assert(dma_start_calls == 1u);
        assert(fake_slots[0].full);
        assert(release_calls == 0u);

        // 準備
        dma_start_result = HAL_OK;

        // 実行
        try_dispatch_tx();

        // 検証
        assert(dma_start_calls == 2u);
}

static void invalid_queued_frame_is_dropped_before_starting_the_next_frame(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 0u, 0x71u);
        queue_frame(1u, 14u, 0x72u);

        // 実行
        try_dispatch_tx();

        // 検証
        assert(release_calls == 1u);
        assert(!fake_slots[0].full);
        assert(dma_start_calls == 1u);
}

static void receive_completion_records_one_dma_ring_wrap(void)
{
        // 準備
        reset_test_state();

        // 実行
        dma_receive_complete_handler();

        // 検証
        assert(lap_increment_calls == 1u);
}

static void failed_receive_restarts_dma_and_retries_a_failed_restart(void)
{
        reset_test_state();
        assert(!recover_lidar_rx());
        rx_buf.read_idx = 4000;
        rx_buf.lap      = 2u;
        rx_start_result = HAL_BUSY;
        rx_abort_result = HAL_TIMEOUT;
        dma_receive_error_handler();
        assert(rx_start_calls == 0u);
        assert(recover_lidar_rx());
        assert(rx_abort_calls == 1u && rx_start_calls == 0u);

        rx_abort_result = HAL_OK;
        assert(recover_lidar_rx());
        assert(rx_start_calls == 1u);
        assert(rx_buf.read_idx == 0 && rx_buf.lap == 0u);

        rx_start_result = HAL_OK;
        assert(recover_lidar_rx());
        assert(rx_start_calls == 2u && rx_reset_calls == 2u);
        assert(!recover_lidar_rx());
        assert(rx_start_calls == 2u);
}

static void transmit_queue_returns_to_the_first_slot_after_wraparound(void)
{
        // 準備
        reset_test_state();
        for (uint8_t i = 0u; i < CORE_TX_BUF_NUM; ++i)
                queue_frame(i, (uint32_t)(20u + i), (uint8_t)(0x80u + i));
        try_dispatch_tx();

        // 実行
        for (uint8_t i = 0u; i < CORE_TX_BUF_NUM; ++i) {
                huart2.gState = HAL_UART_STATE_READY;
                uart_transmit_complete_handler();
        }
        queue_frame(0u, 30u, 0x90u);
        try_dispatch_tx();

        // 検証
        assert(release_calls == CORE_TX_BUF_NUM);
        assert(dma_start_calls == CORE_TX_BUF_NUM + 1u);
        assert(dma_start_length == 30u);
}

int main(void)
{
        try_dispatch_tx_starts_the_oldest_queued_frame();
        try_dispatch_tx_does_not_start_a_second_concurrent_transfer();
        transmit_completion_releases_the_slot_and_starts_the_next_frame();
        dma_start_failure_keeps_the_frame_queued_for_retry();
        invalid_queued_frame_is_dropped_before_starting_the_next_frame();
        receive_completion_records_one_dma_ring_wrap();
        failed_receive_restarts_dma_and_retries_a_failed_restart();
        transmit_queue_returns_to_the_first_slot_after_wraparound();
        return 0;
}
