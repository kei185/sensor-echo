#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "arbiter.h"
#include "lidar/core.h"
#include "lidar/sys.h"
#include "stm32f4xx_hal_uart.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

const uint16_t       MSG_SIZE         = 2u;
static const uint8_t health_command[] = {0xa5u, 0x92u};
static const uint8_t device_command[] = {0xa5u, 0x90u};
const uint8_t*       MSG[]            = {
        [MSG_TYPE_RX_HEALTH]   = health_command,
        [MSG_TYPE_RX_SYS_INFO] = device_command,
};

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
        reset_arbiter();
}

static void queue_frame(uint8_t index, uint32_t length, uint8_t marker)
{
        fake_slots[index].full    = true;
        fake_slots[index].length  = length;
        fake_slots[index]._buf[0] = (int8_t)marker;
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

HAL_StatusTypeDef
HAL_UART_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length)
{
        assert(huart == &huart2);
        ++dma_start_calls;
        dma_start_data   = data;
        dma_start_length = length;
        return dma_start_result;
}

HAL_StatusTypeDef HAL_UART_Transmit(
        UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length, uint32_t timeout)
{
        (void)huart;
        (void)data;
        (void)length;
        (void)timeout;
        return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(
        UART_HandleTypeDef* huart, uint8_t* data, uint16_t length, uint32_t timeout)
{
        (void)huart;
        (void)data;
        (void)length;
        (void)timeout;
        return HAL_OK;
}

static void arbitrate_starts_the_oldest_queued_frame(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 24u, 0x31u);

        // 実行
        arbitrate();

        // 検証
        assert(dma_start_calls == 1u);
        assert(dma_start_data == (const uint8_t*)fake_slots[0]._buf);
        assert(dma_start_length == 24u);
        assert(ARBITER->transmitting == &fake_slots[0]);
        assert(ARBITER->tx_started == 1u);
        assert(fake_slots[0].full);
}

static void arbitrate_does_not_start_a_second_concurrent_transfer(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 12u, 0x41u);
        queue_frame(1u, 13u, 0x42u);
        arbitrate();

        // 実行
        arbitrate();

        // 検証
        assert(dma_start_calls == 1u);
        assert(ARBITER->transmitting == &fake_slots[0]);
        assert(release_calls == 0u);
}

static void transmit_completion_releases_the_slot_and_starts_the_next_frame(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 12u, 0x51u);
        queue_frame(1u, 13u, 0x52u);
        arbitrate();

        // 実行
        uart_transmit_complete_handler();

        // 検証
        assert(release_calls == 1u);
        assert(!fake_slots[0].full);
        assert(dma_start_calls == 2u);
        assert(dma_start_data == (const uint8_t*)fake_slots[1]._buf);
        assert(dma_start_length == 13u);
        assert(ARBITER->transmitting == &fake_slots[1]);
        assert(ARBITER->tx_completed == 1u);
        assert(ARBITER->tx_started == 2u);
}

static void dma_start_failure_keeps_the_frame_queued_for_retry(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 32u, 0x61u);
        dma_start_result = HAL_BUSY;

        // 実行
        arbitrate();

        // 検証
        assert(dma_start_calls == 1u);
        assert(ARBITER->transmitting == NULL);
        assert(ARBITER->tx_start_failures == 1u);
        assert(fake_slots[0].full);
        assert(release_calls == 0u);

        // 準備
        dma_start_result = HAL_OK;

        // 実行
        arbitrate();

        // 検証
        assert(dma_start_calls == 2u);
        assert(ARBITER->transmitting == &fake_slots[0]);
        assert(ARBITER->tx_started == 1u);
}

static void invalid_queued_frame_is_dropped_before_starting_the_next_frame(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 0u, 0x71u);
        queue_frame(1u, 14u, 0x72u);

        // 実行
        arbitrate();

        // 検証
        assert(release_calls == 1u);
        assert(!fake_slots[0].full);
        assert(ARBITER->invalid_tx_frames == 1u);
        assert(dma_start_calls == 1u);
        assert(ARBITER->transmitting == &fake_slots[1]);
}

static void transmit_error_drops_the_active_frame_and_starts_the_next_frame(void)
{
        // 準備
        reset_test_state();
        queue_frame(0u, 20u, 0x21u);
        queue_frame(1u, 21u, 0x22u);
        arbitrate();

        // 実行
        uart_transmit_error_handler();

        // 検証
        assert(release_calls == 1u);
        assert(!fake_slots[0].full);
        assert(ARBITER->tx_transfer_failures == 1u);
        assert(dma_start_calls == 2u);
        assert(ARBITER->transmitting == &fake_slots[1]);
}

static void receive_completion_records_one_dma_ring_wrap(void)
{
        // 準備
        reset_test_state();

        // 実行
        dma_receive_complete_handler();

        // 検証
        assert(lap_increment_calls == 1u);
        assert(ARBITER->rx_wraps == 1u);
}

int main(void)
{
        arbitrate_starts_the_oldest_queued_frame();
        arbitrate_does_not_start_a_second_concurrent_transfer();
        transmit_completion_releases_the_slot_and_starts_the_next_frame();
        dma_start_failure_keeps_the_frame_queued_for_retry();
        invalid_queued_frame_is_dropped_before_starting_the_next_frame();
        transmit_error_drops_the_active_frame_and_starts_the_next_frame();
        receive_completion_records_one_dma_ring_wrap();
        return 0;
}
