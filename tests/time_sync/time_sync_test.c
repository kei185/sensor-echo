#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lidar/core.h"
#include "time_sync.h"
#include "tx/frame.h"

static TxBufSlot tx_slot;
static bool      slot_available;
static uint32_t  queued_length;
static uint32_t  dispatch_calls;

static void reset_test_state(void)
{
        memset(&tx_slot, 0, sizeof(tx_slot));
        slot_available = true;
        queued_length  = 0u;
        dispatch_calls = 0u;
}

uint32_t HAL_GetTick(void) { return 0x12345678u; }

TxBufSlot* get_empty_buf(void) { return slot_available ? &tx_slot : NULL; }

void push_full_slot(TxBufSlot* slot, uint32_t length)
{
        assert(slot == &tx_slot);
        slot->length  = length;
        slot->full    = true;
        queued_length = length;
}

void try_dispatch_tx(void) { ++dispatch_calls; }

static void report_queues_a_readable_host_frame(void)
{
        // 準備
        reset_test_state();
        const TimeSyncMeasurement measurement = {
                .req_time = 1000u,
                .res_time = 1008u,
                .unixtime = UINT64_C(1791160123),
        };
        const char expected_payload[] =
                "TIME SYNC: req_tick=1000 ms | res_tick=1008 ms | "
                "round_trip=8 ms | unix_time=1791160123";

        // 実行
        bool reported = report_time_sync_measurement(&measurement);

        // 検証
        assert(reported);
        assert(tx_slot.full);
        assert(dispatch_calls == 1u);
        assert(queued_length == TX_FRAME_HEADER_SIZE + strlen(expected_payload));
        assert(tx_slot._buf[0] == START_OF_FRAME[0]);
        assert(tx_slot._buf[1] == START_OF_FRAME[1]);
        assert(tx_slot._buf[5] == FRAME_TYPE_TIME_SYNC_REPORT);
        assert(tx_slot._buf[6] == 0x78u);
        assert(tx_slot._buf[7] == 0x56u);
        assert(tx_slot._buf[8] == 0x34u);
        assert(tx_slot._buf[9] == 0x12u);
        assert(memcmp(tx_slot._buf + TX_FRAME_HEADER_SIZE,
                      expected_payload,
                      strlen(expected_payload)) == 0);
}

static void report_rejects_missing_input_or_tx_slot(void)
{
        // 準備
        reset_test_state();
        const TimeSyncMeasurement measurement = {0};

        // 実行・検証: 計測値がなければqueueへ触らない。
        assert(!report_time_sync_measurement(NULL));
        assert(dispatch_calls == 0u);

        // 準備
        slot_available = false;

        // 実行・検証: 空きslotがなければ送信を開始しない。
        assert(!report_time_sync_measurement(&measurement));
        assert(dispatch_calls == 0u);
}

static void report_calculates_round_trip_across_tick_wrap(void)
{
        // 準備
        reset_test_state();
        const TimeSyncMeasurement measurement = {
                .req_time = UINT32_MAX - 2u,
                .res_time = 3u,
                .unixtime = UINT64_C(1791160123),
        };
        const char expected_round_trip[] = "round_trip=6 ms";

        // 実行
        bool reported = report_time_sync_measurement(&measurement);

        // 検証: unsigned tick差分なら1回のwrapをまたいでも経過時間を得られる。
        assert(reported);
        assert(strstr((char*)tx_slot._buf + TX_FRAME_HEADER_SIZE, expected_round_trip) !=
               NULL);
}

static void time_sync_report_command_has_the_documented_bytes(void)
{
        // 検証
        assert(HOST_COMMANDS[HOST_COMMAND_TIME_SYNC_REPORT][0] == 0xAAu);
        assert(HOST_COMMANDS[HOST_COMMAND_TIME_SYNC_REPORT][1] == 0xA4u);
}

int main(void)
{
        report_queues_a_readable_host_frame();
        report_rejects_missing_input_or_tx_slot();
        report_calculates_round_trip_across_tick_wrap();
        time_sync_report_command_has_the_documented_bytes();
        return 0;
}
