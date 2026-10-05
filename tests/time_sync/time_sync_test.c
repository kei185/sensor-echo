#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "time_sync.h"
#include "tx/frame.h"

static void unix_time_is_decoded_from_little_endian_bytes(void)
{
        // 準備
        const uint8_t encoded[TIME_SYNC_UNIX_TIME_SIZE] = {
                0x08u,
                0x07u,
                0x06u,
                0x05u,
                0x04u,
                0x03u,
                0x02u,
                0x01u,
        };
        uint64_t unixtime = 0u;

        // 実行
        bool decoded = time_sync_decode_unix_time(encoded, sizeof(encoded), &unixtime);

        // 検証
        assert(decoded);
        assert(unixtime == UINT64_C(0x0102030405060708));
}

static void session_records_exactly_five_measurements(void)
{
        // 準備
        TimeSyncSession session = {0};

        // 実行・検証
        for (uint8_t i = 0u; i < TIME_SYNC_SAMPLE_COUNT; ++i) {
                assert(time_sync_record(
                        &session,
                        (uint32_t)(1000u + i * 10u),
                        (uint32_t)(1004u + i * 10u),
                        UINT64_C(1791160123000) + i));
        }
        assert(session.count == TIME_SYNC_SAMPLE_COUNT);
        assert(!time_sync_record(&session, 2000u, 2001u, UINT64_C(1791160124000)));
}

static void report_formats_all_five_measurements(void)
{
        // 準備
        TimeSyncSession session = {0};
        for (uint8_t i = 0u; i < TIME_SYNC_SAMPLE_COUNT; ++i) {
                assert(time_sync_record(
                        &session,
                        (uint32_t)(1000u + i * 10u),
                        (uint32_t)(1004u + i * 10u),
                        UINT64_C(1791160123000) + i));
        }
        char report[1024];

        // 実行
        size_t length = time_sync_format_report(report, sizeof(report), &session);

        // 検証
        assert(length == strlen(report));
        const char expected_prefix[] = "TIME SYNC: samples=5 | #1";
        assert(strncmp(report, expected_prefix, strlen(expected_prefix)) == 0);
        assert(strstr(report, "#1 req_tick=1000 ms res_tick=1004 ms round_trip=4 ms") !=
               NULL);
        assert(strstr(report,
                      "#5 req_tick=1040 ms res_tick=1044 ms round_trip=4 ms "
                      "unix_time=1791160123004 ms") != NULL);
}

static void report_calculates_round_trip_across_tick_wrap(void)
{
        // 準備
        TimeSyncSession session = {0};
        assert(time_sync_record(&session, UINT32_MAX - 2u, 3u, UINT64_C(1791160123000)));
        char report[256];

        // 実行
        size_t length = time_sync_format_report(report, sizeof(report), &session);

        // 検証: unsigned tick差分なら1回のwrapをまたいでも経過時間を得られる。
        assert(length > 0u);
        assert(strstr(report, "round_trip=6 ms") != NULL);
}

static void time_sync_start_command_has_the_documented_bytes(void)
{
        // 検証
        assert(HOST_COMMANDS[HOST_COMMAND_TIME_SYNC_START][0] == 0xAAu);
        assert(HOST_COMMANDS[HOST_COMMAND_TIME_SYNC_START][1] == 0xA4u);
}

int main(void)
{
        unix_time_is_decoded_from_little_endian_bytes();
        session_records_exactly_five_measurements();
        report_formats_all_five_measurements();
        report_calculates_round_trip_across_tick_wrap();
        time_sync_start_command_has_the_documented_bytes();
        return 0;
}
