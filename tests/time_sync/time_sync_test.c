#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "lidar/core.h"
#include "time_sync.h"
#include "tx/frame.h"

UART_HandleTypeDef huart2;

typedef enum
{
        EVENT_TIME_SYNC_START_ACK,
        EVENT_TIME_ACK,
        EVENT_TIME_SYNC_REPORT,
} Event;

#define EVENT_CAPACITY (TIME_SYNC_SAMPLE_COUNT * 2u + 1u)

static Event    events[EVENT_CAPACITY];
static uint8_t  event_count;
static uint8_t  receive_count;
static uint32_t fake_tick;
static bool     invalid_time_command;

static bool
payload_equals(const uint8_t* frame, uint16_t frame_length, const char* message)
{
        size_t message_length = strlen(message);
        return frame_length == TX_FRAME_HEADER_SIZE + message_length &&
               memcmp(frame + TX_FRAME_HEADER_SIZE, message, message_length) == 0;
}

static void reset_uart_state(void)
{
        memset(events, 0, sizeof(events));
        event_count          = 0u;
        receive_count        = 0u;
        fake_tick            = 1000u;
        invalid_time_command = false;
}

uint32_t HAL_GetTick(void) { return fake_tick++; }

HAL_StatusTypeDef HAL_UART_Transmit(
        UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length, uint32_t timeout)
{
        assert(huart == &huart2);
        assert(timeout == 100u);
        assert(event_count < EVENT_CAPACITY);

        if (payload_equals(data, length, FRAME_MESSAGE_TIME_SYNC_START_ACK)) {
                assert(data[5] == FRAME_TYPE_TIME_SYNC_START_ACK);
                events[event_count++] = EVENT_TIME_SYNC_START_ACK;
        } else if (payload_equals(data, length, FRAME_MESSAGE_TIME_ACK)) {
                assert(data[5] == FRAME_TYPE_TIME_ACK);
                events[event_count++] = EVENT_TIME_ACK;
        } else {
                assert(data[5] == FRAME_TYPE_TIME_SYNC_REPORT);
                assert(memcmp(data + TX_FRAME_HEADER_SIZE,
                              "TIME SYNC: samples=5",
                              strlen("TIME SYNC: samples=5")) == 0);
                events[event_count++] = EVENT_TIME_SYNC_REPORT;
        }
        return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(
        UART_HandleTypeDef* huart, uint8_t* data, uint16_t length, uint32_t timeout)
{
        assert(huart == &huart2);
        assert(length == TIME_SYNC_TIME_FRAME_SIZE);
        assert(timeout == HAL_MAX_DELAY);

        memcpy(data, HOST_COMMANDS[HOST_COMMAND_TIME], HOST_COMMAND_SIZE);
        if (invalid_time_command)
                data[1] ^= 0x01u;

        uint64_t unixtime = UINT64_C(1791160123000) + receive_count;
        for (uint8_t i = 0u; i < TIME_SYNC_UNIX_TIME_SIZE; ++i)
                data[HOST_COMMAND_SIZE + i] = (uint8_t)(unixtime >> (i * 8u));

        ++receive_count;
        return HAL_OK;
}

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
        assert(HOST_COMMANDS[HOST_COMMAND_TIME][0] == 0xAAu);
        assert(HOST_COMMANDS[HOST_COMMAND_TIME][1] == 0xA5u);
}

static void handler_collects_five_command_prefixed_time_frames(void)
{
        // 準備: callerがAA A4を受け取った後に同じsessionを5回渡す。
        reset_uart_state();
        TimeSyncSession session                    = {0};
        uint8_t         tx_frame[CORE_TX_BUF_SIZE] = {0};

        // 実行
        for (uint8_t i = 0u; i < TIME_SYNC_SAMPLE_COUNT; ++i)
                assert(time_sync_handle_start(tx_frame, &session));

        // 検証: 各sampleをACKし、5件目のACK直後にreportを送る。
        const Event expected[] = {
                EVENT_TIME_SYNC_START_ACK,
                EVENT_TIME_ACK,
                EVENT_TIME_SYNC_START_ACK,
                EVENT_TIME_ACK,
                EVENT_TIME_SYNC_START_ACK,
                EVENT_TIME_ACK,
                EVENT_TIME_SYNC_START_ACK,
                EVENT_TIME_ACK,
                EVENT_TIME_SYNC_START_ACK,
                EVENT_TIME_ACK,
                EVENT_TIME_SYNC_REPORT,
        };
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
        assert(receive_count == TIME_SYNC_SAMPLE_COUNT);
        assert(session.count == 0u);
}

static void handler_rejects_a_time_frame_with_the_wrong_command(void)
{
        // 準備
        reset_uart_state();
        invalid_time_command                       = true;
        TimeSyncSession session                    = {0};
        uint8_t         tx_frame[CORE_TX_BUF_SIZE] = {0};

        // 実行
        bool handled = time_sync_handle_start(tx_frame, &session);

        // 検証: start ACKの後、不正なtime frameを保存・ACKしない。
        assert(!handled);
        assert(event_count == 1u);
        assert(events[0] == EVENT_TIME_SYNC_START_ACK);
        assert(session.count == 0u);
}

int main(void)
{
        unix_time_is_decoded_from_little_endian_bytes();
        session_records_exactly_five_measurements();
        report_formats_all_five_measurements();
        report_calculates_round_trip_across_tick_wrap();
        time_sync_start_command_has_the_documented_bytes();
        handler_collects_five_command_prefixed_time_frames();
        handler_rejects_a_time_frame_with_the_wrong_command();
        return 0;
}
