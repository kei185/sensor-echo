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
static char     report_payload[CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE + 1u];
static size_t   report_length;

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
        memset(report_payload, 0, sizeof(report_payload));
        report_length = 0u;
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
                report_length = length - TX_FRAME_HEADER_SIZE;
                assert(report_length < sizeof(report_payload));
                memcpy(report_payload, data + TX_FRAME_HEADER_SIZE, report_length);
                report_payload[report_length] = '\0';
                events[event_count++]         = EVENT_TIME_SYNC_REPORT;
        }
        return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(
        UART_HandleTypeDef* huart, uint8_t* data, uint16_t length, uint32_t timeout)
{
        assert(huart == &huart2);
        assert(timeout == HAL_MAX_DELAY);

        if (length == HOST_COMMAND_SIZE) {
                memcpy(data,
                       HOST_COMMANDS[HOST_COMMAND_TIME_SYNC_START],
                       HOST_COMMAND_SIZE);
                return HAL_OK;
        }

        assert(length == HOST_COMMAND_SIZE + sizeof(uint64_t) * 2u);

        memcpy(data, HOST_COMMANDS[HOST_COMMAND_TIME], HOST_COMMAND_SIZE);
        if (invalid_time_command)
                data[1] ^= 0x01u;

        uint64_t ack_time  = UINT64_C(1791160123000) + (uint64_t)receive_count * 10u;
        uint64_t send_time = ack_time + 1u;
        for (uint8_t i = 0u; i < sizeof(uint64_t); ++i) {
                data[HOST_COMMAND_SIZE + i] = (uint8_t)(ack_time >> (i * 8u));
                data[HOST_COMMAND_SIZE + sizeof(uint64_t) + i] =
                        (uint8_t)(send_time >> (i * 8u));
        }

        ++receive_count;
        return HAL_OK;
}

static void time_sync_start_command_has_the_documented_bytes(void)
{
        // 検証
        assert(HOST_COMMANDS[HOST_COMMAND_TIME_SYNC_START][0] == 0xAAu);
        assert(HOST_COMMANDS[HOST_COMMAND_TIME_SYNC_START][1] == 0xA4u);
        assert(HOST_COMMANDS[HOST_COMMAND_TIME][0] == 0xAAu);
        assert(HOST_COMMANDS[HOST_COMMAND_TIME][1] == 0xA5u);
}

static void handler_collects_and_reports_five_measurements(void)
{
        // 準備
        reset_uart_state();
        TimeSyncSession session                    = {0};
        uint8_t         tx_frame[CORE_TX_BUF_SIZE] = {0};

        // 実行
        for (uint8_t i = 0u; i < TIME_SYNC_SAMPLE_COUNT; ++i)
                assert(time_sync_handle_start(tx_frame, &session));
        assert(time_sync_send_report_if_ready(tx_frame, &session));

        // 検証: 各sampleをACKした後、5件をまとめてreportする。
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
        assert(report_length == strlen(report_payload));
        assert(strstr(report_payload,
                      "#1 req_tick=1000 ms res_tick=1002 ms round_trip=2 ms "
                      "ack_time=1791160123000 ms send_time=1791160123001 ms") != NULL);
        assert(strstr(report_payload,
                      "#5 req_tick=1016 ms res_tick=1018 ms round_trip=2 ms "
                      "ack_time=1791160123040 ms send_time=1791160123041 ms") != NULL);
}

static void handler_reports_round_trip_across_tick_wrap(void)
{
        // 準備: 1件目のreqとresの間でHAL tickをwrapさせる。
        reset_uart_state();
        fake_tick                                  = UINT32_MAX - 1u;
        TimeSyncSession session                    = {0};
        uint8_t         tx_frame[CORE_TX_BUF_SIZE] = {0};

        // 実行
        for (uint8_t i = 0u; i < TIME_SYNC_SAMPLE_COUNT; ++i)
                assert(time_sync_handle_start(tx_frame, &session));
        assert(time_sync_send_report_if_ready(tx_frame, &session));

        // 検証: unsigned tick差分なら1回のwrapをまたいでも経過時間を得られる。
        assert(strstr(report_payload,
                      "#1 req_tick=4294967294 ms res_tick=0 ms round_trip=2 ms") != NULL);
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
        time_sync_start_command_has_the_documented_bytes();
        handler_collects_and_reports_five_measurements();
        handler_reports_round_trip_across_tick_wrap();
        handler_rejects_a_time_frame_with_the_wrong_command();
        return 0;
}
