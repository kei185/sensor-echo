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
RTC_HandleTypeDef  hrtc = {.Init = {.SynchPrediv = 255u}};

typedef enum
{
        EVENT_TIME_SYNC_START_ACK,
        EVENT_TIME_ACK,
        EVENT_TIME_SYNC_REPORT,
} Event;

#define EVENT_CAPACITY 3u

static Event           events[EVENT_CAPACITY];
static uint8_t         event_count;
static uint8_t         receive_count;
static uint32_t        fake_tick;
static bool            invalid_time_command;
static char            report_payload[CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE + 1u];
static size_t          report_length;
static RTC_DateTypeDef set_date;
static RTC_TimeTypeDef set_time;
static uint32_t        shift_add_one_second;
static uint32_t        shift_subtract_fraction;
static uint8_t         rtc_set_date_count;
static uint8_t         rtc_set_time_count;
static uint8_t         rtc_shift_count;

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
        memset(&set_date, 0, sizeof(set_date));
        memset(&set_time, 0, sizeof(set_time));
        shift_add_one_second    = 0u;
        shift_subtract_fraction = 0u;
        rtc_set_date_count      = 0u;
        rtc_set_time_count      = 0u;
        rtc_shift_count         = 0u;
}

uint32_t HAL_GetTick(void) { return fake_tick++; }

HAL_StatusTypeDef
HAL_RTC_SetDate(RTC_HandleTypeDef* handle, RTC_DateTypeDef* date, uint32_t format)
{
        assert(handle == &hrtc);
        assert(format == RTC_FORMAT_BIN);
        set_date = *date;
        ++rtc_set_date_count;
        return HAL_OK;
}

HAL_StatusTypeDef
HAL_RTC_SetTime(RTC_HandleTypeDef* handle, RTC_TimeTypeDef* time, uint32_t format)
{
        assert(handle == &hrtc);
        assert(format == RTC_FORMAT_BIN);
        set_time = *time;
        ++rtc_set_time_count;
        return HAL_OK;
}

HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(
        RTC_HandleTypeDef* handle, uint32_t add_one_second, uint32_t subtract_fraction)
{
        assert(handle == &hrtc);
        shift_add_one_second    = add_one_second;
        shift_subtract_fraction = subtract_fraction;
        ++rtc_shift_count;
        return HAL_OK;
}

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

static void handler_collects_and_reports_one_measurement(void)
{
        // 準備
        reset_uart_state();
        TimeSyncSession session                    = {0};
        uint8_t         tx_frame[CORE_TX_BUF_SIZE] = {0};

        // 実行
        assert(time_sync_handle_start(tx_frame, &session));
        assert(time_sync_send_report_if_ready(tx_frame, &session));

        // 検証: 1回の時刻交換をACKした後、その計測値をreportする。
        const Event expected[] = {
                EVENT_TIME_SYNC_START_ACK,
                EVENT_TIME_ACK,
                EVENT_TIME_SYNC_REPORT,
        };
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
        assert(receive_count == 1u);
        assert(!session.complete);
        assert(report_length == strlen(report_payload));
        assert(strstr(report_payload,
                      "#1 req_tick=1000 ms res_tick=1002 ms round_trip=2 ms "
                      "ack_time=1791160123000 ms send_time=1791160123001 ms") != NULL);
}

static void handler_reports_round_trip_across_tick_wrap(void)
{
        // 準備: 1件目のreqとresの間でHAL tickをwrapさせる。
        reset_uart_state();
        fake_tick                                  = UINT32_MAX - 1u;
        TimeSyncSession session                    = {0};
        uint8_t         tx_frame[CORE_TX_BUF_SIZE] = {0};

        // 実行
        assert(time_sync_handle_start(tx_frame, &session));
        assert(time_sync_send_report_if_ready(tx_frame, &session));

        // 検証: unsigned tick差分なら1回のwrapをまたいでも経過時間を得られる。
        assert(strstr(report_payload,
                      "#1 req_tick=4294967294 ms res_tick=0 ms round_trip=2 ms") != NULL);
}

static void rtc_uses_network_delay_and_elapsed_time(void)
{
        // 準備: 2024-02-29 12:34:56.789 UTCになる4時刻を作る。
        reset_uart_state();
        fake_tick               = 1020u;
        TimeSyncSession session = {
                .measurement =
                        {
                                .req_time  = 1000u,
                                .res_time  = 1010u,
                                .ack_time  = UINT64_C(1709210096772),
                                .send_time = UINT64_C(1709210096776),
                        },
                .complete = true,
        };

        // 実行
        assert(time_sync_set_rtc(&session));

        // 検証: network RTT 6 msの片道3 msと受信後10 msを加える。
        assert(rtc_set_date_count == 1u);
        assert(rtc_set_time_count == 1u);
        assert(set_date.Year == 24u);
        assert(set_date.Month == 2u);
        assert(set_date.Date == 29u);
        assert(set_date.WeekDay == 4u);
        assert(set_time.Hours == 12u);
        assert(set_time.Minutes == 34u);
        assert(set_time.Seconds == 56u);
        assert(rtc_shift_count == 1u);
        assert(shift_add_one_second == RTC_SHIFTADD1S_SET);
        assert(shift_subtract_fraction == 55u);
}

static void rtc_rejects_an_impossible_host_interval(void)
{
        // 準備: host処理時間がcontroller側の往復時間より長い不整合を作る。
        reset_uart_state();
        TimeSyncSession session = {
                .measurement =
                        {
                                .req_time  = 100u,
                                .res_time  = 102u,
                                .ack_time  = UINT64_C(1709210096000),
                                .send_time = UINT64_C(1709210096003),
                        },
                .complete = true,
        };

        // 実行と検証
        assert(!time_sync_set_rtc(&session));
        assert(rtc_set_date_count == 0u);
        assert(rtc_set_time_count == 0u);
        assert(rtc_shift_count == 0u);
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
        assert(!session.complete);
}

int main(void)
{
        time_sync_start_command_has_the_documented_bytes();
        handler_collects_and_reports_one_measurement();
        handler_reports_round_trip_across_tick_wrap();
        handler_rejects_a_time_frame_with_the_wrong_command();
        rtc_uses_network_delay_and_elapsed_time();
        rtc_rejects_an_impossible_host_interval();
        return 0;
}
