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
static uint64_t        frame_timestamps[EVENT_CAPACITY];
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
static bool            rtc_time_was_read;
static bool            rtc_read_fails;

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
        memset(frame_timestamps, 0, sizeof(frame_timestamps));
        event_count          = 0u;
        receive_count        = 0u;
        fake_tick            = 1000u;
        invalid_time_command = false;
        memset(report_payload, 0, sizeof(report_payload));
        report_length = 0u;
        set_date      = (RTC_DateTypeDef){.Month = 1u, .Date = 1u};
        set_time      = (RTC_TimeTypeDef){.SubSeconds = 255u, .SecondFraction = 255u};
        shift_add_one_second    = 0u;
        shift_subtract_fraction = 0u;
        rtc_set_date_count      = 0u;
        rtc_set_time_count      = 0u;
        rtc_shift_count         = 0u;
        rtc_time_was_read       = false;
        rtc_read_fails          = false;
}

uint32_t HAL_GetTick(void) { return fake_tick++; }

HAL_StatusTypeDef
HAL_RTC_GetTime(RTC_HandleTypeDef* handle, RTC_TimeTypeDef* time, uint32_t format)
{
        assert(handle == &hrtc);
        assert(format == RTC_FORMAT_BIN);
        assert(!rtc_time_was_read);
        *time             = set_time;
        rtc_time_was_read = true;
        return HAL_OK;
}

HAL_StatusTypeDef
HAL_RTC_GetDate(RTC_HandleTypeDef* handle, RTC_DateTypeDef* date, uint32_t format)
{
        assert(handle == &hrtc);
        assert(format == RTC_FORMAT_BIN);
        // 時刻の後に日付を読み、次のRTC読み取りへ進めることを確認する。
        assert(rtc_time_was_read);
        rtc_time_was_read = false;
        *date             = set_date;
        return rtc_read_fails ? HAL_ERROR : HAL_OK;
}

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

        // ヘッダの8バイトを読み、HAL tickではなくUnix時刻が届くか調べる。
        for (size_t i = 0u; i < sizeof(uint64_t); ++i)
                frame_timestamps[event_count] |= (uint64_t)data[6u + i] << (8u * i);

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
                      "#1 req_tick=1000 ms res_tick=1001 ms round_trip=1 ms "
                      "ack_time=1791160123000 ms send_time=1791160123001 ms") != NULL);
        for (size_t i = 0u; i < EVENT_CAPACITY; ++i)
                assert(frame_timestamps[i] == UINT64_C(946684800000));
}

static void handler_reports_round_trip_across_tick_wrap(void)
{
        // 準備: 1件目のreqとresの間でHAL tickをwrapさせる。
        reset_uart_state();
        fake_tick                                  = UINT32_MAX;
        TimeSyncSession session                    = {0};
        uint8_t         tx_frame[CORE_TX_BUF_SIZE] = {0};

        // 実行
        assert(time_sync_handle_start(tx_frame, &session));
        assert(time_sync_send_report_if_ready(tx_frame, &session));

        // 検証: unsigned tick差分なら1回のwrapをまたいでも経過時間を得られる。
        assert(strstr(report_payload,
                      "#1 req_tick=4294967295 ms res_tick=0 ms round_trip=1 ms") != NULL);
}

static void rtc_returns_unix_milliseconds(void)
{
        // 準備: RTCが表せる2000年の先頭を使う。
        reset_uart_state();
        assert(time_sync_get_unix_time_ms() == UINT64_C(946684800000));

        // 閏日のUTC日時と、subsecondの192/256秒を変換する。
        set_date = (RTC_DateTypeDef){.Year = 24u, .Month = 2u, .Date = 29u};
        set_time = (RTC_TimeTypeDef){
                .Hours          = 12u,
                .Minutes        = 34u,
                .Seconds        = 56u,
                .SubSeconds     = 63u,
                .SecondFraction = 255u,
        };
        assert(time_sync_get_unix_time_ms() == UINT64_C(1709210096750));

        // 閏日の次の月にも、1日分の補正が反映されることを確認する。
        set_date.Month = 3u;
        set_date.Date  = 1u;
        set_time       = (RTC_TimeTypeDef){.SubSeconds = 255u, .SecondFraction = 255u};
        assert(time_sync_get_unix_time_ms() == UINT64_C(1709251200000));

        // 2038年のsigned 32bit秒の範囲を超えてもUnixミリ秒を保つ。
        set_date = (RTC_DateTypeDef){.Year = 38u, .Month = 1u, .Date = 19u};
        set_time = (RTC_TimeTypeDef){
                .Hours          = 3u,
                .Minutes        = 14u,
                .Seconds        = 8u,
                .SubSeconds     = 255u,
                .SecondFraction = 255u,
        };
        assert(time_sync_get_unix_time_ms() == UINT64_C(2147483648000));

        // RTC設定の上限である2099年末も、UTC日時のまま扱う。
        set_date = (RTC_DateTypeDef){.Year = 99u, .Month = 12u, .Date = 31u};
        set_time = (RTC_TimeTypeDef){
                .Hours          = 23u,
                .Minutes        = 59u,
                .Seconds        = 59u,
                .SubSeconds     = 0u,
                .SecondFraction = 255u,
        };
        assert(time_sync_get_unix_time_ms() == UINT64_C(4102444799996));
}

static void rtc_handles_subseconds_after_a_shift(void)
{
        // 準備: 1秒進めてから1/4秒を差し引いた直後のRTC値を作る。
        reset_uart_state();
        set_time.Seconds    = 1u;
        set_time.SubSeconds = 319u;

        // 検証: カウンタが255を超えていても、00:00:00.750と解釈する。
        assert(time_sync_get_unix_time_ms() == UINT64_C(946684800750));
}

static void rtc_returns_zero_on_read_failure(void)
{
        reset_uart_state();
        rtc_read_fails = true;
        assert(time_sync_get_unix_time_ms() == 0u);
        assert(!rtc_time_was_read);
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
        rtc_returns_unix_milliseconds();
        rtc_handles_subseconds_after_a_shift();
        rtc_returns_zero_on_read_failure();
        return 0;
}
