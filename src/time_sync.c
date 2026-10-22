#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "time_sync.h"
#include "startup.h"
#include "lidar/core.h"
#include "tx/frame.h"
#include "tx/header.h"

#define TIME_SYNC_UART_TIMEOUT_MS  100u
#define TIME_SYNC_HOST_TIME_SIZE   8u
#define TIME_SYNC_ACK_TIME_OFFSET  HOST_COMMAND_SIZE
#define TIME_SYNC_SEND_TIME_OFFSET (TIME_SYNC_ACK_TIME_OFFSET + TIME_SYNC_HOST_TIME_SIZE)
#define TIME_SYNC_TIME_FRAME_SIZE  (TIME_SYNC_SEND_TIME_OFFSET + TIME_SYNC_HOST_TIME_SIZE)
#define UINT64_DECIMAL_TEXT_SIZE   21u
#define UNIX_MILLISECONDS_PER_SECOND UINT64_C(1000)
#define RTC_MIN_UNIX_TIME_MS         UINT64_C(946684800000)
#define RTC_MAX_UNIX_TIME_MS         UINT64_C(4102444800000)

_Static_assert(sizeof(time_t) >= sizeof(int64_t), "64-bit time_t is required");

typedef struct
{
        RTC_DateTypeDef date;
        RTC_TimeTypeDef time;
        uint16_t        millisecond;
} RtcCalendar;

// static uint64_t synchronized_unix_time_ms;
// static uint32_t synchronized_tick;

uint64_t time_sync_get_unix_time_ms(void)
{
        /*
        if (synchronized_unix_time_ms != 0u) {
                uint32_t tick = HAL_GetTick();
                synchronized_unix_time_ms += (uint32_t)(tick - synchronized_tick);
                synchronized_tick = tick;
                return synchronized_unix_time_ms;
        }
        // 同期後はLSIで動くRTCを使わず、HAL tickの差分で時刻を進める。
        // unsignedの差分により、継続して読み出す間はtickの周回も扱える。
        */
        // RTCのずれを観測するため、HAL tickで時刻を進める修正は一旦無効化。

        RTC_TimeTypeDef time;
        RTC_DateTypeDef date;

        // 時刻の後に日付を読み、RTCのshadow registerを解放する。
        if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
            HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
                return 0u;

        static const uint16_t days_before_month[] = {
                0u,
                0u,
                31u,
                59u,
                90u,
                120u,
                151u,
                181u,
                212u,
                243u,
                273u,
                304u,
                334u,
        };
        // RTCのYearは2000年からの年数。時刻設定と同じ2000～2099年を扱う。
        uint32_t year = date.Year;
        // 1970～1999年の10957日に、経過年・閏日・年内の経過日を足す。
        uint32_t days = 10957u + year * 365u + (year + 3u) / 4u +
                        days_before_month[date.Month] + date.Date - 1u;
        if (date.Month > 2u && year % 4u == 0u)
                ++days;

        uint64_t unix_seconds =
                (((uint64_t)days * 24u + time.Hours) * 60u + time.Minutes) * 60u +
                time.Seconds;
        // RTCの時刻補正直後は、subsecondのカウンタが1秒分を超えることがある。
        if (time.SubSeconds > time.SecondFraction) {
                --unix_seconds;
                time.SubSeconds -= time.SecondFraction + 1u;
        }
        uint32_t milliseconds = (time.SecondFraction - time.SubSeconds) * 1000u /
                                (time.SecondFraction + 1u);
        return unix_seconds * UNIX_MILLISECONDS_PER_SECOND + milliseconds;
}

static bool
send_host_frame(uint8_t* tx_frame, FrameType frame_type, size_t payload_length)
{
        size_t frame_length = tx_frame_write_header(
                tx_frame,
                CORE_TX_BUF_SIZE,
                (uint16_t)payload_length,
                frame_type,
                time_sync_get_unix_time_ms());
        if (frame_length == 0u)
                return false;

        HAL_StatusTypeDef transmit_status = HAL_UART_Transmit(
                &huart2,
                tx_frame,
                (uint16_t)frame_length,
                TIME_SYNC_UART_TIMEOUT_MS);
        const char* operation = frame_type == FRAME_TYPE_TIME_SYNC_START_ACK
                                        ? "sync ACK TX"
                                : frame_type == FRAME_TYPE_TIME_ACK ? "time ACK TX"
                                                                    : "sync report TX";
        return startup_uart_ok(&huart2, transmit_status, operation);
}

static void uint64_to_decimal(uint64_t value, char to[UINT64_DECIMAL_TEXT_SIZE])
{
        char   reversed[UINT64_DECIMAL_TEXT_SIZE - 1u];
        size_t length = 0u;

        do {
                reversed[length++] = (char)('0' + value % 10u);
                value /= 10u;
        } while (value != 0u);

        for (size_t i = 0u; i < length; ++i)
                to[i] = reversed[length - i - 1u];
        to[length] = '\0';
}

static bool unix_time_to_rtc(uint64_t unix_time_ms, RtcCalendar* calendar)
{
        if (calendar == NULL || unix_time_ms < RTC_MIN_UNIX_TIME_MS ||
            unix_time_ms >= RTC_MAX_UNIX_TIME_MS)
                return false;

        time_t     unix_seconds = (time_t)(unix_time_ms / UNIX_MILLISECONDS_PER_SECOND);
        struct tm* utc          = gmtime(&unix_seconds);
        if (utc == NULL)
                return false;

        *calendar = (RtcCalendar){
                .date =
                        {
                                // struct tm uses Sunday=0; HAL uses Sunday=7.
                                .WeekDay =
                                        (uint8_t)(utc->tm_wday == 0 ? 7 : utc->tm_wday),
                                .Month = (uint8_t)(utc->tm_mon + 1),
                                .Date  = (uint8_t)utc->tm_mday,
                                .Year  = (uint8_t)(utc->tm_year - 100),
                        },
                .time =
                        {
                                .Hours          = (uint8_t)utc->tm_hour,
                                .Minutes        = (uint8_t)utc->tm_min,
                                .Seconds        = (uint8_t)utc->tm_sec,
                                .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
                                .StoreOperation = RTC_STOREOPERATION_RESET,
                        },
                .millisecond = (uint16_t)(unix_time_ms % UNIX_MILLISECONDS_PER_SECOND),
        };
        return true;
}

static bool estimate_current_unix_time(
        const TimeSyncSession* session, uint32_t current_tick, uint64_t* unix_time_ms)
{
        if (session == NULL || unix_time_ms == NULL || !session->complete)
                return false;

        const TimeSyncMeasurement* sample = &session->measurement;
        uint32_t controller_round_trip    = sample->res_time - sample->req_time;
        // req_time/res_timeはログのreq_tick/res_tick。符号なし32bitの差で周回も扱う。

        if (sample->send_time < sample->ack_time)
                return false;

        uint64_t host_processing = sample->send_time - sample->ack_time;
        if (host_processing > controller_round_trip)
                return false;

        uint64_t network_round_trip = controller_round_trip - host_processing;
        // ホストのUnix時刻同士の差を引き、ACKからTime受信までの通信時間を求める。

        uint64_t one_way_delay         = network_round_trip / 2u;
        uint64_t elapsed_after_receive = current_tick - sample->res_time;
        // 行き帰りの遅延は同じと仮定する。受信後の経過時間もHAL tickの差で求める。
        if (sample->send_time > UINT64_MAX - one_way_delay ||
            sample->send_time + one_way_delay > UINT64_MAX - elapsed_after_receive)
                return false;

        *unix_time_ms = sample->send_time + one_way_delay + elapsed_after_receive;
        // current_tickはRTC設定の計算前に読む。RTCへの書き込み時間は含めない。
        return true;
}

bool time_sync_set_rtc(const TimeSyncSession* session)
{
        uint64_t unix_time_ms;
        uint32_t tick = HAL_GetTick();
        if (!estimate_current_unix_time(session, tick, &unix_time_ms))
                return false;

        RtcCalendar calendar;
        if (!unix_time_to_rtc(unix_time_ms, &calendar))
                return false;

        if (HAL_RTC_SetDate(&hrtc, &calendar.date, RTC_FORMAT_BIN) != HAL_OK ||
            HAL_RTC_SetTime(&hrtc, &calendar.time, RTC_FORMAT_BIN) != HAL_OK)
                return false;

        uint32_t subsecond_count = (uint32_t)((uint64_t)calendar.millisecond *
                                              (hrtc.Init.SynchPrediv + 1u) / 1000u);
        // 現在はSynchPrediv=255なので256段階。ミリ秒からカウントへの変換で端数を切り捨てる。
        uint32_t shift_subfs     = hrtc.Init.SynchPrediv + 1u - subsecond_count;
        if (subsecond_count != 0u &&
            HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_SET, shift_subfs) != HAL_OK)
                return false;
        // ADD1Sで1秒足し、SUBFSカウント分を引く。0カウントなら補正しない。
        // 例: 500msならSUBFS=128で、1 - 128/256 = 0.5秒進める。
        // 初回の秒未満の時刻合わせであり、RTCの進む速さを補正する処理ではない。

        // synchronized_unix_time_ms = unix_time_ms;
        // synchronized_tick         = tick;
        // RTCの設定中に経過した時間も、最初に取得したtickを基準に含める。
        return true;
}

static size_t
time_sync_format_report(char* to, size_t capacity, const TimeSyncSession* session)
{
        if (to == NULL || capacity == 0u || session == NULL || !session->complete)
                return 0u;

        const TimeSyncMeasurement* sample     = &session->measurement;
        uint32_t                   round_trip = sample->res_time - sample->req_time;
        char                       ack_time[UINT64_DECIMAL_TEXT_SIZE];
        char                       send_time[UINT64_DECIMAL_TEXT_SIZE];
        uint64_to_decimal(sample->ack_time, ack_time);
        uint64_to_decimal(sample->send_time, send_time);

        int written = snprintf(
                to,
                capacity,
                "TIME SYNC: samples=1 | #1 req_tick=%" PRIu32 " ms res_tick=%" PRIu32
                " ms round_trip=%" PRIu32 " ms ack_time=%s ms send_time=%s ms\r\n",
                sample->req_time,
                sample->res_time,
                round_trip,
                ack_time,
                send_time);
        if (written < 0 || (size_t)written >= capacity)
                return 0u;
        return (size_t)written;
}

bool time_sync_send_report_if_ready(uint8_t* tx_frame, TimeSyncSession* session)
{

        char*  payload        = (char*)(tx_frame + TX_FRAME_HEADER_SIZE);
        size_t payload_length = time_sync_format_report(
                payload,
                CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE,
                session);
        if (payload_length == 0u ||
            !send_host_frame(tx_frame, FRAME_TYPE_TIME_SYNC_REPORT, payload_length))
                return false;

        *session = (TimeSyncSession){0};
        return true;
}

bool time_sync_handle_start(uint8_t* tx_frame, TimeSyncSession* session)
{
        if (session->complete)
                return false;

        // prepare receive buffer
        uint8_t time_sync_start_frame[HOST_COMMAND_SIZE];
        // receive time sync start
        HAL_StatusTypeDef time_sync_start_status = HAL_UART_Receive(
                &huart2,
                time_sync_start_frame,
                HOST_COMMAND_SIZE,
                HAL_MAX_DELAY);
        if (!startup_uart_ok(&huart2, time_sync_start_status, "sync start RX"))
                return false;
        // verify command if its time sync start
        if (memcmp(time_sync_start_frame,
                   HOST_COMMANDS[HOST_COMMAND_TIME_SYNC_START],
                   HOST_COMMAND_SIZE) != 0)
                return false;

        // prepare request time
        uint32_t req_time = HAL_GetTick();

        // prepare start ack message
        memcpy(tx_frame + TX_FRAME_HEADER_SIZE,
               FRAME_MESSAGE_TIME_SYNC_START_ACK,
               sizeof(FRAME_MESSAGE_TIME_SYNC_START_ACK) - 1u);

        // send time sync start ack
        if (!send_host_frame(
                    tx_frame,
                    FRAME_TYPE_TIME_SYNC_START_ACK,
                    sizeof(FRAME_MESSAGE_TIME_SYNC_START_ACK) - 1u))
                return false;

        // receive time
        uint8_t           time_frame[TIME_SYNC_TIME_FRAME_SIZE];
        HAL_StatusTypeDef receive_status = HAL_UART_Receive(
                &huart2,
                time_frame,
                TIME_SYNC_TIME_FRAME_SIZE,
                HAL_MAX_DELAY);
        if (!startup_uart_ok(&huart2, receive_status, "time RX"))
                return false;

        // prepare response time
        uint32_t res_time = HAL_GetTick();

        //  verify command if its time frame
        if (memcmp(time_frame, HOST_COMMANDS[HOST_COMMAND_TIME], HOST_COMMAND_SIZE) != 0)
                return false;

        // decode  host times
        uint64_t ack_time  = 0u;
        uint64_t send_time = 0u;
        for (size_t i = 0u; i < TIME_SYNC_HOST_TIME_SIZE; ++i) {
                ack_time |= (uint64_t)time_frame[TIME_SYNC_ACK_TIME_OFFSET + i]
                            << (i * 8u);
                send_time |= (uint64_t)time_frame[TIME_SYNC_SEND_TIME_OFFSET + i]
                             << (i * 8u);
        }
        // 18バイトのTimeから、位置2と10の64bit Unixミリ秒をlittle-endianで読む。

        session->measurement = (TimeSyncMeasurement){
                .req_time  = req_time,
                .res_time  = res_time,
                .ack_time  = ack_time,
                .send_time = send_time,
        };
        session->complete = true;

        // send time ack
        memcpy(tx_frame + TX_FRAME_HEADER_SIZE,
               FRAME_MESSAGE_TIME_ACK,
               sizeof(FRAME_MESSAGE_TIME_ACK) - 1u);
        if (!send_host_frame(
                    tx_frame,
                    FRAME_TYPE_TIME_ACK,
                    sizeof(FRAME_MESSAGE_TIME_ACK) - 1u))
                return false;

        return true;
}
