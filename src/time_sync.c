#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "time_sync.h"
#include "lidar/core.h"
#include "tx/frame.h"
#include "tx/header.h"

#define TIME_SYNC_UART_TIMEOUT_MS  100u
#define TIME_SYNC_HOST_TIME_SIZE   8u
#define TIME_SYNC_ACK_TIME_OFFSET  HOST_COMMAND_SIZE
#define TIME_SYNC_SEND_TIME_OFFSET (TIME_SYNC_ACK_TIME_OFFSET + TIME_SYNC_HOST_TIME_SIZE)
#define TIME_SYNC_TIME_FRAME_SIZE  (TIME_SYNC_SEND_TIME_OFFSET + TIME_SYNC_HOST_TIME_SIZE)
#define UINT64_DECIMAL_TEXT_SIZE   21u
#define UNIX_MILLISECONDS_PER_DAY  UINT64_C(86400000)
#define UNIX_MILLISECONDS_PER_SECOND UINT64_C(1000)
#define UNIX_DAYS_TO_2000            UINT64_C(10957)
#define RTC_MIN_UNIX_TIME_MS         UINT64_C(946684800000)
#define RTC_MAX_UNIX_TIME_MS         UINT64_C(4102444800000)

typedef struct
{
        RTC_DateTypeDef date;
        RTC_TimeTypeDef time;
        uint16_t        millisecond;
} RtcCalendar;

static bool
send_host_frame(uint8_t* tx_frame, FrameType frame_type, size_t payload_length)
{
        size_t frame_length = tx_frame_write_header(
                tx_frame,
                CORE_TX_BUF_SIZE,
                (uint16_t)payload_length,
                frame_type,
                HAL_GetTick());
        if (frame_length == 0u)
                return false;

        HAL_StatusTypeDef transmit_status = HAL_UART_Transmit(
                &huart2,
                tx_frame,
                (uint16_t)frame_length,
                TIME_SYNC_UART_TIMEOUT_MS);
        return transmit_status == HAL_OK;
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

static bool is_leap_year(uint16_t year)
{
        return year % 4u == 0u && (year % 100u != 0u || year % 400u == 0u);
}

static uint8_t days_in_month(uint16_t year, uint8_t month)
{
        static const uint8_t DAYS[] = {
                31u,
                28u,
                31u,
                30u,
                31u,
                30u,
                31u,
                31u,
                30u,
                31u,
                30u,
                31u,
        };

        if (month == 2u && is_leap_year(year))
                return 29u;
        return DAYS[month - 1u];
}

static bool unix_time_to_rtc(uint64_t unix_time_ms, RtcCalendar* calendar)
{
        if (calendar == NULL || unix_time_ms < RTC_MIN_UNIX_TIME_MS ||
            unix_time_ms >= RTC_MAX_UNIX_TIME_MS)
                return false;

        uint64_t days_since_epoch    = unix_time_ms / UNIX_MILLISECONDS_PER_DAY;
        uint64_t milliseconds_of_day = unix_time_ms % UNIX_MILLISECONDS_PER_DAY;
        uint64_t days                = days_since_epoch - UNIX_DAYS_TO_2000;
        uint16_t year                = 2000u;

        while (days >= (is_leap_year(year) ? 366u : 365u)) {
                days -= is_leap_year(year) ? 366u : 365u;
                ++year;
        }

        uint8_t month = 1u;
        while (days >= days_in_month(year, month)) {
                days -= days_in_month(year, month);
                ++month;
        }

        uint32_t seconds_of_day =
                (uint32_t)(milliseconds_of_day / UNIX_MILLISECONDS_PER_SECOND);
        *calendar = (RtcCalendar){
                .date =
                        {
                                // 1970-01-01 was Thursday; HAL numbers Monday as 1.
                                .WeekDay = (uint8_t)((days_since_epoch + 3u) % 7u + 1u),
                                .Month   = month,
                                .Date    = (uint8_t)(days + 1u),
                                .Year    = (uint8_t)(year - 2000u),
                        },
                .time =
                        {
                                .Hours          = (uint8_t)(seconds_of_day / 3600u),
                                .Minutes        = (uint8_t)((seconds_of_day / 60u) % 60u),
                                .Seconds        = (uint8_t)(seconds_of_day % 60u),
                                .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
                                .StoreOperation = RTC_STOREOPERATION_RESET,
                        },
                .millisecond = (uint16_t)(milliseconds_of_day % 1000u),
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

        if (sample->send_time < sample->ack_time)
                return false;

        uint64_t host_processing = sample->send_time - sample->ack_time;
        if (host_processing > controller_round_trip)
                return false;

        // Remove host work between t2 and t3; the remainder is UART travel time.
        uint64_t network_round_trip = controller_round_trip - host_processing;
        // Assume equal travel in both directions, then advance from t4 to now.
        uint64_t one_way_delay         = network_round_trip / 2u;
        uint64_t elapsed_after_receive = current_tick - sample->res_time;
        if (sample->send_time > UINT64_MAX - one_way_delay ||
            sample->send_time + one_way_delay > UINT64_MAX - elapsed_after_receive)
                return false;

        *unix_time_ms = sample->send_time + one_way_delay + elapsed_after_receive;
        return true;
}

bool time_sync_set_rtc(const TimeSyncSession* session)
{
        uint64_t unix_time_ms;
        if (!estimate_current_unix_time(session, HAL_GetTick(), &unix_time_ms))
                return false;

        RtcCalendar calendar;
        if (!unix_time_to_rtc(unix_time_ms, &calendar))
                return false;

        if (HAL_RTC_SetDate(&hrtc, &calendar.date, RTC_FORMAT_BIN) != HAL_OK ||
            HAL_RTC_SetTime(&hrtc, &calendar.time, RTC_FORMAT_BIN) != HAL_OK)
                return false;

        uint32_t subsecond_count = (uint32_t)((uint64_t)calendar.millisecond *
                                              (hrtc.Init.SynchPrediv + 1u) / 1000u);
        if (subsecond_count == 0u)
                return true;

        // RTC advances by 1 - SUBFS/(PREDIV_S + 1), matching the millisecond part.
        uint32_t shift_subfs = hrtc.Init.SynchPrediv + 1u - subsecond_count;
        return HAL_RTCEx_SetSynchroShift(&hrtc, RTC_SHIFTADD1S_SET, shift_subfs) ==
               HAL_OK;
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
        if (time_sync_start_status != HAL_OK)
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
        if (receive_status != HAL_OK)
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
