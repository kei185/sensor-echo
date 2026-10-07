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

static size_t
time_sync_format_report(char* to, size_t capacity, const TimeSyncSession* session)
{
        if (to == NULL || capacity == 0u || session == NULL || session->count == 0u ||
            session->count > TIME_SYNC_SAMPLE_COUNT)
                return 0u;

        int written =
                snprintf(to, capacity, "TIME SYNC: samples=%u", (unsigned)session->count);
        if (written < 0 || (size_t)written >= capacity)
                return 0u;

        size_t used = (size_t)written;
        for (uint8_t i = 0u; i < session->count; ++i) {
                const TimeSyncMeasurement* sample = &session->samples[i];
                const uint32_t round_trip         = sample->res_time - sample->req_time;
                char           ack_time[UINT64_DECIMAL_TEXT_SIZE];
                char           send_time[UINT64_DECIMAL_TEXT_SIZE];
                uint64_to_decimal(sample->ack_time, ack_time);
                uint64_to_decimal(sample->send_time, send_time);

                written = snprintf(
                        to + used,
                        capacity - used,
                        " | #%u req_tick=%" PRIu32 " ms res_tick=%" PRIu32
                        " ms round_trip=%" PRIu32
                        " ms ack_time=%s ms send_time=%s ms\r\n",
                        (unsigned)(i + 1u),
                        sample->req_time,
                        sample->res_time,
                        round_trip,
                        ack_time,
                        send_time);
                if (written < 0 || (size_t)written >= capacity - used)
                        return 0u;

                used += (size_t)written;
        }

        return used;
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

        session->samples[session->count++] = (TimeSyncMeasurement){
                .req_time  = req_time,
                .res_time  = res_time,
                .ack_time  = ack_time,
                .send_time = send_time,
        };

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

bool inti_time()
{
        HAL_RTC_SetDate(
                &hrtc,
                &(RTC_DateTypeDef){
                        .WeekDay = RTC_WEEKDAY_MONDAY,
                        .Month   = RTC_MONTH_JANUARY,
                        .Date    = 1u,
                        .Year    = 70u,
                },
                RTC_FORMAT_BIN);

        HAL_RTC_SetTime(
                &hrtc,
                &(RTC_TimeTypeDef){
                        .Hours   = 0u,
                        .Minutes = 0u,
                        .Seconds = 0u,
                },
                RTC_FORMAT_BIN);
}
bool set_time(TimeSyncSession tss)
{
        HAL_RTC_SetDate(
                &hrtc,
                &(RTC_DateTypeDef){
                        .WeekDay = RTC_WEEKDAY_MONDAY,
                        .Month   = RTC_MONTH_JANUARY,
                        .Date    = (tss.samples[0].send_time / 86400000) % 31 + 1,
                        .Year    = (tss.samples[0].send_time / 31536000000) % 100,
                },
                RTC_FORMAT_BIN);

        HAL_RTC_SetTime(
                &hrtc,
                &(RTC_TimeTypeDef){

                        .Hours   = (tss.samples[0].send_time / 3600000) % 24,
                        .Minutes = (tss.samples[0].send_time / 60000) % 60,
                        .Seconds = (tss.samples[0].send_time / 1000) % 60,
                },
                RTC_FORMAT_BIN);
}
