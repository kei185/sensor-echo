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

#define TIME_SYNC_UART_TIMEOUT_MS 100u

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

static bool send_host_message(
        uint8_t*    tx_frame,
        FrameType   frame_type,
        const char* message,
        uint16_t    message_length)
{
        memcpy(tx_frame + TX_FRAME_HEADER_SIZE, message, message_length);
        return send_host_frame(tx_frame, frame_type, message_length);
}

bool time_sync_decode_unix_time(
        const uint8_t* encoded, size_t encoded_size, uint64_t* unixtime)
{
        if (encoded == NULL || encoded_size != TIME_SYNC_UNIX_TIME_SIZE ||
            unixtime == NULL)
                return false;

        uint64_t decoded = 0u;
        for (size_t i = 0u; i < TIME_SYNC_UNIX_TIME_SIZE; ++i)
                decoded |= (uint64_t)encoded[i] << (i * 8u);

        *unixtime = decoded;
        return true;
}

bool time_sync_record(
        TimeSyncSession* session, uint32_t req_time, uint32_t res_time, uint64_t unixtime)
{
        if (session == NULL || session->count >= TIME_SYNC_SAMPLE_COUNT)
                return false;

        session->samples[session->count++] = (TimeSyncMeasurement){
                .req_time = req_time,
                .res_time = res_time,
                .unixtime = unixtime,
        };
        return true;
}

size_t time_sync_format_report(char* to, size_t capacity, const TimeSyncSession* session)
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

                written = snprintf(
                        to + used,
                        capacity - used,
                        " | #%u req_tick=%" PRIu32 " ms res_tick=%" PRIu32
                        " ms round_trip=%" PRIu32 " ms unix_time=%" PRIu64 " ms",
                        (unsigned)(i + 1u),
                        sample->req_time,
                        sample->res_time,
                        round_trip,
                        sample->unixtime);
                if (written < 0 || (size_t)written >= capacity - used)
                        return 0u;

                used += (size_t)written;
        }

        return used;
}

bool time_sync_handle_start(uint8_t* tx_frame, TimeSyncSession* session)
{
        if (tx_frame == NULL || session == NULL)
                return false;

        uint32_t req_time = HAL_GetTick();
        if (!send_host_message(
                    tx_frame,
                    FRAME_TYPE_TIME_SYNC_START_ACK,
                    FRAME_MESSAGE_TIME_SYNC_START_ACK,
                    sizeof(FRAME_MESSAGE_TIME_SYNC_START_ACK) - 1u))
                return false;

        uint8_t           time_frame[TIME_SYNC_TIME_FRAME_SIZE];
        HAL_StatusTypeDef receive_status = HAL_UART_Receive(
                &huart2,
                time_frame,
                TIME_SYNC_TIME_FRAME_SIZE,
                HAL_MAX_DELAY);
        if (receive_status != HAL_OK)
                return false;

        uint32_t res_time = HAL_GetTick();
        if (memcmp(time_frame, HOST_COMMANDS[HOST_COMMAND_TIME], HOST_COMMAND_SIZE) != 0)
                return false;

        uint64_t unixtime;
        if (!time_sync_decode_unix_time(
                    time_frame + HOST_COMMAND_SIZE,
                    TIME_SYNC_UNIX_TIME_SIZE,
                    &unixtime) ||
            !time_sync_record(session, req_time, res_time, unixtime))
                return false;

        if (!send_host_message(
                    tx_frame,
                    FRAME_TYPE_TIME_ACK,
                    FRAME_MESSAGE_TIME_ACK,
                    sizeof(FRAME_MESSAGE_TIME_ACK) - 1u))
                return false;

        if (session->count < TIME_SYNC_SAMPLE_COUNT)
                return true;

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
