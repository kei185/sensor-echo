#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
        uint32_t req_time;  // HAL tick before the controller sends the start ACK.
        uint32_t res_time;  // HAL tick immediately after the Time frame arrives.
        uint64_t ack_time;  // Host Unix time when the start ACK arrives.
        uint64_t send_time; // Host Unix time immediately before sending Time.
} TimeSyncMeasurement;

typedef struct
{
        TimeSyncMeasurement measurement;
        bool                complete;
} TimeSyncSession;

/** Set the RTC from a complete time-sync exchange. */
bool time_sync_set_rtc(const TimeSyncSession* session);

/** Send the measurement report and clear the completed session. */
bool time_sync_send_report_if_ready(uint8_t* tx_frame, TimeSyncSession* session);

/**
 * Complete one time-sync measurement.
 *
 * The caller owns the session and a TX frame buffer of `CORE_TX_BUF_SIZE`
 * bytes. The function receives `0xAA 0xA4`, sends the start ACK, then receives
 * and validates `0xAA 0xA5` plus the host ACK and send times. It records one
 * sample and sends the time ACK.
 */
bool time_sync_handle_start(uint8_t* tx_frame, TimeSyncSession* session);

#endif /* TIME_SYNC_H */
