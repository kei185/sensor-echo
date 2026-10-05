#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tx/frame.h"

#define TIME_SYNC_SAMPLE_COUNT    5u
#define TIME_SYNC_UNIX_TIME_SIZE  8u
#define TIME_SYNC_TIME_FRAME_SIZE (HOST_COMMAND_SIZE + TIME_SYNC_UNIX_TIME_SIZE)

typedef struct
{
        uint32_t req_time; // HAL tick before the controller sends the start ACK.
        uint32_t res_time; // HAL tick immediately after the Unix time arrives.
        uint64_t unixtime; // Unix time in milliseconds, supplied by the host.
} TimeSyncMeasurement;

typedef struct
{
        TimeSyncMeasurement samples[TIME_SYNC_SAMPLE_COUNT];
        uint8_t             count;
} TimeSyncSession;

/** Decode one little-endian 64-bit Unix time received from the host. */
bool time_sync_decode_unix_time(
        const uint8_t* encoded, size_t encoded_size, uint64_t* unixtime);

/** Add one completed measurement. Returns false when the session is full. */
bool time_sync_record(
        TimeSyncSession* session,
        uint32_t         req_time,
        uint32_t         res_time,
        uint64_t         unixtime);

/** Format all recorded measurements as one host-frame ASCII payload. */
size_t time_sync_format_report(char* to, size_t capacity, const TimeSyncSession* session);

/**
 * Complete one time-sync exchange after the caller receives `0xAA 0xA4`.
 *
 * The caller owns the session and a TX frame buffer of `CORE_TX_BUF_SIZE`
 * bytes. The function sends the start ACK, receives and validates `0xAA 0xA5`
 * plus the 64-bit time, records one sample, and sends the time ACK. The fifth
 * call also sends the complete report and resets the session.
 */
bool time_sync_handle_start(uint8_t* tx_frame, TimeSyncSession* session);

#endif /* TIME_SYNC_H */
