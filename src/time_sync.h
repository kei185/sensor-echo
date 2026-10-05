#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TIME_SYNC_SAMPLE_COUNT   5u
#define TIME_SYNC_UNIX_TIME_SIZE 8u

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

#endif /* TIME_SYNC_H */
