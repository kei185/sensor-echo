#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "time_sync.h"

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
