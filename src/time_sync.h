#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
        uint32_t req_time; // HAL tick when the controller requests Unix time.
        uint32_t res_time; // HAL tick when the host response arrives.
        uint64_t unixtime; // Unmodified Unix-time value supplied by the host.
} TimeSyncMeasurement;

/**
 * Queue one human-readable time-sync measurement for UART2 transmission.
 *
 * The function returns after the frame is queued. UART2 DMA may still be
 * transmitting this frame or an older frame. Call this from the main context;
 * the TX queue has one writer.
 */
bool report_time_sync_measurement(const TimeSyncMeasurement* measurement);

#endif /* TIME_SYNC_H */
