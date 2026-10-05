#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "main.h"

#include "arbiter.h"
#include "lidar/core.h"
#include "time_sync.h"
#include "tx/frame.h"
#include "tx/header.h"

static const char TIME_SYNC_REPORT_FORMAT[] =
        "TIME SYNC: req_tick=%" PRIu32 " ms | res_tick=%" PRIu32
        " ms | round_trip=%" PRIu32 " ms | unix_time=%" PRIu64;

bool report_time_sync_measurement(const TimeSyncMeasurement* measurement)
{
        if (measurement == NULL)
                return false;

        TxBufSlot* slot = get_empty_buf();
        if (slot == NULL)
                return false;

        char*          payload    = (char*)(slot->_buf + TX_FRAME_HEADER_SIZE);
        const size_t   capacity   = CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE;
        const uint32_t round_trip = measurement->res_time - measurement->req_time;

        int written = snprintf(
                payload,
                capacity,
                TIME_SYNC_REPORT_FORMAT,
                measurement->req_time,
                measurement->res_time,
                round_trip,
                measurement->unixtime);
        if (written < 0 || (size_t)written >= capacity)
                return false;

        size_t frame_length = tx_frame_write_header(
                slot->_buf,
                CORE_TX_BUF_SIZE,
                (uint16_t)written,
                FRAME_TYPE_TIME_SYNC_REPORT,
                HAL_GetTick());
        if (frame_length == 0u)
                return false;

        push_full_slot(slot, (uint32_t)frame_length);
        try_dispatch_tx();
        return true;
}
