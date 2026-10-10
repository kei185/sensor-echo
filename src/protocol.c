#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"

#include "arbiter.h"
#include "protocol.h"
#include "startup.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "tx/frame.h"
#include "tx/header.h"
#include "lidar/core.h"
#include "lidar/translate.h"
#include "imu/handler.h"

// bool enc_arrived = false;
void loop()
{
        // 起動に失敗したらCPUを止めず、次のhandshakeを待つ。
        while (!run_startup_sequence())
                ;

        size_t len;

        while (1) {
                // Retry a queued TX frame if a previous DMA start was busy.
                try_dispatch_tx();

                // if (scan_stop_requested)
                // stop scan and send ack

                TxBufSlot* tbs = get_empty_buf();
                if (tbs == NULL)
                        continue;

                if (imu_accel_ready && imu_rot_ready) {
                        imu_accel_ready = imu_rot_ready = false;

                        if (!imu_handler(tbs->_buf + TX_FRAME_HEADER_SIZE))
                                continue;

                        len = tx_frame_write_header(
                                tbs->_buf,
                                CORE_TX_BUF_SIZE,
                                IMU_BUF_SIZE,
                                FRAME_TYPE_IMU,
                                HAL_GetTick());

                        if (len == 0)
                                continue;

                        push_full_slot(tbs, len);

                        continue;
                }

                // if (enc_arrived)
                //  translate_enc();

                if (is_lapped()) {
                        reset_read_idx();
                        continue;
                }

                len = translate_scan_frame(tbs->_buf);
                if (len == 0u || is_lapped()) {
                        reset_read_idx();
                        continue;
                }

                push_full_slot(tbs, (uint32_t)len);
                try_dispatch_tx();
        }
}
