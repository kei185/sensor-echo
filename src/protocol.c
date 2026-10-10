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
#include "lidar/sys.h"
#include "lidar/core.h"
#include "lidar/parser/meta.h"
#include "lidar/translate.h"
#include "imu/handler.h"

// bool enc_arrived = false;
void loop()
{
        if (!run_startup_sequence())
                Error_Handler();

        // // send start scan command
        HAL_StatusTypeDef scan_start_status =
                HAL_UART_Transmit(&huart4, MSG[MSG_TYPE_SCAN], MSG_SIZE, 100);

        if (scan_start_status != HAL_OK) {
                LD2_GPIO_Port->ODR ^= LD2_Pin;
                Error_Handler();
        }

        // // wait for meta frame for scan
        ParserMeta meta = {0};
        read_meta(&meta);
        if (meta.res_mode != SYS_RES_MODE_CONTINUOUS ||
            meta.type_code != SYS_TYPE_CODE_SCAN) {
                HAL_GPIO_WritePin(
                        INITIAL_HANDSHAKE_FAILED_GPIO_Port,
                        INITIAL_HANDSHAKE_FAILED_Pin,
                        1);

                Error_Handler();
        }

        if (!imu_setup()) {
                LD2_GPIO_Port->ODR ^= LD2_Pin;
                Error_Handler();
        }

        SCANNING_GPIO_Port->ODR ^= SCANNING_Pin;

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
