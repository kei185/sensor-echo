#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"

#include "arbiter.h"
#include "protocol.h"
#include "startup.h"
#include "time_sync.h"
#include "stm32f4xx_hal_def.h"
#include "tx/frame.h"
#include "tx/header.h"
#include "lidar/sys.h"
#include "lidar/core.h"
#include "lidar/parser/meta.h"
#include "lidar/translate.h"
#include "imu/handler.h"
#include "motor.h"

static uint8_t host_command_buf[HOST_MOTOR_COMMAND_SIZE];

void loop()
{
        PROTOCOL_RUNNING_GPIO_Port->ODR ^= PROTOCOL_RUNNING_Pin;
        while (!run_startup_sequence())
                ;

        if (HAL_UART_Receive_IT(&huart2, host_command_buf, HOST_MOTOR_COMMAND_SIZE) !=
            HAL_OK)
                Error_Handler();

        // // send start scan command
        HAL_StatusTypeDef scan_start_status =
                HAL_UART_Transmit(&huart4, MSG[MSG_TYPE_SCAN], MSG_SIZE, 100);

        if (scan_start_status != HAL_OK)
                Error_Handler();

        // // wait for meta frame for scan
        ParserMeta meta = {0};
        read_meta(&meta);
        if (meta.res_mode != SYS_RES_MODE_CONTINUOUS ||
            meta.type_code != SYS_TYPE_CODE_SCAN)
                Error_Handler();

        if (!imu_setup())
                Error_Handler();

        LIDAR_SCANNING_GPIO_Port->ODR ^= LIDAR_SCANNING_Pin;

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

                        IMU_RUNNING_GPIO_Port->ODR ^= IMU_RUNNING_Pin;
                        if (!imu_handler(tbs->_buf + TX_FRAME_HEADER_SIZE))
                                continue;
                        IMU_RUNNING_GPIO_Port->ODR ^= IMU_RUNNING_Pin;
                        len = tx_frame_write_header(
                                tbs->_buf,
                                CORE_TX_BUF_SIZE,
                                IMU_BUF_SIZE,
                                FRAME_TYPE_IMU,
                                time_sync_get_unix_time_ms());

                        if (len == 0)
                                continue;

                        push_full_slot(tbs, len);

                        continue;
                }

                if (motor_encoder_timeout) {
                        motor_encoder_timeout = false;

                        MOTOR_RUNNING_GPIO_Port->ODR ^= MOTOR_RUNNING_Pin;
                        motor_timeout_handler(tbs->_buf + TX_FRAME_HEADER_SIZE);
                        MOTOR_RUNNING_GPIO_Port->ODR ^= MOTOR_RUNNING_Pin;

                        len = tx_frame_write_header(
                                tbs->_buf,
                                CORE_TX_BUF_SIZE,
                                ENCODER_FRAME_PAYLOAD_SIZE,
                                FRAME_TYPE_ENC,
                                time_sync_get_unix_time_ms());

                        if (len == 0)
                                continue;

                        push_full_slot(tbs, len);

                        continue;
                }

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

void host_command_handler(void)
{
        HostCommand type = HOST_COMMAND_COUNT;
        for (HostCommand candidate = 0; candidate < HOST_COMMAND_COUNT; candidate++) {
                if (memcmp(host_command_buf,
                           HOST_COMMANDS[candidate],
                           HOST_COMMAND_SIZE) == 0) {
                        type = candidate;
                        break;
                }
        }

        switch (type) {
                case HOST_COMMAND_MOTOR:
                        motor_command_handler(host_command_buf + HOST_COMMAND_SIZE);
                        break;
                default:
                        break;
        }

        if (HAL_UART_Receive_IT(&huart2, host_command_buf, HOST_MOTOR_COMMAND_SIZE) !=
            HAL_OK)
                Error_Handler();
}
