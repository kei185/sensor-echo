#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "arbiter.h"
#include "protocol.h"
#include "startup.h"
#include "time_sync.h"
#include "imu/handler.h"
#include "lidar/core.h"
#include "lidar/translate.h"
#include "tx/frame.h"
#include "tx/header.h"

#define PROTOCOL_UART_TIMEOUT_MS    100u
#define PROTOCOL_COMMAND_TIMEOUT_MS HOST_HANDSHAKE_RETRY_INTERVAL_MS

typedef enum
{
        WAIT_HANDSHAKE,
        WAIT_TIME_SYNC_START,
        WAIT_TIME,
        WAIT_SCAN_START,
} StartupPhase;

static bool send_system_frame(
        uint8_t* tx_frame, FrameType type, const char* message, uint16_t message_length)
{
        memcpy(tx_frame + TX_FRAME_HEADER_SIZE, message, message_length);
        size_t frame_length = tx_frame_write_header(
                tx_frame,
                CORE_TX_BUF_SIZE,
                message_length,
                type,
                HAL_GetTick());
        return frame_length != 0u && HAL_UART_Transmit(
                                             &huart2,
                                             tx_frame,
                                             (uint16_t)frame_length,
                                             PROTOCOL_UART_TIMEOUT_MS) == HAL_OK;
}

static bool receive_host_command(HostCommand* command, bool waiting_handshake)
{
        uint8_t bytes[HOST_COMMAND_SIZE];

        // handshake待ちでは、古いコマンドの残りを飛ばして先頭AAを探す。
        do {
                uint32_t timeout =
                        waiting_handshake ? HAL_MAX_DELAY : PROTOCOL_COMMAND_TIMEOUT_MS;
                if (HAL_UART_Receive(&huart2, bytes, 1u, timeout) != HAL_OK)
                        return false;
        } while (waiting_handshake && bytes[0] != 0xAAu);

        if (bytes[0] != 0xAAu ||
            HAL_UART_Receive(&huart2, bytes + 1u, 1u, PROTOCOL_UART_TIMEOUT_MS) != HAL_OK)
                return false;

        *command = HOST_COMMAND_COUNT;
        for (uint8_t i = 0u; i < HOST_COMMAND_COUNT; ++i) {
                if (memcmp(bytes, HOST_COMMANDS[i], HOST_COMMAND_SIZE) == 0) {
                        *command = (HostCommand)i;
                        break;
                }
        }
        return true;
}

static bool fail_startup(uint8_t* tx_frame)
{
        startup_cancel();
        (void)HAL_UART_Abort(&huart2);
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        HAL_GPIO_WritePin(SCANNING_GPIO_Port, SCANNING_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(
                INITIAL_HANDSHAKE_FAILED_GPIO_Port,
                INITIAL_HANDSHAKE_FAILED_Pin,
                GPIO_PIN_SET);

        if (tx_frame != NULL)
                (void)send_system_frame(
                        tx_frame,
                        FRAME_TYPE_STARTUP_FAILED,
                        FRAME_MESSAGE_STARTUP_FAILED,
                        sizeof(FRAME_MESSAGE_STARTUP_FAILED) - 1u);
        return false;
}

bool run_startup_sequence(void)
{
        // 起動中はこの空きスロットだけを使い、DMAのTXキューには公開しない。
        TxBufSlot* tx_slot = get_empty_buf();
        if (tx_slot == NULL)
                return fail_startup(NULL);
        uint8_t*        tx_frame = tx_slot->_buf;
        StartupPhase    phase    = WAIT_HANDSHAKE;
        TimeSyncSession session  = {0};

        while (true) {
                HostCommand command;
                if (!receive_host_command(&command, phase == WAIT_HANDSHAKE))
                        return fail_startup(phase == WAIT_HANDSHAKE ? NULL : tx_frame);

                if (command == HOST_COMMAND_HANDSHAKE) {
                        // 新しいhandshakeは、以前の起動処理と時刻サンプルを取り消す。
                        startup_cancel();
                        session = (TimeSyncSession){0};
                        HAL_GPIO_WritePin(
                                INITIAL_HANDSHAKE_FAILED_GPIO_Port,
                                INITIAL_HANDSHAKE_FAILED_Pin,
                                GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(
                                SCANNING_GPIO_Port,
                                SCANNING_Pin,
                                GPIO_PIN_RESET);
                        if (!send_system_frame(
                                    tx_frame,
                                    FRAME_TYPE_HANDSHAKE_ACK,
                                    FRAME_MESSAGE_HANDSHAKE_ACK,
                                    sizeof(FRAME_MESSAGE_HANDSHAKE_ACK) - 1u) ||
                            !send_system_frame(
                                    tx_frame,
                                    FRAME_TYPE_INITIALIZING,
                                    FRAME_MESSAGE_INITIALIZING,
                                    sizeof(FRAME_MESSAGE_INITIALIZING) - 1u) ||
                            !startup_initialize_sensors(tx_frame) ||
                            !send_system_frame(
                                    tx_frame,
                                    FRAME_TYPE_READY,
                                    FRAME_MESSAGE_READY,
                                    sizeof(FRAME_MESSAGE_READY) - 1u))
                                return fail_startup(tx_frame);
                        phase = WAIT_TIME_SYNC_START;
                        continue;
                }

                if (phase == WAIT_HANDSHAKE)
                        continue;

                switch (phase) {
                        case WAIT_TIME_SYNC_START:
                                if (command != HOST_COMMAND_TIME_SYNC_START ||
                                    !time_sync_send_start_ack(tx_frame, &session))
                                        return fail_startup(tx_frame);
                                phase = WAIT_TIME;
                                break;

                        case WAIT_TIME: {
                                if (command != HOST_COMMAND_TIME)
                                        return fail_startup(tx_frame);
                                uint8_t payload[TIME_SYNC_PAYLOAD_SIZE];
                                if (HAL_UART_Receive(
                                            &huart2,
                                            payload,
                                            sizeof(payload),
                                            TIME_SYNC_RESPONSE_TIMEOUT_MS) != HAL_OK)
                                        return fail_startup(tx_frame);
                                uint32_t received_tick = HAL_GetTick();
                                if (!time_sync_accept_time(
                                            tx_frame,
                                            &session,
                                            payload,
                                            received_tick) ||
                                    !time_sync_set_rtc(&session) ||
                                    !time_sync_send_report_if_ready(tx_frame, &session))
                                        return fail_startup(tx_frame);
                                phase = WAIT_SCAN_START;
                                break;
                        }

                        case WAIT_SCAN_START:
                                if (command != HOST_COMMAND_START_SCAN ||
                                    !send_system_frame(
                                            tx_frame,
                                            FRAME_TYPE_START_SCAN_ACK,
                                            FRAME_MESSAGE_START_SCAN_ACK,
                                            sizeof(FRAME_MESSAGE_START_SCAN_ACK) - 1u) ||
                                    !startup_start_scan())
                                        return fail_startup(tx_frame);
                                HAL_GPIO_WritePin(
                                        SCANNING_GPIO_Port,
                                        SCANNING_Pin,
                                        GPIO_PIN_SET);
                                return true;

                        default:
                                return fail_startup(tx_frame);
                }
        }
}

void loop(void)
{
        // 失敗でCPUを止めず、次のhandshakeから初期化をやり直す。
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
