#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "startup.h"
#include "time_sync.h"
#include "imu/handler.h"
#include "lidar/core.h"
#include "lidar/parser/meta.h"
#include "lidar/sys.h"
#include "lidar/translate.h"
#include "tx/frame.h"
#include "tx/header.h"

#define STARTUP_UART_TIMEOUT_MS 100u
#define STARTUP_RX_IDLE_MS      2u

/**
 * @brief Build and send one system-message frame to the host.
 *
 * The payload is written after the reserved TX header area. The header is
 * filled in last so it contains the final payload length and timestamp.
 *
 * @param tx_frame Writable TX slot used for the complete host frame.
 * @param frame_type System-message type written into the host frame header.
 * @param message Fixed message written into the frame payload.
 * @param message_length Compile-time length of message, excluding its NUL byte.
 * @return true when the complete frame was sent to the host.
 */
static bool send_host_system_message(
        uint8_t*    tx_frame,
        FrameType   frame_type,
        const char* message,
        uint16_t    message_length)
{
        // write message into the frame
        memcpy(tx_frame + TX_FRAME_HEADER_SIZE, message, message_length);

        // write header into the frame
        size_t frame_length = tx_frame_write_header(
                tx_frame,
                CORE_TX_BUF_SIZE,
                message_length,
                frame_type,
                HAL_GetTick());

        if (frame_length == 0u)
                return false;

        HAL_StatusTypeDef transmit_status = HAL_UART_Transmit(
                &huart2,
                tx_frame,
                (uint16_t)frame_length,
                STARTUP_UART_TIMEOUT_MS);

        return transmit_status == HAL_OK;
}

/**
 * @brief Check whether a blocking LiDAR response matches the sent command.
 *
 * A successful blocking UART receive guarantees that expected_reply_length
 * bytes arrived. This function then checks the LiDAR meta header, content
 * length, response mode, and type before the stream parser consumes it.
 *
 * @param reply Received LiDAR frame beginning with its meta header.
 * @param expected_reply_length Number of bytes requested from the UART.
 * @param expected_content_length Content size defined for the sent command.
 * @param expected_type Type code defined for the sent command.
 * @return true when the received meta header describes the expected response.
 */
static bool is_expected_lidar_reply(
        const uint8_t* reply,
        size_t         expected_reply_length,
        uint32_t       expected_content_length,
        SysTypeCode    expected_type)
{
        // Validate the fixed meta header before handing the bytes to the stream parser.
        if (expected_reply_length != SYS_PACKET_META_SIZE + expected_content_length)
                return false;

        // Meta bytes 2-5 store the 32-bit length/mode field in little-endian order.
        uint32_t length_mode = (uint32_t)reply[2] | ((uint32_t)reply[3] << 8u) |
                               ((uint32_t)reply[4] << 16u) | ((uint32_t)reply[5] << 24u);

        // Meta bytes 0-1 are the fixed 0xA5, 0x5A response header.
        bool has_expected_header =
                reply[0] == SYS_PACKET_HEADER_MSB && reply[1] == SYS_PACKET_HEADER_LSB;
        // 0x3fffffff keeps the content length stored in bits 0-29.
        bool has_expected_length = (length_mode & 0x3fffffffu) == expected_content_length;
        // Shifting by 30 moves the response mode from bits 30-31 to bits 0-1.
        bool is_single_response = (length_mode >> 30u) == SYS_RES_MODE_SINGLE;
        // Meta byte 6 identifies the content that follows the meta header.
        bool has_expected_type = reply[6] == (uint8_t)expected_type;

        return has_expected_header && has_expected_length && is_single_response &&
               has_expected_type;
}

/**
 * @brief Request one LiDAR message, convert it, and forward it to the host.
 *
 * The response is received directly into the shared RX buffer. Blocking RX
 * setup exposes exactly that received range to the existing LiDAR parser.
 *
 * @param tx_frame Writable TX slot used for the converted host frame.
 * @param command LiDAR command sent before receiving the response.
 * @param content_length Expected LiDAR content size in bytes.
 * @param type Expected LiDAR response type.
 * @return true when request, validation, conversion, and forwarding succeed.
 */
static bool request_and_forward_lidar_message(
        uint8_t* tx_frame, MsgType command, uint32_t content_length, SysTypeCode type)
{
        size_t reply_length = SYS_PACKET_META_SIZE + content_length;

        // send command to lidar
        HAL_StatusTypeDef request_status = HAL_UART_Transmit(
                &huart4,
                MSG[command],
                MSG_SIZE,
                STARTUP_UART_TIMEOUT_MS);

        if (request_status != HAL_OK)
                return false;

        setup_blocking_rx(reply_length);

        uint8_t* rx_buf = RX_BUF->_buf;

        // receive lidar response
        HAL_StatusTypeDef receive_status = HAL_UART_Receive(
                &huart4,
                rx_buf,
                (uint16_t)reply_length,
                STARTUP_UART_TIMEOUT_MS);

        if (receive_status != HAL_OK)
                return false;

        // verify received frame
        bool is_expected_reply =
                is_expected_lidar_reply(rx_buf, reply_length, content_length, type);

        if (!is_expected_reply)
                return false;

        size_t frame_length = translate_single(tx_frame);

        if (frame_length == 0u)
                return false;

        HAL_StatusTypeDef forward_status = HAL_UART_Transmit(
                &huart2,
                tx_frame,
                (uint16_t)frame_length,
                STARTUP_UART_TIMEOUT_MS);

        return forward_status == HAL_OK;
}

// handshake待ちだけは古いデータを飛ばす。それ以降の不正な入力は失敗にする。
static HostCommand receive_host_command(bool waiting_handshake)
{
        uint8_t bytes[HOST_COMMAND_SIZE];
        while (true) {
                do {
                        uint32_t timeout = waiting_handshake
                                                   ? HAL_MAX_DELAY
                                                   : HOST_HANDSHAKE_RETRY_INTERVAL_MS;
                        if (HAL_UART_Receive(&huart2, bytes, 1u, timeout) != HAL_OK)
                                return HOST_COMMAND_COUNT;
                } while (waiting_handshake && bytes[0] != 0xAAu);

                if (bytes[0] != 0xAAu ||
                    HAL_UART_Receive(&huart2, bytes + 1u, 1u, STARTUP_UART_TIMEOUT_MS) !=
                            HAL_OK)
                        return HOST_COMMAND_COUNT;

                for (uint8_t i = 0u; i < HOST_COMMAND_COUNT; ++i) {
                        if (memcmp(bytes, HOST_COMMANDS[i], HOST_COMMAND_SIZE) == 0)
                                return (HostCommand)i;
                }
                if (!waiting_handshake)
                        return HOST_COMMAND_COUNT;
        }
}

static void cancel_startup(void)
{
        // 起動のやり直しと失敗時の両方で、DMAを止めてからLiDARを停止する。
        (void)HAL_UART_Abort(&huart4);
        (void)HAL_UART_Transmit(
                &huart4,
                MSG[MSG_TYPE_STOP],
                MSG_SIZE,
                STARTUP_UART_TIMEOUT_MS);
        __HAL_UART_CLEAR_OREFLAG(&huart4);
        setup_blocking_rx(0u);
        imu_accel_ready = imu_rot_ready = false;
        HAL_GPIO_WritePin(SCANNING_GPIO_Port, SCANNING_Pin, GPIO_PIN_RESET);
}

static bool fail_startup(uint8_t* tx_frame)
{
        cancel_startup();
        (void)HAL_UART_Abort(&huart2);
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        HAL_GPIO_WritePin(
                INITIAL_HANDSHAKE_FAILED_GPIO_Port,
                INITIAL_HANDSHAKE_FAILED_Pin,
                GPIO_PIN_SET);

        if (tx_frame != NULL)
                (void)send_host_system_message(
                        tx_frame,
                        FRAME_TYPE_STARTUP_FAILED,
                        FRAME_MESSAGE_STARTUP_FAILED,
                        sizeof(FRAME_MESSAGE_STARTUP_FAILED) - 1u);
        return false;
}

bool run_startup_sequence(void)
{
        // 起動中の返信には同じ空きTXスロットを使い、キューのheadは進めない。
        TxBufSlot* tx_slot = get_empty_buf();
        if (tx_slot == NULL)
                return fail_startup(NULL);
        uint8_t*    tx_frame = tx_slot->_buf;
        HostCommand command  = HOST_COMMAND_COUNT;

        while (true) {
                while (command != HOST_COMMAND_HANDSHAKE) {
                        command = receive_host_command(true);
                        if (command == HOST_COMMAND_COUNT)
                                return fail_startup(NULL);
                }

                cancel_startup();
                HAL_GPIO_WritePin(
                        INITIAL_HANDSHAKE_FAILED_GPIO_Port,
                        INITIAL_HANDSHAKE_FAILED_Pin,
                        GPIO_PIN_RESET);
                if (!send_host_system_message(
                            tx_frame,
                            FRAME_TYPE_HANDSHAKE_ACK,
                            FRAME_MESSAGE_HANDSHAKE_ACK,
                            sizeof(FRAME_MESSAGE_HANDSHAKE_ACK) - 1u))
                        return fail_startup(tx_frame);

                bool initializing_sent = send_host_system_message(
                        tx_frame,
                        FRAME_TYPE_INITIALIZING,
                        FRAME_MESSAGE_INITIALIZING,
                        sizeof(FRAME_MESSAGE_INITIALIZING) - 1u);
                if (!initializing_sent)
                        return fail_startup(tx_frame);

                // STOP後に残ったscanを捨てる。2ms途切れたら進み、100msで諦める。
                uint32_t started = HAL_GetTick();
                while (true) {
                        uint8_t           byte;
                        HAL_StatusTypeDef status =
                                HAL_UART_Receive(&huart4, &byte, 1u, STARTUP_RX_IDLE_MS);
                        if (status == HAL_TIMEOUT)
                                break;
                        if (status != HAL_OK ||
                            HAL_GetTick() - started >= STARTUP_UART_TIMEOUT_MS)
                                return fail_startup(tx_frame);
                }

                bool device_info_forwarded = request_and_forward_lidar_message(
                        tx_frame,
                        MSG_TYPE_RX_SYS_INFO,
                        SYS_PACKET_DEVICE_INFO_CONTENT_SIZE,
                        SYS_TYPE_CODE_DEVICE_INFO);
                if (!device_info_forwarded)
                        return fail_startup(tx_frame);

                bool health_status_forwarded = request_and_forward_lidar_message(
                        tx_frame,
                        MSG_TYPE_RX_HEALTH,
                        SYS_PACKET_HEALTH_CONTENT_SIZE,
                        SYS_TYPE_CODE_HEALTH);
                if (!health_status_forwarded || !imu_setup())
                        return fail_startup(tx_frame);

                bool ready_sent = send_host_system_message(
                        tx_frame,
                        FRAME_TYPE_READY,
                        FRAME_MESSAGE_READY,
                        sizeof(FRAME_MESSAGE_READY) - 1u);
                if (!ready_sent)
                        return fail_startup(tx_frame);

                TimeSyncSession session = {0};
                TimeSyncResult  result  = time_sync_handle_start(tx_frame, &session);
                // handshakeは時刻同期側ですでに受信済みなので、再受信せずACKへ戻る。
                if (result == TIME_SYNC_RESTART)
                        continue;
                if (result != TIME_SYNC_COMPLETE || !time_sync_set_rtc(&session) ||
                    !time_sync_send_report_if_ready(tx_frame, &session))
                        return fail_startup(tx_frame);

                command = receive_host_command(false);
                if (command == HOST_COMMAND_HANDSHAKE)
                        continue;
                if (command != HOST_COMMAND_START_SCAN)
                        return fail_startup(tx_frame);

                bool start_scan_ack_sent = send_host_system_message(
                        tx_frame,
                        FRAME_TYPE_START_SCAN_ACK,
                        FRAME_MESSAGE_START_SCAN_ACK,
                        sizeof(FRAME_MESSAGE_START_SCAN_ACK) - 1u);
                if (!start_scan_ack_sent)
                        return fail_startup(tx_frame);

                // DMAを先に開始してからscanを要求し、最初の受信を取り逃さない。
                setup_nonblocking_rx();
                if (HAL_UART_Receive_DMA(&huart4, RX_BUF->_buf, CORE_RX_BUF_SIZE) !=
                    HAL_OK)
                        return fail_startup(tx_frame);
                if (HAL_UART_Transmit(
                            &huart4,
                            MSG[MSG_TYPE_SCAN],
                            MSG_SIZE,
                            STARTUP_UART_TIMEOUT_MS) != HAL_OK)
                        return fail_startup(tx_frame);

                // descriptor 7byteとread_byte()の余裕2byteを、期限付きで待つ。
                started = HAL_GetTick();
                while (CORE_RX_BUF_SIZE - *RX_BUF->remain_bytes <
                       SYS_PACKET_META_SIZE + 2u) {
                        if (HAL_GetTick() - started >= STARTUP_UART_TIMEOUT_MS)
                                return fail_startup(tx_frame);
                }
                if (RX_BUF->_buf[0] != SYS_PACKET_HEADER_MSB ||
                    RX_BUF->_buf[1] != SYS_PACKET_HEADER_LSB)
                        return fail_startup(tx_frame);

                ParserMeta meta = {0};
                if (read_meta(&meta) == NULL ||
                    meta.res_mode != SYS_RES_MODE_CONTINUOUS ||
                    meta.type_code != SYS_TYPE_CODE_SCAN)
                        return fail_startup(tx_frame);
                HAL_GPIO_WritePin(SCANNING_GPIO_Port, SCANNING_Pin, GPIO_PIN_SET);
                return true;
        }
}
