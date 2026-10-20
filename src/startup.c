#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "startup.h"
#include "time_sync.h"
#include "lidar/core.h"
#include "lidar/sys.h"
#include "lidar/translate.h"
#include "tx/frame.h"
#include "tx/header.h"

#define STARTUP_UART_TIMEOUT_MS 100u

static char uart_failure[96];

bool startup_uart_ok(
        UART_HandleTypeDef* uart, HAL_StatusTypeDef status, const char* operation)
{
        if (status == HAL_OK)
                return true;

        const char* name = status == HAL_TIMEOUT ? "HAL_TIMEOUT"
                           : status == HAL_BUSY  ? "HAL_BUSY"
                                                 : "HAL_ERROR";
        snprintf(
                uart_failure,
                sizeof(uart_failure),
                "UART%u %s %s error=0x%08lX",
                uart == &huart2 ? 2u : 4u,
                operation,
                name,
                (unsigned long)uart->ErrorCode);
        return false;
}
// UART2への通知でErrorCodeが消える前に、元のステータスとエラービットを保存する。

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
                time_sync_get_unix_time_ms());

        if (frame_length == 0u)
                return false;

        HAL_StatusTypeDef transmit_status = HAL_UART_Transmit(
                &huart2,
                tx_frame,
                (uint16_t)frame_length,
                STARTUP_UART_TIMEOUT_MS);

        return startup_uart_ok(&huart2, transmit_status, "TX");
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

        if (!startup_uart_ok(&huart4, request_status, "TX"))
                return false;

        setup_blocking_rx(reply_length);

        uint8_t* rx_buf = RX_BUF->_buf;

        // receive lidar response
        HAL_StatusTypeDef receive_status = HAL_UART_Receive(
                &huart4,
                rx_buf,
                (uint16_t)reply_length,
                STARTUP_UART_TIMEOUT_MS);

        if (!startup_uart_ok(&huart4, receive_status, "RX"))
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

        return startup_uart_ok(&huart2, forward_status, "forward TX");
}

/**
 * @brief Wait until the host sends the command that permits scanning to start.
 *
 * Unknown two-byte commands are ignored. A UART receive failure stops startup.
 *
 * @return true after receiving the start-scan command.
 */
static bool wait_for_start_scan(void)
{
        uint8_t command[HOST_COMMAND_SIZE];

        // Startup remains blocking until the host sends the exact two-byte start command.
        while (true) {
                HAL_StatusTypeDef receive_status = HAL_UART_Receive(
                        &huart2,
                        command,
                        HOST_COMMAND_SIZE,
                        HAL_MAX_DELAY);

                if (!startup_uart_ok(&huart2, receive_status, "RX"))
                        return false;

                bool is_start_scan = memcmp(command,
                                            HOST_COMMANDS[HOST_COMMAND_START_SCAN],
                                            HOST_COMMAND_SIZE) == 0;

                if (is_start_scan)
                        return true;
        }
}

/**
 * @brief Record a startup failure and report it to the host when possible.
 *
 * @param tx_frame Writable TX slot, or NULL when no slot was available.
 * @return false so callers can return this function directly.
 */
static bool fail_startup(uint8_t* tx_frame, const char* stage)
{
        char message[160];
        int  length = snprintf(
                message,
                sizeof(message),
                "%s: %s%s%s",
                FRAME_MESSAGE_STARTUP_FAILED,
                stage,
                uart_failure[0] ? ": " : "",
                uart_failure);
        if (tx_frame != NULL && length > 0 && (size_t)length < sizeof(message))
                (void)send_host_system_message(
                        tx_frame,
                        FRAME_TYPE_STARTUP_FAILED,
                        message,
                        (uint16_t)length);
        return false;
}

/**
 * @brief Complete the blocking handshake and start continuous LiDAR reception.
 *
 * The sequence reports initialization, forwards device information and health,
 * reports readiness, waits for host permission, acknowledges it, arms RX DMA,
 * and starts scanning. Any failed step sets the startup-failure pin and stops
 * the sequence.
 *
 * @return true after the ACK and LiDAR start-scan command are both sent.
 */
bool run_startup_sequence(void)
{
        uart_failure[0] = '\0';
        // Startup owns this free slot until all blocking host transmissions finish.
        TxBufSlot* tx_slot = get_empty_buf();
        if (tx_slot == NULL)
                return fail_startup(NULL, "TX buffer");
        uint8_t* tx_frame = tx_slot->_buf;

        // hostが3秒ごとに送るhandshakeを待つ。それ以外のコマンドでは進まない。
        uint8_t command[HOST_COMMAND_SIZE];
        do {
                HAL_StatusTypeDef receive_status = HAL_UART_Receive(
                        &huart2,
                        command,
                        HOST_COMMAND_SIZE,
                        HAL_MAX_DELAY);
                if (!startup_uart_ok(&huart2, receive_status, "RX"))
                        return fail_startup(tx_frame, "handshake");
        } while (memcmp(command,
                        HOST_COMMANDS[HOST_COMMAND_HANDSHAKE],
                        HOST_COMMAND_SIZE) != 0);

        // ACKを送れたら、既存の起動処理を続ける。
        if (!send_host_system_message(
                    tx_frame,
                    FRAME_TYPE_HANDSHAKE_ACK,
                    FRAME_MESSAGE_HANDSHAKE_ACK,
                    sizeof(FRAME_MESSAGE_HANDSHAKE_ACK) - 1u))
                return fail_startup(tx_frame, "handshake ACK");

        /**
         * send initializing message
         */
        bool initializing_sent = send_host_system_message(
                tx_frame,
                FRAME_TYPE_INITIALIZING,
                FRAME_MESSAGE_INITIALIZING,
                sizeof(FRAME_MESSAGE_INITIALIZING) - 1u);
        if (!initializing_sent)
                return fail_startup(tx_frame, "initializing");

        /**
         * forward device info
         */
        bool device_info_forwarded = request_and_forward_lidar_message(
                tx_frame,
                MSG_TYPE_RX_SYS_INFO,
                SYS_PACKET_DEVICE_INFO_CONTENT_SIZE,
                SYS_TYPE_CODE_DEVICE_INFO);
        if (!device_info_forwarded)
                return fail_startup(tx_frame, "lidar info");

        /**
         * forward health
         */
        bool health_status_forwarded = request_and_forward_lidar_message(
                tx_frame,
                MSG_TYPE_RX_HEALTH,
                SYS_PACKET_HEALTH_CONTENT_SIZE,
                SYS_TYPE_CODE_HEALTH);
        if (!health_status_forwarded)
                return fail_startup(tx_frame, "lidar health");

        /**
         * send ready
         */
        bool ready_sent = send_host_system_message(
                tx_frame,
                FRAME_TYPE_READY,
                FRAME_MESSAGE_READY,
                sizeof(FRAME_MESSAGE_READY) - 1u);
        if (!ready_sent)
                return fail_startup(tx_frame, "ready");

        TimeSyncSession session         = {0};
        uint8_t*        time_sync_frame = get_empty_buf()->_buf;
        if (!time_sync_handle_start(time_sync_frame, &session))
                return fail_startup(tx_frame, "time sync");
        if (!time_sync_set_rtc(&session))
                return fail_startup(tx_frame, "RTC set");
        if (!time_sync_send_report_if_ready(time_sync_frame, &session))
                return fail_startup(tx_frame, "time sync report");

        /**
         * wait for start command from the host
         */
        bool start_scan_requested = wait_for_start_scan();
        if (!start_scan_requested)
                return fail_startup(tx_frame, "start scan");

        /**
         * send ack for start scan command
         */
        bool start_scan_ack_sent = send_host_system_message(
                tx_frame,
                FRAME_TYPE_START_SCAN_ACK,
                FRAME_MESSAGE_START_SCAN_ACK,
                sizeof(FRAME_MESSAGE_START_SCAN_ACK) - 1u);
        if (!start_scan_ack_sent)
                return fail_startup(tx_frame, "start scan ACK");

        // Arm RX before the scan command so the first scan bytes cannot be lost.
        // set global buffer object
        setup_nonblocking_rx();

        // setting dma peripheral
        HAL_StatusTypeDef dma_start_status =
                HAL_UART_Receive_DMA(&huart4, RX_BUF->_buf, CORE_RX_BUF_SIZE);

        if (!startup_uart_ok(&huart4, dma_start_status, "RX DMA"))
                return fail_startup(tx_frame, "lidar DMA");

        return true;
}
