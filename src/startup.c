#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "startup.h"
#include "protocol.h"
#include "lidar/core.h"
#include "lidar/sys.h"
#include "tx/frame.h"
#include "tx/header.h"

#define STARTUP_UART_TIMEOUT_MS 100u

static const char INITIALIZING_MESSAGE[] = "INITIALIZING\r\n";
static const char READY_MESSAGE[]        = "READY\r\n";
static const char FAILURE_MESSAGE[]      = "STARTUP FAILED\r\n";

/**
 * @brief Build and send one system-message frame to the host.
 *
 * The payload is written after the reserved TX header area. The header is
 * filled in last so it contains the final payload length and timestamp.
 *
 * @param tx_frame Writable TX slot used for the complete host frame.
 * @param frame_type System-message type written into the host frame header.
 * @param message NUL-terminated message written into the frame payload.
 * @return true when the complete frame was sent to the host.
 */
static bool
send_host_system_message(uint8_t* tx_frame, FrameType frame_type, const char* message)
{
        const size_t payload_length = strlen(message);

        if (payload_length > CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE)
                return false;

        // write message into the frame
        memcpy(tx_frame + TX_FRAME_HEADER_SIZE, message, payload_length);

        // write header into the frame
        size_t frame_length = tx_frame_write_header(
                tx_frame,
                CORE_TX_BUF_SIZE,
                (uint16_t)payload_length,
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

        // Bits 0-29 contain the content length; bits 30-31 contain the response mode.
        uint32_t length_mode = (uint32_t)reply[2] | ((uint32_t)reply[3] << 8u) |
                               ((uint32_t)reply[4] << 16u) | ((uint32_t)reply[5] << 24u);

        bool has_expected_header =
                reply[0] == SYS_PACKET_HEADER_MSB && reply[1] == SYS_PACKET_HEADER_LSB;
        bool has_expected_length = (length_mode & 0x3fffffffu) == expected_content_length;
        bool is_single_response  = (length_mode >> 30u) == SYS_RES_MODE_SINGLE;
        bool has_expected_type   = reply[6] == (uint8_t)expected_type;

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

        uint8_t* rx_buf = (uint8_t*)RX_BUF->_buf;

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

        size_t frame_length = translate((int8_t*)tx_frame);

        if (frame_length == 0u)
                return false;

        HAL_StatusTypeDef forward_status = HAL_UART_Transmit(
                &huart2,
                tx_frame,
                (uint16_t)frame_length,
                STARTUP_UART_TIMEOUT_MS);

        return forward_status == HAL_OK;
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

                if (receive_status != HAL_OK)
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
static bool fail_startup(uint8_t* tx_frame)
{
        HAL_GPIO_WritePin(
                INITIAL_HANDSHAKE_FAILED_GPIO_Port,
                INITIAL_HANDSHAKE_FAILED_Pin,
                GPIO_PIN_SET);

        if (tx_frame != NULL)
                (void)send_host_system_message(
                        tx_frame,
                        FRAME_TYPE_STARTUP_FAILED,
                        FAILURE_MESSAGE);
        return false;
}

/**
 * @brief Complete the blocking handshake and start continuous LiDAR reception.
 *
 * The sequence reports initialization, forwards device information and health,
 * reports readiness, waits for host permission, arms RX DMA, and starts scanning.
 * Any failed step sets the startup-failure pin and stops the sequence.
 *
 * @return true after RX DMA and LiDAR scanning have both started.
 */
bool run_startup_sequence(void)
{
        // Startup owns this free slot until all blocking host transmissions finish.
        TxBufSlot* tx_slot = get_empty_buf();
        if (tx_slot == NULL)
                return fail_startup(NULL);
        uint8_t* tx_frame = (uint8_t*)tx_slot->_buf;

        bool initializing_sent = send_host_system_message(
                tx_frame,
                FRAME_TYPE_INITIALIZING,
                INITIALIZING_MESSAGE);
        if (!initializing_sent)
                return fail_startup(tx_frame);

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
        if (!health_status_forwarded)
                return fail_startup(tx_frame);

        bool ready_sent =
                send_host_system_message(tx_frame, FRAME_TYPE_READY, READY_MESSAGE);
        if (!ready_sent)
                return fail_startup(tx_frame);

        bool start_scan_requested = wait_for_start_scan();
        if (!start_scan_requested)
                return fail_startup(tx_frame);

        // Arm RX before the scan command so the first scan bytes cannot be lost.
        setup_nonblocking_rx();

        HAL_StatusTypeDef dma_start_status =
                HAL_UART_Receive_DMA(&huart4, (uint8_t*)RX_BUF->_buf, CORE_RX_BUF_SIZE);

        if (dma_start_status != HAL_OK)
                return fail_startup(tx_frame);

        HAL_StatusTypeDef scan_start_status = HAL_UART_Transmit(
                &huart4,
                MSG[MSG_TYPE_SCAN],
                MSG_SIZE,
                STARTUP_UART_TIMEOUT_MS);

        if (scan_start_status != HAL_OK)
                return fail_startup(tx_frame);

        return true;
}
