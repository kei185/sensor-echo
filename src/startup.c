#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "startup.h"
#include "protocol.h"
#include "time_sync.h"
#include "lidar/core.h"
#include "lidar/sys.h"
#include "tx/frame.h"
#include "tx/header.h"

#define STARTUP_UART_TIMEOUT_MS 100u

static const char INITIALIZING_MESSAGE[]        = "INITIALIZING";
static const char READY_MESSAGE[]               = "READY";
static const char FAILURE_MESSAGE[]             = "STARTUP FAILED";
static const char START_SCAN_ACK_MESSAGE[]      = "START SCAN ACK";
static const char TIME_SYNC_START_ACK_MESSAGE[] = "TIME SYNC START ACK";
static const char TIME_ACK_MESSAGE[]            = "TIME ACK";

/**
 * @brief Finish and transmit a payload already stored in the TX frame.
 */
static bool
send_host_frame(uint8_t* tx_frame, FrameType frame_type, size_t payload_length)
{
        if (payload_length > CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE)
                return false;

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
        return send_host_frame(tx_frame, frame_type, payload_length);
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

/**
 * @brief Handle optional time-sync measurements while waiting to start scanning.
 *
 * Unknown two-byte commands are ignored. Five completed time-sync measurements
 * produce one report. A UART receive failure stops startup.
 *
 * @return true after receiving the start-scan command.
 */
static bool measure_host_time(uint8_t* tx_frame, TimeSyncSession* time_sync_session)
{
        // The start ACK asks the host to send its current Unix time.
        uint32_t req_time       = HAL_GetTick();
        bool     start_ack_sent = send_host_system_message(
                tx_frame,
                FRAME_TYPE_TIME_SYNC_START_ACK,
                TIME_SYNC_START_ACK_MESSAGE);
        if (!start_ack_sent)
                return false;

        uint8_t           encoded_unix_time[TIME_SYNC_UNIX_TIME_SIZE];
        HAL_StatusTypeDef receive_status = HAL_UART_Receive(
                &huart2,
                encoded_unix_time,
                TIME_SYNC_UNIX_TIME_SIZE,
                HAL_MAX_DELAY);
        if (receive_status != HAL_OK)
                return false;

        uint32_t res_time = HAL_GetTick();
        uint64_t unixtime;
        if (!time_sync_decode_unix_time(
                    encoded_unix_time,
                    sizeof(encoded_unix_time),
                    &unixtime) ||
            !time_sync_record(time_sync_session, req_time, res_time, unixtime))
                return false;

        bool time_ack_sent =
                send_host_system_message(tx_frame, FRAME_TYPE_TIME_ACK, TIME_ACK_MESSAGE);
        if (!time_ack_sent)
                return false;

        if (time_sync_session->count < TIME_SYNC_SAMPLE_COUNT)
                return true;

        char*  payload        = (char*)(tx_frame + TX_FRAME_HEADER_SIZE);
        size_t payload_length = time_sync_format_report(
                payload,
                CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE,
                time_sync_session);
        if (payload_length == 0u ||
            !send_host_frame(tx_frame, FRAME_TYPE_TIME_SYNC_REPORT, payload_length))
                return false;

        *time_sync_session = (TimeSyncSession){0};
        return true;
}

static bool wait_for_start_scan(uint8_t* tx_frame)
{
        uint8_t         command[HOST_COMMAND_SIZE];
        TimeSyncSession time_sync_session = {0};

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

                bool is_time_sync_start =
                        memcmp(command,
                               HOST_COMMANDS[HOST_COMMAND_TIME_SYNC_START],
                               HOST_COMMAND_SIZE) == 0;
                if (is_time_sync_start &&
                    !measure_host_time(tx_frame, &time_sync_session))
                        return false;
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
 * reports readiness, waits for host permission, acknowledges it, arms RX DMA,
 * and starts scanning. Any failed step sets the startup-failure pin and stops
 * the sequence.
 *
 * @return true after the ACK and LiDAR start-scan command are both sent.
 */
bool run_startup_sequence(void)
{
        // Startup owns this free slot until all blocking host transmissions finish.
        TxBufSlot* tx_slot = get_empty_buf();
        if (tx_slot == NULL)
                return fail_startup(NULL);
        uint8_t* tx_frame = tx_slot->_buf;

        /**
         * send initializing message
         */
        bool initializing_sent = send_host_system_message(
                tx_frame,
                FRAME_TYPE_INITIALIZING,
                INITIALIZING_MESSAGE);
        if (!initializing_sent)
                return fail_startup(tx_frame);

        /**
         * forward device info
         */
        bool device_info_forwarded = request_and_forward_lidar_message(
                tx_frame,
                MSG_TYPE_RX_SYS_INFO,
                SYS_PACKET_DEVICE_INFO_CONTENT_SIZE,
                SYS_TYPE_CODE_DEVICE_INFO);
        if (!device_info_forwarded)
                return fail_startup(tx_frame);

        /**
         * forward health
         */
        bool health_status_forwarded = request_and_forward_lidar_message(
                tx_frame,
                MSG_TYPE_RX_HEALTH,
                SYS_PACKET_HEALTH_CONTENT_SIZE,
                SYS_TYPE_CODE_HEALTH);
        if (!health_status_forwarded)
                return fail_startup(tx_frame);

        /**
         * send ready
         */
        bool ready_sent =
                send_host_system_message(tx_frame, FRAME_TYPE_READY, READY_MESSAGE);
        if (!ready_sent)
                return fail_startup(tx_frame);

        /**
         * wait for start command from the host
         */
        bool start_scan_requested = wait_for_start_scan(tx_frame);
        if (!start_scan_requested)
                return fail_startup(tx_frame);

        /**
         * send ack for start scan command
         */
        bool start_scan_ack_sent = send_host_system_message(
                tx_frame,
                FRAME_TYPE_START_SCAN_ACK,
                START_SCAN_ACK_MESSAGE);
        if (!start_scan_ack_sent)
                return fail_startup(tx_frame);

        // Arm RX before the scan command so the first scan bytes cannot be lost.
        // set global buffer object
        setup_nonblocking_rx();

        // setting dma peripheral
        HAL_StatusTypeDef dma_start_status =
                HAL_UART_Receive_DMA(&huart4, RX_BUF->_buf, CORE_RX_BUF_SIZE);

        if (dma_start_status != HAL_OK)
                return fail_startup(tx_frame);

        return true;
}
