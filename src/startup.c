#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "startup.h"
#include "imu/handler.h"
#include "lidar/core.h"
#include "lidar/parser/meta.h"
#include "lidar/sys.h"
#include "lidar/translate.h"

#define STARTUP_UART_TIMEOUT_MS 100u
#define STARTUP_RX_IDLE_MS      2u

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

void startup_cancel(void)
{
        // RX DMAを止めてから、LiDARにも停止コマンドを送る。
        (void)HAL_UART_Abort(&huart4);
        (void)HAL_UART_Transmit(
                &huart4,
                MSG[MSG_TYPE_STOP],
                MSG_SIZE,
                STARTUP_UART_TIMEOUT_MS);
        __HAL_UART_CLEAR_OREFLAG(&huart4);
        setup_blocking_rx(0u);
        imu_accel_ready = imu_rot_ready = false;
}

static bool drain_lidar_rx(void)
{
        uint32_t started = HAL_GetTick();
        while (HAL_GetTick() - started < STARTUP_UART_TIMEOUT_MS) {
                uint8_t           byte;
                HAL_StatusTypeDef status =
                        HAL_UART_Receive(&huart4, &byte, 1u, STARTUP_RX_IDLE_MS);
                if (status == HAL_TIMEOUT)
                        return true;
                if (status != HAL_OK)
                        return false;
        }
        return false;
}

bool startup_initialize_sensors(uint8_t* tx_frame)
{
        // 停止コマンドの後に残ったscanの末尾も、応答バッファへ混ぜない。
        if (!drain_lidar_rx())
                return false;

        return request_and_forward_lidar_message(
                       tx_frame,
                       MSG_TYPE_RX_SYS_INFO,
                       SYS_PACKET_DEVICE_INFO_CONTENT_SIZE,
                       SYS_TYPE_CODE_DEVICE_INFO) &&
               request_and_forward_lidar_message(
                       tx_frame,
                       MSG_TYPE_RX_HEALTH,
                       SYS_PACKET_HEALTH_CONTENT_SIZE,
                       SYS_TYPE_CODE_HEALTH) &&
               imu_setup();
}

bool startup_start_scan(void)
{
        // scanコマンドより先にDMAを動かして、descriptorと最初の点を受信する。
        setup_nonblocking_rx();
        if (HAL_UART_Receive_DMA(&huart4, RX_BUF->_buf, CORE_RX_BUF_SIZE) != HAL_OK)
                return false;
        if (HAL_UART_Transmit(
                    &huart4,
                    MSG[MSG_TYPE_SCAN],
                    MSG_SIZE,
                    STARTUP_UART_TIMEOUT_MS) != HAL_OK)
                return false;

        // read_byte()の2byte分の余裕も揃えてから、固定長descriptorを解析する。
        uint32_t started = HAL_GetTick();
        while (CORE_RX_BUF_SIZE - *RX_BUF->remain_bytes < SYS_PACKET_META_SIZE + 2u) {
                if (HAL_GetTick() - started >= STARTUP_UART_TIMEOUT_MS)
                        return false;
        }

        if (RX_BUF->_buf[0] != SYS_PACKET_HEADER_MSB ||
            RX_BUF->_buf[1] != SYS_PACKET_HEADER_LSB)
                return false;

        ParserMeta meta = {0};
        return read_meta(&meta) != NULL && meta.res_mode == SYS_RES_MODE_CONTINUOUS &&
               meta.type_code == SYS_TYPE_CODE_SCAN;
}
