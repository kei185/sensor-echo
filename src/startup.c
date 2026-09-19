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

static const char INITIALIZING_MESSAGE[] = "[SENSOR-ECHO] INITIALIZING\r\n";
static const char READY_MESSAGE[]        = "[SENSOR-ECHO] READY\r\n";
static const char FAILURE_MESSAGE[]      = "[SENSOR-ECHO] STARTUP FAILED\r\n";

static bool send_pc_system_message(uint8_t* tx_frame, const char* message)
{
        const size_t payload_length = strlen(message);
        if (payload_length > CORE_TX_BUF_SIZE - TX_FRAME_HEADER_SIZE)
                return false;

        memcpy(tx_frame + TX_FRAME_HEADER_SIZE, message, payload_length);
        size_t frame_length = tx_frame_write_header(
                tx_frame,
                CORE_TX_BUF_SIZE,
                (uint16_t)payload_length,
                FRAME_TYPE_SYS,
                HAL_GetTick());

        return frame_length > 0u && HAL_UART_Transmit(
                                            &huart2,
                                            tx_frame,
                                            (uint16_t)frame_length,
                                            STARTUP_UART_TIMEOUT_MS) == HAL_OK;
}

static bool is_expected_lidar_reply(
        const uint8_t* reply,
        size_t         reply_length,
        uint32_t       content_length,
        SysTypeCode    type)
{
        // Validate the fixed meta header before handing the bytes to the stream parser.
        if (reply_length != SYS_PACKET_META_SIZE + content_length)
                return false;

        uint32_t length_mode = (uint32_t)reply[2] | ((uint32_t)reply[3] << 8u) |
                               ((uint32_t)reply[4] << 16u) | ((uint32_t)reply[5] << 24u);

        return reply[0] == SYS_PACKET_HEADER_MSB && reply[1] == SYS_PACKET_HEADER_LSB &&
               (length_mode & 0x3fffffffu) == content_length &&
               (length_mode >> 30u) == SYS_RES_MODE_SINGLE && reply[6] == (uint8_t)type;
}

static bool request_and_forward_lidar_reply(
        uint8_t* tx_frame, MsgType command, uint32_t content_length, SysTypeCode type)
{
        uint8_t reply[SYS_PACKET_DEVICE_INFO_FRAME_SIZE] = {0};
        size_t  reply_length = SYS_PACKET_META_SIZE + content_length;

        if (HAL_UART_Transmit(&huart4, MSG[command], MSG_SIZE, STARTUP_UART_TIMEOUT_MS) !=
                    HAL_OK ||
            HAL_UART_Receive(
                    &huart4,
                    reply,
                    (uint16_t)reply_length,
                    STARTUP_UART_TIMEOUT_MS) != HAL_OK ||
            !is_expected_lidar_reply(reply, reply_length, content_length, type) ||
            !load_blocking_rx(reply, reply_length))
                return false;

        size_t frame_length = translate((int8_t*)tx_frame);
        return frame_length > 0u && HAL_UART_Transmit(
                                            &huart2,
                                            tx_frame,
                                            (uint16_t)frame_length,
                                            STARTUP_UART_TIMEOUT_MS) == HAL_OK;
}

static bool wait_for_start_scan(void)
{
        uint8_t command[PC_COMMAND_SIZE];

        // Startup remains blocking until the PC sends the exact two-byte start command.
        while (true) {
                if (HAL_UART_Receive(&huart2, command, PC_COMMAND_SIZE, HAL_MAX_DELAY) !=
                    HAL_OK)
                        return false;

                if (memcmp(command,
                           PC_COMMANDS[PC_COMMAND_START_SCAN],
                           PC_COMMAND_SIZE) == 0)
                        return true;
        }
}

static bool fail_startup(uint8_t* tx_frame)
{
        (void)send_pc_system_message(tx_frame, FAILURE_MESSAGE);
        return false;
}

bool run_startup_sequence(void)
{
        // Startup owns this free slot until all blocking PC transmissions finish.
        TxBufSlot* tx_slot = get_empty_buf();
        if (tx_slot == NULL)
                return false;
        uint8_t* tx_frame = (uint8_t*)tx_slot->_buf;

        if (!send_pc_system_message(tx_frame, INITIALIZING_MESSAGE))
                return false;

        if (!request_and_forward_lidar_reply(
                    tx_frame,
                    MSG_TYPE_RX_SYS_INFO,
                    SYS_PACKET_DEVICE_INFO_CONTENT_SIZE,
                    SYS_TYPE_CODE_DEVICE_INFO))
                return fail_startup(tx_frame);

        if (!request_and_forward_lidar_reply(
                    tx_frame,
                    MSG_TYPE_RX_HEALTH,
                    SYS_PACKET_HEALTH_CONTENT_SIZE,
                    SYS_TYPE_CODE_HEALTH))
                return fail_startup(tx_frame);

        if (!send_pc_system_message(tx_frame, READY_MESSAGE) || !wait_for_start_scan())
                return fail_startup(tx_frame);

        // Arm RX before the scan command so the first scan bytes cannot be lost.
        if (!start_lidar_rx_dma() || HAL_UART_Transmit(
                                             &huart4,
                                             MSG[MSG_TYPE_SCAN],
                                             MSG_SIZE,
                                             STARTUP_UART_TIMEOUT_MS) != HAL_OK)
                return fail_startup(tx_frame);

        return true;
}
