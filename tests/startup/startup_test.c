#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "startup.h"
#include "lidar/core.h"
#include "lidar/sys.h"
#include "tx/frame.h"
#include "tx/header.h"
#include "stm32f4xx_hal_uart.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
GPIO_TypeDef       initial_handshake_failed_port;

typedef enum
{
        EVENT_HOST_INITIALIZING,
        EVENT_LIDAR_DEVICE_REQUEST,
        EVENT_HOST_DEVICE_INFO,
        EVENT_LIDAR_HEALTH_REQUEST,
        EVENT_HOST_HEALTH_STATUS,
        EVENT_HOST_READY,
        EVENT_DMA_STARTED,
        EVENT_LIDAR_SCAN_REQUEST,
        EVENT_FAILURE_PIN_SET,
        EVENT_HOST_FAILURE,
} Event;

#define EVENT_CAPACITY 16u

static Event     events[EVENT_CAPACITY];
static size_t    event_count;
static uint32_t  lidar_receive_count;
static uint32_t  host_receive_count;
static bool      malformed_health_reply;
static TxBufSlot startup_slot;
static uint8_t   rx_storage[SYS_PACKET_DEVICE_INFO_FRAME_SIZE];
static uint32_t  rx_remain_bytes;
static RxBuf     rx_buf = {
        .remain_bytes = &rx_remain_bytes,
        ._buf         = rx_storage,
};
const RxBuf* const RX_BUF = &rx_buf;

static void record_event(Event event)
{
        assert(event_count < EVENT_CAPACITY);
        events[event_count++] = event;
}

static bool
payload_equals(const uint8_t* frame, uint16_t frame_length, const char* message)
{
        size_t message_length = strlen(message);
        return frame_length == TX_FRAME_HEADER_SIZE + message_length &&
               memcmp(frame + TX_FRAME_HEADER_SIZE, message, message_length) == 0;
}

static void reset_test_state(void)
{
        memset(events, 0, sizeof(events));
        event_count            = 0u;
        lidar_receive_count    = 0u;
        host_receive_count     = 0u;
        malformed_health_reply = false;
        memset(&startup_slot, 0, sizeof(startup_slot));
        memset(rx_storage, 0, sizeof(rx_storage));
}

static void
write_lidar_reply(uint8_t* data, uint16_t length, uint8_t type, uint32_t content_length)
{
        assert(length == SYS_PACKET_META_SIZE + content_length);
        memset(data, 0, length);
        data[0] = SYS_PACKET_HEADER_MSB;
        data[1] = SYS_PACKET_HEADER_LSB;
        data[2] = (uint8_t)content_length;
        data[3] = (uint8_t)(content_length >> 8u);
        data[4] = (uint8_t)(content_length >> 16u);
        data[5] = (uint8_t)(content_length >> 24u);
        data[6] = type;
}

HAL_StatusTypeDef HAL_UART_Transmit(
        UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length, uint32_t timeout)
{
        (void)timeout;

        if (huart == &huart4) {
                assert(length == MSG_SIZE);
                if (memcmp(data, MSG[MSG_TYPE_RX_SYS_INFO], MSG_SIZE) == 0)
                        record_event(EVENT_LIDAR_DEVICE_REQUEST);
                else if (memcmp(data, MSG[MSG_TYPE_RX_HEALTH], MSG_SIZE) == 0)
                        record_event(EVENT_LIDAR_HEALTH_REQUEST);
                else {
                        assert(memcmp(data, MSG[MSG_TYPE_SCAN], MSG_SIZE) == 0);
                        record_event(EVENT_LIDAR_SCAN_REQUEST);
                }
                return HAL_OK;
        }

        assert(huart == &huart2);
        if (payload_equals(data, length, "INITIALIZING\r\n")) {
                assert(data[5] == FRAME_TYPE_INITIALIZING);
                record_event(EVENT_HOST_INITIALIZING);
        } else if (payload_equals(data, length, "READY\r\n")) {
                assert(data[5] == FRAME_TYPE_READY);
                record_event(EVENT_HOST_READY);
        } else if (payload_equals(data, length, "STARTUP FAILED\r\n")) {
                assert(data[5] == FRAME_TYPE_STARTUP_FAILED);
                record_event(EVENT_HOST_FAILURE);
        } else {
                assert(length == 1u);
                if (data[0] == SYS_TYPE_CODE_DEVICE_INFO)
                        record_event(EVENT_HOST_DEVICE_INFO);
                else {
                        assert(data[0] == SYS_TYPE_CODE_HEALTH);
                        record_event(EVENT_HOST_HEALTH_STATUS);
                }
        }
        return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(
        UART_HandleTypeDef* huart, uint8_t* data, uint16_t length, uint32_t timeout)
{
        if (huart == &huart4) {
                assert(timeout != HAL_MAX_DELAY);
                if (lidar_receive_count++ == 0u) {
                        write_lidar_reply(
                                data,
                                length,
                                SYS_TYPE_CODE_DEVICE_INFO,
                                SYS_PACKET_DEVICE_INFO_CONTENT_SIZE);
                } else {
                        write_lidar_reply(
                                data,
                                length,
                                malformed_health_reply ? SYS_TYPE_CODE_DEVICE_INFO
                                                       : SYS_TYPE_CODE_HEALTH,
                                SYS_PACKET_HEALTH_CONTENT_SIZE);
                }
                return HAL_OK;
        }

        assert(huart == &huart2);
        assert(length == HOST_COMMAND_SIZE);
        assert(timeout == HAL_MAX_DELAY);
        if (host_receive_count++ == 0u)
                memcpy(data, HOST_COMMANDS[HOST_COMMAND_GET_STATUS], HOST_COMMAND_SIZE);
        else
                memcpy(data, HOST_COMMANDS[HOST_COMMAND_START_SCAN], HOST_COMMAND_SIZE);
        return HAL_OK;
}

uint32_t HAL_GetTick(void) { return 1234u; }

TxBufSlot* get_empty_buf(void) { return &startup_slot; }

void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state)
{
        assert(port == INITIAL_HANDSHAKE_FAILED_GPIO_Port);
        assert(pin == INITIAL_HANDSHAKE_FAILED_Pin);
        assert(state == GPIO_PIN_SET);
        record_event(EVENT_FAILURE_PIN_SET);
}

void setup_blocking_rx(size_t length)
{
        assert(length <= sizeof(rx_storage));
        rx_buf.read_idx = 0u;
        rx_buf.lap      = 0u;
}

size_t translate(uint8_t* to)
{
        assert(to != NULL);
        uint8_t reply_type = RX_BUF->_buf[6];
        assert(reply_type == SYS_TYPE_CODE_DEVICE_INFO ||
               reply_type == SYS_TYPE_CODE_HEALTH);
        to[0] = reply_type;
        return 1u;
}

void setup_nonblocking_rx(void)
{
        rx_buf.read_idx = 0u;
        rx_buf.lap      = 0u;
}

HAL_StatusTypeDef
HAL_UART_Receive_DMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t length)
{
        assert(huart == &huart4);
        assert(data == RX_BUF->_buf);
        assert(length == CORE_RX_BUF_SIZE);
        record_event(EVENT_DMA_STARTED);
        return HAL_OK;
}

static void startup_follows_the_documented_order(void)
{
        // 準備: hostから無関係なコマンドを1回受けてからscan開始を受ける。
        reset_test_state();

        // 実行
        assert(run_startup_sequence());

        // 検証: DMAはLiDARへscan開始を送る前に開始する。
        const Event expected[] = {
                EVENT_HOST_INITIALIZING,
                EVENT_LIDAR_DEVICE_REQUEST,
                EVENT_HOST_DEVICE_INFO,
                EVENT_LIDAR_HEALTH_REQUEST,
                EVENT_HOST_HEALTH_STATUS,
                EVENT_HOST_READY,
                EVENT_DMA_STARTED,
                EVENT_LIDAR_SCAN_REQUEST,
        };
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
        assert(host_receive_count == 2u);
}

static void invalid_health_reply_stops_startup(void)
{
        // 準備: health応答のtypeだけを不正にする。
        reset_test_state();
        malformed_health_reply = true;

        // 実行
        assert(!run_startup_sequence());

        // 検証: 失敗をhostへ通知し、ready・DMA・scan開始へ進まない。
        const Event expected[] = {
                EVENT_HOST_INITIALIZING,
                EVENT_LIDAR_DEVICE_REQUEST,
                EVENT_HOST_DEVICE_INFO,
                EVENT_LIDAR_HEALTH_REQUEST,
                EVENT_FAILURE_PIN_SET,
                EVENT_HOST_FAILURE,
        };
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
        assert(host_receive_count == 0u);
}

int main(void)
{
        startup_follows_the_documented_order();
        invalid_health_reply_stops_startup();
        return 0;
}
