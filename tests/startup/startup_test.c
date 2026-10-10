#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "startup.h"
#include "time_sync.h"
#include "lidar/core.h"
#include "lidar/sys.h"
#include "lidar/translate.h"
#include "tx/frame.h"
#include "stm32f4xx_hal_uart.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
GPIO_TypeDef       initial_handshake_failed_port;

typedef enum
{
        EVENT_HOST_HANDSHAKE_ACK,
        EVENT_HOST_INITIALIZING,
        EVENT_LIDAR_DEVICE_REQUEST,
        EVENT_HOST_DEVICE_INFO,
        EVENT_LIDAR_HEALTH_REQUEST,
        EVENT_HOST_HEALTH_STATUS,
        EVENT_HOST_READY,
        EVENT_TIME_SYNC,
        EVENT_RTC_SET,
        EVENT_TIME_SYNC_REPORT,
        EVENT_DMA_STARTED,
        EVENT_LIDAR_SCAN_REQUEST,
        EVENT_HOST_START_SCAN_ACK,
        EVENT_FAILURE_PIN_SET,
        EVENT_HOST_FAILURE,
} Event;

#define EVENT_CAPACITY 16u

static Event     events[EVENT_CAPACITY];
static size_t    event_count;
static uint32_t  lidar_receive_count;
static uint32_t  host_receive_count;
static bool      malformed_health_reply;
static bool      handshake_receive_failed;
static bool      handshake_ack_failed;
static bool      time_sync_failed;
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
        event_count              = 0u;
        lidar_receive_count      = 0u;
        host_receive_count       = 0u;
        malformed_health_reply   = false;
        handshake_receive_failed = false;
        handshake_ack_failed     = false;
        time_sync_failed         = false;
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
        if (payload_equals(data, length, "HANDSHAKE ACK")) {
                assert(data[5] == FRAME_TYPE_HANDSHAKE_ACK);
                record_event(EVENT_HOST_HANDSHAKE_ACK);
                if (handshake_ack_failed)
                        return HAL_ERROR;
        } else if (payload_equals(data, length, "INITIALIZING")) {
                assert(data[5] == FRAME_TYPE_INITIALIZING);
                record_event(EVENT_HOST_INITIALIZING);
        } else if (payload_equals(data, length, "READY")) {
                assert(data[5] == FRAME_TYPE_READY);
                record_event(EVENT_HOST_READY);
        } else if (payload_equals(data, length, "STARTUP FAILED")) {
                assert(data[5] == FRAME_TYPE_STARTUP_FAILED);
                record_event(EVENT_HOST_FAILURE);
        } else if (payload_equals(data, length, "START SCAN ACK")) {
                assert(data[5] == FRAME_TYPE_START_SCAN_ACK);
                record_event(EVENT_HOST_START_SCAN_ACK);
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
        if (handshake_receive_failed)
                return HAL_ERROR;

        uint32_t receive_index = host_receive_count++;
        if (receive_index == 0u || receive_index == 2u) {
                // handshakeとstart scanの前で、無関係なコマンドを無視する。
                const uint8_t unsupported_command[HOST_COMMAND_SIZE] = {0xAA, 0xA1};
                memcpy(data, unsupported_command, HOST_COMMAND_SIZE);
        } else if (receive_index == 1u) {
                memcpy(data, HOST_COMMANDS[HOST_COMMAND_HANDSHAKE], HOST_COMMAND_SIZE);
        } else {
                assert(receive_index == 3u);
                memcpy(data, HOST_COMMANDS[HOST_COMMAND_START_SCAN], HOST_COMMAND_SIZE);
        }
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

size_t translate_single(uint8_t* to)
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

// 時刻同期本体は変更せず、startupから従来の順に呼ばれることを確認する。
bool time_sync_handle_start(uint8_t* tx_frame, TimeSyncSession* session)
{
        assert(tx_frame == startup_slot._buf);
        assert(!session->complete);
        record_event(EVENT_TIME_SYNC);
        if (time_sync_failed)
                return false;
        session->complete = true;
        return true;
}

bool time_sync_set_rtc(const TimeSyncSession* session)
{
        assert(session->complete);
        record_event(EVENT_RTC_SET);
        return true;
}

bool time_sync_send_report_if_ready(uint8_t* tx_frame, TimeSyncSession* session)
{
        assert(tx_frame == startup_slot._buf);
        assert(session->complete);
        record_event(EVENT_TIME_SYNC_REPORT);
        *session = (TimeSyncSession){0};
        return true;
}

static void startup_follows_the_documented_order(void)
{
        // 準備: handshakeとscan開始の前に無関係なコマンドを受ける。
        reset_test_state();

        // 実行
        assert(run_startup_sequence());

        // 検証: handshake ACKの後に既存の起動処理を続け、最後にDMAを開始する。
        const Event expected[] = {
                EVENT_HOST_HANDSHAKE_ACK,
                EVENT_HOST_READY,
                EVENT_TIME_SYNC,
                EVENT_RTC_SET,
                EVENT_TIME_SYNC_REPORT,
                EVENT_HOST_START_SCAN_ACK,
                EVENT_DMA_STARTED,
        };
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
        assert(host_receive_count == 4u);
        // mainでコメントアウト中のセンサー情報取得は再開していない。
        assert(lidar_receive_count == 0u);
}

static void failed_handshake_receive_stops_startup(void)
{
        // handshakeを受け取れなければ、READYや時刻同期へ進まない。
        reset_test_state();
        handshake_receive_failed = true;
        assert(!run_startup_sequence());
        const Event expected[] = {
                EVENT_FAILURE_PIN_SET,
                EVENT_HOST_FAILURE,
        };
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
        assert(host_receive_count == 0u);
}

static void failed_handshake_ack_stops_startup(void)
{
        // ACKの送信が失敗しても既存の起動処理へ進まない。
        reset_test_state();
        handshake_ack_failed = true;
        assert(!run_startup_sequence());
        const Event expected[] = {
                EVENT_HOST_HANDSHAKE_ACK,
                EVENT_FAILURE_PIN_SET,
                EVENT_HOST_FAILURE,
        };
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
        assert(host_receive_count == 2u);
}

static void failed_time_sync_returns_to_handshake_on_retry(void)
{
        // 時刻同期の失敗をstartupがfalseで返し、再実行でhandshakeを待つ。
        reset_test_state();
        time_sync_failed = true;
        assert(!run_startup_sequence());
        const Event expected[] = {
                EVENT_HOST_HANDSHAKE_ACK,
                EVENT_HOST_READY,
                EVENT_TIME_SYNC,
                EVENT_FAILURE_PIN_SET,
                EVENT_HOST_FAILURE,
        };
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);

        // protocolと同様に、失敗後にstartupをもう一度呼ぶ。
        event_count        = 0u;
        host_receive_count = 0u;
        time_sync_failed   = false;
        assert(run_startup_sequence());
        assert(events[0] == EVENT_HOST_HANDSHAKE_ACK);
        assert(events[event_count - 1u] == EVENT_DMA_STARTED);
        assert(host_receive_count == 4u);
}

int main(void)
{
        startup_follows_the_documented_order();
        failed_handshake_receive_stops_startup();
        failed_handshake_ack_stops_startup();
        failed_time_sync_returns_to_handshake_on_retry();
        return 0;
}
