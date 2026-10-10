#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "startup.h"
#include "imu/handler.h"
#include "lidar/core.h"
#include "lidar/parser/meta.h"
#include "lidar/sys.h"
#include "lidar/translate.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
bool volatile imu_accel_ready;
bool volatile imu_rot_ready;

typedef enum
{
        EVENT_ABORT,
        EVENT_STOP,
        EVENT_FLUSH,
        EVENT_RESET_RX,
        EVENT_DEVICE_REQUEST,
        EVENT_DEVICE_FORWARD,
        EVENT_HEALTH_REQUEST,
        EVENT_HEALTH_FORWARD,
        EVENT_IMU_SETUP,
        EVENT_DMA_START,
        EVENT_SCAN_REQUEST,
        EVENT_DESCRIPTOR_READ,
} Event;

static Event             events[16];
static size_t            event_count;
static uint32_t          tick;
static uint8_t           rx_storage[CORE_RX_BUF_SIZE];
static uint32_t          remain_bytes;
static RxBuf             rx_buf = {.remain_bytes = &remain_bytes, ._buf = rx_storage};
const RxBuf* const       RX_BUF = &rx_buf;
static uint8_t           tx_frame[CORE_TX_BUF_SIZE];
static MsgType           last_request;
static bool              invalid_health;
static bool              imu_ok;
static HAL_StatusTypeDef dma_status;
static HAL_StatusTypeDef scan_status;
static HAL_StatusTypeDef reply_status;
static HAL_StatusTypeDef drain_status;
static uint16_t          scan_received_size;
static bool              valid_scan_marker;
static SysResMode        scan_mode;
static SysTypeCode       scan_type;

static void record(Event event)
{
        assert(event_count < sizeof(events) / sizeof(events[0]));
        events[event_count++] = event;
}

static void reset_test_state(void)
{
        memset(rx_storage, 0, sizeof(rx_storage));
        event_count = tick = 0u;
        rx_buf.read_idx    = 0;
        rx_buf.lap         = 0u;
        remain_bytes       = CORE_RX_BUF_SIZE;
        invalid_health     = false;
        imu_ok             = true;
        dma_status = scan_status = reply_status = HAL_OK;
        drain_status                            = HAL_TIMEOUT;
        scan_received_size                      = SYS_PACKET_META_SIZE + 2u;
        valid_scan_marker                       = true;
        scan_mode                               = SYS_RES_MODE_CONTINUOUS;
        scan_type                               = SYS_TYPE_CODE_SCAN;
        imu_accel_ready = imu_rot_ready = true;
}

uint32_t HAL_GetTick(void) { return tick++; }

HAL_StatusTypeDef HAL_UART_Transmit(
        UART_HandleTypeDef* uart, const uint8_t* data, uint16_t length, uint32_t timeout)
{
        assert(timeout == 100u);
        if (uart == &huart2) {
                // translate_singleの出力をそのままhostへ渡すことを検証する。
                assert(data == tx_frame && length == 1u);
                if (data[0] == SYS_TYPE_CODE_DEVICE_INFO)
                        record(EVENT_DEVICE_FORWARD);
                else {
                        assert(data[0] == SYS_TYPE_CODE_HEALTH);
                        record(EVENT_HEALTH_FORWARD);
                }
                return HAL_OK;
        }
        assert(uart == &huart4 && length == MSG_SIZE);
        if (memcmp(data, MSG[MSG_TYPE_STOP], MSG_SIZE) == 0) {
                record(EVENT_STOP);
                return HAL_OK;
        }
        if (memcmp(data, MSG[MSG_TYPE_RX_SYS_INFO], MSG_SIZE) == 0) {
                last_request = MSG_TYPE_RX_SYS_INFO;
                record(EVENT_DEVICE_REQUEST);
        } else if (memcmp(data, MSG[MSG_TYPE_RX_HEALTH], MSG_SIZE) == 0) {
                last_request = MSG_TYPE_RX_HEALTH;
                record(EVENT_HEALTH_REQUEST);
        } else {
                assert(memcmp(data, MSG[MSG_TYPE_SCAN], MSG_SIZE) == 0);
                // DMAが先に動いていないと、最初のdescriptorを取り逃す。
                assert(event_count > 0u && events[event_count - 1u] == EVENT_DMA_START);
                record(EVENT_SCAN_REQUEST);
                rx_storage[0] = valid_scan_marker ? SYS_PACKET_HEADER_MSB : 0x00u;
                rx_storage[1] = SYS_PACKET_HEADER_LSB;
                remain_bytes  = CORE_RX_BUF_SIZE - scan_received_size;
                return scan_status;
        }
        return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(
        UART_HandleTypeDef* uart, uint8_t* data, uint16_t length, uint32_t timeout)
{
        assert(uart == &huart4);
        if (length == 1u) {
                assert(timeout == 2u);
                *data = 0u;
                return drain_status;
        }
        assert(timeout == 100u);
        assert(data == rx_storage);
        if (reply_status != HAL_OK)
                return reply_status;
        uint32_t content_size = last_request == MSG_TYPE_RX_SYS_INFO
                                        ? SYS_PACKET_DEVICE_INFO_CONTENT_SIZE
                                        : SYS_PACKET_HEALTH_CONTENT_SIZE;
        assert(length == SYS_PACKET_META_SIZE + content_size);
        memset(data, 0, length);
        data[0] = SYS_PACKET_HEADER_MSB;
        data[1] = SYS_PACKET_HEADER_LSB;
        data[2] = (uint8_t)content_size;
        data[6] = last_request == MSG_TYPE_RX_SYS_INFO || invalid_health
                          ? SYS_TYPE_CODE_DEVICE_INFO
                          : SYS_TYPE_CODE_HEALTH;
        return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef* uart)
{
        assert(uart == &huart4);
        record(EVENT_ABORT);
        return HAL_OK;
}

void clear_uart_overrun(UART_HandleTypeDef* uart)
{
        assert(uart == &huart4);
        record(EVENT_FLUSH);
}

void setup_blocking_rx(size_t length)
{
        assert(length <= sizeof(rx_storage));
        rx_buf.read_idx = 0;
        rx_buf.lap      = 0u;
        if (length == 0u)
                record(EVENT_RESET_RX);
}

void setup_nonblocking_rx(void)
{
        rx_buf.read_idx = 0;
        rx_buf.lap      = 0u;
        remain_bytes    = CORE_RX_BUF_SIZE;
}

HAL_StatusTypeDef
HAL_UART_Receive_DMA(UART_HandleTypeDef* uart, uint8_t* data, uint16_t length)
{
        assert(uart == &huart4 && data == rx_storage && length == CORE_RX_BUF_SIZE);
        record(EVENT_DMA_START);
        return dma_status;
}

size_t translate_single(uint8_t* to)
{
        assert(to == tx_frame);
        to[0] = rx_storage[6];
        return 1u;
}

ParserMeta* read_meta(ParserMeta* meta)
{
        // 7byteのdescriptorだけでは不足。read_byteの2byte余裕まで待たせる。
        assert(CORE_RX_BUF_SIZE - remain_bytes >= SYS_PACKET_META_SIZE + 2u);
        record(EVENT_DESCRIPTOR_READ);
        *meta = (ParserMeta){.res_mode = scan_mode, .type_code = scan_type};
        return meta;
}

bool imu_setup(void)
{
        record(EVENT_IMU_SETUP);
        return imu_ok;
}

static void initialize_requests_and_forwards_both_sensor_messages(void)
{
        reset_test_state();
        assert(startup_initialize_sensors(tx_frame));
        const Event expected[] = {
                EVENT_DEVICE_REQUEST,
                EVENT_DEVICE_FORWARD,
                EVENT_HEALTH_REQUEST,
                EVENT_HEALTH_FORWARD,
                EVENT_IMU_SETUP,
        };
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
}

static void initialization_rejects_bad_replies_and_peripheral_failures(void)
{
        reset_test_state();
        invalid_health = true;
        assert(!startup_initialize_sensors(tx_frame));
        assert(event_count == 3u && events[2] == EVENT_HEALTH_REQUEST);

        reset_test_state();
        reply_status = HAL_TIMEOUT;
        assert(!startup_initialize_sensors(tx_frame));
        assert(event_count == 1u && events[0] == EVENT_DEVICE_REQUEST);

        reset_test_state();
        imu_ok = false;
        assert(!startup_initialize_sensors(tx_frame));
        assert(events[event_count - 1u] == EVENT_IMU_SETUP);
}

static void scan_tail_drain_has_a_deadline(void)
{
        reset_test_state();
        // STOP後もバイトが流れ続けたら100ms以内で諦め、要求を送らない。
        drain_status = HAL_OK;
        assert(!startup_initialize_sensors(tx_frame));
        assert(event_count == 0u && tick <= 102u);
}

static void cancellation_stops_dma_and_clears_receive_state(void)
{
        reset_test_state();
        rx_buf.read_idx = 123;
        rx_buf.lap      = 1u;
        startup_cancel();
        const Event expected[] = {EVENT_ABORT, EVENT_STOP, EVENT_FLUSH, EVENT_RESET_RX};
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
        assert(rx_buf.read_idx == 0 && rx_buf.lap == 0u);
        assert(!imu_accel_ready && !imu_rot_ready);
}

static void scan_starts_dma_before_request_and_reads_descriptor(void)
{
        reset_test_state();
        assert(startup_start_scan());
        const Event expected[] = {EVENT_DMA_START,
                                  EVENT_SCAN_REQUEST,
                                  EVENT_DESCRIPTOR_READ};
        assert(event_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(events, expected, sizeof(expected)) == 0);
}

static void scan_rejects_bad_or_missing_descriptors(void)
{
        for (unsigned failure = 0u; failure < 6u; ++failure) {
                reset_test_state();
                if (failure == 0u)
                        dma_status = HAL_ERROR;
                if (failure == 1u)
                        scan_status = HAL_ERROR;
                // descriptorだけ到着して最初の点が来ない場合もbusy waitし続けない。
                if (failure == 2u)
                        scan_received_size = SYS_PACKET_META_SIZE;
                if (failure == 3u)
                        valid_scan_marker = false;
                if (failure == 4u)
                        scan_mode = SYS_RES_MODE_SINGLE;
                if (failure == 5u)
                        scan_type = SYS_TYPE_CODE_HEALTH;
                assert(!startup_start_scan());
                if (failure < 4u)
                        assert(events[event_count - 1u] != EVENT_DESCRIPTOR_READ);
        }
}

int main(void)
{
        initialize_requests_and_forwards_both_sensor_messages();
        initialization_rejects_bad_replies_and_peripheral_failures();
        scan_tail_drain_has_a_deadline();
        cancellation_stops_dma_and_clears_receive_state();
        scan_starts_dma_before_request_and_reads_descriptor();
        scan_rejects_bad_or_missing_descriptors();
        return 0;
}
