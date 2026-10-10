#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "startup.h"
#include "time_sync.h"
#include "imu/handler.h"
#include "lidar/core.h"
#include "lidar/parser/meta.h"
#include "lidar/sys.h"
#include "lidar/translate.h"
#include "tx/frame.h"
#include "tx/header.h"

#define TIME_SYNC_HOST_TIME_SIZE 8u
#define TIME_SYNC_PAYLOAD_SIZE   16u

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
RTC_HandleTypeDef  hrtc = {.Init = {.SynchPrediv = 255u}};
GPIO_TypeDef       scanning_port;
GPIO_TypeDef       initial_handshake_failed_port;
bool volatile imu_accel_ready;
bool volatile imu_rot_ready;

static TxBufSlot         slot;
static uint8_t           input[128];
static size_t            input_length;
static size_t            input_position;
static FrameType         sent_types[64];
static size_t            sent_count;
static uint32_t          fake_tick;
static uint32_t          last_receive_timeout;
static size_t            cancel_count;
static size_t            initialize_count;
static size_t            scan_count;
static size_t            abort_count;
static size_t            flush_count;
static size_t            rtc_count;
static GPIO_PinState     scanning;
static GPIO_PinState     failed;
static bool              slot_available;
static bool              initialize_ok;
static bool              scan_ok;
static bool              rtc_ok;
static FrameType         failed_transmit_type;
static uint8_t           rx_storage[CORE_RX_BUF_SIZE];
static uint32_t          remain_bytes;
static RxBuf             rx_buf = {.remain_bytes = &remain_bytes, ._buf = rx_storage};
const RxBuf* const       RX_BUF = &rx_buf;
static MsgType           last_lidar_command;
static bool              invalid_health;
static bool              imu_ok;
static bool              dma_ok;
static bool              valid_scan_marker;
static SysResMode        scan_mode;
static SysTypeCode       scan_type;
static uint16_t          scan_received_size;
static HAL_StatusTypeDef drain_status;
static size_t            imu_count;
static size_t            descriptor_count;
static bool              dma_started;

static void reset_test_state(void)
{
        memset(&slot, 0, sizeof(slot));
        memset(input, 0, sizeof(input));
        input_length = input_position = sent_count = 0u;
        cancel_count = initialize_count = scan_count = 0u;
        abort_count = flush_count = rtc_count = 0u;
        fake_tick                             = 1000u;
        last_receive_timeout                  = 0u;
        scanning = failed = GPIO_PIN_RESET;
        slot_available = initialize_ok = scan_ok = rtc_ok = true;
        failed_transmit_type                              = 0;
        memset(rx_storage, 0, sizeof(rx_storage));
        rx_buf.read_idx = 0;
        rx_buf.lap      = 0u;
        remain_bytes    = CORE_RX_BUF_SIZE;
        invalid_health  = false;
        imu_ok = dma_ok = valid_scan_marker = true;
        dma_started                         = false;
        scan_mode                           = SYS_RES_MODE_CONTINUOUS;
        scan_type                           = SYS_TYPE_CODE_SCAN;
        scan_received_size                  = SYS_PACKET_META_SIZE + 2u;
        drain_status                        = HAL_TIMEOUT;
        imu_count = descriptor_count = 0u;
}

static void append_command(HostCommand command)
{
        assert(input_length + HOST_COMMAND_SIZE <= sizeof(input));
        memcpy(input + input_length, HOST_COMMANDS[command], HOST_COMMAND_SIZE);
        input_length += HOST_COMMAND_SIZE;
}

static void append_time(uint64_t ack_time, uint64_t send_time)
{
        append_command(HOST_COMMAND_TIME);
        assert(input_length + TIME_SYNC_PAYLOAD_SIZE <= sizeof(input));
        for (size_t i = 0u; i < TIME_SYNC_HOST_TIME_SIZE; ++i) {
                input[input_length + i] = (uint8_t)(ack_time >> (i * 8u));
                input[input_length + TIME_SYNC_HOST_TIME_SIZE + i] =
                        (uint8_t)(send_time >> (i * 8u));
        }
        input_length += TIME_SYNC_PAYLOAD_SIZE;
}

static void append_valid_startup(void)
{
        append_command(HOST_COMMAND_HANDSHAKE);
        append_command(HOST_COMMAND_TIME_SYNC_START);
        append_time(UINT64_C(1791160123000), UINT64_C(1791160123001));
        append_command(HOST_COMMAND_START_SCAN);
}

uint32_t HAL_GetTick(void) { return fake_tick++; }

HAL_StatusTypeDef HAL_UART_Receive(
        UART_HandleTypeDef* uart, uint8_t* data, uint16_t length, uint32_t timeout)
{
        if (uart == &huart4) {
                if (length == 1u) {
                        assert(timeout == 2u);
                        *data = 0u;
                        return drain_status;
                }
                assert(timeout == 100u && data == rx_storage);
                if (!initialize_ok)
                        return HAL_ERROR;
                uint32_t content_size = last_lidar_command == MSG_TYPE_RX_SYS_INFO
                                                ? SYS_PACKET_DEVICE_INFO_CONTENT_SIZE
                                                : SYS_PACKET_HEALTH_CONTENT_SIZE;
                assert(length == SYS_PACKET_META_SIZE + content_size);
                memset(data, 0, length);
                data[0] = SYS_PACKET_HEADER_MSB;
                data[1] = SYS_PACKET_HEADER_LSB;
                data[2] = (uint8_t)content_size;
                data[6] = last_lidar_command == MSG_TYPE_RX_SYS_INFO || invalid_health
                                  ? SYS_TYPE_CODE_DEVICE_INFO
                                  : SYS_TYPE_CODE_HEALTH;
                return HAL_OK;
        }
        assert(uart == &huart2);
        last_receive_timeout = timeout;
        // 2byte目と時刻payloadは必ず期限付き。入力不足は待ち時間切れにする。
        if (length == TIME_SYNC_PAYLOAD_SIZE)
                assert(timeout == HOST_HANDSHAKE_RETRY_INTERVAL_MS);
        else {
                assert(length == 1u);
                assert(timeout == HAL_MAX_DELAY || timeout == 100u ||
                       timeout == HOST_HANDSHAKE_RETRY_INTERVAL_MS);
        }
        if (input_position + length > input_length)
                return HAL_TIMEOUT;
        memcpy(data, input + input_position, length);
        input_position += length;
        return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(
        UART_HandleTypeDef* uart, const uint8_t* data, uint16_t length, uint32_t timeout)
{
        assert(timeout == 100u);
        if (uart == &huart4) {
                assert(length == MSG_SIZE);
                if (memcmp(data, MSG[MSG_TYPE_STOP], MSG_SIZE) == 0)
                        return HAL_OK;
                if (memcmp(data, MSG[MSG_TYPE_RX_SYS_INFO], MSG_SIZE) == 0) {
                        assert(sent_types[sent_count - 1u] == FRAME_TYPE_INITIALIZING);
                        ++initialize_count;
                        last_lidar_command = MSG_TYPE_RX_SYS_INFO;
                } else if (memcmp(data, MSG[MSG_TYPE_RX_HEALTH], MSG_SIZE) == 0) {
                        assert(sent_types[sent_count - 1u] == FRAME_TYPE_DEVICE_INFO);
                        last_lidar_command = MSG_TYPE_RX_HEALTH;
                } else {
                        assert(memcmp(data, MSG[MSG_TYPE_SCAN], MSG_SIZE) == 0);
                        // ACK送信完了後にDMAを動かしてからscanを要求する。
                        assert(dma_started);
                        assert(sent_types[sent_count - 1u] == FRAME_TYPE_START_SCAN_ACK);
                        ++scan_count;
                        rx_storage[0] = valid_scan_marker ? SYS_PACKET_HEADER_MSB : 0u;
                        rx_storage[1] = SYS_PACKET_HEADER_LSB;
                        remain_bytes  = CORE_RX_BUF_SIZE - scan_received_size;
                        return scan_ok ? HAL_OK : HAL_ERROR;
                }
                return HAL_OK;
        }
        assert(uart == &huart2);
        assert(length >= TX_FRAME_HEADER_SIZE);
        assert(data[0] == 0xAAu && data[1] == 0x55u);
        assert(((uint16_t)data[3] << 8u | data[2]) == length - TX_FRAME_HEADER_SIZE);
        assert(sent_count < sizeof(sent_types) / sizeof(sent_types[0]));
        FrameType type           = (FrameType)data[5];
        sent_types[sent_count++] = type;
        const char* message      = NULL;
        switch (type) {
                case FRAME_TYPE_HANDSHAKE_ACK:
                        message = FRAME_MESSAGE_HANDSHAKE_ACK;
                        break;
                case FRAME_TYPE_INITIALIZING:
                        message = FRAME_MESSAGE_INITIALIZING;
                        break;
                case FRAME_TYPE_READY:
                        message = FRAME_MESSAGE_READY;
                        break;
                case FRAME_TYPE_STARTUP_FAILED:
                        message = FRAME_MESSAGE_STARTUP_FAILED;
                        break;
                case FRAME_TYPE_TIME_SYNC_START_ACK:
                        message = FRAME_MESSAGE_TIME_SYNC_START_ACK;
                        break;
                case FRAME_TYPE_TIME_ACK:
                        message = FRAME_MESSAGE_TIME_ACK;
                        break;
                case FRAME_TYPE_START_SCAN_ACK:
                        message = FRAME_MESSAGE_START_SCAN_ACK;
                        break;
                case FRAME_TYPE_DEVICE_INFO:
                case FRAME_TYPE_HEALTH_STATUS:
                case FRAME_TYPE_TIME_SYNC_REPORT:
                        break;
                default:
                        assert(false);
        }
        if (message != NULL) {
                assert(length == TX_FRAME_HEADER_SIZE + strlen(message));
                assert(memcmp(data + TX_FRAME_HEADER_SIZE, message, strlen(message)) ==
                       0);
        }
        return type == failed_transmit_type ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef* uart)
{
        if (uart == &huart4) {
                ++cancel_count;
                dma_started = false;
        } else {
                assert(uart == &huart2);
                ++abort_count;
        }
        return HAL_OK;
}

void clear_uart_overrun(UART_HandleTypeDef* uart)
{
        if (uart == &huart2)
                ++flush_count;
        else
                assert(uart == &huart4);
}

void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state)
{
        if (port == SCANNING_GPIO_Port) {
                assert(pin == SCANNING_Pin);
                scanning = state;
        } else {
                assert(port == INITIAL_HANDSHAKE_FAILED_GPIO_Port);
                assert(pin == INITIAL_HANDSHAKE_FAILED_Pin);
                failed = state;
        }
}

HAL_StatusTypeDef
HAL_RTC_SetDate(RTC_HandleTypeDef* handle, RTC_DateTypeDef* date, uint32_t format)
{
        assert(handle == &hrtc && format == RTC_FORMAT_BIN);
        // 送ったUnix時刻は2026年。実際のtime_sync.cがUTC変換した値を検証する。
        assert(date->Year == 26u);
        ++rtc_count;
        return rtc_ok ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef
HAL_RTC_SetTime(RTC_HandleTypeDef* handle, RTC_TimeTypeDef* time, uint32_t format)
{
        assert(handle == &hrtc && format == RTC_FORMAT_BIN);
        assert(time->Hours < 24u);
        return HAL_OK;
}

HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(
        RTC_HandleTypeDef* handle, uint32_t add_second, uint32_t subtract)
{
        assert(handle == &hrtc);
        assert(add_second == RTC_SHIFTADD1S_SET && subtract <= 256u);
        return HAL_OK;
}

TxBufSlot* get_empty_buf(void) { return slot_available ? &slot : NULL; }
void       setup_blocking_rx(size_t length)
{
        assert(length <= sizeof(rx_storage));
        rx_buf.read_idx = 0;
        rx_buf.lap      = 0u;
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
        assert(sent_types[sent_count - 1u] == FRAME_TYPE_START_SCAN_ACK);
        dma_started = dma_ok;
        return dma_ok ? HAL_OK : HAL_ERROR;
}

size_t translate_single(uint8_t* to)
{
        assert(to == slot._buf);
        FrameType type           = rx_storage[6] == SYS_TYPE_CODE_DEVICE_INFO
                                           ? FRAME_TYPE_DEVICE_INFO
                                           : FRAME_TYPE_HEALTH_STATUS;
        to[TX_FRAME_HEADER_SIZE] = rx_storage[6];
        return tx_frame_write_header(to, CORE_TX_BUF_SIZE, 1u, type, HAL_GetTick());
}

ParserMeta* read_meta(ParserMeta* meta)
{
        assert(CORE_RX_BUF_SIZE - remain_bytes >= SYS_PACKET_META_SIZE + 2u);
        ++descriptor_count;
        *meta = (ParserMeta){.res_mode = scan_mode, .type_code = scan_type};
        return meta;
}

bool imu_setup(void)
{
        assert(sent_types[sent_count - 1u] == FRAME_TYPE_HEALTH_STATUS);
        ++imu_count;
        return imu_ok;
}

static void assert_failed_startup(void)
{
        assert(!run_startup_sequence());
        assert(scanning == GPIO_PIN_RESET);
        assert(failed == GPIO_PIN_SET);
        assert(abort_count == 1u && flush_count == 1u);
        assert(cancel_count >= 1u);
        assert(!slot.full);
        if (sent_count != 0u)
                assert(sent_types[sent_count - 1u] == FRAME_TYPE_STARTUP_FAILED);
}

static void startup_follows_the_wire_order(void)
{
        reset_test_state();
        // handshake前の古いデータと無関係なコマンドを捨てる。
        input[input_length++] = 0x00u;
        append_command(HOST_COMMAND_START_SCAN);
        append_valid_startup();
        assert(run_startup_sequence());
        const FrameType expected[] = {
                FRAME_TYPE_HANDSHAKE_ACK,
                FRAME_TYPE_INITIALIZING,
                FRAME_TYPE_DEVICE_INFO,
                FRAME_TYPE_HEALTH_STATUS,
                FRAME_TYPE_READY,
                FRAME_TYPE_TIME_SYNC_START_ACK,
                FRAME_TYPE_TIME_ACK,
                FRAME_TYPE_TIME_SYNC_REPORT,
                FRAME_TYPE_START_SCAN_ACK,
        };
        assert(sent_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(sent_types, expected, sizeof(expected)) == 0);
        assert(initialize_count == 1u && scan_count == 1u && rtc_count == 1u);
        assert(imu_count == 1u && descriptor_count == 1u);
        assert(scanning == GPIO_PIN_SET && failed == GPIO_PIN_RESET);
        assert(abort_count == 0u && flush_count == 0u);
        assert(!slot.full);
        assert(input_position == input_length);
}

static void handshake_restarts_each_startup_phase(void)
{
        // READY直後、時刻payload待ち、scan開始待ちのそれぞれでやり直せる。
        for (unsigned phase = 0u; phase < 3u; ++phase) {
                reset_test_state();
                append_command(HOST_COMMAND_HANDSHAKE);
                if (phase >= 1u)
                        append_command(HOST_COMMAND_TIME_SYNC_START);
                if (phase >= 2u)
                        append_time(UINT64_C(1791160123000), UINT64_C(1791160123001));
                append_valid_startup();
                assert(run_startup_sequence());
                assert(initialize_count == 2u && cancel_count == 2u);
                assert(scan_count == 1u && failed == GPIO_PIN_RESET);
        }
}

static void incomplete_commands_and_payloads_have_deadlines(void)
{
        reset_test_state();
        append_command(HOST_COMMAND_HANDSHAKE);
        assert_failed_startup();
        assert(last_receive_timeout == HOST_HANDSHAKE_RETRY_INTERVAL_MS);
        assert(scan_count == 0u);

        reset_test_state();
        append_command(HOST_COMMAND_HANDSHAKE);
        input[input_length++] = 0xAAu;
        assert_failed_startup();
        assert(last_receive_timeout == 100u);

        reset_test_state();
        append_command(HOST_COMMAND_HANDSHAKE);
        append_command(HOST_COMMAND_TIME_SYNC_START);
        append_command(HOST_COMMAND_TIME);
        input[input_length++] = 0x01u;
        assert_failed_startup();
        assert(last_receive_timeout == HOST_HANDSHAKE_RETRY_INTERVAL_MS);
        assert(rtc_count == 0u && scan_count == 0u);
}

static void unexpected_startup_commands_are_rejected(void)
{
        const HostCommand expected[] = {HOST_COMMAND_TIME_SYNC_START,
                                        HOST_COMMAND_TIME,
                                        HOST_COMMAND_START_SCAN};
        // 3つの待機段階で、handshakeと期待するコマンド以外を拒否する。
        for (unsigned phase = 0u; phase < 3u; ++phase) {
                for (HostCommand command = 0; command < HOST_COMMAND_COUNT; ++command) {
                        if (command == expected[phase] ||
                            command == HOST_COMMAND_HANDSHAKE)
                                continue;
                        reset_test_state();
                        append_command(HOST_COMMAND_HANDSHAKE);
                        if (phase >= 1u)
                                append_command(HOST_COMMAND_TIME_SYNC_START);
                        if (phase >= 2u)
                                append_time(
                                        UINT64_C(1791160123000),
                                        UINT64_C(1791160123001));
                        append_command(command);
                        assert_failed_startup();
                        assert(scan_count == 0u);
                }
        }
        reset_test_state();
        append_command(HOST_COMMAND_HANDSHAKE);
        input[input_length++] = 0xAAu;
        input[input_length++] = 0xFFu;
        assert_failed_startup();
}

static void peripheral_failures_stop_startup(void)
{
        // センサー初期化、RTC設定、scan開始、ACK送信の失敗を個別に作る。
        for (unsigned failure = 0u; failure < 4u; ++failure) {
                reset_test_state();
                append_valid_startup();
                if (failure == 0u)
                        initialize_ok = false;
                if (failure == 1u)
                        rtc_ok = false;
                if (failure == 2u)
                        scan_ok = false;
                if (failure == 3u)
                        failed_transmit_type = FRAME_TYPE_START_SCAN_ACK;
                assert_failed_startup();
                assert(scan_count == (failure == 2u ? 1u : 0u));
        }
}

static void failed_host_replies_stop_before_scan_start(void)
{
        const FrameType replies[] = {
                FRAME_TYPE_HANDSHAKE_ACK,
                FRAME_TYPE_INITIALIZING,
                FRAME_TYPE_DEVICE_INFO,
                FRAME_TYPE_HEALTH_STATUS,
                FRAME_TYPE_READY,
                FRAME_TYPE_TIME_SYNC_START_ACK,
                FRAME_TYPE_TIME_ACK,
                FRAME_TYPE_TIME_SYNC_REPORT,
        };
        for (size_t i = 0u; i < sizeof(replies) / sizeof(replies[0]); ++i) {
                reset_test_state();
                append_valid_startup();
                failed_transmit_type = replies[i];
                assert_failed_startup();
                assert(scan_count == 0u);
        }
}

static void invalid_time_samples_stop_startup(void)
{
        reset_test_state();
        append_command(HOST_COMMAND_HANDSHAKE);
        append_command(HOST_COMMAND_TIME_SYNC_START);
        // hostの時刻が逆行しているのでRTCへ反映せず失敗に戻す。
        append_time(UINT64_C(1791160123001), UINT64_C(1791160123000));
        assert_failed_startup();
        assert(rtc_count == 0u && scan_count == 0u);
}

static void another_handshake_recovers_after_a_failed_attempt(void)
{
        reset_test_state();
        append_command(HOST_COMMAND_HANDSHAKE);
        append_command(HOST_COMMAND_START_SCAN);
        // 段階外のstart scanで失敗した後、次のhandshakeが入力に残っている。
        append_valid_startup();
        assert_failed_startup();
        assert(run_startup_sequence());
        assert(initialize_count == 2u && scan_count == 1u);
        assert(scanning == GPIO_PIN_SET && failed == GPIO_PIN_RESET);
}

static void sensor_failures_and_scan_descriptors_are_checked(void)
{
        // 各失敗でDMAを止め、RXとIMUフラグを消してhandshake待ちへ戻せる。
        for (unsigned failure = 0u; failure < 9u; ++failure) {
                reset_test_state();
                append_valid_startup();
                rx_buf.read_idx = 123;
                rx_buf.lap      = 1u;
                imu_accel_ready = imu_rot_ready = true;
                if (failure == 0u)
                        invalid_health = true;
                if (failure == 1u)
                        imu_ok = false;
                if (failure == 2u)
                        drain_status = HAL_OK;
                if (failure == 3u)
                        dma_ok = false;
                if (failure == 4u)
                        scan_received_size = SYS_PACKET_META_SIZE;
                if (failure == 5u)
                        valid_scan_marker = false;
                if (failure == 6u)
                        scan_mode = SYS_RES_MODE_SINGLE;
                if (failure == 7u)
                        scan_type = SYS_TYPE_CODE_HEALTH;
                if (failure == 8u)
                        drain_status = HAL_ERROR;
                assert_failed_startup();
                assert(!dma_started);
                assert(rx_buf.read_idx == 0 && rx_buf.lap == 0u);
                assert(!imu_accel_ready && !imu_rot_ready);
                if (failure < 6u || failure == 8u)
                        assert(descriptor_count == 0u);
        }
}

static void missing_slot_does_not_start_scanning(void)
{
        reset_test_state();
        slot_available = false;
        assert_failed_startup();
        assert(sent_count == 0u && initialize_count == 0u && scan_count == 0u);
}

int main(void)
{
        startup_follows_the_wire_order();
        handshake_restarts_each_startup_phase();
        incomplete_commands_and_payloads_have_deadlines();
        unexpected_startup_commands_are_rejected();
        peripheral_failures_stop_startup();
        failed_host_replies_stop_before_scan_start();
        invalid_time_samples_stop_startup();
        another_handshake_recovers_after_a_failed_attempt();
        missing_slot_does_not_start_scanning();
        sensor_failures_and_scan_descriptors_are_checked();
        return 0;
}
