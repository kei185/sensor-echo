#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "protocol.h"
#include "startup.h"
#include "time_sync.h"
#include "imu/handler.h"
#include "lidar/core.h"
#include "lidar/translate.h"
#include "tx/frame.h"

UART_HandleTypeDef huart2;
RTC_HandleTypeDef  hrtc = {.Init = {.SynchPrediv = 255u}};
GPIO_TypeDef       scanning_port;
GPIO_TypeDef       initial_handshake_failed_port;
bool volatile imu_accel_ready;
bool volatile imu_rot_ready;

static TxBufSlot     slot;
static uint8_t       input[128];
static size_t        input_length;
static size_t        input_position;
static FrameType     sent_types[32];
static size_t        sent_count;
static uint32_t      fake_tick;
static uint32_t      last_receive_timeout;
static size_t        cancel_count;
static size_t        initialize_count;
static size_t        scan_count;
static size_t        abort_count;
static size_t        flush_count;
static size_t        rtc_count;
static GPIO_PinState scanning;
static GPIO_PinState failed;
static bool          slot_available;
static bool          initialize_ok;
static bool          scan_ok;
static bool          rtc_ok;
static FrameType     failed_transmit_type;

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
        assert(uart == &huart2);
        last_receive_timeout = timeout;
        // 2byte目と時刻payloadは必ず期限付き。入力不足は待ち時間切れにする。
        if (length == TIME_SYNC_PAYLOAD_SIZE)
                assert(timeout == TIME_SYNC_RESPONSE_TIMEOUT_MS);
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
        assert(uart == &huart2);
        assert(timeout == 100u);
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
        assert(uart == &huart2);
        ++abort_count;
        return HAL_OK;
}

void clear_uart_overrun(UART_HandleTypeDef* uart)
{
        assert(uart == &huart2);
        ++flush_count;
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
void       startup_cancel(void) { ++cancel_count; }
bool       startup_initialize_sensors(uint8_t* tx_frame)
{
        assert(tx_frame == slot._buf);
        // センサー初期化はhandshake ACKとINITIALIZINGの送信後に始まる。
        assert(sent_count >= 2u);
        assert(sent_types[sent_count - 2u] == FRAME_TYPE_HANDSHAKE_ACK);
        assert(sent_types[sent_count - 1u] == FRAME_TYPE_INITIALIZING);
        ++initialize_count;
        return initialize_ok;
}
bool startup_start_scan(void)
{
        // UARTのACK送信完了後にのみDMAとscanを開始する。
        assert(sent_types[sent_count - 1u] == FRAME_TYPE_START_SCAN_ACK);
        ++scan_count;
        return scan_ok;
}

// loop()のリンク用。起動テストではTXキューやscan解析を使わない。
void try_dispatch_tx(void) { assert(false); }
void push_full_slot(TxBufSlot* target, uint32_t length)
{
        (void)target;
        (void)length;
        assert(false);
}
bool is_lapped(void)
{
        assert(false);
        return false;
}
void reset_read_idx(void) { assert(false); }
bool imu_handler(uint8_t* to)
{
        (void)to;
        assert(false);
        return false;
}
size_t translate_scan_frame(uint8_t* to)
{
        (void)to;
        assert(false);
        return 0u;
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
                FRAME_TYPE_READY,
                FRAME_TYPE_TIME_SYNC_START_ACK,
                FRAME_TYPE_TIME_ACK,
                FRAME_TYPE_TIME_SYNC_REPORT,
                FRAME_TYPE_START_SCAN_ACK,
        };
        assert(sent_count == sizeof(expected) / sizeof(expected[0]));
        assert(memcmp(sent_types, expected, sizeof(expected)) == 0);
        assert(initialize_count == 1u && scan_count == 1u && rtc_count == 1u);
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
        assert(last_receive_timeout == TIME_SYNC_RESPONSE_TIMEOUT_MS);
        assert(rtc_count == 0u && scan_count == 0u);
}

static void unexpected_startup_commands_are_rejected(void)
{
        const HostCommand unexpected[] = {
                HOST_COMMAND_START_SCAN,
                HOST_COMMAND_TIME,
                HOST_COMMAND_MOTOR,
                HOST_COMMAND_SOFT_RESET,
        };
        for (size_t i = 0u; i < sizeof(unexpected) / sizeof(unexpected[0]); ++i) {
                reset_test_state();
                append_command(HOST_COMMAND_HANDSHAKE);
                append_command(unexpected[i]);
                assert_failed_startup();
                assert(scan_count == 0u && rtc_count == 0u);
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
        return 0;
}
