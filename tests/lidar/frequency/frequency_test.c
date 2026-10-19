#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "stm32f446xx.h"

#include "lidar/core.h"
#include "lidar/frequency.h"
#include "lidar/sys.h"

UART_HandleTypeDef huart4;
DMA_Stream_TypeDef mock_dma1_stream2;

static uint32_t frequency;
static uint8_t  commands[64];
static size_t   command_count;
static size_t   receive_count;
static int      fail_transmit_at;
static int      fail_receive_at;
static int      bad_header_byte;
static int      step_direction;

static void reset_test_state(uint32_t initial_frequency)
{
        frequency        = initial_frequency;
        command_count    = 0u;
        receive_count    = 0u;
        fail_transmit_at = -1;
        fail_receive_at  = -1;
        bad_header_byte  = -1;
        step_direction   = 1;
        memset(commands, 0, sizeof(commands));
}

HAL_StatusTypeDef HAL_UART_Transmit(
        UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length, uint32_t timeout)
{
        assert(huart == &huart4);
        assert(length == 2u);
        assert(data[0] == 0xA5);
        assert(timeout == 100u);
        assert(command_count < sizeof(commands));
        commands[command_count++] = data[1];
        if ((int)(command_count - 1u) == fail_transmit_at)
                return HAL_ERROR;

        // 実機と同じコマンドの意味で設定値を動かす。単位は0.01 Hz。
        int change;
        switch (data[1]) {
                case 0x0D:
                        change = 0;
                        break;
                case 0x0B:
                        change = 100;
                        break;
                case 0x0C:
                        change = -100;
                        break;
                case 0x09:
                        change = 10;
                        break;
                case 0x0A:
                        change = -10;
                        break;
                default:
                        assert(false);
                        change = 0;
        }
        frequency = (uint32_t)((int64_t)frequency + change * step_direction);
        return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(
        UART_HandleTypeDef* huart, uint8_t* data, uint16_t length, uint32_t timeout)
{
        assert(huart == &huart4);
        assert(data == RX_BUF->_buf);
        assert(length == 11u);
        assert(timeout == 100u);
        assert(command_count == receive_count + 1u);
        if ((int)receive_count++ == fail_receive_at)
                return HAL_TIMEOUT;

        // マニュアルの11 byte応答を作り、実際のread_byteとdecoderに読ませる。
        const uint8_t header[] = {0xA5, 0x5A, 0x04, 0x00, 0x00, 0x00, 0x04};
        memcpy(data, header, sizeof(header));
        for (size_t i = 0u; i < 4u; ++i)
                data[sizeof(header) + i] = (uint8_t)(frequency >> (i * 8u));
        if (bad_header_byte >= 0)
                data[bad_header_byte] ^= 0x01;
        return HAL_OK;
}

static void expect_commands(const uint8_t* expected, size_t length)
{
        assert(command_count == length);
        assert(receive_count == length);
        assert(memcmp(commands, expected, length) == 0);
        assert(frequency == 1000u);
        // 応答の最後のbyteまで読み終わり、次のコマンドに残していない。
        assert(RX_BUF->read_idx == 11);
}

static void adjusts_up_and_down_with_both_step_sizes(void)
{
        // デフォルト6 Hzからは、現在値の取得後に1 Hzずつ4回増やす。
        reset_test_state(600u);
        assert(lidar_set_scan_frequency(10u));
        const uint8_t from_default[] = {0x0D, 0x0B, 0x0B, 0x0B, 0x0B};
        expect_commands(from_default, sizeof(from_default));

        reset_test_state(1200u);
        assert(lidar_set_scan_frequency(10u));
        const uint8_t from_upper[] = {0x0D, 0x0C, 0x0C};
        expect_commands(from_upper, sizeof(from_upper));

        // 8.8 Hzからは1 Hzを1回と0.1 Hzを2回足す。
        reset_test_state(880u);
        assert(lidar_set_scan_frequency(10u));
        const uint8_t from_fraction_below[] = {0x0D, 0x0B, 0x09, 0x09};
        expect_commands(from_fraction_below, sizeof(from_fraction_below));

        // 11.2 Hzからは同じ組み合わせで減らす。
        reset_test_state(1120u);
        assert(lidar_set_scan_frequency(10u));
        const uint8_t from_fraction_above[] = {0x0D, 0x0C, 0x0A, 0x0A};
        expect_commands(from_fraction_above, sizeof(from_fraction_above));

        // すでに10 Hzなら、取得だけで調整コマンドを送らない。
        reset_test_state(1000u);
        assert(lidar_set_scan_frequency(10u));
        const uint8_t already_at_target[] = {0x0D};
        expect_commands(already_at_target, sizeof(already_at_target));
}

static void rejects_invalid_target_and_reply(void)
{
        // 機種の範囲外の目標はUARTに触る前に拒否する。
        reset_test_state(600u);
        assert(!lidar_set_scan_frequency(5u));
        assert(!lidar_set_scan_frequency(13u));
        assert(command_count == 0u);

        // 開始記号・長さ・mode・typeのどのbyteが違っても調整を続けない。
        for (int i = 0; i < 7; ++i) {
                reset_test_state(600u);
                bad_header_byte = i;
                assert(!lidar_set_scan_frequency(10u));
                assert(command_count == 1u);
        }

        const uint32_t invalid_frequency[] = {0u, 599u, 1201u, UINT32_MAX};
        for (size_t i = 0u; i < sizeof(invalid_frequency) / sizeof(invalid_frequency[0]);
             ++i) {
                reset_test_state(invalid_frequency[i]);
                assert(!lidar_set_scan_frequency(10u));
                assert(command_count == 1u);
        }
}

static void transfer_failures_stop_adjustment(void)
{
        // 取得時と調整の途中、どちらで送受信が失敗してもfalseを返す。
        for (int i = 0; i < 3; ++i) {
                reset_test_state(600u);
                fail_transmit_at = i;
                assert(!lidar_set_scan_frequency(10u));
                assert(command_count == (size_t)i + 1u);
                assert(receive_count == (size_t)i);

                reset_test_state(600u);
                fail_receive_at = i;
                assert(!lidar_set_scan_frequency(10u));
                assert(command_count == (size_t)i + 1u);
                assert(receive_count == (size_t)i + 1u);
        }
}

static void nonprogress_and_unreachable_target_do_not_loop_forever(void)
{
        reset_test_state(600u);
        step_direction = 0;
        assert(!lidar_set_scan_frequency(10u));
        assert(command_count == 2u);

        reset_test_state(900u);
        step_direction = -1;
        assert(!lidar_set_scan_frequency(10u));
        assert(command_count == 2u);

        // 9.95 Hzは0.1 Hz刻みで10.00 Hzに合わない。上限回数で失敗する。
        reset_test_state(995u);
        assert(!lidar_set_scan_frequency(10u));
        assert(command_count == 61u);
}

int main(void)
{
        adjusts_up_and_down_with_both_step_sizes();
        rejects_invalid_target_and_reply();
        transfer_failures_stop_adjustment();
        nonprogress_and_unreachable_target_do_not_loop_forever();
        return 0;
}
