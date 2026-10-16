#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#include "lidar/core.h"
#include "lidar/frequency.h"
#include "lidar/sys.h"

#define FREQUENCY_UART_TIMEOUT_MS 100u
#define FREQUENCY_CONTENT_SIZE    4u
#define FREQUENCY_SCALE           100u
#define FREQUENCY_FINE_STEP       10u
#define FREQUENCY_MAX_ADJUSTMENTS                                                        \
        ((SYS_SCAN_FREQ_UPPER - SYS_SCAN_FREQ_LOWER) *                                   \
         (FREQUENCY_SCALE / FREQUENCY_FINE_STEP))

// マニュアル3.5〜3.6: 内容は4 byte、single response、typeは0x04。
static const uint8_t FREQUENCY_REPLY_HEADER[SYS_PACKET_META_SIZE] = {
        SYS_PACKET_HEADER_MSB,
        SYS_PACKET_HEADER_LSB,
        FREQUENCY_CONTENT_SIZE,
        0x00,
        0x00,
        0x00,
        0x04,
};

static bool request_frequency(MsgType command, uint32_t* frequency)
{
        if (HAL_UART_Transmit(
                    &huart4,
                    MSG[command],
                    MSG_SIZE,
                    FREQUENCY_UART_TIMEOUT_MS) != HAL_OK)
                return false;

        // 設定コマンドにも応答がある。毎回最後まで受け取り、次に残さない。
        const size_t reply_size = SYS_PACKET_META_SIZE + FREQUENCY_CONTENT_SIZE;
        setup_blocking_rx(reply_size);
        if (HAL_UART_Receive(
                    &huart4,
                    RX_BUF->_buf,
                    (uint16_t)reply_size,
                    FREQUENCY_UART_TIMEOUT_MS) != HAL_OK)
                return false;

        // 固定ヘッダを確認してから、既存のreaderで4 byteを読む。
        for (size_t i = 0u; i < sizeof(FREQUENCY_REPLY_HEADER); ++i) {
                if (read_byte() != FREQUENCY_REPLY_HEADER[i])
                        return false;
        }
        *frequency = dec_little_endian(FREQUENCY_CONTENT_SIZE);

        // 応答値の単位は0.01 Hz。例: 1000なら10.00 Hz。
        return *frequency >= SYS_SCAN_FREQ_LOWER * FREQUENCY_SCALE &&
               *frequency <= SYS_SCAN_FREQ_UPPER * FREQUENCY_SCALE;
}

bool lidar_set_scan_frequency(uint8_t hz)
{
        if (hz < SYS_SCAN_FREQ_LOWER || hz > SYS_SCAN_FREQ_UPPER)
                return false;

        uint32_t frequency;
        if (!request_frequency(MSG_TYPE_RX_FREQ, &frequency))
                return false;

        const uint32_t target = (uint32_t)hz * FREQUENCY_SCALE;

        // 6〜12 Hzを0.1 Hzずつ動かしても60回。異常な応答で無限に調整しない。
        for (uint32_t i = 0u; i < FREQUENCY_MAX_ADJUSTMENTS; ++i) {
                if (frequency == target)
                        return true;

                bool     increase   = frequency < target;
                uint32_t difference = increase ? target - frequency : frequency - target;

                // 差が1 Hz以上なら大きく動かし、残りを0.1 Hzずつ合わせる。
                MsgType command;
                if (difference >= FREQUENCY_SCALE)
                        command =
                                increase ? MSG_TYPE_INC_FREQ_1HZ : MSG_TYPE_DEC_FREQ_1HZ;
                else
                        command = increase ? MSG_TYPE_INC_FREQ_0_1HZ
                                           : MSG_TYPE_DEC_FREQ_0_1HZ;

                uint32_t previous = frequency;
                if (!request_frequency(command, &frequency))
                        return false;

                // 設定値が変わらない、または逆方向へ動いた場合は起動を止める。
                if ((increase && frequency <= previous) ||
                    (!increase && frequency >= previous))
                        return false;
        }

        return frequency == target;
}
