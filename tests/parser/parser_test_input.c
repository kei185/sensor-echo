#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include "lidar/core.h"
#include "parser_test_input.h"

static const uint8_t* test_input;
static size_t         test_input_length;
static size_t         test_read_index;
static bool           test_read_timed_out;

void parser_test_set_input(const uint8_t* input, size_t length)
{
        assert(input != NULL || length == 0u);
        test_input        = input;
        test_input_length = length;
        test_read_index   = 0u;
}

size_t parser_test_bytes_read(void) { return test_read_index; }

void start_read_timeout(void) { test_read_timed_out = false; }

bool has_read_timed_out(void) { return test_read_timed_out; }

/* parserが使うRXリングバッファ読み出し処理をホストテスト用に置き換える。 */
uint8_t read_byte(void)
{
        // 入力が途切れた場合は、実機の受信待ちタイムアウトと同じ失敗を返す。
        if (test_read_index >= test_input_length) {
                test_read_timed_out = true;
                return 0u;
        }
        return test_input[test_read_index++];
}

/* バイト列のデコード方法をターゲット実装と同じにする。 */
uint32_t dec_little_endian(const uint8_t length)
{
        uint32_t value = 0u;

        for (uint8_t i = 0u; i < length; ++i)
                value |= (uint32_t)read_byte() << (i * 8u);

        return value;
}
