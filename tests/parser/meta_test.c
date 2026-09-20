#include <assert.h>
#include <stdint.h>
#include "lidar/parser/meta.h"
#include "parser_test_input.h"

static void read_meta_skips_noise_and_reads_device_info_header(void)
{
        // 準備
        const uint8_t input[] = {
                0xffu,
                0x5au,
                0x00u,
                0xa5u,
                0x5au,
                0x14u,
                0x00u,
                0x00u,
                0x00u,
                0x04u,
        };
        ParserMeta       actual   = {0};
        const ParserMeta expected = {
                .res_len   = 20u,
                .res_mode  = SYS_RES_MODE_SINGLE,
                .type_code = SYS_TYPE_CODE_DEVICE_INFO,
        };
        parser_test_set_input(input, sizeof(input));

        // 実行
        ParserMeta* result = read_meta(&actual);

        // 検証
        assert(result == &actual);
        assert(actual.res_len == expected.res_len);
        assert(actual.res_mode == expected.res_mode);
        assert(actual.type_code == expected.type_code);
        assert(parser_test_bytes_read() == sizeof(input));
}

static void read_meta_decodes_continuous_scan_header(void)
{
        // 準備
        const uint8_t input[] = {
                0xa5u,
                0x5au,
                0x05u,
                0x00u,
                0x00u,
                0x40u,
                0x81u,
        };
        ParserMeta       actual   = {0};
        const ParserMeta expected = {
                .res_len   = 5u,
                .res_mode  = SYS_RES_MODE_CONTINUOUS,
                .type_code = SYS_TYPE_CODE_SCAN,
        };
        parser_test_set_input(input, sizeof(input));

        // 実行
        read_meta(&actual);

        // 検証
        assert(actual.res_len == expected.res_len);
        assert(actual.res_mode == expected.res_mode);
        assert(actual.type_code == expected.type_code);
}

static void read_meta_preserves_all_30_response_length_bits(void)
{
        // 準備
        const uint8_t input[] = {
                0xa5u,
                0x5au,
                0x78u,
                0x56u,
                0x34u,
                0x12u,
                0x06u,
        };
        ParserMeta     actual                   = {0};
        const uint32_t expected_response_length = 0x12345678u;
        parser_test_set_input(input, sizeof(input));

        // 実行
        read_meta(&actual);

        // 検証
        assert(actual.res_len == expected_response_length);
        assert(actual.res_mode == SYS_RES_MODE_SINGLE);
        assert(actual.type_code == SYS_TYPE_CODE_HEALTH);
}

static void read_meta_marks_unknown_mode_and_type_as_undefined(void)
{
        // 準備
        const uint8_t input[] = {
                0xa5u,
                0x5au,
                0x00u,
                0x00u,
                0x00u,
                0xc0u,
                0x99u,
        };
        ParserMeta actual = {0};
        parser_test_set_input(input, sizeof(input));

        // 実行
        read_meta(&actual);

        // 検証
        assert(actual.res_mode == SYS_RES_MODE_UNDEFINED);
        assert(actual.type_code == SYS_TYPE_CODE_UNDEFINED);
}

static void read_meta_preserves_high_bit_wire_bytes(void)
{
        // 準備: noise、length、typeへ0x7fより大きいbyteを入れる。
        const uint8_t input[] = {
                0x80u,
                0xffu,
                0xa5u,
                0x5au,
                0x80u,
                0x00u,
                0x00u,
                0x40u,
                0x81u,
        };
        ParserMeta actual = {0};
        parser_test_set_input(input, sizeof(input));

        // 実行
        read_meta(&actual);

        // 検証: high bitを符号として解釈せず、wire値のまま処理する。
        assert(actual.res_len == 128u);
        assert(actual.res_mode == SYS_RES_MODE_CONTINUOUS);
        assert(actual.type_code == SYS_TYPE_CODE_SCAN);
        assert(parser_test_bytes_read() == sizeof(input));
}

int main(void)
{
        read_meta_skips_noise_and_reads_device_info_header();
        read_meta_decodes_continuous_scan_header();
        read_meta_preserves_all_30_response_length_bits();
        read_meta_marks_unknown_mode_and_type_as_undefined();
        read_meta_preserves_high_bit_wire_bytes();
        return 0;
}
