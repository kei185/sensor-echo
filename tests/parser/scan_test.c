#include <assert.h>
#include <stdint.h>
#include "lidar/parser/scan.h"
#include "parser_test_input.h"

static void read_scan_frame_reads_points_after_two_byte_checksum(void)
{
        // 準備
        /*
         * PH、CT、LSN=3、FSA=10度、LSA=20度、未検証のCS、3個のSiを並べる。
         * 最初のSi（64 E5 6F）はマニュアル3.1.4節の7161 mmの例を使う。
         */
        const uint8_t input[] = {
                0xaau, 0x55u, 0x00u, 0x03u, 0x01u, 0x05u, 0x01u, 0x0au, 0x34u, 0x12u,
                0x64u, 0xe5u, 0x6fu, 0x11u, 0x90u, 0x01u, 0x22u, 0x00u, 0x10u,
        };
        const ParserScannedPoint expected[] = {
                {.dist = 7161u, .angle = 10u * 64u},
                {.dist = 100u, .angle = 15u * 64u},
                {.dist = 1024u, .angle = 20u * 64u},
        };
        ParserScannedPoint actual[3] = {0};
        parser_test_set_input(input, sizeof(input));

        // 実行
        uint32_t result = read_scan_frame(actual);

        // 検証
        assert(result == 3u);
        for (uint32_t i = 0u; i < result; ++i) {
                assert(actual[i].angle == expected[i].angle);
                assert(actual[i].dist == expected[i].dist);
        }
        assert(parser_test_bytes_read() == sizeof(input));
}

static void read_scan_frame_interpolates_angles_across_zero_degrees(void)
{
        // 準備
        const uint8_t input[] = {
                0xaau, 0x55u, 0x00u, 0x03u, 0x01u, 0xafu, 0x01u, 0x05u, 0x00u, 0x00u,
                0x00u, 0x04u, 0x00u, 0x00u, 0x08u, 0x00u, 0x00u, 0x0cu, 0x00u,
        };
        const uint16_t     expected_angles[] = {350u * 64u, 0u, 10u * 64u};
        ParserScannedPoint actual[3]         = {0};
        parser_test_set_input(input, sizeof(input));

        // 実行
        uint32_t result = read_scan_frame(actual);

        // 検証
        assert(result == 3u);
        for (uint32_t i = 0u; i < result; ++i)
                assert(actual[i].angle == expected_angles[i]);
}

static void read_scan_frame_rejects_invalid_packet_header(void)
{
        // 準備
        const uint8_t      input[] = {0x00u, 0x00u};
        ParserScannedPoint actual  = {0};
        parser_test_set_input(input, sizeof(input));

        // 実行
        uint32_t result = read_scan_frame(&actual);

        // 検証
        assert(result == 0u);
        assert(parser_test_bytes_read() == sizeof(input));
}

static void read_scan_frame_rejects_angle_without_check_bit(void)
{
        // 準備
        const uint8_t input[] = {
                0xaau,
                0x55u,
                0x00u,
                0x01u,
                0x00u,
                0x05u,
                0x01u,
                0x05u,
                0x00u,
                0x00u,
                0x00u,
                0x00u,
                0x00u,
        };
        ParserScannedPoint actual = {0};
        parser_test_set_input(input, sizeof(input));

        // 実行
        uint32_t result = read_scan_frame(&actual);

        // 検証
        assert(result == 0u);
}

int main(void)
{
        read_scan_frame_reads_points_after_two_byte_checksum();
        read_scan_frame_interpolates_angles_across_zero_degrees();
        read_scan_frame_rejects_invalid_packet_header();
        read_scan_frame_rejects_angle_without_check_bit();
        return 0;
}
