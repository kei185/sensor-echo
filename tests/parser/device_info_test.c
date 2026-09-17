#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "lidar/parser/device_info.h"
#include "parser_test_input.h"

static void read_device_info_frame_preserves_wire_byte_order(void)
{
        // 準備
        const uint8_t input[] = {
                151u,  2u,    7u,    3u,    0x00u, 0x11u, 0x22u, 0x33u, 0x44u, 0x55u,
                0x66u, 0x77u, 0x88u, 0x99u, 0xaau, 0xbbu, 0xccu, 0xddu, 0xeeu, 0xffu,
        };
        ParserDeviceInfo actual = {0};
        parser_test_set_input(input, sizeof(input));

        // 実行
        bool result = read_device_info_frame(&actual);

        // 検証
        assert(result);
        assert(actual.model == 151u);
        assert(actual.firmware_major == 2u);
        assert(actual.firmware_minor == 7u);
        assert(actual.hardware_version == 3u);
        assert(memcmp(actual.serial_number, &input[4], sizeof(actual.serial_number)) ==
               0);
        assert(parser_test_bytes_read() == sizeof(input));
}

static void read_device_info_frame_rejects_null_output_without_reading(void)
{
        // 準備
        const uint8_t input[] = {0x97u};
        parser_test_set_input(input, sizeof(input));

        // 実行
        bool result = read_device_info_frame(NULL);

        // 検証
        assert(!result);
        assert(parser_test_bytes_read() == 0u);
}

int main(void)
{
        read_device_info_frame_preserves_wire_byte_order();
        read_device_info_frame_rejects_null_output_without_reading();
        return 0;
}
