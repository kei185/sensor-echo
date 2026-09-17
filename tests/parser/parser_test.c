#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "lidar/parser/device_info.h"
#include "lidar/parser/health.h"
#include "lidar/parser/meta.h"
#include "lidar/parser/scan.h"
#include "parser_test_input.h"

static void test_meta_skips_noise_and_reads_device_info_header(void)
{
        const uint8_t input[] = {
                0xffu,
                0x5au,
                0x00u,
                0x5au,
                0xa5u,
                0x14u,
                0x00u,
                0x00u,
                0x00u,
                0x04u,
        };
        ParserMeta meta = {0};

        parser_test_set_input(input, sizeof(input));

        assert(read_meta(&meta) == &meta);
        assert(meta.res_len == 20u);
        assert(meta.res_mode == SYS_RES_MODE_SINGLE);
        assert(meta.type_code == SYS_TYPE_CODE_DEVICE_INFO);
        assert(parser_test_bytes_read() == sizeof(input));
}

static void test_meta_reads_continuous_scan_header(void)
{
        const uint8_t input[] = {
                0x5au,
                0xa5u,
                0x05u,
                0x00u,
                0x00u,
                0x40u,
                0x81u,
        };
        ParserMeta meta = {0};

        parser_test_set_input(input, sizeof(input));
        read_meta(&meta);

        assert(meta.res_len == 5u);
        assert(meta.res_mode == SYS_RES_MODE_CONTINUOUS);
        assert(meta.type_code == SYS_TYPE_CODE_SCAN);
}

static void test_meta_keeps_all_30_response_length_bits(void)
{
        const uint8_t input[] = {
                0x5au,
                0xa5u,
                0x78u,
                0x56u,
                0x34u,
                0x12u,
                0x06u,
        };
        ParserMeta meta = {0};

        parser_test_set_input(input, sizeof(input));
        read_meta(&meta);

        assert(meta.res_len == 0x12345678u);
        assert(meta.res_mode == SYS_RES_MODE_SINGLE);
        assert(meta.type_code == SYS_TYPE_CODE_HEALTH);
}

static void test_meta_marks_unknown_mode_and_type(void)
{
        const uint8_t input[] = {
                0x5au,
                0xa5u,
                0x00u,
                0x00u,
                0x00u,
                0xc0u,
                0x99u,
        };
        ParserMeta meta = {0};

        parser_test_set_input(input, sizeof(input));
        read_meta(&meta);

        assert(meta.res_mode == SYS_RES_MODE_UNDEFINED);
        assert(meta.type_code == SYS_TYPE_CODE_UNDEFINED);
}

static void test_health_zero_clears_every_flag(void)
{
        const uint8_t input[] = {0x00u};
        ParserHealth  health;
        memset(&health, 0xff, sizeof(health));

        parser_test_set_input(input, sizeof(input));

        assert(read_health_frame(&health));
        assert(health.health == 0u);
        assert(!health.sensor_abnormal);
        assert(!health.encoder_abnormal);
        assert(!health.wireless_power_abnormal);
        assert(!health.laser_feedback_abnormal);
        assert(!health.laser_drive_abnormal);
        assert(!health.lidar_data_abnormal);
}

static void test_health_decodes_each_status_bit(void)
{
        const uint8_t input[] = {0x25u};
        ParserHealth  health  = {0};

        parser_test_set_input(input, sizeof(input));
        assert(read_health_frame(&health));

        assert(health.sensor_abnormal);
        assert(!health.encoder_abnormal);
        assert(health.wireless_power_abnormal);
        assert(!health.laser_feedback_abnormal);
        assert(!health.laser_drive_abnormal);
        assert(health.lidar_data_abnormal);
}

static void test_health_rejects_null_output_without_reading(void)
{
        const uint8_t input[] = {0x3fu};

        parser_test_set_input(input, sizeof(input));

        assert(!read_health_frame(NULL));
        assert(parser_test_bytes_read() == 0u);
}

static void test_device_info_reads_wire_bytes_in_order(void)
{
        const uint8_t input[] = {
                151u,  2u,    7u,    3u,    0x00u, 0x11u, 0x22u, 0x33u, 0x44u, 0x55u,
                0x66u, 0x77u, 0x88u, 0x99u, 0xaau, 0xbbu, 0xccu, 0xddu, 0xeeu, 0xffu,
        };
        ParserDeviceInfo info = {0};

        parser_test_set_input(input, sizeof(input));

        assert(read_device_info_frame(&info));
        assert(info.model == 151u);
        assert(info.firmware_major == 2u);
        assert(info.firmware_minor == 7u);
        assert(info.hardware_version == 3u);
        assert(memcmp(info.serial_number, &input[4], sizeof(info.serial_number)) == 0);
        assert(parser_test_bytes_read() == sizeof(input));
}

static void test_device_info_rejects_null_output_without_reading(void)
{
        const uint8_t input[] = {0x97u};

        parser_test_set_input(input, sizeof(input));

        assert(!read_device_info_frame(NULL));
        assert(parser_test_bytes_read() == 0u);
}

static void test_scan_reads_points_after_two_byte_checksum(void)
{
        /*
         * PH, CT, LSN=3, FSA=10 deg, LSA=20 deg, ignored CS, then three Si.
         * The first Si (64 E5 6F) is the 7161 mm example from manual section 3.1.4.
         */
        const uint8_t input[] = {
                0xaau, 0x55u, 0x00u, 0x03u, 0x01u, 0x05u, 0x01u, 0x0au, 0x34u, 0x12u,
                0x64u, 0xe5u, 0x6fu, 0x11u, 0x90u, 0x01u, 0x22u, 0x00u, 0x10u,
        };
        ParserScannedPoint points[3] = {0};

        parser_test_set_input(input, sizeof(input));

        assert(read_scan_frame(points) == 3u);
        assert(points[0].angle == 10u * 64u);
        assert(points[1].angle == 15u * 64u);
        assert(points[2].angle == 20u * 64u);
        assert(points[0].dist == 7161u);
        assert(points[1].dist == 100u);
        assert(points[2].dist == 1024u);
        assert(parser_test_bytes_read() == sizeof(input));
}

static void test_scan_interpolates_across_zero_degrees(void)
{
        const uint8_t input[] = {
                0xaau, 0x55u, 0x00u, 0x03u, 0x01u, 0xafu, 0x01u, 0x05u, 0x00u, 0x00u,
                0x00u, 0x04u, 0x00u, 0x00u, 0x08u, 0x00u, 0x00u, 0x0cu, 0x00u,
        };
        ParserScannedPoint points[3] = {0};

        parser_test_set_input(input, sizeof(input));

        assert(read_scan_frame(points) == 3u);
        assert(points[0].angle == 350u * 64u);
        assert(points[1].angle == 0u);
        assert(points[2].angle == 10u * 64u);
}

static void test_scan_rejects_invalid_packet_header(void)
{
        const uint8_t      input[] = {0x00u, 0x00u};
        ParserScannedPoint point   = {0};

        parser_test_set_input(input, sizeof(input));

        assert(read_scan_frame(&point) == 0u);
        assert(parser_test_bytes_read() == sizeof(input));
}

static void test_scan_rejects_angle_without_check_bit(void)
{
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
        ParserScannedPoint point = {0};

        parser_test_set_input(input, sizeof(input));

        assert(read_scan_frame(&point) == 0u);
}

int main(void)
{
        test_meta_skips_noise_and_reads_device_info_header();
        test_meta_reads_continuous_scan_header();
        test_meta_keeps_all_30_response_length_bits();
        test_meta_marks_unknown_mode_and_type();
        test_health_zero_clears_every_flag();
        test_health_decodes_each_status_bit();
        test_health_rejects_null_output_without_reading();
        test_device_info_reads_wire_bytes_in_order();
        test_device_info_rejects_null_output_without_reading();
        test_scan_reads_points_after_two_byte_checksum();
        test_scan_interpolates_across_zero_degrees();
        test_scan_rejects_invalid_packet_header();
        test_scan_rejects_angle_without_check_bit();
        return 0;
}
