#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "lidar/parser/health.h"
#include "parser_test_input.h"

static void read_health_frame_clears_every_flag_for_zero_status(void)
{
        // 準備
        const uint8_t input[] = {0x00u};
        ParserHealth  actual;
        memset(&actual, 0xff, sizeof(actual));
        parser_test_set_input(input, sizeof(input));

        // 実行
        bool result = read_health_frame(&actual);

        // 検証
        assert(result);
        assert(actual.health == 0u);
        assert(!actual.sensor_abnormal);
        assert(!actual.encoder_abnormal);
        assert(!actual.wireless_power_abnormal);
        assert(!actual.laser_feedback_abnormal);
        assert(!actual.laser_drive_abnormal);
        assert(!actual.lidar_data_abnormal);
}

static void read_health_frame_decodes_each_status_bit(void)
{
        // 準備
        const uint8_t input[] = {0x25u};
        ParserHealth  actual  = {0};
        parser_test_set_input(input, sizeof(input));

        // 実行
        bool result = read_health_frame(&actual);

        // 検証
        assert(result);
        assert(actual.sensor_abnormal);
        assert(!actual.encoder_abnormal);
        assert(actual.wireless_power_abnormal);
        assert(!actual.laser_feedback_abnormal);
        assert(!actual.laser_drive_abnormal);
        assert(actual.lidar_data_abnormal);
}

static void read_health_frame_rejects_null_output_without_reading(void)
{
        // 準備
        const uint8_t input[] = {0x3fu};
        parser_test_set_input(input, sizeof(input));

        // 実行
        bool result = read_health_frame(NULL);

        // 検証
        assert(!result);
        assert(parser_test_bytes_read() == 0u);
}

int main(void)
{
        read_health_frame_clears_every_flag_for_zero_status();
        read_health_frame_decodes_each_status_bit();
        read_health_frame_rejects_null_output_without_reading();
        return 0;
}
