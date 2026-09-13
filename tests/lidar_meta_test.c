#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "lidar/core.h"
#include "lidar/parser.h"

// The caller makes a completed RX span available; the parser must read every
// descriptor and content byte through the same checked reader used by scans.
static const int8_t* ready_begin;
static uint32_t      ready_len;
static uint32_t      read_count;

static void set_ready(const void* buf, uint32_t len)
{
        ready_begin = buf;
        ready_len   = len;
        read_count  = 0;
}

int8_t read_byte(int8_t* buf)
{
        uintptr_t addr  = (uintptr_t)buf;
        uintptr_t start = (uintptr_t)ready_begin;
        assert(ready_begin != NULL && addr >= start && addr - start < ready_len);
        ++read_count;
        return *buf;
}

uint32_t dec_little_endian(int8_t* buf, uint8_t len)
{
        uint32_t value = 0;
        for (uint8_t i = 0; i < len; ++i)
                value |= (uint32_t)(uint8_t)read_byte(buf + i) << (i * 8u);
        return value;
}

static void test_meta_length_and_mode(void)
{
        // The upper two bits of the fourth field byte are the response mode.
        const int8_t frame[] = {(int8_t)0xa5,
                                0x5a,
                                (int8_t)0xff,
                                (int8_t)0xff,
                                (int8_t)0xff,
                                0x7f,
                                (int8_t)0x81};
        ParserMeta   meta;
        set_ready(frame, sizeof(frame));
        assert(read_meta(frame, sizeof(frame), &meta) == &meta);
        assert(read_count == SYS_PACKET_META_SIZE);
        assert(meta.res_len == 0x3fffffffu);
        assert(meta.res_mode == SYS_RES_MODE_CONTINUOUS);
        assert(meta.type_code == SYS_TYPE_CODE_SCAN);

        // Generic metadata search may start after unrelated bytes.
        const int8_t prefixed[] = {0, (int8_t)0xa5, 0x5a, 3, 0, 0, 0, 6};
        set_ready(prefixed, sizeof(prefixed));
        assert(read_meta(prefixed, sizeof(prefixed), &meta) == &meta);
        assert(read_count == sizeof(prefixed));
        assert(meta.res_len == 3u);
        assert(meta.res_mode == SYS_RES_MODE_SINGLE);
        assert(meta.type_code == SYS_TYPE_CODE_HEALTH);
}

static void test_meta_rejects_short_buffers(void)
{
        const int8_t frame[] = {(int8_t)0xa5, 0x5a, 3, 0, 0, 0, 6};
        ParserMeta   meta    = {.res_len   = 42u,
                                .res_mode  = SYS_RES_MODE_CONTINUOUS,
                                .type_code = SYS_TYPE_CODE_SCAN};
        set_ready(frame, sizeof(frame));
        assert(read_meta(frame, SYS_PACKET_META_SIZE - 1u, &meta) == NULL);
        assert(read_meta(frame + SYS_PACKET_META_SIZE - 1u, 1u, &meta) == NULL);
        assert(read_meta(NULL, sizeof(frame), &meta) == NULL);
        assert(read_meta(frame, sizeof(frame), NULL) == NULL);
        assert(read_count == 0u);
        assert(meta.res_len == 42u);
        assert(meta.res_mode == SYS_RES_MODE_CONTINUOUS);
        assert(meta.type_code == SYS_TYPE_CODE_SCAN);
}

static void check_health_rejected(const uint8_t* frame, uint32_t len)
{
        ParserHealth output;
        ParserHealth before;
        memset(&output, 0xa5, sizeof(output));
        memcpy(&before, &output, sizeof(output));
        assert(!read_health_frame(frame, len, &output));
        assert(memcmp(&output, &before, sizeof(output)) == 0);
}

static void test_health_reply(void)
{
        uint8_t frame[SYS_PACKET_HEALTH_FRAME_SIZE] =
                {0xa5, 0x5a, 3, 0, 0, 0, 6, 0, 0, 0};
        ParserHealth health;
        set_ready(frame, sizeof(frame));
        assert(read_health_frame(frame, sizeof(frame), &health));
        assert(read_count == SYS_PACKET_META_SIZE + 1u);
        assert(health.health == 0u);
        assert(!health.sensor_abnormal);
        assert(!health.lidar_data_abnormal);

        frame[SYS_PACKET_META_SIZE] = 0x21u;
        set_ready(frame, sizeof(frame));
        assert(read_health_frame(frame, sizeof(frame), &health));
        assert(health.sensor_abnormal);
        assert(health.lidar_data_abnormal);
        assert(!health.encoder_abnormal);

        ParserSingleReply reply;
        set_ready(frame, sizeof(frame));
        assert(parse_single_response((const int8_t*)frame, sizeof(frame), &reply));
        assert(reply.meta.type_code == SYS_TYPE_CODE_HEALTH);
        assert(reply.content.health.health == 0x21u);

        check_health_rejected(frame, sizeof(frame) - 1u);
        frame[6] = SYS_TYPE_CODE_DEVICE_INFO;
        check_health_rejected(frame, sizeof(frame));
        frame[6] = SYS_TYPE_CODE_HEALTH;
        frame[5] = 0x40u;
        check_health_rejected(frame, sizeof(frame));
        frame[5] = 0;
        frame[2] = 4;
        check_health_rejected(frame, sizeof(frame));
        frame[2] = 3;
        frame[0] = 0;
        check_health_rejected(frame, sizeof(frame));
        check_health_rejected(NULL, sizeof(frame));
        assert(!read_health_frame(frame, sizeof(frame), NULL));
}

static void check_device_info_rejected(const uint8_t* frame, uint32_t len)
{
        ParserDeviceInfo output;
        ParserDeviceInfo before;
        memset(&output, 0xa5, sizeof(output));
        memcpy(&before, &output, sizeof(output));
        assert(!read_device_info_frame(frame, len, &output));
        assert(memcmp(&output, &before, sizeof(output)) == 0);
}

static void test_device_info_reply(void)
{
        uint8_t frame[SYS_PACKET_DEVICE_INFO_FRAME_SIZE] = {0xa5, 0x5a, 20, 0, 0, 0, 4};
        frame[7]                                         = 0x12u;
        frame[8]                                         = 0x34u;
        frame[9]                                         = 0x56u;
        frame[10]                                        = 0x78u;
        for (uint8_t i = 0; i < SYS_PACKET_DEVICE_SERIAL_SIZE; ++i)
                frame[11u + i] = i;

        ParserDeviceInfo info;
        set_ready(frame, sizeof(frame));
        assert(read_device_info_frame(frame, sizeof(frame), &info));
        assert(read_count == SYS_PACKET_META_SIZE + SYS_PACKET_DEVICE_INFO_CONTENT_SIZE);
        assert(info.model == 0x12u);
        assert(info.firmware_major == 0x34u);
        assert(info.firmware_minor == 0x56u);
        assert(info.hardware_version == 0x78u);
        assert(memcmp(info.serial_number, frame + 11u, SYS_PACKET_DEVICE_SERIAL_SIZE) ==
               0);

        ParserSingleReply reply;
        set_ready(frame, sizeof(frame));
        assert(parse_single_response((const int8_t*)frame, sizeof(frame), &reply));
        assert(reply.meta.type_code == SYS_TYPE_CODE_DEVICE_INFO);
        assert(memcmp(reply.content.device_info.serial_number,
                      info.serial_number,
                      SYS_PACKET_DEVICE_SERIAL_SIZE) == 0);

        check_device_info_rejected(frame, sizeof(frame) - 1u);
        frame[6] = SYS_TYPE_CODE_HEALTH;
        check_device_info_rejected(frame, sizeof(frame));
        frame[6] = SYS_TYPE_CODE_DEVICE_INFO;
        frame[5] = 0x40u;
        check_device_info_rejected(frame, sizeof(frame));
        frame[5] = 0;
        frame[2] = 19;
        check_device_info_rejected(frame, sizeof(frame));
        frame[2] = 20;
        frame[0] = 0;
        check_device_info_rejected(frame, sizeof(frame));
        check_device_info_rejected(NULL, sizeof(frame));
        assert(!read_device_info_frame(frame, sizeof(frame), NULL));
}

int main(void)
{
        test_meta_length_and_mode();
        test_meta_rejects_short_buffers();
        test_health_reply();
        test_device_info_reply();
        return 0;
}
