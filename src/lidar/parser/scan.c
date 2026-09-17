#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "lidar/core.h"
#include "lidar/parser/scan.h"
#include "lidar/sys.h"

/**
 * SCAN
 */

static bool is_valid_scan_header(void)
{
        return SYS_PACKET_SCAN_HEADER == dec_little_endian(SYS_PACKET_SCAN_HEADER_SIZE);
}

static bool is_start_frame(void)
{
        return SYS_PACKET_SCAN_CT_START ==
               (SYS_PACKET_SCAN_CT_START_MASK &
                (uint8_t)dec_little_endian(SYS_PACKET_SCAN_CT_SIZE));
}

/**
 *
 */
static uint8_t read_qty(void) { return (uint8_t)read_byte(); }

/**
 * @return angle in Q6 degrees, or PARSER_SCAN_ANGLE_INVALID_Q6 if the field is invalid
 */
static uint16_t read_angle(void)
{
        uint32_t       raw          = dec_little_endian(SYS_PACKET_SCAN_ANGLE_SIZE);
        uint32_t       angle_q6     = raw >> 1;
        const uint32_t full_turn_q6 = 360u * 64u;

        // Bit 0 is the required check bit; the other bits encode degrees * 64.
        if ((raw & 1u) == 0u || angle_q6 > full_turn_q6)
                return PARSER_SCAN_ANGLE_INVALID_Q6;

        return (uint16_t)(angle_q6 % full_turn_q6);
}

static uint32_t distance(void)
{
        // Si[0] is intensity; the low two bits of Si[1] are flags.
        (void)read_byte();
        uint8_t low  = (uint8_t)read_byte();
        uint8_t high = (uint8_t)read_byte();
        return ((uint32_t)high << 6) | (low >> 2);
}

static uint16_t angle(const ParserScanMeta* meta, uint32_t point_idx)
{
        const uint32_t full_turn_q6 = 360u * 64u;
        // LSN is one byte, so the interpolation product fits in uint32_t.
        if (meta == NULL || meta->data_num == 0u || point_idx >= meta->data_num ||
            meta->start_angle >= full_turn_q6 || meta->end_angle >= full_turn_q6)
                return PARSER_SCAN_ANGLE_INVALID_Q6;

        uint32_t clockwise_q6 =
                ((uint32_t)meta->end_angle + full_turn_q6 - (uint32_t)meta->start_angle) %
                full_turn_q6;
        uint32_t point_angle_q6 = (uint32_t)meta->start_angle;
        if (meta->data_num > 1u)
                point_angle_q6 += clockwise_q6 * point_idx / (meta->data_num - 1u);

        return (uint16_t)(point_angle_q6 % full_turn_q6);
}

/**
 * @param p output array with room for data_num points
 * @return number of decoded points, or zero for invalid arguments or angles
 */
static uint32_t read_points(const ParserScanMeta* meta, ParserScannedPoint* p)
{
        if (meta == NULL || p == NULL || angle(meta, 0) == PARSER_SCAN_ANGLE_INVALID_Q6)
                return 0;

        for (uint32_t i = 0; i < meta->data_num; ++i) {
                p[i]         = (ParserScannedPoint){.angle = angle(meta, i),
                                                    .dist  = distance()};
        }

        return meta->data_num;
}

/**
 * @brief pack point data to array interpreting bytes of the buffer
 * @return number of point data packed into points array
 */
uint32_t read_scan_frame(ParserScannedPoint* points)
{
        // read header
        if (!is_valid_scan_header())
                return 0;

        // read ct
        (void)is_start_frame();

        ParserScanMeta scanMeta = {.start_angle     = 0,
                                   .end_angle       = 0,
                                   .data_num        = 0};

        // read data num
        scanMeta.data_num = read_qty();

        // read angles
        scanMeta.start_angle = read_angle();

        scanMeta.end_angle = read_angle();

        // CSはまだ検証しないが、Siの前にある2byteを読み飛ばして位置を合わせる。
        (void)dec_little_endian(SYS_PACKET_SCAN_CS_SIZE);

        // read points and pack them into buf
        uint32_t read_num = read_points(&scanMeta, points);

        return read_num;
}
