#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "lidar/core.h"
#include "lidar/parser/scan.h"
#include "lidar/sys.h"

/**
 * SCAN
 */

/**
 * @param buf supposed to point to packet header fields
 */
static bool is_valid_scan_header(int8_t* buf)
{
        return SYS_PACKET_SCAN_HEADER_LE ==
               dec_little_endian(buf, SYS_PACKET_SCAN_HEADER_SIZE);
}

/**
 * @param buf supposed to point to CT fields
 */
static bool is_start_frame(int8_t* buf)
{
        return SYS_PACKET_SCAN_CT_START ==
               (SYS_PACKET_SCAN_CT_START_MASK &
                (uint8_t)dec_little_endian(buf, SYS_PACKET_SCAN_CT_SIZE));
}

/**
 *
 */
static uint8_t read_qty(int8_t* buf) { return (uint8_t)read_byte(buf); }

/**
 * @param buf supposed to point to angle fields
 * @return angle in Q6 degrees, or PARSER_SCAN_ANGLE_INVALID_Q6 if the field is invalid
 */
static uint16_t read_angle(int8_t* buf)
{
        uint32_t       raw          = dec_little_endian(buf, SYS_PACKET_SCAN_ANGLE_SIZE);
        uint32_t       angle_q6     = raw >> 1;
        const uint32_t full_turn_q6 = 360u * 64u;

        // Bit 0 is the required check bit; the other bits encode degrees * 64.
        if ((raw & 1u) == 0u || angle_q6 > full_turn_q6)
                return PARSER_SCAN_ANGLE_INVALID_Q6;

        return (uint16_t)(angle_q6 % full_turn_q6);
}

static uint32_t distance(int8_t* node)
{
        // Si[0] is intensity; the low two bits of Si[1] are flags.
        uint8_t low  = (uint8_t)read_byte(node + 1);
        uint8_t high = (uint8_t)read_byte(node + 2);
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
 * @param meta data_frame_head must point to the first three-byte Si field
 * @param p output array with room for data_num points
 * @return number of decoded points, or zero for invalid arguments or angles
 */
static uint32_t read_points(const ParserScanMeta* meta, ParserScannedPoint* p)
{
        if (meta == NULL || p == NULL || meta->data_frame_head == NULL ||
            angle(meta, 0) == PARSER_SCAN_ANGLE_INVALID_Q6)
                return 0;

        for (uint32_t i = 0; i < meta->data_num; ++i) {
                // Move to this point's three-byte Si field before decoding its distance.
                int8_t* node = meta->data_frame_head + i * SYS_PACKET_POINT_DATA_SIZE;
                p[i]         = (ParserScannedPoint){.angle = angle(meta, i),
                                                    .dist  = distance(node)};
        }

        return meta->data_num;
}

/**
 * @brief pack point data to array interpreting bytes of the buffer
 * @return number of point data packed into points array
 */
uint32_t read_scan_frame(int8_t* buf, ParserScannedPoint* points)
{
        int8_t* frame_head = buf;
        // read header
        if (is_valid_scan_header(frame_head))
                return 0;

        frame_head += SYS_PACKET_SCAN_HEADER_SIZE;

        // read ct
        bool isf = is_start_frame(frame_head);
        frame_head += SYS_PACKET_SCAN_CT_SIZE;

        ParserScanMeta scanMeta = {.start_angle     = 0,
                                   .end_angle       = 0,
                                   .data_num        = 0,
                                   .data_frame_head = NULL};

        // read data num
        scanMeta.data_num = read_qty(buf);
        frame_head += SYS_PACKET_SCAN_DATA_QTY_SIZE;

        // read angles
        scanMeta.start_angle = read_angle(buf);
        frame_head += SYS_PACKET_SCAN_ANGLE_SIZE;

        scanMeta.end_angle = read_angle(buf);
        frame_head += SYS_PACKET_SCAN_ANGLE_SIZE;

        // read points and pack them into buf
        scanMeta.data_frame_head = frame_head;
        uint32_t read_num        = read_points(&scanMeta, points);

        return read_num;
}
