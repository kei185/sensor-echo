#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "lidar/core.h"
#include "lidar/sys.h"
#include "lidar/parser.h"

/**
 * META
 */

static ParserMeta* set_res_mode(ParserMeta* pm, uint8_t rm)
{
        switch (rm) {
                case SYS_RES_MODE_SINGLE:
                        pm->res_mode = SYS_RES_MODE_SINGLE;
                        break;
                case SYS_RES_MODE_CONTINUOUS:
                        pm->res_mode = SYS_RES_MODE_CONTINUOUS;
                        break;
                default:
                        pm->res_mode = SYS_RES_MODE_UNDEFINED;
                        break;
        }

        return pm;
}

static ParserMeta* set_type_code(ParserMeta* pm, uint8_t tc)
{
        switch (tc) {
                case SYS_TYPE_CODE_DEVICE_INFO:
                        pm->type_code = SYS_TYPE_CODE_DEVICE_INFO;
                        break;
                case SYS_TYPE_CODE_HEALTH:
                        pm->type_code = SYS_TYPE_CODE_HEALTH;
                        break;
                case SYS_TYPE_CODE_SCAN:
                        pm->type_code = SYS_TYPE_CODE_SCAN;
                        break;
                default:
                        pm->type_code = SYS_TYPE_CODE_UNDEFINED;
                        break;
        }

        return pm;
}

/**
 * @param buf: pointer to the buffer to be searched
 * @param len: length of the buffer
 * @return: pointer to the start sign in the buffer, or NULL if not found
 */
static const int8_t* find_start_sign(const int8_t* buf, uint32_t len)
{
        if (buf == NULL || len < SYS_PACKET_META_SIZE)
                return NULL;

        for (uint32_t idx = 0; idx <= len - SYS_PACKET_META_SIZE; ++idx) {
                if ((uint8_t)buf[idx] == SYS_PACKET_HEADER_MSB &&
                    (uint8_t)buf[idx + 1] == SYS_PACKET_HEADER_LSB)
                        return buf + idx;
        }

        return NULL;
}

/**
 * @brief Decode the response length and mode from an already-received descriptor.
 * @param buf Pointer to the four-byte length/mode field.
 * @param pm Parsed metadata to update.
 */
static ParserMeta* read_res_len(const int8_t* buf, ParserMeta* pm)
{
        // The four-byte field is little-endian: its top two bits are the mode.
        uint32_t len_mode = 0;
        for (uint32_t i = 0; i < SYS_PACKET_LEN_MODE_SIZE; ++i)
                len_mode |= (uint32_t)(uint8_t)buf[i] << (i * 8U);

        pm->res_len = len_mode & 0x3FFFFFFFU;
        set_res_mode(pm, (uint8_t)(len_mode >> 30));

        return pm;
}

ParserMeta* read_meta(const int8_t* buf, uint32_t len, ParserMeta* rfm)
{
        if (rfm == NULL)
                return NULL;

        const int8_t* frame_head = find_start_sign(buf, len);
        if (frame_head == NULL)
                return NULL;

        ParserMeta parsed = {0};
        read_res_len(frame_head + SYS_PACKET_HEADER_SIZE, &parsed);

        uint8_t tc =
                (uint8_t)frame_head[SYS_PACKET_HEADER_SIZE + SYS_PACKET_LEN_MODE_SIZE];
        set_type_code(&parsed, tc);

        *rfm = parsed;
        return rfm;
}

/**
 * HEALTH
 */

bool health_parse(ParserHealth* this)
{
        if (this->health == 0U)
                return false;

        this->sensor_abnormal         = (this->health & (1U << 0)) != 0U;
        this->encoder_abnormal        = (this->health & (1U << 1)) != 0U;
        this->wireless_power_abnormal = (this->health & (1U << 2)) != 0U;
        this->laser_feedback_abnormal = (this->health & (1U << 3)) != 0U;
        this->laser_drive_abnormal    = (this->health & (1U << 4)) != 0U;
        this->lidar_data_abnormal     = (this->health & (1U << 5)) != 0U;

        return true;
}

bool read_health_frame(const uint8_t* buf, uint32_t len, ParserHealth* health)
{
        if (buf == NULL || health == NULL || len < SYS_PACKET_HEALTH_FRAME_SIZE)
                return false;

        ParserMeta meta;
        if (read_meta((const int8_t*)buf, SYS_PACKET_META_SIZE, &meta) == NULL ||
            meta.res_mode != SYS_RES_MODE_SINGLE ||
            meta.type_code != SYS_TYPE_CODE_HEALTH ||
            meta.res_len != SYS_PACKET_HEALTH_CONTENT_SIZE)
                return false;

        const uint8_t* content = buf + SYS_PACKET_META_SIZE;
        ParserHealth   parsed  = {
                .health = content[0],
        };
        health_parse(&parsed);
        *health = parsed;
        return true;
}

/**
 * DEVICE INFO
 */
bool read_device_info_frame(const uint8_t* buf, uint32_t len, ParserDeviceInfo* info)
{
        if (buf == NULL || info == NULL || len < SYS_PACKET_DEVICE_INFO_FRAME_SIZE)
                return false;

        ParserMeta meta;
        if (read_meta((const int8_t*)buf, SYS_PACKET_META_SIZE, &meta) == NULL ||
            meta.res_mode != SYS_RES_MODE_SINGLE ||
            meta.type_code != SYS_TYPE_CODE_DEVICE_INFO ||
            meta.res_len != SYS_PACKET_DEVICE_INFO_CONTENT_SIZE)
                return false;

        const uint8_t* content = buf + SYS_PACKET_META_SIZE;
        // Manual section 3.3: firmware low byte is major, high byte is minor.
        ParserDeviceInfo parsed = {.model            = content[0],
                                   .firmware_major   = content[1],
                                   .firmware_minor   = content[2],
                                   .hardware_version = content[3]};
        // Preserve the serial bytes in wire order; no integer endian conversion.
        memcpy(parsed.serial_number, content + 4, SYS_PACKET_DEVICE_SERIAL_SIZE);
        *info = parsed;
        return true;
}

/**
 * SCAN
 */
static ParserScanMeta scanMeta = {
        .start_angle = 0, .end_angle = 0, .data_num = 0, .data_frame_head = NULL};

const ParserScanMeta* const PARSER_SCAN_META = &scanMeta;

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
