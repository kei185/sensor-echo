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
static int8_t* find_start_sign(int8_t* buf, uint32_t len)
{
        if (buf == NULL || len < SYS_PACKET_HEADER_SIZE)
                return NULL;

        for (uint32_t i = 0u; i < len - 1u; ++i)
                if ((uint8_t)buf[i] == SYS_PACKET_HEADER_FIRST_BYTE &&
                    (uint8_t)buf[i + 1u] == SYS_PACKET_HEADER_SECOND_BYTE)
                        return buf + i;

        return NULL;
}

/**
 * @brief read response length and response mode field and set them  ParserMeta
 * corresponding fields
 * @param buf: pointer to the buffer containing the response length byte field
 * @param rfm
 */
static ParserMeta* read_res_len(int8_t* buf, ParserMeta* pm)
{

        pm->res_len = dec_little_endian(buf, SYS_PACKET_LEN_MODE_SIZE - 1);

        uint8_t  last_byte = (uint8_t)read_byte(buf + SYS_PACKET_LEN_MODE_SIZE - 1);
        uint8_t  rm        = (last_byte & SYS_PACKET_MODE_BIT_MASK) >> 6;
        uint32_t len       = (uint32_t)(last_byte & SYS_PACKET_LEN_BIT_MASK) << 24;

        set_res_mode(pm, rm);
        pm->res_len |= len;

        return pm;
}

/**
 * @return NULL if the start of frame  not found
 */
ParserMeta* read_meta(int8_t* buf, uint32_t len, ParserMeta* rfm)
{
        if (buf == NULL || rfm == NULL || len < SYS_PACKET_META_SIZE)
                return NULL;

        int8_t* frame_head = find_start_sign(buf, len);
        if (frame_head == NULL ||
            len - (uint32_t)(frame_head - buf) < SYS_PACKET_META_SIZE)
                return NULL;

        read_res_len(frame_head + SYS_PACKET_HEADER_SIZE, rfm);

        uint8_t tc = (uint8_t)dec_little_endian(
                frame_head + SYS_PACKET_HEADER_SIZE + SYS_PACKET_LEN_MODE_SIZE,
                SYS_PACKET_TYPE_CODE_SIZE);

        set_type_code(rfm, tc);

        return rfm;
}

/**
 * HEALTH
 */

static bool health_parse(ParserHealth* this)
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

/**
 * @param buf points to health byte field
 */
bool read_health_frame(const int8_t* buf, ParserHealth* health)
{
        if (health == NULL)
                return false;

        health->health = (uint8_t)read_byte(buf);

        health_parse(health);
        return true;
}

/**
 * DEVICE INFO
 */
bool read_device_info_frame(const int8_t* buf, ParserDeviceInfo* info)
{
        if (info)
                return false;

        // Manual section 3.3: firmware low byte is major, high byte is minor.
        info->model            = buf[0];
        info->firmware_major   = buf[1];
        info->firmware_minor   = buf[2];
        info->hardware_version = buf[3];
        // Preserve the serial bytes in wire order; no integer endian conversion.
        memcpy(info->serial_number, buf + 4, SYS_PACKET_DEVICE_SERIAL_SIZE);

        return true;
};

/**
 * SCAN
 */

/**
 * @param buf supposed to point to packet header fields
 */
static bool is_valid_scan_header(const uint8_t* buf)
{
        return buf[0] == 0xaau && buf[1] == 0x55u;
}

/**
 * @param buf supposed to point to angle fields
 * @return angle in Q6 degrees, or PARSER_SCAN_ANGLE_INVALID_Q6 if the field is invalid
 */
static uint16_t read_angle(const uint8_t* buf)
{
        // This packet has already passed its length check. Read its two bytes
        // directly so a copy outside the DMA RX slots can also be parsed.
        uint32_t       raw          = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8);
        uint32_t       angle_q6     = raw >> 1;
        const uint32_t full_turn_q6 = 360u * 64u;

        // Bit 0 is the required check bit; the other bits encode degrees * 64.
        if ((raw & 1u) == 0u || angle_q6 > full_turn_q6)
                return PARSER_SCAN_ANGLE_INVALID_Q6;

        return (uint16_t)(angle_q6 % full_turn_q6);
}

static uint16_t distance(const uint8_t* node)
{
        // Si[0] is intensity; the low two bits of Si[1] are flags.
        uint8_t low  = node[1];
        uint8_t high = node[2];
        return (uint16_t)(((uint16_t)high << 6) | (low >> 2));
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
 * @param payload output bytes with room for data_num four-byte points
 * @return number of decoded points, or zero for invalid arguments or angles
 */
static size_t read_points(const ParserScanMeta* meta, uint8_t* payload)
{
        if (meta == NULL || payload == NULL || meta->data_frame_head == NULL ||
            angle(meta, 0) == PARSER_SCAN_ANGLE_INVALID_Q6)
                return 0;

        for (size_t i = 0; i < meta->data_num; ++i) {
                // Move to this point's three-byte Si field before decoding its distance.
                const uint8_t* node =
                        meta->data_frame_head + i * SYS_PACKET_POINT_DATA_SIZE;
                uint16_t dist     = distance(node);
                uint16_t angle_q6 = angle(meta, i);
                size_t   offset   = i * sizeof(ParserScannedPoint);
                // PC payload order is distance, then angle, each little-endian.
                payload[offset]      = (uint8_t)dist;
                payload[offset + 1u] = (uint8_t)(dist >> 8);
                payload[offset + 2u] = (uint8_t)angle_q6;
                payload[offset + 3u] = (uint8_t)(angle_q6 >> 8);
        }

        return meta->data_num;
}

/**
 * @brief Decode a complete LiDAR scan packet into PC payload bytes.
 * @return Number of decoded points, or zero for invalid input.
 */
size_t read_scan_frame(
        const uint8_t* buf, size_t packet_len, uint8_t* payload, size_t payload_capacity)
{
        if (buf == NULL || payload == NULL || packet_len < SYS_PACKET_SCAN_FIXED_SIZE ||
            !is_valid_scan_header(buf))
                return 0u;

        // AA 55, CT, LSN, FSA, LSA, CS, then LSN three-byte point records.
        const uint8_t ct  = buf[SYS_PACKET_SCAN_HEADER_SIZE];
        const uint8_t lsn = buf[SYS_PACKET_SCAN_HEADER_SIZE + SYS_PACKET_SCAN_CT_SIZE];
        const size_t  needed =
                SYS_PACKET_SCAN_FIXED_SIZE + (size_t)lsn * SYS_PACKET_POINT_DATA_SIZE;
        if (lsn == 0u || payload_capacity / sizeof(ParserScannedPoint) < lsn ||
            packet_len < needed ||
            ((ct & SYS_PACKET_SCAN_CT_START_MASK) != 0u && lsn != 1u))
                return 0u;

        ParserScanMeta scan_meta = {
                .start_angle     = read_angle(buf + 4u),
                .end_angle       = read_angle(buf + 6u),
                .data_num        = lsn,
                .data_frame_head = buf + SYS_PACKET_SCAN_FIXED_SIZE,
        };
        if (scan_meta.start_angle == PARSER_SCAN_ANGLE_INVALID_Q6 ||
            scan_meta.end_angle == PARSER_SCAN_ANGLE_INVALID_Q6)
                return 0u;

        // CS is at bytes 8 and 9. XOR checking remains a separate TODO.
        return read_points(&scan_meta, payload);
}
