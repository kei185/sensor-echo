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
        uint32_t next_filed_idx = 0;

        for (; next_filed_idx < len; ++next_filed_idx)
                if (SYS_PACKET_HEADER_LE ==
                    (buf[next_filed_idx] | (buf[next_filed_idx + 1] << 8)))
                        break;

        if (next_filed_idx >= len)
                return NULL;

        return buf + next_filed_idx;
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

        int32_t last_byte = read_byte(buf + SYS_PACKET_LEN_MODE_SIZE - 1);
        int8_t  rm        = last_byte & SYS_PACKET_MODE_BIT_MASK >> 6;
        int32_t len       = last_byte & SYS_PACKET_LEN_BIT_MASK << 24;

        set_res_mode(pm, rm);
        pm->res_len |= len;

        return pm;
}

ParserMeta* read_meta(int8_t* buf, uint32_t len, ParserMeta* rfm)
{
        int8_t* frame_head = find_start_sign(buf, len);
        if (frame_head == NULL)
                return NULL;

        read_res_len(frame_head + SYS_PACKET_HEADER_SIZE, rfm);

        int8_t tc = dec_little_endian(
                frame_head + SYS_PACKET_HEADER_SIZE + SYS_PACKET_LEN_MODE_SIZE,
                SYS_PACKET_TYPE_CODE_SIZE);

        set_type_code(rfm, tc);

        return rfm;
}

/**
 * Health and device-info replies have fixed single-response descriptors.
 * Match their wire bytes directly so blocking RX does not depend on the DMA reader.
 */
static const uint8_t* single_response_content(
        const uint8_t* buf, uint32_t len, uint8_t content_size, SysTypeCode type_code)
{
        const uint8_t descriptor[SYS_PACKET_META_SIZE] =
                {0xA5, 0x5A, content_size, 0x00, 0x00, 0x00, (uint8_t)type_code};

        if (buf == NULL || len < SYS_PACKET_META_SIZE + content_size)
                return NULL;

        if (memcmp(buf, descriptor, sizeof(descriptor)) != 0)
                return NULL;

        return buf + SYS_PACKET_META_SIZE;
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
        const uint8_t* content = single_response_content(
                buf,
                len,
                SYS_PACKET_HEALTH_CONTENT_SIZE,
                SYS_TYPE_CODE_HEALTH);
        if (content == NULL || health == NULL)
                return false;

        ParserHealth parsed = {
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
        const uint8_t* content = single_response_content(
                buf,
                len,
                SYS_PACKET_DEVICE_INFO_CONTENT_SIZE,
                SYS_TYPE_CODE_DEVICE_INFO);
        if (content == NULL || info == NULL)
                return false;

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
static ParserScanMeta scanMeta = {0};

const ParserScanMeta* const PARSER_SCAN_META = &scanMeta;

/**
 * @param buf supposed to point to packet header fields
 */
static bool is_valid_scan_header(const uint8_t* buf)
{
        return buf[0] == 0xAA && buf[1] == 0x55;
}

/**
 * @param buf supposed to point to CT fields
 */
static bool is_start_frame(const uint8_t* buf)
{
        return SYS_PACKET_SCAN_CT_START == (SYS_PACKET_SCAN_CT_START_MASK & buf[0]);
}

static uint8_t read_qty(const uint8_t* buf) { return buf[0]; }

/**
 * @param buf supposed to point to angle fields
 */
static bool read_angle(const uint8_t* buf, uint16_t* angle_q6)
{
        uint16_t raw = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
        if ((raw & 1u) == 0u || (raw >> 1) > SYS_PACKET_SCAN_FULL_TURN_Q6)
                return false;

        *angle_q6 = (raw >> 1) % SYS_PACKET_SCAN_FULL_TURN_Q6;
        return true;
}

static uint16_t distance(const uint8_t* node)
{
        // Si[0] is intensity; Si[1] has flags in bits 7:6 and distance in 5:2.
        return ((uint16_t)node[2] << 6) | ((uint16_t)node[1] >> 2);
}

static int16_t angle(const ParserScanMeta* meta, uint32_t point_idx)
{
        uint32_t clockwise_q6 = (meta->end_angle_q6 + SYS_PACKET_SCAN_FULL_TURN_Q6 -
                                 meta->start_angle_q6) %
                                SYS_PACKET_SCAN_FULL_TURN_Q6;
        uint32_t value_q6     = meta->start_angle_q6;

        if (meta->data_num > 1u)
                value_q6 += clockwise_q6 * point_idx / (meta->data_num - 1u);

        // Keep Q6 precision until the final integer-degree PC payload value.
        uint32_t degrees = (value_q6 % SYS_PACKET_SCAN_FULL_TURN_Q6) /
                           SYS_PACKET_SCAN_ANGLE_Q6_PER_DEGREE;
        return (int16_t)(degrees > 180u ? (int32_t)degrees - 360 : (int32_t)degrees);
}

/**
 * @param buf supposed to point to the head of data fields
 */
static uint32_t read_points(const ParserScanMeta* meta, ParserScannedPoint* p)
{
        for (uint32_t i = 0; i < meta->data_num; ++i) {
                const uint8_t* node =
                        meta->data_frame_head + i * SYS_PACKET_POINT_DATA_SIZE;
                p[i] = (ParserScannedPoint){.angle = angle(meta, i),
                                            .dist  = distance(node)};
        }

        return meta->data_num;
}

/**
 * @brief pack point data to array interpreting bytes of the buffer
 * @return number of point data packed into points array
 */
uint32_t read_scan_frame(
        const uint8_t* buf, uint32_t len, ParserScannedPoint* points, uint32_t capacity)
{
        if (buf == NULL || points == NULL || len < SYS_PACKET_SCAN_FIXED_SIZE ||
            !is_valid_scan_header(buf))
                return 0;

        const uint8_t* frame_head = buf + SYS_PACKET_SCAN_HEADER_SIZE;
        bool           isf        = is_start_frame(frame_head);
        frame_head += SYS_PACKET_SCAN_CT_SIZE;

        ParserScanMeta parsed = {.data_num = read_qty(frame_head)};
        if (parsed.data_num == 0u || capacity < parsed.data_num ||
            (isf && parsed.data_num != 1u) ||
            len < SYS_PACKET_SCAN_FIXED_SIZE +
                            parsed.data_num * SYS_PACKET_POINT_DATA_SIZE)
                return 0;
        frame_head += SYS_PACKET_SCAN_DATA_QTY_SIZE;

        if (!read_angle(frame_head, &parsed.start_angle_q6))
                return 0;
        frame_head += SYS_PACKET_SCAN_ANGLE_SIZE;

        if (!read_angle(frame_head, &parsed.end_angle_q6))
                return 0;
        frame_head += SYS_PACKET_SCAN_ANGLE_SIZE;

        // Skip the two-byte CS field; XOR verification is intentionally deferred.
        parsed.data_frame_head = frame_head + SYS_PACKET_SCAN_CS_SIZE;
        uint32_t read_num      = read_points(&parsed, points);
        scanMeta               = parsed;

        return read_num;
}
