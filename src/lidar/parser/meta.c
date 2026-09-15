#include <stddef.h>
#include <stdint.h>
#include "lidar/core.h"
#include "lidar/parser/meta.h"

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

/**
 * @return NULL if the start of frame  not found
 */
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
