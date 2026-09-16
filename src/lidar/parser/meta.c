#include <stdbool.h>
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

static void find_start_sign(void)
{
        uint8_t previous = (uint8_t)read_byte();

        while (true) {
                uint8_t current = (uint8_t)read_byte();
                uint16_t header = (uint16_t)previous | ((uint16_t)current << 8);

                if (header == SYS_PACKET_HEADER_LE)
                        return;

                previous = current;
        }
}

/**
 * @brief read response length and response mode field and set them  ParserMeta
 * corresponding fields
 * @param rfm
 */
static ParserMeta* read_res_len(ParserMeta* pm)
{

        pm->res_len = dec_little_endian(SYS_PACKET_LEN_MODE_SIZE - 1);

        int32_t last_byte = read_byte();
        int8_t  rm        = last_byte & SYS_PACKET_MODE_BIT_MASK >> 6;
        int32_t len       = last_byte & SYS_PACKET_LEN_BIT_MASK << 24;

        set_res_mode(pm, rm);
        pm->res_len |= len;

        return pm;
}

/**
 * @return parsed meta fields
 */
ParserMeta* read_meta(ParserMeta* rfm)
{
        find_start_sign();

        read_res_len(rfm);

        int8_t tc = (int8_t)dec_little_endian(SYS_PACKET_TYPE_CODE_SIZE);
        set_type_code(rfm, tc);

        return rfm;
}
