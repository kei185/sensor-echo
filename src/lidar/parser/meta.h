#ifndef LIDAR_PARSER_META_H
#define LIDAR_PARSER_META_H

#include <stdint.h>
#include "lidar/sys.h"

typedef struct RxFrameMeta
{
        uint16_t    res_len;
        SysResMode  res_mode;
        SysTypeCode type_code;
} ParserMeta;

ParserMeta* read_meta(ParserMeta*);

#endif /* LIDAR_PARSER_META_H */
