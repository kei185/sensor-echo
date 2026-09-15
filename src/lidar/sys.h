
#ifndef LIDAR_SYS_H
#define LIDAR_SYS_H

#include <stdint.h>

extern const uint16_t MSG_SIZE;

extern const uint8_t* MSG[];
typedef enum
{
        MSG_TYPE_SCAN = 0,
        MSG_TYPE_STOP,
        MSG_TYPE_RX_SYS_INFO,
        MSG_TYPE_RX_HEALTH,
        MSG_TYPE_INC_FREQ_0_1HZ,
        MSG_TYPE_DEC_FREQ_0_1HZ,
        MSG_TYPE_INC_FREQ_1HZ,
        MSG_TYPE_DEC_FREQ_1HZ,
        MSG_TYPE_RX_FREQ,
        MSG_TYPE_SOFT_RESTART,
} MsgType;

#define SYS_RANGING_FREQ    4000u
#define SYS_SCAN_FREQ_UPPER 12u
#define SYS_SCAN_FREQ_LOWER 6u

#define SYS_PACKET_HEADER_SIZE    2u // byte
#define SYS_PACKET_LEN_MODE_SIZE  4u // byte
#define SYS_PACKET_MODE_BIT_MASK  0b11000000
#define SYS_PACKET_LEN_BIT_MASK   0b00111111
#define SYS_PACKET_TYPE_CODE_SIZE 1u // byte
#define SYS_PACKET_HEADER_LSB     0x5Au
#define SYS_PACKET_HEADER_MSB     0xA5u
#define SYS_PACKET_HEADER         0xA55Au
#define SYS_PACKET_HEADER_LE      (SYS_PACKET_HEADER_LSB | (SYS_PACKET_HEADER_MSB << 8))
// LE Little Endian

#define SYS_PACKET_META_SIZE                                                             \
        (SYS_PACKET_HEADER_SIZE + SYS_PACKET_LEN_MODE_SIZE + SYS_PACKET_TYPE_CODE_SIZE)
#define SYS_PACKET_HEALTH_CONTENT_SIZE      3u  // byte
#define SYS_PACKET_DEVICE_INFO_CONTENT_SIZE 20u // byte
#define SYS_PACKET_DEVICE_SERIAL_SIZE       16u // byte
#define SYS_PACKET_HEALTH_FRAME_SIZE                                                     \
        (SYS_PACKET_META_SIZE + SYS_PACKET_HEALTH_CONTENT_SIZE)
#define SYS_PACKET_DEVICE_INFO_FRAME_SIZE                                                \
        (SYS_PACKET_META_SIZE + SYS_PACKET_DEVICE_INFO_CONTENT_SIZE)

#define SYS_PACKET_SCAN_HEADER_SIZE   2u // byte
#define SYS_PACKET_SCAN_HEADER        0x55AA
#define SYS_PACKET_SCAN_HEADER_LE     0xAA55
#define SYS_PACKET_SCAN_CT_SIZE       1u // byte
#define SYS_PACKET_SCAN_CT_START      0b00000001
#define SYS_PACKET_SCAN_CT_START_MASK 0b00000001
#define SYS_PACKET_SCAN_DATA_QTY_SIZE 1u // byte
#define SYS_PACKET_SCAN_ANGLE_SIZE    2u // byte
#define SYS_PACKET_SCAN_CS_SIZE       2u // bytes
#define SYS_PACKET_POINT_DATA_SIZE    3u // byte
#define SYS_PACKET_SCAN_FIXED_SIZE                                                       \
        (SYS_PACKET_SCAN_HEADER_SIZE + SYS_PACKET_SCAN_CT_SIZE +                         \
         SYS_PACKET_SCAN_DATA_QTY_SIZE + 2u * SYS_PACKET_SCAN_ANGLE_SIZE +               \
         SYS_PACKET_SCAN_CS_SIZE)

typedef enum
{
        SYS_RES_MODE_SINGLE     = 0x00,
        SYS_RES_MODE_CONTINUOUS = 0x01,
        SYS_RES_MODE_UNDEFINED  = 0x02
} SysResMode;

typedef enum
{
        SYS_TYPE_CODE_DEVICE_INFO = 0x04,
        SYS_TYPE_CODE_HEALTH      = 0x06,
        SYS_TYPE_CODE_SCAN        = 0x81,
        SYS_TYPE_CODE_UNDEFINED   = 0xFF
} SysTypeCode;

typedef struct
{
        uint8_t  scan_freq;
        uint16_t point_num;
} SysInfo;

#endif /* LIDAR_SYS_H */
