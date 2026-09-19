#ifndef FRAME_H
#define FRAME_H
#include <stdint.h>

#define SOF_SIZE             2u
#define TX_FRAME_HEADER_SIZE 10u
#define TX_FRAME_CAPACITY    1344u

extern const uint8_t START_OF_FRAME[SOF_SIZE];

typedef enum
{
        FRAME_TYPE_SYS = 0,
        FRAME_TYPE_LIDAR,
        FRAME_TYPE_IMU,
        FRAME_TYPE_ENC,
} FrameType;

typedef enum
{
        PC_COMMAND_GET_STATUS,
        PC_COMMAND_START_SCAN,
        PC_COMMAND_END_SCAN,
        PC_COMMAND_COUNT,
} PcCommand;

#define PC_COMMAND_SIZE 2u

extern const uint8_t PC_COMMANDS[PC_COMMAND_COUNT][PC_COMMAND_SIZE];

#endif
