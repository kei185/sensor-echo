#ifndef FRAME_H
#define FRAME_H
#include <stdint.h>

#define SOF_SIZE             2u
#define TX_FRAME_HEADER_SIZE 10u

extern const uint8_t START_OF_FRAME[SOF_SIZE];

typedef enum
{
        FRAME_TYPE_SYS = 0,
        FRAME_TYPE_LIDAR,
        FRAME_TYPE_IMU,
        FRAME_TYPE_ENC,
} FrameType;

typedef struct
{
        FrameType type;
        uint32_t  len;
        uint8_t*  payload_head;
} TxFrame;

#define COMMAND_NUM 4u
typedef enum
{
        COMMAND_DEVICE_READY,
        COMMAND_GET_STAT,
        COMMAND_SRT_SCAN,
        COMMAND_END_SCAN,
        UNDEFINED,
} COMMAND;

#define COMMAND_SIZE 2u
#define ACK_SIZE     13u
typedef struct
{
        uint8_t command[COMMAND_SIZE];
        char    ack[ACK_SIZE];
} Operation;

extern const char      MSG_DEVICE_READY[ACK_SIZE];
extern const Operation OP[COMMAND_NUM];

#endif
