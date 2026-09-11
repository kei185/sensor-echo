#ifndef FRAME_H
#define FRAME_H
#include <stdint.h>

#define SOF_SIZE 2u
const uint8_t START_OF_FRAME[SOF_SIZE] = {0xAA, 0x55};

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

const char      MSG_DEVICE_READY[ACK_SIZE] = "DEVICE READY";
const Operation OP[COMMAND_NUM]            = {
        {.command = {0xAA, 0xA1}, .ack = "GET STAT ACK"},
        {.command = {0xAA, 0xA2}, .ack = "SRT SCAN ACK"},
        {.command = {0xAA, 0xA3}, .ack = "END SCAN ACK"},
};

#endif