#include "tx/frame.h"

const uint8_t START_OF_FRAME[SOF_SIZE] = {0xAA, 0x55};

const char      MSG_DEVICE_READY[ACK_SIZE] = "DEVICE READY";
const Operation OP[COMMAND_NUM]            = {
        {.command = {0xAA, 0xA1}, .ack = "GET STAT ACK"},
        {.command = {0xAA, 0xA2}, .ack = "SRT SCAN ACK"},
        {.command = {0xAA, 0xA3}, .ack = "END SCAN ACK"},
};
