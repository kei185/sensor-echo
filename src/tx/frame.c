#include "tx/frame.h"

const uint8_t START_OF_FRAME[SOF_SIZE] = {0xAA, 0x55};

const uint8_t PC_COMMANDS[PC_COMMAND_COUNT][PC_COMMAND_SIZE] = {
        [PC_COMMAND_GET_STATUS] = {0xAA, 0xA1},
        [PC_COMMAND_START_SCAN] = {0xAA, 0xA2},
        [PC_COMMAND_END_SCAN]   = {0xAA, 0xA3},
};
