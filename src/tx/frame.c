#include "tx/frame.h"

const uint8_t START_OF_FRAME[SOF_SIZE] = {0xAA, 0x55};

const uint8_t HOST_COMMANDS[HOST_COMMAND_COUNT][HOST_COMMAND_SIZE] = {
        // [HOST_COMMAND_GET_STATUS] = {0xAA, 0xA1},
        [HOST_COMMAND_START_SCAN] = {0xAA, 0xA2},
        // [HOST_COMMAND_END_SCAN] = {0xAA, 0xA3},
};
