#include "tx/frame.h"

const uint8_t START_OF_FRAME[SOF_SIZE] = {0xAA, 0x55};

const uint8_t HOST_COMMANDS[HOST_COMMAND_COUNT][HOST_COMMAND_SIZE] = {
        // [HOST_COMMAND_GET_STATUS] = {0xAA, 0xA1},
        [HOST_COMMAND_START_SCAN] = {0xAA, 0xA2},
        // [HOST_COMMAND_END_SCAN] = {0xAA, 0xA3},
        [HOST_COMMAND_TIME_SYNC_START] = {0xAA, 0xA4},
        [HOST_COMMAND_TIME]            = {0xAA, 0xA5},
};

const char FRAME_MESSAGE_INITIALIZING[]        = "INITIALIZING";
const char FRAME_MESSAGE_READY[]               = "READY";
const char FRAME_MESSAGE_STARTUP_FAILED[]      = "STARTUP FAILED";
const char FRAME_MESSAGE_START_SCAN_ACK[]      = "START SCAN ACK";
const char FRAME_MESSAGE_TIME_SYNC_START_ACK[] = "TIME SYNC START ACK";
const char FRAME_MESSAGE_TIME_ACK[]            = "TIME ACK";
