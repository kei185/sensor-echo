#include "stdint.h"
#include "lidar/sys.h"

//  https://ydlidar.com/static/upload/file/20260615/1781511822498018.pdf
/* LIDAR TX Related */
const uint16_t MSG_SIZE = 2;

static const uint8_t SCAN[]           = {0xA5, 0x60};
static const uint8_t STOP[]           = {0xA5, 0x65};
static const uint8_t RX_SYS_INFO[]    = {0xA5, 0x90};
static const uint8_t RX_HEALTH[]      = {0xA5, 0x92};
static const uint8_t INC_FREQ_0_1HZ[] = {0xA5, 0x09};
static const uint8_t DEC_FREQ_0_1HZ[] = {0xA5, 0x0A};
static const uint8_t INC_FREQ_1HZ[]   = {0xA5, 0x0B};
static const uint8_t DEC_FREQ_1HZ[]   = {0xA5, 0x0C};
static const uint8_t RX_FREQ[]        = {0xA5, 0x0D};
static const uint8_t SOFT_RESTART[]   = {0xA5, 0x40};

const uint8_t* MSG[] = {
        [MSG_TYPE_SCAN]           = SCAN,
        [MSG_TYPE_STOP]           = STOP,
        [MSG_TYPE_RX_SYS_INFO]    = RX_SYS_INFO,
        [MSG_TYPE_RX_HEALTH]      = RX_HEALTH,
        [MSG_TYPE_INC_FREQ_0_1HZ] = INC_FREQ_0_1HZ,
        [MSG_TYPE_DEC_FREQ_0_1HZ] = DEC_FREQ_0_1HZ,
        [MSG_TYPE_INC_FREQ_1HZ]   = INC_FREQ_1HZ,
        [MSG_TYPE_DEC_FREQ_1HZ]   = DEC_FREQ_1HZ,
        [MSG_TYPE_RX_FREQ]        = RX_FREQ,
        [MSG_TYPE_SOFT_RESTART]   = SOFT_RESTART,
};

/* LIDAR RX Related */
// ranging frequency 4000hz
// suppose scan frequency is 0.1hz, content size per scan becomes 400
