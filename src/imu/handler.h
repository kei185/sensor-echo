#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct __attribute__((packed))
{
        uint16_t x; // roll
        uint16_t y; // pitch
        uint16_t z; // yaw
} Acceleration;

#define IMU_BUF_SIZE 2 * 6 // byte
typedef struct __attribute__((packed))
{
        Acceleration trans;
        Acceleration rot;
} ImuRxBuf;

extern const ImuRxBuf* const IMU_RX_BUF;

extern bool volatile imu_accel_ready;
extern bool volatile imu_rot_ready;

bool imu_handler(uint8_t*);
bool imu_setup(void);
#endif