#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct __attribute__((packed))
{
        uint16_t x; // roll
        uint16_t y; // pitch
        uint16_t z; // yaw
} Acceleration;

typedef struct __attribute__((packed))
{
        Acceleration trans;
        Acceleration rot;
} ImuRxBuf;

extern const ImuRxBuf* const IMU_RX_BUF;

extern bool imu_accel_ready;
extern bool imu_rot_ready;

#endif