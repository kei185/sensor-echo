#ifndef IMU_FRAME_HJ
#define IMU_FRAME_HJ

#include <stdint.h>
typedef enum
{
        RW_WRITE = 0,
        RW_READ  = 1
} RW;

typedef struct
{
        RW      rw;
        uint8_t address;

} ImuFrame;

#endif