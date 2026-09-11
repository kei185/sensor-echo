
#include <stdint.h>
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
