#ifndef LIDAR_PARSER_HEALTH_H
#define LIDAR_PARSER_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

typedef struct RxFrameHealth
{
        uint8_t health;
        bool    sensor_abnormal;
        bool    encoder_abnormal;
        bool    wireless_power_abnormal;
        bool    laser_feedback_abnormal;
        bool    laser_drive_abnormal;
        bool    lidar_data_abnormal;
} ParserHealth;

bool read_health_frame(ParserHealth*);

#endif /* LIDAR_PARSER_HEALTH_H */
