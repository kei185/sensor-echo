#include <stdbool.h>
#include <stddef.h>
#include "lidar/core.h"
#include "lidar/parser/health.h"

/**
 * HEALTH
 */

static bool health_parse(ParserHealth* this)
{
        if (this->health == 0U)
                return false;

        this->sensor_abnormal         = (this->health & (1U << 0)) != 0U;
        this->encoder_abnormal        = (this->health & (1U << 1)) != 0U;
        this->wireless_power_abnormal = (this->health & (1U << 2)) != 0U;
        this->laser_feedback_abnormal = (this->health & (1U << 3)) != 0U;
        this->laser_drive_abnormal    = (this->health & (1U << 4)) != 0U;
        this->lidar_data_abnormal     = (this->health & (1U << 5)) != 0U;

        return true;
}

bool read_health_frame(ParserHealth* health)
{
        if (health == NULL)
                return false;

        health->health = (uint8_t)read_byte();

        health_parse(health);
        return true;
}
