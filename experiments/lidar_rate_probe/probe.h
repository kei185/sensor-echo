#ifndef LIDAR_RATE_PROBE_H
#define LIDAR_RATE_PROBE_H

#include "stm32f4xx_hal.h"

void lidar_rate_probe_run(UART_HandleTypeDef* lidar, UART_HandleTypeDef* pc);

#endif
