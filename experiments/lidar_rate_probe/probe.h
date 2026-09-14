#ifndef LIDAR_RATE_PROBE_H
#define LIDAR_RATE_PROBE_H

#include "stm32f4xx_hal.h"

// Run one blocking measurement using UART4 (LiDAR) and USART2 (PC output).
// The probe uses the LiDAR's current scan setting and stops it when finished.
void lidar_rate_probe_run(UART_HandleTypeDef* lidar, UART_HandleTypeDef* pc);

#endif
