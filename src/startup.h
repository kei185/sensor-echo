#ifndef STARTUP_H
#define STARTUP_H

#include <stdbool.h>
#include "stm32f4xx_hal_uart.h"

bool run_startup_sequence(void);
bool startup_uart_ok(
        UART_HandleTypeDef* uart, HAL_StatusTypeDef status, const char* operation);
// UARTの失敗理由を保存し、startup失敗時にUART2へまとめて通知する。

#endif /* STARTUP_H */
