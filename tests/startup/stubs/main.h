#ifndef STARTUP_TEST_MAIN_H
#define STARTUP_TEST_MAIN_H

#include "stm32f4xx_hal_uart.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

typedef struct
{
        uint8_t unused;
} GPIO_TypeDef;

typedef enum
{
        GPIO_PIN_RESET = 0u,
        GPIO_PIN_SET,
} GPIO_PinState;

extern GPIO_TypeDef initial_handshake_failed_port;

#define INITIAL_HANDSHAKE_FAILED_GPIO_Port (&initial_handshake_failed_port)
#define INITIAL_HANDSHAKE_FAILED_Pin       0x0004u

void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);

#endif /* STARTUP_TEST_MAIN_H */
