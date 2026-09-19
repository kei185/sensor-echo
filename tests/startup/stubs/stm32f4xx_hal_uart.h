#ifndef STARTUP_TEST_STM32F4XX_HAL_UART_H
#define STARTUP_TEST_STM32F4XX_HAL_UART_H

#include <stdint.h>

typedef enum
{
        HAL_OK      = 0x00u,
        HAL_ERROR   = 0x01u,
        HAL_BUSY    = 0x02u,
        HAL_TIMEOUT = 0x03u,
} HAL_StatusTypeDef;

typedef struct
{
        void* Instance;
} UART_HandleTypeDef;

#define HAL_MAX_DELAY UINT32_MAX

HAL_StatusTypeDef HAL_UART_Transmit(
        UART_HandleTypeDef* huart,
        const uint8_t*      data,
        uint16_t            length,
        uint32_t            timeout);
HAL_StatusTypeDef HAL_UART_Receive(
        UART_HandleTypeDef* huart, uint8_t* data, uint16_t length, uint32_t timeout);
uint32_t HAL_GetTick(void);

#endif /* STARTUP_TEST_STM32F4XX_HAL_UART_H */
