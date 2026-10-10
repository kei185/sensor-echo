#ifndef STARTUP_TEST_MAIN_H
#define STARTUP_TEST_MAIN_H

#include "stm32f4xx_hal_uart.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

typedef struct
{
        uint32_t SynchPrediv;
} RTC_InitTypeDef;

typedef struct
{
        RTC_InitTypeDef Init;
} RTC_HandleTypeDef;

typedef struct
{
        uint8_t  Hours;
        uint8_t  Minutes;
        uint8_t  Seconds;
        uint8_t  TimeFormat;
        uint32_t SubSeconds;
        uint32_t SecondFraction;
        uint32_t DayLightSaving;
        uint32_t StoreOperation;
} RTC_TimeTypeDef;

typedef struct
{
        uint8_t WeekDay;
        uint8_t Month;
        uint8_t Date;
        uint8_t Year;
} RTC_DateTypeDef;

extern RTC_HandleTypeDef hrtc;

#define RTC_FORMAT_BIN           0u
#define RTC_DAYLIGHTSAVING_NONE  0u
#define RTC_STOREOPERATION_RESET 0u
#define RTC_SHIFTADD1S_SET       0x80000000u

HAL_StatusTypeDef
HAL_RTC_SetDate(RTC_HandleTypeDef* handle, RTC_DateTypeDef* date, uint32_t format);
HAL_StatusTypeDef
HAL_RTC_SetTime(RTC_HandleTypeDef* handle, RTC_TimeTypeDef* time, uint32_t format);
HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(
        RTC_HandleTypeDef* handle, uint32_t add_one_second, uint32_t subtract_fraction);

typedef struct
{
        uint8_t unused;
} GPIO_TypeDef;
typedef enum
{
        GPIO_PIN_RESET = 0u,
        GPIO_PIN_SET
} GPIO_PinState;
extern GPIO_TypeDef scanning_port;
extern GPIO_TypeDef initial_handshake_failed_port;
#define SCANNING_GPIO_Port                 (&scanning_port)
#define SCANNING_Pin                       0x0002u
#define INITIAL_HANDSHAKE_FAILED_GPIO_Port (&initial_handshake_failed_port)
#define INITIAL_HANDSHAKE_FAILED_Pin       0x0004u
void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);
#endif /* STARTUP_TEST_MAIN_H */
