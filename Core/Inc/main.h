/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

        /* Private includes ----------------------------------------------------------*/
        /* USER CODE BEGIN Includes */

        /* USER CODE END Includes */

        /* Exported types ------------------------------------------------------------*/
        /* USER CODE BEGIN ET */

        extern RTC_HandleTypeDef  hrtc;
        extern SPI_HandleTypeDef  hspi2;
        extern UART_HandleTypeDef huart4;
        extern UART_HandleTypeDef huart2;
        extern TIM_HandleTypeDef  htim2;
        extern TIM_HandleTypeDef  htim3;
        extern TIM_HandleTypeDef  htim4;

        extern DMA_HandleTypeDef hdma_uart4_rx;
        extern DMA_HandleTypeDef hdma_usart2_tx;

        /* USER CODE END ET */

        /* Exported constants --------------------------------------------------------*/
        /* USER CODE BEGIN EC */

        /* USER CODE END EC */

        /* Exported macro ------------------------------------------------------------*/
        /* USER CODE BEGIN EM */

        /* USER CODE END EM */

        /* Exported functions prototypes ---------------------------------------------*/
        void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin                    GPIO_PIN_13
#define B1_GPIO_Port              GPIOC
#define IMU_INT1_Pin              GPIO_PIN_0
#define IMU_INT1_GPIO_Port        GPIOC
#define IMU_INT1_EXTI_IRQn        EXTI0_IRQn
#define IMU_INT2_Pin              GPIO_PIN_3
#define IMU_INT2_GPIO_Port        GPIOC
#define IMU_INT2_EXTI_IRQn        EXTI3_IRQn
#define USART_TX_Pin              GPIO_PIN_2
#define USART_TX_GPIO_Port        GPIOA
#define USART_RX_Pin              GPIO_PIN_3
#define USART_RX_GPIO_Port        GPIOA
#define LD2_Pin                   GPIO_PIN_5
#define LD2_GPIO_Port             GPIOA
#define LIDAR_SCANNING_Pin        GPIO_PIN_1
#define LIDAR_SCANNING_GPIO_Port  GPIOB
#define SPI2_CS_Pin               GPIO_PIN_12
#define SPI2_CS_GPIO_Port         GPIOB
#define IMU_RUNNING_Pin           GPIO_PIN_13
#define IMU_RUNNING_GPIO_Port     GPIOB
#define MOTOR_RUNNING_Pin         GPIO_PIN_14
#define MOTOR_RUNNING_GPIO_Port   GPIOB
#define PROTOCOL_FAILED_Pin       GPIO_PIN_15
#define PROTOCOL_FAILED_GPIO_Port GPIOB
#define M_MODE2_Pin               GPIO_PIN_7
#define M_MODE2_GPIO_Port         GPIOC
#define BIN1_Pin                  GPIO_PIN_8
#define BIN1_GPIO_Port            GPIOC
#define BIN2_Pin                  GPIO_PIN_9
#define BIN2_GPIO_Port            GPIOC
#define TMS_Pin                   GPIO_PIN_13
#define TMS_GPIO_Port             GPIOA
#define TCK_Pin                   GPIO_PIN_14
#define TCK_GPIO_Port             GPIOA
#define AIN1_Pin                  GPIO_PIN_10
#define AIN1_GPIO_Port            GPIOC
#define AIN2_Pin                  GPIO_PIN_11
#define AIN2_GPIO_Port            GPIOC
#define M_MODE1_Pin               GPIO_PIN_12
#define M_MODE1_GPIO_Port         GPIOC
#define SWO_Pin                   GPIO_PIN_3
#define SWO_GPIO_Port             GPIOB

        /* USER CODE BEGIN Private defines */

        /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
