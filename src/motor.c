
#include "main.h"
#include "motor.h"
#include "tx/frame.h"
#include <string.h>

// 車輪1回転あたりのCNTカウント数。
#define MOTOR_COUNTS_PER_REVOLUTION INT64_C(2800)
#define MOTOR_COUNTER_PERIOD        (MOTOR_COUNTS_PER_REVOLUTION * MOTOR_COUNTER_SCALER)

#define MOTOR_MAX_DUTY_PERCENT 100

enum MOTOR_MODE GLOBAL_MOTOR_MODE = MOTOR_MODE_PHASE_ENABLE;

const uint32_t   MOTOR_COUNTER_SCALER  = 10u;
volatile bool    motor_encoder_timeout = false;
volatile int32_t motor_right_counter   = 0u;
volatile int32_t motor_left_counter    = 0u;

void init_motor(void)
{
        motor_encoder_timeout = false;
        motor_right_counter = motor_left_counter = 0u;
        __HAL_TIM_SET_COUNTER(&htim3, 0u);
        __HAL_TIM_SET_COUNTER(&htim4, 0u);
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start_IT(&htim3);
        HAL_TIM_Base_Start_IT(&htim4);
        HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
        // 周回は更新割り込みで数える。エンコーダ入力ごとの割り込みは使わない。
        HAL_TIM_Base_Start_IT(&htim5);

        // ENABLEを0%にしてからPHASE/ENABLEモードとPWMを有効にする。
        motor_stop();
        set_motor_mode(MOTOR_MODE_PHASE_ENABLE);
        if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
                Error_Handler();
}

void set_motor_mode(enum MOTOR_MODE mode)
{
        if (mode == MOTOR_MODE_IN_IN)
                M_MODE_GPIO_Port->ODR &= ~M_MODE_Pin;

        if (mode == MOTOR_MODE_PHASE_ENABLE)
                M_MODE_GPIO_Port->ODR |= M_MODE_Pin;

        GLOBAL_MOTOR_MODE = mode;
}

void motor1_forward(int8_t velocity)
{
        if (velocity < 0 || velocity > MOTOR_MAX_DUTY_PERCENT)
                return;

        const uint32_t duty = velocity;
        HAL_GPIO_WritePin(M_PHASE1_GPIO_Port, M_PHASE1_Pin, GPIO_PIN_RESET);
        // PHASEは正転のまま、ENABLEのPWMだけを変更する。

        __HAL_TIM_SET_COMPARE(
                &htim2,
                TIM_CHANNEL_1,
                duty * (TIM2->ARR + 1u) / MOTOR_MAX_DUTY_PERCENT);
}

void motor2_forward(int8_t velocity)
{
        if (velocity < 0 || velocity > MOTOR_MAX_DUTY_PERCENT)
                return;

        const uint32_t duty = velocity;
        HAL_GPIO_WritePin(M_PHASE2_GPIO_Port, M_PHASE2_Pin, GPIO_PIN_RESET);
        // PHASEは正転のまま、ENABLEのPWMだけを変更する。

        __HAL_TIM_SET_COMPARE(
                &htim2,
                TIM_CHANNEL_2,
                duty * (TIM2->ARR + 1u) / MOTOR_MAX_DUTY_PERCENT);
}

void motor_stop(void)
{
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0u);
}

void motor_timeout_handler(uint8_t* buf)
{
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        int32_t  left_counter  = motor_left_counter;
        uint32_t left_count    = htim4.Instance->CNT;
        int32_t  right_counter = motor_right_counter;
        uint32_t right_count   = htim3.Instance->CNT;

        if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)) {
                left_counter += (htim4.Instance->CR1 & TIM_CR1_DIR) ? -1 : 1;
                left_count = htim4.Instance->CNT;
        }
        if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
                right_counter += (htim3.Instance->CR1 & TIM_CR1_DIR) ? -1 : 1;
                right_count = htim3.Instance->CNT;
        }

        motor_left_counter = htim4.Instance->CNT = 0u;
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        motor_right_counter = htim3.Instance->CNT = 0u;
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        // 左右を先に取得し、IRQ未処理の周回も含めてCNTと更新フラグをリセットする。
        // CNTを読み取ってからゼロにするまでの入力は失われる。

        __set_PRIMASK(primask);

        const int32_t left = (int32_t)(left_counter * MOTOR_COUNTER_PERIOD + left_count);
        const int32_t right =
                (int32_t)(right_counter * MOTOR_COUNTER_PERIOD + right_count);
        memcpy(buf, &left, sizeof left);
        memcpy(buf + sizeof left, &right, sizeof right);
}

void motor_command_handler(uint8_t* buf)
{
        const int8_t left  = (int8_t)buf[0];
        const int8_t right = (int8_t)buf[1];

        // 左右とも範囲内と確認してから、両輪の出力を更新する。
        if (left < 0 || left > MOTOR_MAX_DUTY_PERCENT || right < 0 ||
            right > MOTOR_MAX_DUTY_PERCENT)
                return;

        motor2_forward(left);
        motor1_forward(right);
        // 実機の配線は右がモーター1、左がモーター2。エンコーダは右TIM3・左TIM4。
}
