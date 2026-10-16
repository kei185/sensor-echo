
#include "main.h"
#include "motor.h"
#include "tx/frame.h"
#include <string.h>

// 車輪1回転あたりのCNTカウント数。
#define MOTOR_COUNTS_PER_REVOLUTION INT64_C(2800)
#define MOTOR_COUNTER_PERIOD        (MOTOR_COUNTS_PER_REVOLUTION * MOTOR_COUNTER_SCALER)

#define MOTOR_MAX_DUTY_PERCENT 100

enum MOTOR_MODE GLOBAL_MOTOR_MODE = MOTOR_MODE_PHASE_ENABLE;

const uint32_t MOTOR_COUNTER_SCALER  = 10u;
bool           motor_encoder_timeout = false;
uint32_t       motor_right_counter   = 0u;
uint32_t       motor_left_counter    = 0u;

void init_motor(void)
{
        motor_encoder_timeout = false;
        motor_right_counter = motor_left_counter = 0u;
        HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
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
        const uint32_t duty = velocity < 0 ? -(int32_t)velocity : velocity;
        HAL_GPIO_WritePin(
                M_PHASE1_GPIO_Port,
                M_PHASE1_Pin,
                velocity < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(
                &htim2,
                TIM_CHANNEL_1,
                duty * (TIM2->ARR + 1u) / MOTOR_MAX_DUTY_PERCENT);
}

void motor2_forward(int8_t velocity)
{
        const uint32_t duty = velocity < 0 ? -(int32_t)velocity : velocity;
        HAL_GPIO_WritePin(
                M_PHASE2_GPIO_Port,
                M_PHASE2_Pin,
                velocity < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

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
        // 周回数はIRQで反映済みの前提。取得とリセットの間はIRQを止める。
        // CNTの読み取りからゼロクリアまでに増えたカウントは失われる。
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        const int64_t left =
                (int32_t)motor_left_counter * MOTOR_COUNTER_PERIOD + htim4.Instance->CNT;
        motor_left_counter = htim4.Instance->CNT = 0u;

        const int64_t right =
                (int32_t)motor_right_counter * MOTOR_COUNTER_PERIOD + htim3.Instance->CNT;
        motor_right_counter = htim3.Instance->CNT = 0u;

        __set_PRIMASK(primask);

        const int16_t left_q6 =
                (int16_t)(left * WHEEL_ROTATION_Q6_SCALE / MOTOR_COUNTS_PER_REVOLUTION);
        memcpy(buf, &left_q6, sizeof left_q6);

        const int16_t right_q6 =
                (int16_t)(right * WHEEL_ROTATION_Q6_SCALE / MOTOR_COUNTS_PER_REVOLUTION);
        memcpy(buf + sizeof left_q6, &right_q6, sizeof right_q6);
}

void motor_command_handler(uint8_t* buf)
{
        const int8_t left  = (int8_t)buf[0];
        const int8_t right = (int8_t)buf[1];

        // 左右とも範囲内と確認してから、両輪の出力を更新する。
        if (left < -MOTOR_MAX_DUTY_PERCENT || left > MOTOR_MAX_DUTY_PERCENT ||
            right < -MOTOR_MAX_DUTY_PERCENT || right > MOTOR_MAX_DUTY_PERCENT)
                return;

        motor1_forward(left);
        motor2_forward(right);
}
