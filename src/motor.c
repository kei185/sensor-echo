
#include "main.h"
#include "motor.h"

enum MOTOR_MODE GLOBAL_MOTOR_MODE = MOTOR_MODE_PHASE_ENABLE;

void init_motor(void) {}

void set_motor_mode(enum MOTOR_MODE mode)
{

        if (mode == MOTOR_MODE_IN_IN)
                M_MODE_GPIO_Port->ODR &= ~M_MODE_Pin;

        if (mode == MOTOR_MODE_PHASE_ENABLE)
                M_MODE_GPIO_Port->ODR |= M_MODE_Pin;

        GLOBAL_MOTOR_MODE = mode;
}

void motor_forward(void)
{
        // phase
        AIN1_GPIO_Port->ODR &= ~AIN1_Pin;
        // enable
        AIN2_GPIO_Port->ODR |= AIN2_Pin;
}

void motor_stop(void) { AIN2_GPIO_Port->ODR &= ~AIN2_Pin; }

void motor_toggle(void)
{
        if ((AIN2_GPIO_Port->ODR & AIN2_Pin) == AIN2_Pin)
                motor_stop();
        else
                motor_forward();
}
