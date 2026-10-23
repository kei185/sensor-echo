#include <stdint.h>
#include <stdbool.h>

#ifndef MOTOR_H
#define MOTOR_H

enum MOTOR_MODE
{
        MOTOR_MODE_IN_IN,
        MOTOR_MODE_PHASE_ENABLE,
};

extern enum MOTOR_MODE GLOBAL_MOTOR_MODE;

extern const uint32_t   MOTOR_COUNTER_SCALER;
extern volatile bool    motor_encoder_timeout;
extern volatile int32_t motor_right_counter;
extern volatile int32_t motor_left_counter;

void init_motor(void);
void motor1_forward(int8_t velocity);
void motor2_forward(int8_t velocity);
void motor_stop(void);
void motor_command_handler(uint8_t*);
void motor_timeout_handler(uint8_t*);
void set_motor_mode(enum MOTOR_MODE mode);
void motor_toggle(void);

#endif
