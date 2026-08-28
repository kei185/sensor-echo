
#ifndef MOTOR_H
#define MOTOR_H

enum MOTOR_MODE
{
        MOTOR_MODE_IN_IN,
        MOTOR_MODE_PHASE_ENABLE,
};

extern enum MOTOR_MODE GLOBAL_MOTOR_MODE;

void init_motor(void);
void set_motor_mode(enum MOTOR_MODE mode);
void motor_toggle(void);

#endif
