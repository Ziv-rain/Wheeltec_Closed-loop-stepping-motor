#ifndef _DRIVE_H
#define _DRIVE_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

void motor_left_forward(void);
void motor_left_stop(void);
void motor_left_backward(void);
void motor_right_forward(void);
void motor_right_stop(void);
void motor_right_backward(void);
void set_motor_speed(float left_duty, float right_duty);
uint8_t Drive_GetLeftDutyPercent(void);
uint8_t Drive_GetRightDutyPercent(void);
void Car_Move(double PL,double PR);
void PWM_D1(double Compare);
void PWM_D0(double Compare);

#endif
