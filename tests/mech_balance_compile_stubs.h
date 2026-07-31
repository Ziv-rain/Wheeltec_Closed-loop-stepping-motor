#ifndef MECH_BALANCE_COMPILE_STUBS_H
#define MECH_BALANCE_COMPILE_STUBS_H

#include <stdint.h>

#define CLOSED_LOOP_H
#define DEMO_CONFIG_H
#define ENCODER_H
#define MOTOR_H

#define MOTOR_MAX_ANGLE_POS 50.0f
#define MOTOR_MAX_ANGLE_NEG 35.0f
#define PWM_LIMIT_HIGH 190.0f
#define PWM_LIMIT_LOW 105.0f
#define PWM_LIMIT_MARGIN 2.0f

typedef enum { MOTOR_AXIS_X = 0 } MotorAxis_t;
typedef enum { MOTOR_OK = 0, MOTOR_ERROR, MOTOR_BUSY } MotorStatus_t;

MotorStatus_t CL_SetTargetAngle(MotorAxis_t axis, float target_deg);
void CL_Stop(MotorAxis_t axis);
float CL_GetCurrentAngle(MotorAxis_t axis);
uint8_t Encoder_GetPwmAngle(uint8_t axis, float *angle);

#define ENCODER_AXIS_X 0U

#endif
