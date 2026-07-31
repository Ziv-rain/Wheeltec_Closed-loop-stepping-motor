#ifndef MECH_BALANCE_COMPILE_STUBS_H
#define MECH_BALANCE_COMPILE_STUBS_H

#include <stdint.h>

#define CLOSED_LOOP_H
#define MOTOR_H

typedef enum { MOTOR_AXIS_X = 0 } MotorAxis_t;
typedef enum { MOTOR_OK = 0, MOTOR_ERROR, MOTOR_BUSY } MotorStatus_t;

MotorStatus_t CL_SetTargetAngle(MotorAxis_t axis, float target_deg);
void CL_Stop(MotorAxis_t axis);
float CL_GetCurrentAngle(MotorAxis_t axis);

#endif
