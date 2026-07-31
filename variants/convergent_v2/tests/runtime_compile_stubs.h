#ifndef BBV2_RUNTIME_COMPILE_STUBS_H
#define BBV2_RUNTIME_COMPILE_STUBS_H

#include <stdint.h>

#define CLOSED_LOOP_H
#define DEMO_CONFIG_H
#define ENCODER_H
#define MOTOR_H

#define MOTOR_AXIS_X 0U
#define ENCODER_AXIS_X 0U
#define CL_PERIOD_MS 5U
#define PWM_LIMIT_LOW 105.0f
#define PWM_LIMIT_HIGH 190.0f
#define PWM_LIMIT_MARGIN 2.0f

typedef enum {
    MOTOR_OK = 0,
    MOTOR_ERROR,
    MOTOR_BUSY
} MotorStatus_t;

float CL_GetCurrentAngle(uint8_t axis);
MotorStatus_t CL_SetTargetAngle(uint8_t axis, float target_deg);
void CL_Stop(uint8_t axis);
uint8_t Encoder_GetPwmAngle(uint8_t axis, float *angle_deg);

#endif
