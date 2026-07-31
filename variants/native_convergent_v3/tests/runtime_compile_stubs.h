#ifndef NATIVE_V3_RUNTIME_COMPILE_STUBS_H
#define NATIVE_V3_RUNTIME_COMPILE_STUBS_H

#include <stdint.h>

#define CLOSED_LOOP_H
#define MOTOR_H
#define DEMO_CONFIG_H
#define ENCODER_H
#define PROTO_RX_H

#define MOTOR_AXIS_X 0
#define ENCODER_AXIS_X 0
#define VIS_SETPOINT_CM 0.0f
#define PWM_LIMIT_LOW 105.0f
#define PWM_LIMIT_HIGH 190.0f
#define PWM_LIMIT_MARGIN 2.0f

typedef enum { MOTOR_OK = 0, MOTOR_ERROR, MOTOR_BUSY } MotorStatus_t;
typedef enum {
    CL_FAULT_NONE = 0,
    CL_FAULT_NO_ENCODER,
    CL_FAULT_DIRECTION,
    CL_FAULT_DRIVER
} CL_Fault_t;

typedef struct {
    int16_t position_centi_cm;
    uint8_t confidence;
    uint8_t status;
    uint32_t age_ms;
    uint32_t frame_id;
} BallData_t;

MotorStatus_t CL_SetTargetAngle(int axis, float target_deg);
void CL_Stop(int axis);
float CL_GetCurrentAngle(int axis);
CL_Fault_t CL_GetFault(int axis);
uint8_t Encoder_GetPwmAngle(int axis, float *angle_deg);
uint8_t ProtoRx_GetBall(BallData_t *ball);

#endif
