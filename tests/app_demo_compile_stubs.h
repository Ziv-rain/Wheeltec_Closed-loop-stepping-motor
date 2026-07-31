#ifndef APP_DEMO_COMPILE_STUBS_H
#define APP_DEMO_COMPILE_STUBS_H

#include <stdint.h>

#define _BOARD_H_
#define APP_DEMO_H
#define CLOSED_LOOP_H
#define DEMO_CONFIG_H
#define ENCODER_H
#define MOTOR_H
#define PROTO_RX_H
#define BALL_CONTROL_H
#define TASK_CTRL_H

#define DEMO_SELECT 8
#define CL_PERIOD_MS 5U
#define D36A_MICROSTEP 32U
#define UART_0_INST 0U
#define MOTOR_AXIS_X 0U
#define ENCODER_AXIS_X 0U

typedef enum { MOTOR_OK = 0, MOTOR_ERROR, MOTOR_BUSY } MotorStatus_t;
typedef enum {
    CL_FAULT_NONE = 0,
    CL_FAULT_NO_ENCODER,
    CL_FAULT_DIRECTION,
    CL_FAULT_DRIVER
} CL_Fault_t;
typedef struct {
    int32_t current_count, target_count, error_count;
    float current_angle_deg, target_angle_deg;
    uint8_t active, reached;
    CL_Fault_t fault;
} CL_Snapshot_t;

void uart_puts(const char *text);
void uart_putf(float value, uint8_t decimals);
void uart_putu(uint32_t value);
void uart_puti(int32_t value);
void uart_putc(char value);
uint8_t DL_UART_Main_isRXFIFOEmpty(uint32_t instance);
uint32_t DL_UART_Main_receiveData(uint32_t instance);

void CL_Init(void);
void CL_Process(void);
void CL_StopAll(void);
void CL_Stop(uint8_t axis);
void CL_SetZero(uint8_t axis);
void CL_GetSnapshot(uint8_t axis, CL_Snapshot_t *snapshot);
float CL_GetCurrentAngle(uint8_t axis);
MotorStatus_t CL_SetTargetAngle(uint8_t axis, float angle);

void Encoder_Tick(uint32_t elapsed_ms);
uint8_t Encoder_GetPwmAngle(uint8_t axis, float *angle);
int32_t Encoder_GetZCount(uint8_t axis);

void ProtoRx_Init(void);
void ProtoRx_Tick(uint32_t elapsed_ms);
void TaskCtrl_Init(void);
void BallControl_Init(void);
void BallControl_Tick5ms(void);
uint8_t BallControl_IsHomingReady(void);
uint8_t BallControl_HasFault(void);
void BallControl_EmergencyStop(void);

#endif
