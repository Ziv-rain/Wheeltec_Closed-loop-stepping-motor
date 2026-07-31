#ifndef MECH_BALANCE_H
#define MECH_BALANCE_H
#include <stdint.h>

/* 力学前馈补偿参数 (可通过串口调参) */
typedef struct {
    float gravity;
    float accel_gain_fwd;
    float accel_gain_brake;
    float accel_bias;
    float theta_trim_deg;
    float theta_rate_limit;
    float pitch_deg;
} MechParams_t;

void MechBalance_Init(void);
void MechBalance_Tick5ms(void);
void MechBalance_SetAccel(float ax_mps2);
void MechBalance_SetAccelManual(float ax_mps2); /* A命令: 手动加速度, 不被衰减 */
void MechBalance_SetParam(uint8_t id, float v);
const MechParams_t *MechBalance_GetParams(void);
void MechBalance_SetDirectAngle(float deg);
void MechBalance_ExitDirect(void);
uint8_t MechBalance_IsDirect(void);

#define MP_GRAVITY      0
#define MP_GAIN_FWD     1
#define MP_GAIN_BRAKE   2
#define MP_ACCEL_BIAS   3
#define MP_TRIM         4
#define MP_RATE_LIMIT   5
#define MP_PITCH        6
#define MP_FF_ENABLE    7

/* 视觉PID状态 */
typedef struct {
    uint8_t active;
    uint8_t valid;
    uint8_t fault;
    float   ball_pos_cm;
    float   setpoint_cm;
    float   pid_out_deg;
    float   ball_velocity_cm_s;
    float   vis_kp;
    float   vis_kd;
    float   vis_ki;
    float   out_min;
    float   out_max;
    float   ff_angle_deg;   /* 当前加速度前馈分量 (度) */
    uint8_t ff_merged;      /* 融合是否激活 */
    float   accel_mps2;     /* 当前加速度值 (m/s²) */
} VisionStatus_t;

uint8_t MechBalance_StartVision(void);
void    MechBalance_StopVision(void);
uint8_t MechBalance_IsVisionActive(void);
uint8_t MechBalance_SetVisionSetpoint(float cm);
void    MechBalance_SetVisionTargetOnly(float cm);
void    MechBalance_SetVisKp(float kp);
void    MechBalance_SetVisKd(float kd);
void    MechBalance_SetVisKi(float ki);
void    MechBalance_SetVisOutputMin(float omin);
void    MechBalance_SetVisOutputMax(float omax);
void    MechBalance_EnableFFMerge(uint8_t en);
uint8_t MechBalance_IsFFMerged(void);
void    MechBalance_SetFFLimit(float deg);   /* 前馈角度限幅 (串口E命令) */
void    MechBalance_GetVisionStatus(VisionStatus_t *s);
void    MechBalance_EmergencyStop(void);

#endif
