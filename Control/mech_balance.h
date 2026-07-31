#ifndef MECH_BALANCE_H
#define MECH_BALANCE_H
#include <stdint.h>

/* 纯力学前馈补偿参数 (可通过串口调参) */
typedef struct {
    float gravity;              /* 重力加速度 m/s² */
    float accel_gain_fwd;       /* 前向加速增益 */
    float accel_gain_brake;     /* 制动增益 */
    float accel_bias;           /* 加速度零偏 m/s² */
    float theta_trim_deg;       /* 静态水平微调 ° */
    float theta_rate_limit;     /* 角度变化率限制 °/s */
    float pitch_deg;            /* 车体俯仰角 ° (无IMU则0) */
} MechParams_t;

void MechBalance_Init(void);
void MechBalance_Tick5ms(void);
void MechBalance_SetAccel(float ax_mps2);      /* 设置车辆纵向加速度 */
void MechBalance_SetParam(uint8_t id, float v); /* 调参 (id见枚举) */
const MechParams_t *MechBalance_GetParams(void);
void MechBalance_SetDirectAngle(float deg);    /* 手动倾角模式: 直接设摆杆角 */
void MechBalance_ExitDirect(void);             /* 退出手动模式, 回到力学补偿 */
uint8_t MechBalance_IsDirect(void);            /* 是否处于手动模式 */

/* 角度序列: 4步 (滚向D' → 减速 → 反向刹车 → 回水平), 每步角度/时长可调 */
void MechBalance_SetSeqAngle(uint8_t idx, float deg);   /* idx=0..3 */
void MechBalance_SetSeqTime(uint8_t idx, uint32_t ms);  /* idx=0..3 */
void MechBalance_StartSeq(void);
void MechBalance_StopSeq(void);
uint8_t MechBalance_IsSeqActive(void);

/* 调参ID */
#define MP_GRAVITY      0
#define MP_GAIN_FWD     1
#define MP_GAIN_BRAKE   2
#define MP_ACCEL_BIAS   3
#define MP_TRIM         4
#define MP_RATE_LIMIT   5
#define MP_PITCH        6

/* ---- 视觉PID精调模式 ---- */
typedef struct {
    uint8_t active;        /* 视觉PID是否运行 */
    uint8_t valid;         /* 最近一次视觉数据是否有效 */
    uint8_t fault;         /* 视觉/PWM/驱动故障后置1 */
    float   ball_pos_cm;   /* 最近一次有效球位置(cm) */
    float   setpoint_cm;   /* 目标位置(cm) */
    float   pid_out_deg;   /* 最近一次PID输出(°) */
} VisionStatus_t;

uint8_t MechBalance_StartVision(void);           /* 数据和PWM有效时启动 */
void MechBalance_StopVision(void);               /* 停止视觉PID, 回到力学模式 (W命令) */
uint8_t MechBalance_IsVisionActive(void);
uint8_t MechBalance_SetVisionSetpoint(float cm); /* 目标限制在安全轨道范围 */
void MechBalance_SetSeqAutoVision(uint8_t en);   /* 序列结束后自动启动视觉PID */
void MechBalance_GetVisionStatus(VisionStatus_t *s); /* S命令显示用 */
void MechBalance_EmergencyStop(void);

#endif
