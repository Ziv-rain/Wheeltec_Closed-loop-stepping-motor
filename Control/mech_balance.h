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
    float theta_min_deg;        /* 摆杆角下限 ° */
    float theta_max_deg;        /* 摆杆角上限 ° */
    float theta_rate_limit;     /* 角度变化率限制 °/s */
    float pitch_deg;            /* 车体俯仰角 ° (无IMU则0) */
} MechParams_t;

void MechBalance_Init(void);
void MechBalance_Tick5ms(void);
void MechBalance_SetAccel(float ax_mps2);      /* 设置车辆纵向加速度 */
void MechBalance_SetParam(uint8_t id, float v); /* 调参 (id见枚举) */
const MechParams_t *MechBalance_GetParams(void);

/* 调参ID */
#define MP_GRAVITY      0
#define MP_GAIN_FWD     1
#define MP_GAIN_BRAKE   2
#define MP_ACCEL_BIAS   3
#define MP_TRIM         4
#define MP_MIN_DEG      5
#define MP_MAX_DEG      6
#define MP_RATE_LIMIT   7
#define MP_PITCH        8

#endif
