/**
 * mech_balance.c - 纯力学前馈补偿
 *
 * 核心公式: φ_ff = atan2(-a_x, g)
 * 机构角:   θ_cmd = φ_ff - β + θ_trim
 * 限幅 + 斜坡限制 -> CL_SetTargetAngle
 *
 * 不使用视觉位置误差, 不写位置PID
 */
#include "mech_balance.h"
#include "closed_loop.h"
#include "board.h"
#include <math.h>

static MechParams_t mp = {
    .gravity       = 9.80665f,
    .accel_gain_fwd   = 1.0f,
    .accel_gain_brake = 1.0f,
    .accel_bias    = 0.0f,
    .theta_trim_deg = 0.0f,
    .theta_min_deg  = -3.0f,
    .theta_max_deg  =  3.0f,
    .theta_rate_limit = 80.0f,
    .pitch_deg = 0.0f,
};

static float s_ax = 0.0f;        /* 当前车辆加速度 m/s² */
static float s_theta_prev = 0.0f; /* 上次输出的角度 (斜坡限制用) */
static float s_phi_deg = 0.0f;    /* 当前前馈绝对角 */
static float s_theta_cmd = 0.0f;  /* 当前机构命令角 */

static float clampf(float v, float lo, float hi) { return v<lo?lo:(v>hi?hi:v); }

void MechBalance_Init(void)
{
    s_ax = 0.0f;
    s_theta_prev = 0.0f;
    s_phi_deg = 0.0f;
    s_theta_cmd = 0.0f;
}

void MechBalance_SetAccel(float ax_mps2)
{
    s_ax = ax_mps2;
}

void MechBalance_SetParam(uint8_t id, float v)
{
    switch (id) {
    case MP_GRAVITY:     mp.gravity = v;         break;
    case MP_GAIN_FWD:    mp.accel_gain_fwd = v;   break;
    case MP_GAIN_BRAKE:  mp.accel_gain_brake = v; break;
    case MP_ACCEL_BIAS:  mp.accel_bias = v;       break;
    case MP_TRIM:        mp.theta_trim_deg = v;   break;
    case MP_MIN_DEG:     mp.theta_min_deg = v;    break;
    case MP_MAX_DEG:     mp.theta_max_deg = v;    break;
    case MP_RATE_LIMIT:  mp.theta_rate_limit = v; break;
    case MP_PITCH:       mp.pitch_deg = v;        break;
    }
}

const MechParams_t *MechBalance_GetParams(void) { return &mp; }

void MechBalance_Tick5ms(void)
{
    float gain, a_ff, phi_rad, theta_target, max_change;

    /* 1. 加速度前馈: 加速和制动用不同增益 */
    gain = (s_ax >= 0.0f) ? mp.accel_gain_fwd : mp.accel_gain_brake;
    a_ff = gain * s_ax + mp.accel_bias;

    /* 2. 纯力学补偿角 φ_ff = atan2(-a_ff, g) (弧度) */
    phi_rad = atan2f(-a_ff, mp.gravity);
    s_phi_deg = phi_rad * 57.29578f;  /* rad -> deg */

    /* 3. 机构相对角 θ_cmd = φ_ff - pitch + trim */
    theta_target = s_phi_deg - mp.pitch_deg + mp.theta_trim_deg;

    /* 4. 角度限幅 */
    theta_target = clampf(theta_target, mp.theta_min_deg, mp.theta_max_deg);

    /* 5. 斜坡限制 (角度变化率) */
    max_change = mp.theta_rate_limit * 0.005f;  /* dt=5ms */
    s_theta_cmd = clampf(theta_target, s_theta_prev - max_change, s_theta_prev + max_change);
    s_theta_prev = s_theta_cmd;

    /* 6. 发给电机闭环 */
    (void)CL_SetTargetAngle(MOTOR_AXIS_X, s_theta_cmd);
}
