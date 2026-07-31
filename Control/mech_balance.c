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
static uint8_t s_direct_mode = 0; /* 手动倾角模式 */
static float s_direct_deg = 0.0f; /* 手动倾角值 */

/* 角度序列状态 */
static uint8_t s_seq_active = 0;
static uint8_t s_seq_step = 0;
static uint32_t s_seq_elapsed = 0;
static float s_seq_angles[4] = { -2.0f, -0.5f, 1.5f, 0.0f }; /* 默认: 滚向D'→减速→刹车→水平 */
static uint32_t s_seq_times[4] = { 2000U, 1000U, 1000U, 3000U }; /* 默认时长 ms */

static float clampf(float v, float lo, float hi) { return v<lo?lo:(v>hi?hi:v); }

void MechBalance_Init(void)
{
    s_ax = 0.0f;
    s_theta_prev = 0.0f;
    s_phi_deg = 0.0f;
    s_theta_cmd = 0.0f;
    s_direct_mode = 0;
    s_direct_deg = 0.0f;
}

void MechBalance_SetDirectAngle(float deg)
{
    s_direct_mode = 1;
    s_direct_deg = deg;
}

void MechBalance_ExitDirect(void) { s_direct_mode = 0; s_seq_active = 0; }
uint8_t MechBalance_IsDirect(void) { return s_direct_mode; }

void MechBalance_SetSeqAngle(uint8_t idx, float deg)
{
    if (idx < 4U) s_seq_angles[idx] = deg;
}

void MechBalance_SetSeqTime(uint8_t idx, uint32_t ms)
{
    if (idx < 4U) s_seq_times[idx] = ms;
}

void MechBalance_StartSeq(void)
{
    s_seq_active = 1;
    s_seq_step = 0;
    s_seq_elapsed = 0;
    s_direct_mode = 1;
}

void MechBalance_StopSeq(void) { s_seq_active = 0; }
uint8_t MechBalance_IsSeqActive(void) { return s_seq_active; }

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

    /* 0. 角度序列: 4步依次执行, 每步时长后切换下一步 */
    if (s_seq_active) {
        s_seq_elapsed += 5U;
        if (s_seq_elapsed >= s_seq_times[s_seq_step]) {
            s_seq_elapsed = 0U;
            if (s_seq_step < 3U) s_seq_step++;
            else { s_seq_active = 0; s_seq_step = 0; }  /* 序列结束, 回水平 */
        }
        s_direct_deg = s_seq_angles[s_seq_step];
        theta_target = clampf(s_direct_deg, mp.theta_min_deg, mp.theta_max_deg);
        max_change = mp.theta_rate_limit * 0.005f;
        s_theta_cmd = clampf(theta_target, s_theta_prev - max_change, s_theta_prev + max_change);
        s_theta_prev = s_theta_cmd;
        (void)CL_SetTargetAngle(MOTOR_AXIS_X, s_theta_cmd);
        return;
    }

    /* 0. 手动倾角模式: 直接输出, 球沿坡滚动 */
    if (s_direct_mode) {
        theta_target = s_direct_deg;
        theta_target = clampf(theta_target, mp.theta_min_deg, mp.theta_max_deg);
        max_change = mp.theta_rate_limit * 0.005f;
        s_theta_cmd = clampf(theta_target, s_theta_prev - max_change, s_theta_prev + max_change);
        s_theta_prev = s_theta_cmd;
        (void)CL_SetTargetAngle(MOTOR_AXIS_X, s_theta_cmd);
        return;
    }

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
