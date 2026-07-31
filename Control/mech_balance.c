/**
 * mech_balance.c - 纯力学前馈补偿 + 4步角度序列
 *
 * 力学模式: θ_cmd = -arctan(a_x/g) - pitch + trim
 * 序列模式: 4步 (角度/时长均可调), K 触发, 结束回水平
 */
#include "mech_balance.h"
#include "closed_loop.h"
#include "board.h"
#include <math.h>

#define SEQ_STEPS 4U

static MechParams_t mp = {
    .gravity       = 9.80665f,
    .accel_gain_fwd   = 1.0f,
    .accel_gain_brake = 1.0f,
    .accel_bias    = 0.0f,
    .theta_trim_deg = 0.0f,
    .theta_rate_limit = 80.0f,
    .pitch_deg = 0.0f,
};

static float s_ax = 0.0f;
static float s_theta_prev = 0.0f;
static float s_phi_deg = 0.0f;
static float s_theta_cmd = 0.0f;
static uint8_t s_direct_mode = 0;
static float s_direct_deg = 0.0f;

/* 4步序列: 每步角度(°)和时长(ms)独立可调 */
static float s_seq_angles[SEQ_STEPS]   = { -2.0f, -0.5f, 1.5f, 0.0f };
static uint32_t s_seq_times[SEQ_STEPS] = { 2000U, 1000U, 1000U, 3000U };
static uint8_t s_seq_active = 0;
static uint8_t s_seq_step = 0;
static uint32_t s_seq_elapsed = 0;

static float clampf(float v, float lo, float hi) { return v<lo?lo:(v>hi?hi:v); }

/* 斜坡限制输出: 每5ms最多变化 rate*0.005 度 */
static float slew(float target)
{
    float max_change = mp.theta_rate_limit * 0.005f;
    s_theta_cmd = clampf(target, s_theta_prev - max_change, s_theta_prev + max_change);
    s_theta_prev = s_theta_cmd;
    return s_theta_cmd;
}

void MechBalance_Init(void)
{
    s_ax = 0.0f;
    s_theta_prev = 0.0f;
    s_phi_deg = 0.0f;
    s_theta_cmd = 0.0f;
    s_direct_mode = 0;
    s_direct_deg = 0.0f;
    s_seq_active = 0;
    s_seq_step = 0;
    s_seq_elapsed = 0;
}

void MechBalance_SetDirectAngle(float deg)
{
    s_direct_mode = 1;
    s_direct_deg = deg;
}

void MechBalance_ExitDirect(void) { s_direct_mode = 0; s_seq_active = 0; }
uint8_t MechBalance_IsDirect(void) { return s_direct_mode; }

/* 设置第 idx 步角度 (idx=0..3) */
void MechBalance_SetSeqAngle(uint8_t idx, float deg)
{
    if (idx < SEQ_STEPS) s_seq_angles[idx] = deg;
}

/* 设置第 idx 步时长 (idx=0..3, ms) */
void MechBalance_SetSeqTime(uint8_t idx, uint32_t ms)
{
    if (idx < SEQ_STEPS) s_seq_times[idx] = ms;
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
    case MP_RATE_LIMIT:  mp.theta_rate_limit = v; break;
    case MP_PITCH:       mp.pitch_deg = v;        break;
    }
}

const MechParams_t *MechBalance_GetParams(void) { return &mp; }

void MechBalance_Tick5ms(void)
{
    float gain, a_ff, phi_rad, theta_target;

    /* ---- 模式1: 4步角度序列 ---- */
    if (s_seq_active) {
        s_seq_elapsed += 5U;
        if (s_seq_elapsed >= s_seq_times[s_seq_step]) {
            s_seq_elapsed = 0U;
            if (s_seq_step < SEQ_STEPS - 1U) {
                s_seq_step++;                    /* 进入下一步 */
            } else {
                s_seq_active = 0;                /* 最后一步结束 */
                s_seq_step = 0;
                MechBalance_ExitDirect();        /* 退出直接模式, 回力学补偿(trim) */
            }
        }
        if (s_seq_active) {
            (void)CL_SetTargetAngle(MOTOR_AXIS_X, slew(s_seq_angles[s_seq_step]));
        }
        return;
    }

    /* ---- 模式2: 手动倾角 ---- */
    if (s_direct_mode) {
        (void)CL_SetTargetAngle(MOTOR_AXIS_X, slew(s_direct_deg));
        return;
    }

    /* ---- 模式3: 力学前馈补偿 ---- */
    gain = (s_ax >= 0.0f) ? mp.accel_gain_fwd : mp.accel_gain_brake;
    a_ff = gain * s_ax + mp.accel_bias;

    phi_rad = atan2f(-a_ff, mp.gravity);
    s_phi_deg = phi_rad * 57.29578f;

    theta_target = s_phi_deg - mp.pitch_deg + mp.theta_trim_deg;

    (void)CL_SetTargetAngle(MOTOR_AXIS_X, slew(theta_target));
}
