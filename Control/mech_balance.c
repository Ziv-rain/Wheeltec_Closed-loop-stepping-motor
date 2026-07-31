/**
 * mech_balance.c - 纯力学前馈补偿 + 4步角度序列
 *
 * 力学模式: θ_cmd = -arctan(a_x/g) - pitch + trim
 * 序列模式: 4步 (角度/时长均可调), K 触发, 结束回水平
 */
#include "mech_balance.h"
#include "closed_loop.h"
#include "board.h"
#include "pid.h"
#include "proto_rx.h"
#include "encoder.h"
#include "demo_config.h"
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
static float s_seq_angles[SEQ_STEPS]   = { -10.0f, 10.0f, 0.0f, 0.0f };
static uint32_t s_seq_times[SEQ_STEPS] = { 600U, 750U, 0U, 0U };
static uint8_t s_seq_active = 0;
static uint8_t s_seq_step = 0;
static uint32_t s_seq_elapsed = 0;

/* 视觉PID精调 */
static PID_t    s_vis_pid;
static uint8_t  s_vis_active = 0;       /* 视觉PID运行标志 */
static float    s_vis_setpoint_cm = VIS_SETPOINT_CM;
static uint8_t  s_seq_auto_vis = 0;     /* 序列结束自动启动视觉 */
static float    s_vis_ball_pos_cm = 0.0f; /* 状态显示缓存 */
static float    s_vis_pid_out = 0.0f;     /* 状态显示缓存 */
static uint8_t  s_vis_warned = 0;         /* 丢失警告去抖 */

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
    PID_Init(&s_vis_pid, VIS_KP, VIS_KI, VIS_KD, VIS_INTEGRAL_LIMIT,
             -VIS_OUTPUT_LIMIT_DEG, +VIS_OUTPUT_LIMIT_DEG);
    s_vis_active = 0;
    s_vis_setpoint_cm = VIS_SETPOINT_CM;
    s_seq_auto_vis = 0;
    s_vis_ball_pos_cm = 0.0f;
    s_vis_pid_out = 0.0f;
    s_vis_warned = 0;
}

void MechBalance_SetDirectAngle(float deg)
{
    MechBalance_StopVision();
    s_direct_mode = 1;
    s_direct_deg = deg;
}

void MechBalance_ExitDirect(void) { s_direct_mode = 0; s_seq_active = 0; s_vis_active = 0; PID_Reset(&s_vis_pid); }
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
    MechBalance_StopVision();
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

/* ---- 视觉PID精调 ---- */
static void VisHold(void)
{
    s_vis_active = 0;
    PID_Reset(&s_vis_pid);
    s_direct_mode = 1;
    s_direct_deg = s_theta_cmd;
    if (!s_vis_warned) {
        s_vis_warned = 1;
        uart_puts("VISION LOST\r\n");
    }
}

void MechBalance_StartVision(void)
{
    s_seq_active = 0;
    s_seq_step = 0;
    s_vis_active = 1;
    s_vis_warned = 0;
    s_direct_mode = 0;
    PID_Reset(&s_vis_pid);
}

void MechBalance_StopVision(void)
{
    if (s_vis_active) {
        s_vis_active = 0;
        PID_Reset(&s_vis_pid);
        s_vis_warned = 0;
    }
}

uint8_t MechBalance_IsVisionActive(void) { return s_vis_active; }

void MechBalance_SetVisionSetpoint(float cm) { s_vis_setpoint_cm = cm; }

void MechBalance_SetSeqAutoVision(uint8_t en) { s_seq_auto_vis = en; }

void MechBalance_GetVisionStatus(VisionStatus_t *s)
{
    if (s) {
        s->active = s_vis_active;
        s->ball_pos_cm = s_vis_ball_pos_cm;
        s->setpoint_cm = s_vis_setpoint_cm;
        s->pid_out_deg = s_vis_pid_out;
    }
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
                s_direct_mode = 0;
                if (s_seq_auto_vis) {
                    MechBalance_StartVision();   /* DEMO8: 粗定位后自动进视觉精调 */
                } else {
                    MechBalance_ExitDirect();    /* DEMO7: 原行为, 回力学补偿(trim) */
                }
            }
        }
        if (s_seq_active) {
            (void)CL_SetTargetAngle(MOTOR_AXIS_X, slew(s_seq_angles[s_seq_step]));
        }
        return;
    }

    /* ---- 模式2: 视觉PID精调 ---- */
    if (s_vis_active) {
        BallData_t b; float bp, out, pw, ca;
        if (!ProtoRx_GetBall(&b)) { VisHold(); return; }
        if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, &pw) ||
            pw < PWM_LIMIT_LOW - PWM_LIMIT_MARGIN ||
            pw > PWM_LIMIT_HIGH + PWM_LIMIT_MARGIN) { VisHold(); return; }
        bp = (float)b.position_centi_cm / 100.0f;
        s_vis_ball_pos_cm = bp;
        out = PID_Update(&s_vis_pid, bp - s_vis_setpoint_cm, 0.005f);
        ca = CL_GetCurrentAngle(MOTOR_AXIS_X);
        if (pw >= PWM_LIMIT_HIGH - PWM_LIMIT_MARGIN && out > ca) out = ca;
        if (pw <= PWM_LIMIT_LOW  + PWM_LIMIT_MARGIN && out < ca) out = ca;
        s_vis_pid_out = out;
        (void)CL_SetTargetAngle(MOTOR_AXIS_X, slew(out));
        return;
    }

    /* ---- 模式3: 手动倾角 ---- */
    if (s_direct_mode) {
        (void)CL_SetTargetAngle(MOTOR_AXIS_X, slew(s_direct_deg));
        return;
    }

    /* ---- 模式4: 力学前馈补偿 ---- */
    gain = (s_ax >= 0.0f) ? mp.accel_gain_fwd : mp.accel_gain_brake;
    a_ff = gain * s_ax + mp.accel_bias;

    phi_rad = atan2f(-a_ff, mp.gravity);
    s_phi_deg = phi_rad * 57.29578f;

    theta_target = s_phi_deg - mp.pitch_deg + mp.theta_trim_deg;

    (void)CL_SetTargetAngle(MOTOR_AXIS_X, slew(theta_target));
}
