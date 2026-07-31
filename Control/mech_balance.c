/**
 * mech_balance.c - 力学前馈 + 视觉PID (第4/5问)
 *
 * 模式优先级: 视觉PID > 手动倾角 > 力学前馈
 * PID 每 5ms 运行一次(200Hz), 不依赖视觉帧率
 */
#include "mech_balance.h"
#include "closed_loop.h"
#include "pid.h"
#include "proto_rx.h"
#include "encoder.h"
#include "demo_config.h"
#include <math.h>

#define MECH_ANGLE_MIN_DEG (-30.0f)
#define MECH_ANGLE_MAX_DEG 45.0f

static MechParams_t mp = {
    .gravity = 9.80665f,
    .accel_gain_fwd = 1.0f,
    .accel_gain_brake = 1.0f,
    .accel_bias = 0.0f,
    .theta_trim_deg = 0.0f,
    .theta_rate_limit = 80.0f,
    .pitch_deg = 0.0f,
};

static float s_ax = 0.0f;
static float s_theta_prev = 0.0f;
static float s_theta_cmd = 0.0f;
static uint8_t s_direct_mode = 0;
static float s_direct_deg = 0.0f;

/* 视觉PID */
static PID_t    s_vis_pid;
static uint8_t  s_vis_active = 0;
static float    s_vis_setpoint_cm = 0.0f;
static float    s_vis_ball_pos_cm = 0.0f;
static float    s_vis_pid_out = 0.0f;
static uint8_t  s_vis_valid = 0;
static uint8_t  s_emergency_stop = 0;
static uint32_t s_vis_last_frame_id = 0U;
static uint32_t s_vis_sample_elapsed_ms = 0U;
static float    s_vis_last_sample_cm = 0.0f;
static float    s_vis_velocity_cm_s = 0.0f;
static uint8_t  s_vis_sample_initialized = 0U;

static float clampf(float v, float lo, float hi) { return v<lo?lo:(v>hi?hi:v); }

static void VisionEstimatorReset(void)
{
    s_vis_last_frame_id = 0U;
    s_vis_sample_elapsed_ms = 0U;
    s_vis_last_sample_cm = 0.0f;
    s_vis_velocity_cm_s = 0.0f;
    s_vis_sample_initialized = 0U;
}

static uint8_t VisionEstimatorUpdate(const BallData_t *b, float *position_cm,
                                     float *sample_dt_s)
{
    float bp, dt, raw_velocity;
    if (!b || b->frame_id == s_vis_last_frame_id) return 0U;
    bp = (float)b->position_centi_cm / 100.0f;
    dt = (float)s_vis_sample_elapsed_ms / 1000.0f;
    if (!s_vis_sample_initialized || dt < 0.005f) {
        s_vis_velocity_cm_s = 0.0f;
        s_vis_sample_initialized = 1U;
    } else {
        raw_velocity = (bp - s_vis_last_sample_cm) / dt;
        s_vis_velocity_cm_s =
            VIS_VELOCITY_FILTER_ALPHA * s_vis_velocity_cm_s +
            (1.0f - VIS_VELOCITY_FILTER_ALPHA) * raw_velocity;
    }
    s_vis_last_sample_cm = bp;
    s_vis_last_frame_id = b->frame_id;
    s_vis_sample_elapsed_ms = 0U;
    if (position_cm) *position_cm = bp;
    if (sample_dt_s) *sample_dt_s = dt;
    return 1U;
}

static float slew(float target)
{
    float max_change = mp.theta_rate_limit * 0.005f;
    target = clampf(target, MECH_ANGLE_MIN_DEG, MECH_ANGLE_MAX_DEG);
    s_theta_cmd = clampf(target, s_theta_prev - max_change, s_theta_prev + max_change);
    s_theta_prev = s_theta_cmd;
    return s_theta_cmd;
}

void MechBalance_Init(void)
{
    s_ax = 0.0f;
    s_theta_prev = 0.0f;
    s_theta_cmd = 0.0f;
    s_direct_mode = 0;
    s_direct_deg = 0.0f;
    PID_Init(&s_vis_pid, VIS_KP, VIS_KI, VIS_KD, VIS_INTEGRAL_LIMIT,
             VIS_OUTPUT_MIN_DEG, VIS_OUTPUT_MAX_DEG);
    s_vis_active = 1;  /* 上电自动启动PID, 回零完成后立即生效 */
    s_vis_setpoint_cm = 0.0f;
    s_vis_ball_pos_cm = 0.0f;
    s_vis_pid_out = 0.0f;
    s_vis_valid = 0;
    s_emergency_stop = 0;
    VisionEstimatorReset();
}

void MechBalance_SetDirectAngle(float deg)
{
    if (s_emergency_stop || deg < MECH_ANGLE_MIN_DEG || deg > MECH_ANGLE_MAX_DEG) return;
    MechBalance_StopVision();
    s_direct_mode = 1;
    s_direct_deg = deg;
}

void MechBalance_ExitDirect(void) { s_direct_mode = 0; s_vis_active = 0; PID_Reset(&s_vis_pid); }
uint8_t MechBalance_IsDirect(void) { return s_direct_mode; }
void MechBalance_SetAccel(float ax_mps2) { s_ax = ax_mps2; }

uint8_t MechBalance_StartVision(void)
{
    if (s_emergency_stop) return 0U;
    s_vis_active = 1;
    s_direct_mode = 0;
    s_vis_valid = 0;
    s_vis_pid_out = mp.theta_trim_deg;
    PID_Reset(&s_vis_pid);
    VisionEstimatorReset();
    return 1U;
}

void MechBalance_StopVision(void)
{
    if (s_vis_active) { s_vis_active = 0; PID_Reset(&s_vis_pid); }
}

uint8_t MechBalance_IsVisionActive(void) { return s_vis_active; }

uint8_t MechBalance_SetVisionSetpoint(float cm)
{
    if (cm != cm || cm < -VIS_SETPOINT_LIMIT_CM || cm > VIS_SETPOINT_LIMIT_CM) return 0U;
    s_vis_setpoint_cm = cm;
    PID_Reset(&s_vis_pid);
    return 1U;
}

void MechBalance_SetVisionTargetOnly(float cm)
{
    if (cm != cm || cm < -VIS_SETPOINT_LIMIT_CM || cm > VIS_SETPOINT_LIMIT_CM) return;
    s_vis_setpoint_cm = cm;
}

void MechBalance_SetVisKp(float kp) { if (kp > 0.0f) { s_vis_pid.kp = kp; PID_Reset(&s_vis_pid); } }
void MechBalance_SetVisKd(float kd) { if (kd >= 0.0f) { s_vis_pid.kd = kd; PID_Reset(&s_vis_pid); } }
void MechBalance_SetVisKi(float ki) { s_vis_pid.ki = ki; PID_Reset(&s_vis_pid); }
void MechBalance_SetVisOutputMin(float omin) { s_vis_pid.output_min = omin; }
void MechBalance_SetVisOutputMax(float omax) { s_vis_pid.output_max = omax; }

void MechBalance_GetVisionStatus(VisionStatus_t *s)
{
    if (s) {
        s->active = s_vis_active;
        s->valid = s_vis_valid;
        s->fault = s_emergency_stop;
        s->ball_pos_cm = s_vis_ball_pos_cm;
        s->setpoint_cm = s_vis_setpoint_cm;
        s->pid_out_deg = s_vis_pid_out;
        s->ball_velocity_cm_s = s_vis_velocity_cm_s;
        s->vis_kp = s_vis_pid.kp;
        s->vis_kd = s_vis_pid.kd;
        s->vis_ki = s_vis_pid.ki;
        s->out_min = s_vis_pid.output_min;
        s->out_max = s_vis_pid.output_max;
    }
}

void MechBalance_EmergencyStop(void)
{
    s_emergency_stop = 1U;
    s_vis_active = 0U;
    s_direct_mode = 0U;
    PID_Reset(&s_vis_pid);
    CL_Stop(MOTOR_AXIS_X);
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
    BallData_t ball;
    float gain, a_ff, phi_rad, theta_target, pwm;
    float bp = s_vis_last_sample_cm, sample_dt = 0.005f;
    uint8_t ball_valid, new_ball;

    if (s_emergency_stop) return;

    /* 更新视觉数据 */
    if (s_vis_sample_elapsed_ms <= 0xFFFFFFFFU - 5U) {
        s_vis_sample_elapsed_ms += 5U;
    }
    ball_valid = ProtoRx_GetBall(&ball);
    new_ball = ball_valid ? VisionEstimatorUpdate(&ball, &bp, &sample_dt) : 0U;
    if (!ball_valid) s_vis_valid = 0U;
    if (new_ball) {
        s_vis_ball_pos_cm = bp;
        s_vis_valid = 1U;
    }

    /* PWM 信号丢失 -> 跳过本周期, 等待恢复 */
    if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm)) return;
    /* PWM 越界 -> 钳位保护, 不退出PID (仅X命令或断电可停止) */
    if (pwm <= PWM_LIMIT_LOW || pwm >= PWM_LIMIT_HIGH) {
        s_vis_pid_out = clampf(s_vis_pid_out,
            MECH_ANGLE_MIN_DEG, MECH_ANGLE_MAX_DEG);
    }

    /* ---- 视觉PID模式 ---- */
    if (s_vis_active) {
        float ca, error, out, use_vel;
        float pos = s_vis_ball_pos_cm;

        /* 新帧: 更新位置+速度; 旧帧: 保持上次速度, D项持续提供阻尼防振荡 */
        if (new_ball) {
            pos = clampf(bp, -VIS_SETPOINT_LIMIT_CM, VIS_SETPOINT_LIMIT_CM);
            s_vis_ball_pos_cm = pos;
            use_vel = s_vis_velocity_cm_s;
        } else if (ball_valid) {
            use_vel = s_vis_velocity_cm_s;
        } else {
            /* 数据超时: 保持当前角度, 等待恢复 */
            (void)CL_SetTargetAngle(MOTOR_AXIS_X, s_theta_cmd);
            return;
        }

        /* 每5ms运行一次PID --- 200Hz高频调整, 不依赖视觉帧率 */
        error = pos - s_vis_setpoint_cm;
        s_vis_pid_out = PID_UpdateWithDerivative(&s_vis_pid, error,
            use_vel, 0.005f);
        s_vis_pid_out = clampf(s_vis_pid_out + mp.theta_trim_deg,
                               VIS_OUTPUT_MIN_DEG, VIS_OUTPUT_MAX_DEG);

        out = s_vis_pid_out;
        ca = CL_GetCurrentAngle(MOTOR_AXIS_X);
        if (pwm >= PWM_LIMIT_HIGH - PWM_LIMIT_MARGIN && out > ca) out = ca;
        if (pwm <= PWM_LIMIT_LOW  + PWM_LIMIT_MARGIN && out < ca) out = ca;
        out = clampf(out, MECH_ANGLE_MIN_DEG, MECH_ANGLE_MAX_DEG);

        if (CL_SetTargetAngle(MOTOR_AXIS_X, slew(out)) != MOTOR_OK) {
            return;
        }
        return;
    }

    /* ---- 手动倾角模式 ---- */
    if (s_direct_mode) {
        (void)CL_SetTargetAngle(MOTOR_AXIS_X, slew(s_direct_deg));
        return;
    }

    /* ---- 力学前馈补偿 (默认) ---- */
    gain = (s_ax >= 0.0f) ? mp.accel_gain_fwd : mp.accel_gain_brake;
    a_ff = gain * s_ax + mp.accel_bias;
    phi_rad = atan2f(-a_ff, mp.gravity);
    theta_target = phi_rad * 57.29578f - mp.pitch_deg + mp.theta_trim_deg;
    (void)CL_SetTargetAngle(MOTOR_AXIS_X, slew(theta_target));
}
