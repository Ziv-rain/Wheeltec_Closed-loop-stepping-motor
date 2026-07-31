/**
 * mech_balance.c - 纯力学前馈补偿 + 4步角度序列
 *
 * 力学模式: θ_cmd = -arctan(a_x/g) - pitch + trim
 * 序列模式: 4步 (角度/时长均可调), K 触发, 结束回水平
 */
#include "mech_balance.h"
#include "closed_loop.h"
#include "pid.h"
#include "proto_rx.h"
#include "encoder.h"
#include "demo_config.h"
#include <math.h>

#define SEQ_STEPS 4U
#define MECH_ANGLE_MIN_DEG (-30.0f)
#define MECH_ANGLE_MAX_DEG 45.0f
#define SEQ_TIME_MIN_MS 5U
#define SEQ_TIME_MAX_MS 10000U

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
static float s_seq_angles[SEQ_STEPS]   = { -10.0f, 6.0f, -3.0f, 0.0f };
static uint32_t s_seq_times[SEQ_STEPS] = { 600U, 600U, 300U, 150U };
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
static uint8_t  s_vis_valid = 0;
static uint8_t  s_vis_fault = 0;
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

/* Only update velocity for a new camera frame, never for a cached sample. */
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

static void SeqNextStep(void)
{
    s_seq_elapsed = 0U;
    if (s_seq_step < SEQ_STEPS - 1U) s_seq_step++;
}

/* 斜坡限制输出: 每5ms最多变化 rate*0.005 度 */
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
    s_phi_deg = 0.0f;
    s_theta_cmd = 0.0f;
    s_direct_mode = 0;
    s_direct_deg = 0.0f;
    s_seq_active = 0;
    s_seq_step = 0;
    s_seq_elapsed = 0;
    PID_Init(&s_vis_pid, VIS_KP, VIS_KI, VIS_KD, VIS_INTEGRAL_LIMIT,
             VIS_OUTPUT_MIN_DEG, VIS_OUTPUT_MAX_DEG);
    s_vis_active = 0;
    s_vis_setpoint_cm = VIS_SETPOINT_CM;
    s_seq_auto_vis = 0;
    s_vis_ball_pos_cm = 0.0f;
    s_vis_pid_out = 0.0f;
    s_vis_valid = 0;
    s_vis_fault = 0;
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

void MechBalance_ExitDirect(void) { s_direct_mode = 0; s_seq_active = 0; s_vis_active = 0; PID_Reset(&s_vis_pid); }
uint8_t MechBalance_IsDirect(void) { return s_direct_mode; }

/* 设置第 idx 步角度 (idx=0..3) */
void MechBalance_SetSeqAngle(uint8_t idx, float deg)
{
    if (!s_seq_active && idx < SEQ_STEPS &&
        deg >= MECH_ANGLE_MIN_DEG && deg <= MECH_ANGLE_MAX_DEG) {
        s_seq_angles[idx] = deg;
    }
}

/* 设置第 idx 步时长 (idx=0..3, ms) */
void MechBalance_SetSeqTime(uint8_t idx, uint32_t ms)
{
    if (!s_seq_active && idx < SEQ_STEPS &&
        ms >= SEQ_TIME_MIN_MS && ms <= SEQ_TIME_MAX_MS &&
        (ms % 5U) == 0U) {
        s_seq_times[idx] = ms;
    }
}

void MechBalance_StartSeq(void)
{
    if (s_emergency_stop) return;
    MechBalance_StopVision();
    s_seq_active = 1;
    s_seq_step = 0;
    s_seq_elapsed = 0;
    s_direct_mode = 1;
    VisionEstimatorReset();
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
    s_vis_valid = 0;
    s_vis_fault = 1;
    PID_Reset(&s_vis_pid);
    s_direct_mode = 1;
    s_direct_deg = mp.theta_trim_deg;
    s_vis_pid_out = mp.theta_trim_deg;
}

uint8_t MechBalance_StartVision(void)
{
    if (s_emergency_stop) return 0U;
    s_seq_active = 0;
    s_seq_step = 0;
    s_vis_active = 1;
    s_direct_mode = 0;
    s_vis_valid = 0;
    s_vis_fault = 0;
    s_vis_pid_out = mp.theta_trim_deg;
    PID_Reset(&s_vis_pid);
    VisionEstimatorReset();
    return 1U;
}

void MechBalance_StopVision(void)
{
    if (s_vis_active) {
        s_vis_active = 0;
        PID_Reset(&s_vis_pid);
    }
}

uint8_t MechBalance_IsVisionActive(void) { return s_vis_active; }

uint8_t MechBalance_SetVisionSetpoint(float cm)
{
    if (cm != cm || cm < -VIS_SETPOINT_LIMIT_CM ||
        cm > VIS_SETPOINT_LIMIT_CM) return 0U;
    s_vis_setpoint_cm = cm;
    PID_Reset(&s_vis_pid);
    return 1U;
}

void MechBalance_SetSeqAutoVision(uint8_t en) { s_seq_auto_vis = en; }

uint8_t MechBalance_GetSeqAutoVision(void) { return s_seq_auto_vis; }

uint8_t MechBalance_ToggleSeqAutoVision(void)
{
    s_seq_auto_vis = (uint8_t)!s_seq_auto_vis;
    return s_seq_auto_vis;
}

void MechBalance_SetVisKp(float kp) { if (kp > 0.0f) { s_vis_pid.kp = kp; PID_Reset(&s_vis_pid); } }
void MechBalance_SetVisKd(float kd) { if (kd >= 0.0f) { s_vis_pid.kd = kd; PID_Reset(&s_vis_pid); } }
void MechBalance_SetVisKi(float ki) { s_vis_pid.ki = ki; PID_Reset(&s_vis_pid); }

void MechBalance_GetVisionStatus(VisionStatus_t *s)
{
    if (s) {
        s->active = s_vis_active;
        s->valid = s_vis_valid;
        s->fault = s_vis_fault;
        s->ball_pos_cm = s_vis_ball_pos_cm;
        s->setpoint_cm = s_vis_setpoint_cm;
        s->pid_out_deg = s_vis_pid_out;
        s->ball_velocity_cm_s = s_vis_velocity_cm_s;
        s->seq_step = s_seq_active ? (uint8_t)(s_seq_step + 1U) : 0U;
    }
}

void MechBalance_EmergencyStop(void)
{
    s_emergency_stop = 1U;
    s_seq_active = 0U;
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
    if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm) ||
        pwm <= PWM_LIMIT_LOW || pwm >= PWM_LIMIT_HIGH) {
        MechBalance_EmergencyStop();
        return;
    }
    if (ball_valid &&
        (bp < -VIS_SETPOINT_LIMIT_CM || bp > VIS_SETPOINT_LIMIT_CM)) {
        s_seq_active = 0U;
        VisHold();
        return;
    }

    /* ---- 模式1: 4步角度序列 ---- */
    if (s_seq_active) {
        if (CL_SetTargetAngle(MOTOR_AXIS_X,
                              slew(s_seq_angles[s_seq_step])) != MOTOR_OK) {
            MechBalance_EmergencyStop();
            return;
        }
        s_seq_elapsed += 5U;
        if (new_ball && s_seq_step == 0U && bp >= SEQ_TURNAROUND_CM) {
            SeqNextStep();
        } else if (new_ball && s_seq_step == 1U) {
            float toward_speed = -s_vis_velocity_cm_s;
            float remaining = bp - s_vis_setpoint_cm;
            float stop_distance = 0.0f;
            if (toward_speed > 0.0f) {
                stop_distance = toward_speed * toward_speed /
                                (2.0f * SEQ_BRAKE_DECEL_CM_S2);
            }
            if (bp <= SEQ_BRAKE_LATEST_CM ||
                (toward_speed > SEQ_CAPTURE_VEL_CM_S &&
                 remaining <= stop_distance + SEQ_BRAKE_MARGIN_CM)) {
                SeqNextStep();
            }
        } else if (new_ball && s_seq_step == 2U &&
                   fabsf(bp - s_vis_setpoint_cm) <= SEQ_CAPTURE_POS_TOL_CM &&
                   fabsf(s_vis_velocity_cm_s) <= SEQ_CAPTURE_VEL_CM_S) {
            SeqNextStep();
        } else if (s_seq_elapsed >= s_seq_times[s_seq_step]) {
            s_seq_elapsed = 0U;
            if (s_seq_step < SEQ_STEPS - 1U) {
                s_seq_step++;                    /* 进入下一步 */
            } else {
                s_seq_active = 0;                /* 最后一步结束 */
                s_seq_step = 0;
                s_direct_mode = 0;
                if (s_seq_auto_vis) {
                    (void)MechBalance_StartVision(); /* 数据无效则安全回水平 */
                } else {
                    MechBalance_ExitDirect();    /* DEMO7: 原行为, 回力学补偿(trim) */
                }
            }
        }
        return;
    }

    /* ---- 模式2: 视觉PID精调 ---- */
    if (s_vis_active) {
        float out, pw, ca, error;
        if (!ball_valid) { VisHold(); return; }
        if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, &pw) ||
            pw <= PWM_LIMIT_LOW || pw >= PWM_LIMIT_HIGH) {
            VisHold();
            return;
        }
        bp = (float)ball.position_centi_cm / 100.0f;
        if (bp < -VIS_SETPOINT_LIMIT_CM || bp > VIS_SETPOINT_LIMIT_CM) {
            VisHold();
            return;
        }
        s_vis_valid = 1;
        s_vis_fault = 0;
        s_vis_ball_pos_cm = bp;
        if (new_ball) {
            error = bp - s_vis_setpoint_cm;
            if (fabsf(error) <= VIS_POSITION_DEADBAND_CM &&
                fabsf(s_vis_velocity_cm_s) <= VIS_VELOCITY_DEADBAND_CM_S) {
                PID_Reset(&s_vis_pid);
                s_vis_pid_out = mp.theta_trim_deg;
            } else {
                out = PID_UpdateWithDerivative(&s_vis_pid, error,
                                               s_vis_velocity_cm_s,
                                               sample_dt);
                s_vis_pid_out = clampf(out + mp.theta_trim_deg,
                                       VIS_OUTPUT_MIN_DEG,
                                       VIS_OUTPUT_MAX_DEG);
            }
        }
        out = s_vis_pid_out;
        ca = CL_GetCurrentAngle(MOTOR_AXIS_X);
        if (pw >= PWM_LIMIT_HIGH - PWM_LIMIT_MARGIN && out > ca) out = ca;
        if (pw <= PWM_LIMIT_LOW  + PWM_LIMIT_MARGIN && out < ca) out = ca;
        out = clampf(out, MECH_ANGLE_MIN_DEG, MECH_ANGLE_MAX_DEG);
        if (CL_SetTargetAngle(MOTOR_AXIS_X, slew(out)) != MOTOR_OK) {
            MechBalance_EmergencyStop();
        }
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
