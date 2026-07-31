/**
 * mech_balance.c - V3 状态机控制器 + 加速度前馈融合
 *
 * 球在梁上控制: NativeV3 状态机 (ROLLING/BREAKAWAY/SETTLED/VISION_HOLD)
 * 车体扰动补偿: acceleration_base_angle() 前馈 (可 F<0|1> 开关)
 *
 * 融合公式:
 *   target = ff_base + correction   (F1: 前馈 + 状态机)
 *   target = trim   + correction   (F0: 纯状态机基线)
 *
 * 坐标约定: error = pos - setpoint, 正角推球向负坐标
 */
#include "mech_balance.h"
#include "native_v3_core.h"
#include "closed_loop.h"
#include "demo_config.h"
#include "encoder.h"
#include "proto_rx.h"

#include <math.h>

#define V3_MECH_ANGLE_MIN_DEG (-30.0f)
#define V3_MECH_ANGLE_MAX_DEG 45.0f
#define V3_NORMAL_RATE_MAX_DEG_S 80.0f
#define V3_LOST_RATE_DEG_S 20.0f

static MechParams_t s_params = {
    .gravity = 9.80665f,
    .accel_gain_fwd = 1.0f,   /* 审查: 前馈不应压过PID, 合成<=+-18 */
    .accel_gain_brake = 1.0f,
    .accel_bias = 0.0f,
    .theta_trim_deg = 0.0f,
    .theta_rate_limit = 80.0f,
    .pitch_deg = 0.0f,
};

static NativeV3_Controller_t s_controller;
static float s_accel_mps2;
static float s_theta_previous;
static float s_theta_command;
static float s_direct_deg;
static uint32_t s_last_frame_id;
static uint8_t s_direct_mode;
static uint8_t s_emergency_stop;

/* 前馈融合开关 (串口 F<0|1> 可切换) */
static uint8_t s_ff_merge_enabled = MECH_FF_MERGE_ENABLE;
static float s_ff_angle_deg;
static uint32_t s_accel_age_ms;   /* 距上次SetAccel的时间, 超时前馈衰减回零 */
static uint8_t s_accel_manual;    /* 1=手动A命令设置(不衰减), 0=编码器来源 */

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint8_t finitef(float value)
{
    return (value == value && value <= 3.4028234e38f &&
            value >= -3.4028234e38f) ? 1U : 0U;
}

static float slew_to(float target, float rate_deg_s)
{
    float maximum_change;

    target = clampf(target, V3_MECH_ANGLE_MIN_DEG, V3_MECH_ANGLE_MAX_DEG);
    rate_deg_s = clampf(rate_deg_s, 1.0f, V3_NORMAL_RATE_MAX_DEG_S);
    maximum_change = rate_deg_s * 0.005f;
    s_theta_command = clampf(target,
                             s_theta_previous - maximum_change,
                             s_theta_previous + maximum_change);
    s_theta_previous = s_theta_command;
    return s_theta_command;
}

/* 力学前馈: 加速度 -> 抵消惯性的摆杆倾角 (限幅防电机猛甩) */
#define FF_ANGLE_LIMIT_DEG 30.0f   /* 默认30度(机械极限附近, 用户要求), 串口E命令可调 */
static float s_ff_angle_limit = FF_ANGLE_LIMIT_DEG;

void MechBalance_SetFFLimit(float deg)
{
    if (finitef(deg) && deg >= 4.0f && deg <= 30.0f) s_ff_angle_limit = deg;
}
static float acceleration_base_angle(void)
{
    float gain, acceleration_ff, phi_rad, angle;

    if (fabsf(s_accel_mps2) < 0.001f) return s_params.theta_trim_deg;
    gain = s_accel_mps2 >= 0.0f ? s_params.accel_gain_fwd :
                                  s_params.accel_gain_brake;
    acceleration_ff = gain * s_accel_mps2 + s_params.accel_bias;
    phi_rad = atan2f(-acceleration_ff, s_params.gravity);
    angle = phi_rad * 57.29578f - s_params.pitch_deg +
            s_params.theta_trim_deg;
    return clampf(angle, -s_ff_angle_limit, s_ff_angle_limit);
}

static void latch_emergency(void)
{
    if (s_emergency_stop) return;
    s_emergency_stop = 1U;
    s_direct_mode = 0U;
    NativeV3_EmergencyStop(&s_controller);
    CL_Stop(MOTOR_AXIS_X);
}

void MechBalance_Init(void)
{
    NativeV3_Config_t config;

    NativeV3_DefaultConfig(&config);
    NativeV3_Init(&s_controller, &config);
    (void)NativeV3_SetSetpoint(&s_controller, VIS_SETPOINT_CM, 1U);

    s_accel_mps2 = 0.0f;
    s_theta_previous = 0.0f;
    s_theta_command = 0.0f;
    s_direct_deg = 0.0f;
    s_last_frame_id = 0U;
    s_direct_mode = 0U;
    s_emergency_stop = 0U;
    s_ff_merge_enabled = MECH_FF_MERGE_ENABLE;
    s_ff_angle_deg = 0.0f;
}

void MechBalance_SetDirectAngle(float deg)
{
    if (s_emergency_stop || !finitef(deg) ||
        deg < V3_MECH_ANGLE_MIN_DEG || deg > V3_MECH_ANGLE_MAX_DEG) return;
    NativeV3_Pause(&s_controller);
    s_direct_mode = 1U;
    s_direct_deg = deg;
}

void MechBalance_ExitDirect(void)
{
    s_direct_mode = 0U;
    NativeV3_Pause(&s_controller);
}

uint8_t MechBalance_IsDirect(void) { return s_direct_mode; }

void MechBalance_SetAccel(float ax_mps2)
{
    if (!finitef(ax_mps2)) return;
    /* 物理合理上限: 比赛加速度<=2, 4.94/6.39等异常直接丢弃 */
    if (fabsf(ax_mps2) > 3.0f) return;
    s_accel_age_ms = 0U;
    s_accel_manual = 0U;   /* 编码器来源: 断流时允许衰减 */
    /* EMA低通: 编码器两次差分噪声大, 0.5为新值权重(~2帧响应) */
    s_accel_mps2 = 0.50f * s_accel_mps2 + 0.50f * ax_mps2;
    /* 死区: 微小加速度忽略 */
    if (fabsf(s_accel_mps2) < 0.05f) s_accel_mps2 = 0.0f;
}

/* 手动A命令: 直接设置且不被断流衰减(测试前馈用) */
void MechBalance_SetAccelManual(float ax_mps2)
{
    if (!finitef(ax_mps2) || fabsf(ax_mps2) > 3.0f) return;
    s_accel_age_ms = 0U;
    s_accel_manual = 1U;
    s_accel_mps2 = ax_mps2;
}

uint8_t MechBalance_StartVision(void)
{
    if (s_emergency_stop) return 0U;
    s_direct_mode = 0U;
    s_last_frame_id = 0U;
    NativeV3_Start(&s_controller);
    return 1U;
}

void MechBalance_StopVision(void)
{
    NativeV3_Pause(&s_controller);
}

uint8_t MechBalance_IsVisionActive(void)
{
    NativeV3_Status_t status;
    NativeV3_GetStatus(&s_controller, &status);
    return status.requested;
}

uint8_t MechBalance_SetVisionSetpoint(float cm)
{
    return NativeV3_SetSetpoint(&s_controller, cm, 1U);
}

void MechBalance_SetVisionTargetOnly(float cm)
{
    (void)NativeV3_SetSetpoint(&s_controller, cm, 0U);
}

void MechBalance_SetVisKp(float kp)
{
    NativeV3_SetGains(&s_controller, kp, s_controller.cfg.ki,
                      s_controller.cfg.kd);
}

void MechBalance_SetVisKd(float kd)
{
    NativeV3_SetGains(&s_controller, s_controller.cfg.kp,
                      s_controller.cfg.ki, kd);
}

void MechBalance_SetVisKi(float ki)
{
    NativeV3_SetGains(&s_controller, s_controller.cfg.kp, ki,
                      s_controller.cfg.kd);
}

void MechBalance_SetVisOutputMin(float minimum_deg)
{
    NativeV3_SetOutputLimits(&s_controller, minimum_deg,
                             s_controller.cfg.output_max_deg);
}

void MechBalance_SetVisOutputMax(float maximum_deg)
{
    NativeV3_SetOutputLimits(&s_controller, s_controller.cfg.output_min_deg,
                             maximum_deg);
}

void MechBalance_EnableFFMerge(uint8_t en)
{
    s_ff_merge_enabled = en ? 1U : 0U;
}

uint8_t MechBalance_IsFFMerged(void)
{
    return s_ff_merge_enabled;
}

void MechBalance_GetVisionStatus(VisionStatus_t *status)
{
    NativeV3_Status_t detailed;

    if (status == 0) return;
    NativeV3_GetStatus(&s_controller, &detailed);
    status->active = detailed.requested;
    status->valid = detailed.vision_valid;
    status->fault = s_emergency_stop;
    status->ball_pos_cm = detailed.position_cm;
    status->setpoint_cm = detailed.setpoint_cm;
    status->pid_out_deg = s_theta_command;
    status->ball_velocity_cm_s = detailed.velocity_cm_s;
    status->vis_kp = s_controller.cfg.kp;
    status->vis_kd = s_controller.cfg.kd;
    status->vis_ki = s_controller.cfg.ki;
    status->out_min = s_controller.cfg.output_min_deg;
    status->out_max = s_controller.cfg.output_max_deg;
    status->ff_angle_deg = s_ff_angle_deg;
    status->ff_merged = s_ff_merge_enabled && detailed.requested;
    status->accel_mps2 = s_accel_mps2;
}

void MechBalance_EmergencyStop(void) { latch_emergency(); }

void MechBalance_SetParam(uint8_t id, float value)
{
    if (!finitef(value)) return;
    switch (id) {
    case MP_GRAVITY:
        if (value >= 8.0f && value <= 11.0f) s_params.gravity = value;
        break;
    case MP_GAIN_FWD:
        if (value >= 0.0f) s_params.accel_gain_fwd = value;
        break;
    case MP_GAIN_BRAKE:
        if (value >= 0.0f) s_params.accel_gain_brake = value;
        break;
    case MP_ACCEL_BIAS:
        if (value >= -3.0f && value <= 3.0f) s_params.accel_bias = value;
        break;
    case MP_TRIM:
        if (value >= -4.0f && value <= 4.0f) s_params.theta_trim_deg = value;
        break;
    case MP_RATE_LIMIT:
        if (value >= 10.0f && value <= V3_NORMAL_RATE_MAX_DEG_S) {
            s_params.theta_rate_limit = value;
        }
        break;
    case MP_PITCH:
        if (value >= -10.0f && value <= 10.0f) s_params.pitch_deg = value;
        break;
    case MP_FF_ENABLE:
        s_ff_merge_enabled = (value != 0.0f) ? 1U : 0U;
        break;
    default:
        break;
    }
}

const MechParams_t *MechBalance_GetParams(void) { return &s_params; }

void MechBalance_Tick5ms(void)
{
    BallData_t ball;
    NativeV3_Status_t status;
    float pwm_angle, current_angle, correction, target, rate;
    uint8_t ball_valid, new_frame;

    if (s_emergency_stop) return;
    /* 编码器断流超时: 前馈平滑衰减回零 (手动A命令设置不衰减) */
    if (s_accel_age_ms <= 0xFFFFFFFFU - 5U) s_accel_age_ms += 5U;
    if (!s_accel_manual && s_accel_age_ms > 500U) {
        s_accel_mps2 *= 0.90f;
        if (fabsf(s_accel_mps2) < 0.05f) s_accel_mps2 = 0.0f;
    }
    if (CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE) {
        latch_emergency();
        return;
    }

    if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle)) {
        /* 反馈短暂丢失: 抑制输出但不清除V请求 */
        CL_Stop(MOTOR_AXIS_X);
        return;
    }
    if (pwm_angle < PWM_LIMIT_LOW || pwm_angle > PWM_LIMIT_HIGH) {
        latch_emergency();
        return;
    }

    ball_valid = ProtoRx_GetBall(&ball);
    new_frame = (ball_valid && ball.frame_id != s_last_frame_id) ? 1U : 0U;
    if (new_frame) {
        s_last_frame_id = ball.frame_id;
        if (!NativeV3_ObserveDirect(&s_controller,
                                    (float)ball.position_centi_cm / 100.0f,
                                    ball.confidence)) {
            NativeV3_MarkVisionMissing(&s_controller);
        }
    } else if (!ball_valid) {
        NativeV3_MarkVisionMissing(&s_controller);
    }

    correction = NativeV3_Tick5ms(&s_controller);
    NativeV3_GetStatus(&s_controller, &status);

    /* 每5ms更新前馈分量 (视觉不依赖) */
    s_ff_angle_deg = acceleration_base_angle();

    if (s_direct_mode) {
        target = s_direct_deg;
        rate = s_params.theta_rate_limit;
    } else if (!status.requested) {
        /* 待机(未启动): 保持水平, 前馈不参与, 防球被甩 */
        target = s_params.theta_trim_deg;
        rate = s_params.theta_rate_limit;
    } else if (s_ff_merge_enabled) {
        /* 融合: 前馈基座 + 状态机修正 */
        target = s_ff_angle_deg + correction;
        rate = (status.requested && !status.vision_valid) ?
            V3_LOST_RATE_DEG_S : s_params.theta_rate_limit;
    } else {
        /* 纯状态机: 仅trim基线, 无前馈 */
        target = s_params.theta_trim_deg + correction;
        rate = (status.requested && !status.vision_valid) ?
            V3_LOST_RATE_DEG_S : s_params.theta_rate_limit;
    }

    target = clampf(target, V3_MECH_ANGLE_MIN_DEG, V3_MECH_ANGLE_MAX_DEG);
    current_angle = CL_GetCurrentAngle(MOTOR_AXIS_X);
    if (pwm_angle >= PWM_LIMIT_HIGH - PWM_LIMIT_MARGIN &&
        target > current_angle) target = current_angle;
    if (pwm_angle <= PWM_LIMIT_LOW + PWM_LIMIT_MARGIN &&
        target < current_angle) target = current_angle;

    if (CL_SetTargetAngle(MOTOR_AXIS_X, slew_to(target, rate)) != MOTOR_OK) {
        latch_emergency();
    }
}
