/**
 * mech_balance.c - mechanical acceleration feed-forward compensation
 *
 * Feed-forward angle: phi = atan2(-acceleration, gravity)
 * Motor angle: theta = phi - vehicle_pitch + trim
 *
 * This module reduces acceleration disturbance. It does not replace ball
 * position feedback when the task requires the ball to return to a setpoint.
 */
#include "mech_balance.h"
#include "closed_loop.h"
#include "demo_config.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>

#define MECH_ACCEL_ABS_MAX_MPS2 30.0f
#define MECH_GAIN_MAX           5.0f
#define MECH_GRAVITY_MIN        1.0f
#define MECH_GRAVITY_MAX        20.0f
#define MECH_TRIM_ABS_MAX_DEG   10.0f
#define MECH_PITCH_ABS_MAX_DEG  45.0f
#define MECH_RATE_MIN_DPS       1.0f
#define MECH_RATE_MAX_DPS       360.0f

static MechParams_t mp = {
    .gravity = 9.80665f,
    .accel_gain_fwd = 1.0f,
    .accel_gain_brake = 1.0f,
    .accel_bias = 0.0f,
    .theta_trim_deg = 0.0f,
    .theta_min_deg = -3.0f,
    .theta_max_deg = 3.0f,
    .theta_rate_limit = 80.0f,
    .pitch_deg = 0.0f,
};

static volatile float s_ax;
static float s_theta_prev;
static float s_phi_deg;
static float s_theta_cmd;
static volatile float s_direct_deg;
static volatile uint8_t s_direct_mode;
static volatile uint8_t s_enabled;
static volatile MechStatus_t s_status;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint8_t finite_in_range(float v, float lo, float hi)
{
    return (v == v && v >= lo && v <= hi) ? 1U : 0U;
}

static void fail(MechStatus_t status)
{
    s_enabled = 0U;
    s_status = status;
    CL_Stop(MOTOR_AXIS_X);
}

void MechBalance_Init(void)
{
    s_ax = 0.0f;
    s_theta_prev = 0.0f;
    s_phi_deg = 0.0f;
    s_theta_cmd = 0.0f;
    s_direct_deg = 0.0f;
    s_direct_mode = 0U;
    s_enabled = 1U;
    s_status = MECH_STATUS_ARMED;
}

void MechBalance_EmergencyStop(void)
{
    s_enabled = 0U;
    s_status = MECH_STATUS_STOPPED;
    CL_Stop(MOTOR_AXIS_X);
}

uint8_t MechBalance_SetAccel(float ax_mps2)
{
    if (!finite_in_range(ax_mps2,
                         -MECH_ACCEL_ABS_MAX_MPS2,
                         MECH_ACCEL_ABS_MAX_MPS2)) {
        return 0U;
    }
    s_ax = ax_mps2;
    return 1U;
}

uint8_t MechBalance_SetDirectAngle(float deg)
{
    if (!finite_in_range(deg, mp.theta_min_deg, mp.theta_max_deg)) return 0U;
    s_direct_deg = deg;
    s_direct_mode = 1U;
    return 1U;
}

void MechBalance_ExitDirect(void)
{
    s_direct_mode = 0U;
}

uint8_t MechBalance_IsDirect(void)
{
    return s_direct_mode;
}

uint8_t MechBalance_SetParam(uint8_t id, float v)
{
    switch (id) {
    case MP_GRAVITY:
        if (!finite_in_range(v, MECH_GRAVITY_MIN, MECH_GRAVITY_MAX)) return 0U;
        mp.gravity = v;
        break;
    case MP_GAIN_FWD:
        if (!finite_in_range(v, 0.0f, MECH_GAIN_MAX)) return 0U;
        mp.accel_gain_fwd = v;
        break;
    case MP_GAIN_BRAKE:
        if (!finite_in_range(v, 0.0f, MECH_GAIN_MAX)) return 0U;
        mp.accel_gain_brake = v;
        break;
    case MP_ACCEL_BIAS:
        if (!finite_in_range(v,
                             -MECH_ACCEL_ABS_MAX_MPS2,
                             MECH_ACCEL_ABS_MAX_MPS2)) return 0U;
        mp.accel_bias = v;
        break;
    case MP_TRIM:
        if (!finite_in_range(v,
                             -MECH_TRIM_ABS_MAX_DEG,
                             MECH_TRIM_ABS_MAX_DEG)) return 0U;
        mp.theta_trim_deg = v;
        break;
    case MP_MIN_DEG:
        if (!finite_in_range(v, -MOTOR_MAX_ANGLE_NEG, 0.0f) ||
            v >= mp.theta_max_deg) return 0U;
        mp.theta_min_deg = v;
        break;
    case MP_MAX_DEG:
        if (!finite_in_range(v, 0.0f, MOTOR_MAX_ANGLE_POS) ||
            v <= mp.theta_min_deg) return 0U;
        mp.theta_max_deg = v;
        break;
    case MP_RATE_LIMIT:
        if (!finite_in_range(v, MECH_RATE_MIN_DPS, MECH_RATE_MAX_DPS)) return 0U;
        mp.theta_rate_limit = v;
        break;
    case MP_PITCH:
        if (!finite_in_range(v,
                             -MECH_PITCH_ABS_MAX_DEG,
                             MECH_PITCH_ABS_MAX_DEG)) return 0U;
        mp.pitch_deg = v;
        break;
    default:
        return 0U;
    }
    return 1U;
}

const MechParams_t *MechBalance_GetParams(void)
{
    return &mp;
}

MechStatus_t MechBalance_GetStatus(void)
{
    return s_status;
}

void MechBalance_Tick5ms(void)
{
    float gain;
    float a_ff;
    float phi_rad;
    float theta_target;
    float max_change;
    float pwm_angle;
    float current_angle;

    if (s_enabled == 0U) return;

    if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle)) {
        fail(MECH_STATUS_FAULT_PWM);
        return;
    }
    if (pwm_angle <= PWM_LIMIT_LOW || pwm_angle >= PWM_LIMIT_HIGH) {
        fail(MECH_STATUS_FAULT_LIMIT);
        return;
    }

    if (s_direct_mode != 0U) {
        theta_target = s_direct_deg;
    } else {
        gain = (s_ax >= 0.0f) ? mp.accel_gain_fwd : mp.accel_gain_brake;
        a_ff = gain * s_ax + mp.accel_bias;
        phi_rad = atan2f(-a_ff, mp.gravity);
        s_phi_deg = phi_rad * 57.29578f;
        theta_target = s_phi_deg - mp.pitch_deg + mp.theta_trim_deg;
    }
    theta_target = clampf(theta_target, mp.theta_min_deg, mp.theta_max_deg);

    current_angle = CL_GetCurrentAngle(MOTOR_AXIS_X);
    if (pwm_angle >= PWM_LIMIT_HIGH - PWM_LIMIT_MARGIN &&
        theta_target > current_angle) {
        theta_target = current_angle;
    }
    if (pwm_angle <= PWM_LIMIT_LOW + PWM_LIMIT_MARGIN &&
        theta_target < current_angle) {
        theta_target = current_angle;
    }

    max_change = mp.theta_rate_limit * 0.005f;
    s_theta_cmd = clampf(theta_target,
                         s_theta_prev - max_change,
                         s_theta_prev + max_change);
    s_theta_prev = s_theta_cmd;

    if (CL_SetTargetAngle(MOTOR_AXIS_X, s_theta_cmd) != MOTOR_OK) {
        fail(MECH_STATUS_FAULT_DRIVER);
        return;
    }
    s_status = MECH_STATUS_RUNNING;
}
