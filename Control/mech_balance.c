/**
 * Mechanical feed-forward and seven-phase open-loop ball trajectory.
 *
 * Sequence coordinates:
 *   A' side = negative, D' side = positive
 *   waypoint 1 = -5 cm, waypoint 2 = +5 cm
 *
 * The default motor-angle trajectory is a bench-test starting point. It does
 * not use vision feedback for normal motion. Hardware limits and emergency
 * stop remain active.
 */
#include "mech_balance.h"
#include "closed_loop.h"
#include "demo_config.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>

#define MECH_ACCEL_ABS_MAX_MPS2  30.0f
#define MECH_GAIN_MAX            5.0f
#define MECH_GRAVITY_MIN         1.0f
#define MECH_GRAVITY_MAX         20.0f
#define MECH_TRIM_ABS_MAX_DEG    10.0f
#define MECH_PITCH_ABS_MAX_DEG   45.0f
#define MECH_RATE_MIN_DPS        1.0f
#define MECH_RATE_MAX_DPS        450.0f

static MechParams_t s_params = {
    .gravity = 9.80665f,
    .accel_gain_fwd = 1.0f,
    .accel_gain_brake = 1.0f,
    .accel_bias = 0.0f,
    .theta_trim_deg = 0.0f,
    .theta_rate_limit = 450.0f,
    .pitch_deg = 0.0f,
};

/* Package defaults: level, accelerate/brake to +5, settle, then to -5. */
static float s_seq_angles[MECH_SEQ_STEP_COUNT] = {
    0.0f, -24.2063f, 19.7937f, 0.0f, 34.7937f, -20.2063f, 0.0f
};
static uint32_t s_seq_times[MECH_SEQ_STEP_COUNT] = {
    200U, 365U, 335U, 550U, 405U, 430U, 800U
};

static volatile float s_accel_mps2;
static volatile float s_direct_deg;
static volatile uint8_t s_direct_mode;
static volatile uint8_t s_enabled;
static volatile uint8_t s_seq_active;
static volatile uint8_t s_seq_step;
static volatile uint8_t s_seq_last_step;
static volatile uint32_t s_seq_elapsed_ms;
static float s_previous_command_deg;
static MechStatus_t s_status;

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static uint8_t finite_in_range(float value, float low, float high)
{
    return (value == value && value >= low && value <= high) ? 1U : 0U;
}

static void stop_with_status(MechStatus_t status)
{
    s_enabled = 0U;
    s_seq_active = 0U;
    s_direct_mode = 0U;
    s_status = status;
    CL_Stop(MOTOR_AXIS_X);
}

static uint8_t output_angle(float target_deg)
{
    float pwm_angle;
    float current_angle;
    float max_change;

    if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle)) {
        stop_with_status(MECH_STATUS_FAULT_PWM);
        return 0U;
    }
    if (pwm_angle <= PWM_LIMIT_LOW || pwm_angle >= PWM_LIMIT_HIGH) {
        stop_with_status(MECH_STATUS_FAULT_LIMIT);
        return 0U;
    }

    target_deg = clampf(target_deg, MECH_ANGLE_MIN_DEG, MECH_ANGLE_MAX_DEG);
    current_angle = CL_GetCurrentAngle(MOTOR_AXIS_X);
    if (pwm_angle >= PWM_LIMIT_HIGH - PWM_LIMIT_MARGIN &&
        target_deg > current_angle) {
        target_deg = current_angle;
    }
    if (pwm_angle <= PWM_LIMIT_LOW + PWM_LIMIT_MARGIN &&
        target_deg < current_angle) {
        target_deg = current_angle;
    }

    max_change = s_params.theta_rate_limit * 0.005f;
    target_deg = clampf(target_deg,
                        s_previous_command_deg - max_change,
                        s_previous_command_deg + max_change);
    s_previous_command_deg = target_deg;
    if (CL_SetTargetAngle(MOTOR_AXIS_X, target_deg) != MOTOR_OK) {
        stop_with_status(MECH_STATUS_FAULT_DRIVER);
        return 0U;
    }
    return 1U;
}

void MechBalance_Init(void)
{
    s_accel_mps2 = 0.0f;
    s_direct_deg = s_params.theta_trim_deg;
    s_direct_mode = 0U;
    s_enabled = 1U;
    s_seq_active = 0U;
    s_seq_step = 0U;
    s_seq_last_step = MECH_SEQ_STEP_COUNT - 1U;
    s_seq_elapsed_ms = 0U;
    s_previous_command_deg = 0.0f;
    s_status = MECH_STATUS_ARMED;
}

void MechBalance_EmergencyStop(void)
{
    stop_with_status(MECH_STATUS_STOPPED);
}

uint8_t MechBalance_SetAccel(float ax_mps2)
{
    if (s_seq_active != 0U ||
        !finite_in_range(ax_mps2,
                         -MECH_ACCEL_ABS_MAX_MPS2,
                         MECH_ACCEL_ABS_MAX_MPS2)) {
        return 0U;
    }
    s_accel_mps2 = ax_mps2;
    return 1U;
}

uint8_t MechBalance_SetParam(uint8_t id, float value)
{
    if (s_seq_active != 0U) return 0U;
    switch (id) {
    case MP_GRAVITY:
        if (!finite_in_range(value, MECH_GRAVITY_MIN, MECH_GRAVITY_MAX)) return 0U;
        s_params.gravity = value;
        break;
    case MP_GAIN_FWD:
        if (!finite_in_range(value, 0.0f, MECH_GAIN_MAX)) return 0U;
        s_params.accel_gain_fwd = value;
        break;
    case MP_GAIN_BRAKE:
        if (!finite_in_range(value, 0.0f, MECH_GAIN_MAX)) return 0U;
        s_params.accel_gain_brake = value;
        break;
    case MP_ACCEL_BIAS:
        if (!finite_in_range(value,
                             -MECH_ACCEL_ABS_MAX_MPS2,
                             MECH_ACCEL_ABS_MAX_MPS2)) return 0U;
        s_params.accel_bias = value;
        break;
    case MP_TRIM:
        if (!finite_in_range(value,
                             -MECH_TRIM_ABS_MAX_DEG,
                             MECH_TRIM_ABS_MAX_DEG)) return 0U;
        s_params.theta_trim_deg = value;
        s_seq_angles[0] = value;
        s_seq_angles[3] = value;
        s_seq_angles[6] = value;
        break;
    case MP_RATE_LIMIT:
        if (!finite_in_range(value, MECH_RATE_MIN_DPS, MECH_RATE_MAX_DPS)) return 0U;
        s_params.theta_rate_limit = value;
        break;
    case MP_PITCH:
        if (!finite_in_range(value,
                             -MECH_PITCH_ABS_MAX_DEG,
                             MECH_PITCH_ABS_MAX_DEG)) return 0U;
        s_params.pitch_deg = value;
        break;
    default:
        return 0U;
    }
    return 1U;
}

const MechParams_t *MechBalance_GetParams(void)
{
    return &s_params;
}

uint8_t MechBalance_SetDirectAngle(float deg)
{
    if (s_enabled == 0U || s_seq_active != 0U ||
        !finite_in_range(deg, MECH_ANGLE_MIN_DEG, MECH_ANGLE_MAX_DEG)) {
        return 0U;
    }
    s_direct_deg = deg;
    s_direct_mode = 1U;
    return 1U;
}

void MechBalance_ExitDirect(void)
{
    s_direct_mode = 0U;
    s_seq_active = 0U;
}

uint8_t MechBalance_IsDirect(void)
{
    return s_direct_mode;
}

uint8_t MechBalance_SetSeqAngle(uint8_t index, float deg)
{
    if (s_seq_active != 0U || index >= MECH_SEQ_STEP_COUNT ||
        !finite_in_range(deg, MECH_ANGLE_MIN_DEG, MECH_ANGLE_MAX_DEG)) {
        return 0U;
    }
    s_seq_angles[index] = deg;
    return 1U;
}

uint8_t MechBalance_SetSeqTime(uint8_t index, uint32_t ms)
{
    if (s_seq_active != 0U || index >= MECH_SEQ_STEP_COUNT ||
        ms < MECH_SEQ_TIME_MIN_MS || ms > MECH_SEQ_TIME_MAX_MS ||
        (ms % 5U) != 0U) {
        return 0U;
    }
    s_seq_times[index] = ms;
    return 1U;
}

float MechBalance_GetSeqAngle(uint8_t index)
{
    return index < MECH_SEQ_STEP_COUNT ? s_seq_angles[index] : 0.0f;
}

uint32_t MechBalance_GetSeqTime(uint8_t index)
{
    return index < MECH_SEQ_STEP_COUNT ? s_seq_times[index] : 0U;
}

uint8_t MechBalance_StartSeq(uint8_t run_mode)
{
    if (s_enabled == 0U || s_seq_active != 0U ||
        run_mode > 2U) {
        return 0U;
    }
    s_seq_step = (run_mode == 2U) ? 4U : 0U;
    s_seq_last_step = (run_mode == 1U) ? 3U : (MECH_SEQ_STEP_COUNT - 1U);
    s_seq_elapsed_ms = 0U;
    s_direct_mode = 1U;
    s_seq_active = 1U;
    s_status = MECH_STATUS_RUNNING;
    return 1U;
}

void MechBalance_StopSeq(void)
{
    s_seq_active = 0U;
    s_direct_mode = 0U;
    s_seq_step = 0U;
    s_seq_elapsed_ms = 0U;
    if (s_enabled != 0U) s_status = MECH_STATUS_STOPPED;
}

uint8_t MechBalance_IsSeqActive(void)
{
    return s_seq_active;
}

uint8_t MechBalance_GetSeqStep(void)
{
    return s_seq_step;
}

uint32_t MechBalance_GetSeqElapsed(void)
{
    return s_seq_elapsed_ms;
}

MechStatus_t MechBalance_GetStatus(void)
{
    return s_status;
}

void MechBalance_Tick5ms(void)
{
    float gain;
    float acceleration_ff;
    float target_deg;

    if (s_enabled == 0U) return;

    if (s_seq_active != 0U) {
        target_deg = s_seq_angles[s_seq_step];
        if (!output_angle(target_deg)) return;

        s_seq_elapsed_ms += 5U;
        if (s_seq_elapsed_ms >= s_seq_times[s_seq_step]) {
            s_seq_elapsed_ms = 0U;
            if (s_seq_step < s_seq_last_step) {
                s_seq_step++;
            } else {
                s_seq_active = 0U;
                s_direct_mode = 0U;
                s_status = MECH_STATUS_FINISHED;
            }
        }
        return;
    }

    if (s_direct_mode != 0U) {
        target_deg = s_direct_deg;
    } else {
        gain = s_accel_mps2 >= 0.0f ?
               s_params.accel_gain_fwd : s_params.accel_gain_brake;
        acceleration_ff = gain * s_accel_mps2 + s_params.accel_bias;
        target_deg = atan2f(-acceleration_ff, s_params.gravity) * 57.29578f;
        target_deg = target_deg - s_params.pitch_deg + s_params.theta_trim_deg;
    }
    if (output_angle(target_deg) && s_status != MECH_STATUS_FINISHED) {
        s_status = MECH_STATUS_ARMED;
    }
}
