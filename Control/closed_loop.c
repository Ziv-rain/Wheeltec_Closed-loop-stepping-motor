/**
 * closed_loop.c - Closed-loop stepper position control
 *
 * The balance-control mode establishes the software zero after its guarded
 * PWM absolute-angle homing sequence.
 */
#include "closed_loop.h"
#include "encoder.h"
#include "demo_config.h"

#define CL_SETTLE_CYCLES    3U
#define CL_MAX_BURST_STEPS  ((8U * D36A_MICROSTEP) / 16U)
#define CL_CHECK_STEP_LIMIT 64U
#define CL_MOVE_CONFIRM     2
#define CL_REVERSE_LIMIT    64U
#define CL_REVERSE_COUNTS   3

typedef struct {
    volatile int32_t target_count;
    volatile uint8_t active;
    volatile uint8_t reached;
    volatile CL_Fault_t fault;
    uint8_t settle_cycles;
    uint8_t feedback_pending;
    int8_t expected_sign;
    uint32_t check_steps;
    uint32_t reverse_steps;
    int32_t check_start_pos;
    uint8_t positive_dir_level;
} CL_State_t;

static CL_State_t s_cl;
static uint8_t s_initialized;

static int32_t CL_AngleToCount(float angle)
{
    float scaled = angle * ENCODER_COUNTS_PER_REV / 360.0f;
    return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) :
                              (int32_t)(scaled - 0.5f);
}

static float CL_CountToAngle(int32_t count)
{
    return (float)count * 360.0f / ENCODER_COUNTS_PER_REV;
}

static uint32_t CL_SelectFrequency(uint32_t error)
{
    if (error > 800U) return 8000U;
    if (error > 200U) return 5000U;
    if (error > 50U) return 2500U;
    if (error > 15U) return 1200U;
    return 800U;
}

static uint32_t CL_ErrorToSteps(uint32_t error)
{
    uint64_t numerator = (uint64_t)error * MOTOR_STEPS_PER_REV;
    uint32_t steps = (uint32_t)((numerator + ENCODER_COUNTS_PER_REV - 1U) /
                                ENCODER_COUNTS_PER_REV);
    if (steps == 0U) steps = 1U;
    if (steps > CL_MAX_BURST_STEPS) steps = CL_MAX_BURST_STEPS;
    return steps;
}

static void CL_SetFault(CL_Fault_t fault)
{
    s_cl.fault = fault;
    s_cl.active = 0U;
    s_cl.reached = 0U;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    Motor_Stop(MOTOR_AXIS_X);
}

static void CL_CheckFeedback(int32_t current)
{
    int32_t movement;
    if (s_cl.feedback_pending == 0U) return;
    s_cl.feedback_pending = 0U;
    movement = current - s_cl.check_start_pos;
    if ((s_cl.expected_sign > 0 && movement >= CL_MOVE_CONFIRM) ||
        (s_cl.expected_sign < 0 && movement <= -CL_MOVE_CONFIRM)) {
        s_cl.check_steps = 0U;
        s_cl.reverse_steps = 0U;
        return;
    }
    if ((s_cl.expected_sign > 0 && movement <= -CL_REVERSE_COUNTS) ||
        (s_cl.expected_sign < 0 && movement >= CL_REVERSE_COUNTS)) {
        s_cl.reverse_steps += s_cl.check_steps;
        s_cl.check_steps = 0U;
        s_cl.expected_sign = 0;
        if (s_cl.reverse_steps >= CL_REVERSE_LIMIT) CL_SetFault(CL_FAULT_DIRECTION);
        return;
    }
    if (s_cl.check_steps >= CL_CHECK_STEP_LIMIT) CL_SetFault(CL_FAULT_NO_ENCODER);
}

void CL_Init(void)
{
    s_cl.target_count = 0;
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.settle_cycles = 0U;
    s_cl.feedback_pending = 0U;
    s_cl.expected_sign = 0;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    s_cl.check_start_pos = 0;
    s_cl.positive_dir_level = AXIS_X_POSITIVE_DIR_LEVEL;
    Encoder_SetZero(ENCODER_AXIS_X);
    s_initialized = 1U;
}

void CL_Process(void)
{
    int32_t current;
    int64_t error;
    uint64_t error_magnitude;
    uint32_t error_abs, steps, frequency;
    uint8_t direction;
    int8_t sign;

    if (s_initialized == 0U) return;

    current = Encoder_GetCount(ENCODER_AXIS_X);
    if (Motor_IsBusy(MOTOR_AXIS_X) != 0U) return;
    CL_CheckFeedback(current);
    if (s_cl.active == 0U || s_cl.fault != CL_FAULT_NONE) return;
    error = (int64_t)s_cl.target_count - (int64_t)current;
    error_magnitude = (error >= 0) ? (uint64_t)error : (uint64_t)(-error);
    error_abs = (error_magnitude > UINT32_MAX) ?
                UINT32_MAX : (uint32_t)error_magnitude;
    if (error_abs <= CL_TOLERANCE_COUNTS) {
        if (s_cl.settle_cycles < CL_SETTLE_CYCLES) s_cl.settle_cycles++;
        if (s_cl.settle_cycles >= CL_SETTLE_CYCLES) s_cl.reached = 1U;
        return;
    }
    s_cl.settle_cycles = 0U;
    s_cl.reached = 0U;
    steps = CL_ErrorToSteps(error_abs);
    frequency = CL_SelectFrequency(error_abs);
    if (error > 0) {
        direction = s_cl.positive_dir_level;
        sign = 1;
    } else {
        direction = (uint8_t)!s_cl.positive_dir_level;
        sign = -1;
    }
    if (Motor_SetDirection(MOTOR_AXIS_X, direction) != MOTOR_OK ||
        Motor_Start(MOTOR_AXIS_X, steps, frequency) != MOTOR_OK) {
        CL_SetFault(CL_FAULT_DRIVER);
        return;
    }
    if (s_cl.check_steps == 0U || s_cl.expected_sign != sign) {
        s_cl.check_start_pos = current;
        s_cl.check_steps = 0U;
        s_cl.expected_sign = sign;
    }
    s_cl.check_steps += steps;
    s_cl.feedback_pending = 1U;
}

MotorStatus_t CL_SetTargetAngle(MotorAxis_t axis, float target_deg)
{
    uint32_t primask;
    int32_t new_target;
    int64_t target_delta;

    if (axis != MOTOR_AXIS_X || s_initialized == 0U || target_deg != target_deg ||
        target_deg > 100000.0f || target_deg < -100000.0f) {
        return MOTOR_ERROR;
    }

    new_target = CL_AngleToCount(target_deg);
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_cl.fault != CL_FAULT_NONE) {
        if (primask == 0U) __enable_irq();
        return MOTOR_ERROR;
    }

    target_delta = (int64_t)new_target - (int64_t)s_cl.target_count;
    if (s_cl.active == 0U ||
        target_delta > (int32_t)CL_TOLERANCE_COUNTS ||
        target_delta < -(int32_t)CL_TOLERANCE_COUNTS) {
        s_cl.reached = 0U;
        s_cl.settle_cycles = 0U;
    }
    s_cl.target_count = new_target;
    s_cl.active = 1U;
    if (primask == 0U) __enable_irq();
    return MOTOR_OK;
}

void CL_SetZero(MotorAxis_t axis)
{
    uint32_t primask;

    if (axis != MOTOR_AXIS_X || s_initialized == 0U) return;
    primask = __get_PRIMASK();
    __disable_irq();
    Motor_Stop(axis);
    Encoder_SetZero(ENCODER_AXIS_X);
    s_cl.target_count = 0;
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    if (primask == 0U) __enable_irq();
}

void CL_SetZeroAll(void) { CL_SetZero(MOTOR_AXIS_X); }

void CL_Stop(MotorAxis_t axis)
{
    uint32_t primask;

    if (axis != MOTOR_AXIS_X) return;
    primask = __get_PRIMASK();
    __disable_irq();
    Motor_Stop(axis);
    s_cl.active = 0U;
    s_cl.reached = 0U;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    if (primask == 0U) __enable_irq();
}

void CL_StopAll(void) { CL_Stop(MOTOR_AXIS_X); }

void CL_ClearFault(MotorAxis_t axis)
{
    uint32_t primask;

    if (axis != MOTOR_AXIS_X || s_initialized == 0U) return;
    primask = __get_PRIMASK();
    __disable_irq();
    Motor_Stop(axis);
    s_cl.target_count = Encoder_GetCount(ENCODER_AXIS_X);
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    if (primask == 0U) __enable_irq();
}

MotorStatus_t CL_TogglePositiveDirLevel(MotorAxis_t axis)
{
    uint32_t primask;

    if (axis != MOTOR_AXIS_X) return MOTOR_ERROR;
    primask = __get_PRIMASK();
    __disable_irq();
    if (Motor_IsBusy(axis) != 0U) {
        if (primask == 0U) __enable_irq();
        return MOTOR_ERROR;
    }
    s_cl.positive_dir_level = (uint8_t)!s_cl.positive_dir_level;
    s_cl.reverse_steps = 0U;
    if (primask == 0U) __enable_irq();
    return MOTOR_OK;
}

uint8_t CL_IsReached(MotorAxis_t axis)
{
    return (axis == MOTOR_AXIS_X && s_initialized != 0U) ? s_cl.reached : 0U;
}

CL_Fault_t CL_GetFault(MotorAxis_t axis)
{
    return (axis == MOTOR_AXIS_X && s_initialized != 0U) ? s_cl.fault : CL_FAULT_DRIVER;
}

float CL_GetCurrentAngle(MotorAxis_t axis)
{
    return (axis == MOTOR_AXIS_X) ? Encoder_GetAngle(ENCODER_AXIS_X) : 0.0f;
}

void CL_GetSnapshot(MotorAxis_t axis, CL_Snapshot_t *snapshot)
{
    uint32_t primask;
    int64_t error;

    if (axis != MOTOR_AXIS_X || snapshot == 0 || s_initialized == 0U) return;

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->current_count = Encoder_GetCount(ENCODER_AXIS_X);
    snapshot->target_count = s_cl.target_count;
    snapshot->active = s_cl.active;
    snapshot->reached = s_cl.reached;
    snapshot->fault = s_cl.fault;
    if (primask == 0U) __enable_irq();

    error = (int64_t)snapshot->target_count -
            (int64_t)snapshot->current_count;
    if (error > INT32_MAX) error = INT32_MAX;
    if (error < INT32_MIN) error = INT32_MIN;
    snapshot->error_count = (int32_t)error;
    snapshot->current_angle_deg = CL_CountToAngle(snapshot->current_count);
    snapshot->target_angle_deg = CL_CountToAngle(snapshot->target_count);
}
