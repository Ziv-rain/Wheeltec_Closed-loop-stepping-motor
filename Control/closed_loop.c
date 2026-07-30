/**
  ******************************************************************************
  * @file    closed_loop.c
  * @brief   ����ֶ�����λ�ñջ��ͷ������ϼ��
  ******************************************************************************
  * ����������TIMG8Ӳ��QEI��������λ������ͨ��TIMA1�����������𲽱ƽ�Ŀ�ꡣ
  * ���д�����Ϊ��������ʽ����ǰ����ν�����Ż���㲢������һ�Ρ�
  ******************************************************************************
  */
#include "closed_loop.h"
#include "encoder.h"
#include "demo_config.h"
#include "nvm.h"

/* ����3�����ڴ����ݲ��ڲ��㵽λ����ֹֻ��˲�侭��Ŀ��λ�á� */
#define CL_SETTLE_CYCLES    3U
/* ÿ�����8��΢��(��16ϸ�ֵȱ�������)��������α��ڼ�ʱ�ز�λ�á� */
#define CL_MAX_BURST_STEPS  ((8U * D36A_MICROSTEP) / 16U)
/* ������ֵ���ۼ�64���岻�����޷������ۼ�64���������з������ */
#define CL_CHECK_STEP_LIMIT 64U
#define CL_MOVE_CONFIRM     2
#define CL_REVERSE_LIMIT    64U
#define CL_REVERSE_COUNTS   3

/* ����ջ�״̬�͡�����һ�������˶Է����������ͳ������ */
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

/* �Ƕ�ת�����������������������������롣 */
static int32_t CL_AngleToCount(float angle)
{
    float scaled = angle * ENCODER_COUNTS_PER_REV / 360.0f;
    return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) :
                              (int32_t)(scaled - 0.5f);
}

/* ����������ת�Ƕȣ�4000������Ӧ360�ȡ� */
static float CL_CountToAngle(int32_t count)
{
    return (float)count * 360.0f / ENCODER_COUNTS_PER_REV;
}

/* �ֶ����٣����Խ��Ƶ��Խ�ߣ�����Ŀ����٣����ٳ������񵴡� */
static uint32_t CL_SelectFrequency(uint32_t error)
{
    if (error > 800U) return 3000U;
    if (error > 200U) return 1800U;
    if (error > 50U) return 900U;
    if (error > 15U) return 450U;
    return 400U;
}

/* ���������/�����������������㱾�β���������ȡ�����������γ��� */
static uint32_t CL_ErrorToSteps(uint32_t error)
{
    uint64_t numerator = (uint64_t)error * MOTOR_STEPS_PER_REV;
    uint32_t steps = (uint32_t)((numerator + ENCODER_COUNTS_PER_REV - 1U) /
                                ENCODER_COUNTS_PER_REV);
    if (steps == 0U) steps = 1U;
    if (steps > CL_MAX_BURST_STEPS) steps = CL_MAX_BURST_STEPS;
    return steps;
}

/* �������״̬ʱ����ֹͣ������������δ�˶Եķ���ͳ�ơ� */
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

/**
  * �˶���һ����ε�A/B������
  * 1. �����������ƶ��ﵽ��ֵ����������������ۼƣ�
  * 2. ���Է����ۼƷ������壬���ޱ�������ϣ�
  * 3. �����������ۼ��ѷ����壬���ޱ��ޱ�����������
  */
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

/* �ϵ�λ����Ϊ��㣬�ջ���ʼΪ�������ѵ�λ״̬�� */
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

    /* Load saved settings from flash (restore zero & direction) */
    {
        NVM_Settings_t nvm;
        if (NVM_Load(&nvm) != 0U) {
            Encoder_RestoreZero(ENCODER_AXIS_X, nvm.encoder_zero);
            s_cl.positive_dir_level = nvm.pos_dir_level;
        }
    }

    s_initialized = 1U;
}

/**
  * �ջ����Ĵ������ȴ���ǰ����ν������˶Է���������λ����
  * ��ѡ���򡢲�����Ƶ�ʷ�����һС�����塣
  */
void CL_Process(void)
{
    int32_t current, error;
    uint32_t error_abs, steps, frequency;
    uint8_t direction;
    int8_t sign;
    if (s_initialized == 0U) return;
    current = Encoder_GetCount(ENCODER_AXIS_X);
    if (Motor_IsBusy(MOTOR_AXIS_X) != 0U) return;
    CL_CheckFeedback(current);
    if (s_cl.active == 0U || s_cl.fault != CL_FAULT_NONE) return;
    error = s_cl.target_count - current;
    error_abs = (error >= 0) ? (uint32_t)error : (uint32_t)(-error);
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

/* ��Ŀ�����������Ϊ��׼��ת��ΪQEIĿ������������ջ��� */
MotorStatus_t CL_SetTargetAngle(MotorAxis_t axis, float target_deg)
{
    if (axis != MOTOR_AXIS_X || s_initialized == 0U || target_deg != target_deg ||
        target_deg > 100000.0f || target_deg < -100000.0f ||
        s_cl.fault != CL_FAULT_NONE) return MOTOR_ERROR;
    s_cl.target_count = CL_AngleToCount(target_deg);
    s_cl.active = 1U;
    s_cl.reached = 0U;
    s_cl.settle_cycles = 0U;
    return MOTOR_OK;
}

/* ����ǰλ����Ϊ0��ȡ����ǰĿ�꣬ͬʱ������Ϻͷ���ͳ�ơ� */
void CL_SetZero(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X || s_initialized == 0U) return;
    Motor_Stop(axis);
    Encoder_SetZero(ENCODER_AXIS_X);
    s_cl.target_count = 0;
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;

    /* Save zero & direction to flash */
    {
        NVM_Settings_t nvm;
        nvm.encoder_zero  = Encoder_GetZeroOffset(ENCODER_AXIS_X);
        nvm.pos_dir_level = s_cl.positive_dir_level;
        NVM_Save(&nvm);
    }
}

void CL_SetZeroAll(void) { CL_SetZero(MOTOR_AXIS_X); }

/* ����ֹͣ��ǰĿ�ꣻֹͣ����ͬ�ڵ�λ�����reached��0�� */
void CL_Stop(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X) return;
    Motor_Stop(axis);
    s_cl.active = 0U;
    s_cl.reached = 0U;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
}

void CL_StopAll(void) { CL_Stop(MOTOR_AXIS_X); }

/* ����Ϻ��Ե�ǰλ��ΪĿ�꣬����ԭ�صȴ���һ����� */
void CL_ClearFault(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X || s_initialized == 0U) return;
    Motor_Stop(axis);
    s_cl.target_count = Encoder_GetCount(ENCODER_AXIS_X);
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
}

/* ͣ��ʱ��ת�߼������������ֳ�����DIR�����������һ�¡� */
MotorStatus_t CL_TogglePositiveDirLevel(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X || Motor_IsBusy(axis) != 0U) return MOTOR_ERROR;
    s_cl.positive_dir_level = (uint8_t)!s_cl.positive_dir_level;
    s_cl.reverse_steps = 0U;
    /* Save direction to flash */
    {
        NVM_Settings_t nvm;
        nvm.encoder_zero  = Encoder_GetZeroOffset(ENCODER_AXIS_X);
        nvm.pos_dir_level = s_cl.positive_dir_level;
        NVM_Save(&nvm);
    }
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

/* ���ڲ�״̬���Ƶ����գ����⴮�ڲ�ֱ�������ջ�˽�б����� */
void CL_GetSnapshot(MotorAxis_t axis, CL_Snapshot_t *snapshot)
{
    if (axis != MOTOR_AXIS_X || snapshot == 0 || s_initialized == 0U) return;
    snapshot->current_count = Encoder_GetCount(ENCODER_AXIS_X);
    snapshot->target_count = s_cl.target_count;
    snapshot->error_count = snapshot->target_count - snapshot->current_count;
    snapshot->current_angle_deg = CL_CountToAngle(snapshot->current_count);
    snapshot->target_angle_deg = CL_CountToAngle(snapshot->target_count);
    snapshot->active = s_cl.active;
    snapshot->reached = s_cl.reached;
    snapshot->fault = s_cl.fault;
}
