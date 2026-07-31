#include "ball_beam_v2_runtime.h"

#include "closed_loop.h"
#include "demo_config.h"
#include "encoder.h"

static BBV2_Controller_t s_runtime_controller;

void BBV2_Runtime_Init(const BBV2_Config_t *config)
{
    BBV2_Init(&s_runtime_controller, config);
}

void BBV2_Runtime_Start(void)
{
    BBV2_Start(&s_runtime_controller);
}

void BBV2_Runtime_Pause(void)
{
    BBV2_Pause(&s_runtime_controller);
}

void BBV2_Runtime_EmergencyStop(void)
{
    BBV2_EmergencyStop(&s_runtime_controller);
    CL_Stop(MOTOR_AXIS_X);
}

void BBV2_Runtime_SetSetpoint(float setpoint_cm)
{
    BBV2_SetSetpoint(&s_runtime_controller, setpoint_cm);
}

void BBV2_Runtime_SetTrim(float trim_deg)
{
    BBV2_SetTrim(&s_runtime_controller, trim_deg);
}

void BBV2_Runtime_Tick5ms(const BBV2_VisionSample_t *new_sample)
{
    const BBV2_Output_t *output;
    float pwm_angle = 0.0f;
    float current_angle = CL_GetCurrentAngle(MOTOR_AXIS_X);
    float target_angle;
    uint8_t pwm_valid = Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle);
    uint8_t actuator_safe = pwm_valid &&
                            pwm_angle > PWM_LIMIT_LOW &&
                            pwm_angle < PWM_LIMIT_HIGH;

    if (pwm_valid &&
        (pwm_angle <= PWM_LIMIT_LOW || pwm_angle >= PWM_LIMIT_HIGH)) {
        BBV2_Runtime_EmergencyStop();
        return;
    }

    BBV2_Tick(&s_runtime_controller, CL_PERIOD_MS, current_angle,
              actuator_safe, new_sample);
    output = BBV2_GetOutput(&s_runtime_controller);
    if (!output || !output->target_enabled) {
        CL_Stop(MOTOR_AXIS_X);
        return;
    }

    target_angle = output->target_angle_deg;
    if (pwm_angle >= PWM_LIMIT_HIGH - PWM_LIMIT_MARGIN &&
        target_angle > current_angle) {
        target_angle = current_angle;
    }
    if (pwm_angle <= PWM_LIMIT_LOW + PWM_LIMIT_MARGIN &&
        target_angle < current_angle) {
        target_angle = current_angle;
    }
    if (CL_SetTargetAngle(MOTOR_AXIS_X, target_angle) != MOTOR_OK) {
        BBV2_Runtime_EmergencyStop();
    }
}

const BBV2_Output_t *BBV2_Runtime_GetOutput(void)
{
    return BBV2_GetOutput(&s_runtime_controller);
}
