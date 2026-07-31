#ifndef NATIVE_V3_CORE_H
#define NATIVE_V3_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * This controller uses the project's own coordinate convention:
 *   error = ball_position_cm - setpoint_cm
 *   positive motor angle moves the ball toward the negative coordinate.
 * Therefore a positive position error requires a positive correction angle.
 */
typedef enum {
    NATIVE_V3_PAUSED = 0,
    NATIVE_V3_VISION_HOLD,
    NATIVE_V3_ROLLING,
    NATIVE_V3_BREAKAWAY,
    NATIVE_V3_SETTLED,
    NATIVE_V3_EMERGENCY
} NativeV3_Mode_t;

typedef struct {
    float kp;
    float kd;
    float ki;
    float integral_limit;
    float output_min_deg;
    float output_max_deg;
    float setpoint_limit_cm;

    float velocity_old_weight;
    float prediction_s;
    float prediction_limit_cm;
    float brake_near_cm;
    float brake_far_cm;
    float brake_far_scale;

    float position_deadband_cm;
    float settled_position_cm;
    float settled_exit_position_cm;
    float settled_velocity_cm_s;
    float settled_exit_velocity_cm_s;
    uint32_t settled_confirm_ms;

    float integral_position_cm;
    float integral_velocity_cm_s;

    float stiction_error_cm;
    float stiction_velocity_cm_s;
    float moving_velocity_cm_s;
    uint32_t stiction_confirm_ms;
    float breakaway_positive_deg;
    float breakaway_negative_deg;
    float breakaway_release_cm;
    uint32_t breakaway_max_ms;

    uint32_t vision_timeout_ms;
    uint8_t recovery_frames;
    uint8_t minimum_confidence;
} NativeV3_Config_t;

typedef struct {
    uint8_t requested;
    uint8_t vision_valid;
    uint8_t emergency;
    NativeV3_Mode_t mode;
    float setpoint_cm;
    float position_cm;
    float predicted_position_cm;
    float velocity_cm_s;
    float position_error_cm;
    float p_deg;
    float i_deg;
    float d_deg;
    float brake_scale;
    float desired_correction_deg;
    uint32_t vision_age_ms;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
} NativeV3_Status_t;

typedef struct {
    NativeV3_Config_t cfg;
    NativeV3_Status_t status;

    uint32_t now_ms;
    uint32_t last_sample_ms;
    uint32_t stationary_ms;
    uint32_t settled_ms;
    uint32_t breakaway_ms;
    float integral;
    float breakaway_start_position_cm;

    float history_position_cm[3];
    uint32_t history_time_ms[3];
    uint8_t history_count;
    uint8_t history_head;
    uint8_t recovery_count;
} NativeV3_Controller_t;

void NativeV3_DefaultConfig(NativeV3_Config_t *cfg);
void NativeV3_Init(NativeV3_Controller_t *controller,
                   const NativeV3_Config_t *cfg);
void NativeV3_Start(NativeV3_Controller_t *controller);
void NativeV3_Pause(NativeV3_Controller_t *controller);
void NativeV3_EmergencyStop(NativeV3_Controller_t *controller);

uint8_t NativeV3_SetSetpoint(NativeV3_Controller_t *controller, float cm,
                             uint8_t reset_dynamics);
void NativeV3_SetGains(NativeV3_Controller_t *controller, float kp,
                       float ki, float kd);
void NativeV3_SetOutputLimits(NativeV3_Controller_t *controller,
                              float minimum_deg, float maximum_deg);

/* Called only for a fresh direct camera measurement. */
uint8_t NativeV3_ObserveDirect(NativeV3_Controller_t *controller,
                               float position_cm, uint8_t confidence);
/* Called when the parser reports BALL_LOST/timeout/invalid direct data. */
void NativeV3_MarkVisionMissing(NativeV3_Controller_t *controller);
/* Called at the existing 5 ms firmware rate. Returns correction angle. */
float NativeV3_Tick5ms(NativeV3_Controller_t *controller);
void NativeV3_GetStatus(const NativeV3_Controller_t *controller,
                        NativeV3_Status_t *status);

#ifdef __cplusplus
}
#endif

#endif
