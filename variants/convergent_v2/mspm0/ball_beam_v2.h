#ifndef BALL_BEAM_V2_H
#define BALL_BEAM_V2_H

#include <stdint.h>

#define BBV2_HISTORY_LENGTH 48U

typedef enum {
    BBV2_MODE_IDLE = 0,
    BBV2_MODE_WAIT_VISION,
    BBV2_MODE_TRACKING,
    BBV2_MODE_LEVEL_HOLD,
    BBV2_MODE_SAFETY_INHIBIT,
    BBV2_MODE_EMERGENCY_STOP
} BBV2_Mode_t;

typedef struct {
    uint32_t control_period_ms;
    uint32_t vision_timeout_ms;
    uint32_t nominal_camera_delay_ms;
    uint32_t camera_extra_delay_ms;
    uint32_t delay_min_ms;
    uint32_t delay_max_ms;
    uint32_t stable_time_ms;
    uint32_t stiction_wait_ms;
    uint8_t min_confidence;
    uint8_t reacquire_frames;

    float setpoint_cm;
    float position_limit_cm;
    float innovation_limit_cm;
    float max_measured_speed_cm_s;
    float position_correction_weight;
    float velocity_measurement_weight;

    float position_to_speed_per_s;
    float max_speed_cm_s;
    float max_reference_accel_cm_s2;
    float velocity_gain_deg_per_cm_s;

    float integral_gain_deg_per_cm_s;
    float integral_limit_cm_s;
    float integral_enable_error_cm;
    float integral_enable_speed_cm_s;

    float stiction_error_cm;
    float stiction_speed_cm_s;
    float stiction_comp_deg;

    float normal_angle_limit_deg;
    float brake_angle_limit_deg;
    float feedforward_limit_deg;
    float angle_rate_limit_deg_s;
    float brake_margin_cm;

    float theta_trim_deg;
    float plant_gain_cm_s2_per_deg;
    float plant_gain_min_cm_s2_per_deg;
    float plant_gain_max_cm_s2_per_deg;
    float plant_adapt_rate;
    float velocity_damping_per_s;

    float stable_error_cm;
    float stable_speed_cm_s;
} BBV2_Config_t;

typedef struct {
    float position_cm;
    uint8_t confidence;
    uint8_t status;
    uint8_t valid;
    uint8_t has_timing;
    uint16_t frame_seq;
    uint16_t processing_delay_ms;
    uint16_t receiver_age_ms;
} BBV2_VisionSample_t;

typedef struct {
    float position_cm;
    float velocity_cm_s;
    float actual_angle_deg;
} BBV2_HistorySample_t;

typedef struct {
    BBV2_Mode_t mode;
    uint8_t requested;
    uint8_t vision_valid;
    uint8_t actuator_safe;
    uint8_t target_enabled;
    uint8_t stable;
    uint8_t brake_active;

    float target_angle_deg;
    float estimated_position_cm;
    float estimated_velocity_cm_s;
    float position_error_cm;
    float velocity_reference_cm_s;
    float feedforward_angle_deg;
    float feedback_angle_deg;
    float integral_angle_deg;
    float plant_gain_cm_s2_per_deg;

    uint32_t vision_age_ms;
    uint32_t estimated_delay_ms;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
} BBV2_Output_t;

typedef struct {
    BBV2_Config_t config;
    BBV2_HistorySample_t history[BBV2_HISTORY_LENGTH];
    uint8_t history_head;
    uint8_t history_count;

    float estimated_position_cm;
    float estimated_velocity_cm_s;
    float filtered_measured_velocity_cm_s;
    float previous_filtered_velocity_cm_s;
    float angle_command_deg;
    float velocity_reference_cm_s;
    float integral_cm_s;
    float plant_gain_cm_s2_per_deg;

    float last_measurement_cm;
    uint32_t last_capture_time_ms;
    uint32_t time_ms;
    uint32_t vision_age_ms;
    uint32_t stable_elapsed_ms;
    uint32_t stiction_elapsed_ms;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    uint32_t last_delay_ms;

    uint16_t last_frame_seq;
    uint8_t has_last_frame_seq;
    uint8_t has_measurement;
    uint8_t requested;
    uint8_t vision_valid;
    uint8_t actuator_safe;
    uint8_t emergency_stop;
    uint8_t reacquire_count;
    uint8_t last_output_saturated;

    BBV2_Output_t output;
} BBV2_Controller_t;

void BBV2_DefaultConfig(BBV2_Config_t *config);
void BBV2_Init(BBV2_Controller_t *controller,
               const BBV2_Config_t *config);
void BBV2_Start(BBV2_Controller_t *controller);
void BBV2_Pause(BBV2_Controller_t *controller);
void BBV2_EmergencyStop(BBV2_Controller_t *controller);
void BBV2_SetSetpoint(BBV2_Controller_t *controller, float setpoint_cm);
void BBV2_SetTrim(BBV2_Controller_t *controller, float trim_deg);

void BBV2_Tick(BBV2_Controller_t *controller,
               uint32_t elapsed_ms,
               float actual_angle_deg,
               uint8_t actuator_safe,
               const BBV2_VisionSample_t *new_sample);

const BBV2_Output_t *BBV2_GetOutput(const BBV2_Controller_t *controller);

#endif
