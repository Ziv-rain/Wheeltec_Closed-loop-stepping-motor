#include "ball_beam_v2.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float bbv2_clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float bbv2_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static float bbv2_signf(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

static uint32_t bbv2_add_u32_sat(uint32_t value, uint32_t increment)
{
    if (increment > 0xFFFFFFFFU - value) return 0xFFFFFFFFU;
    return value + increment;
}

static void bbv2_predict(const BBV2_Controller_t *controller,
                         float *position_cm,
                         float *velocity_cm_s,
                         float actual_angle_deg,
                         float dt_s)
{
    float relative_angle = actual_angle_deg -
                           controller->config.theta_trim_deg;
    float acceleration =
        -controller->plant_gain_cm_s2_per_deg * relative_angle -
        controller->config.velocity_damping_per_s * (*velocity_cm_s);

    *position_cm += (*velocity_cm_s) * dt_s +
                    0.5f * acceleration * dt_s * dt_s;
    *velocity_cm_s += acceleration * dt_s;
}

static void bbv2_push_history(BBV2_Controller_t *controller,
                              float actual_angle_deg)
{
    uint8_t next;
    if (controller->history_count == 0U) {
        next = 0U;
        controller->history_count = 1U;
    } else {
        next = (uint8_t)((controller->history_head + 1U) %
                         BBV2_HISTORY_LENGTH);
        if (controller->history_count < BBV2_HISTORY_LENGTH) {
            controller->history_count++;
        }
    }
    controller->history_head = next;
    controller->history[next].position_cm =
        controller->estimated_position_cm;
    controller->history[next].velocity_cm_s =
        controller->estimated_velocity_cm_s;
    controller->history[next].actual_angle_deg = actual_angle_deg;
}

static uint8_t bbv2_history_index_ago(const BBV2_Controller_t *controller,
                                      uint32_t steps_ago)
{
    uint32_t available = controller->history_count > 0U ?
                         (uint32_t)controller->history_count - 1U : 0U;
    uint32_t head = controller->history_head;
    if (steps_ago > available) steps_ago = available;
    return (uint8_t)((head + BBV2_HISTORY_LENGTH - steps_ago) %
                     BBV2_HISTORY_LENGTH);
}

static uint32_t bbv2_sample_delay_ms(const BBV2_Controller_t *controller,
                                     const BBV2_VisionSample_t *sample)
{
    uint32_t delay_ms;
    if (sample->has_timing && sample->processing_delay_ms > 0U) {
        delay_ms = (uint32_t)sample->processing_delay_ms +
                   controller->config.camera_extra_delay_ms +
                   (uint32_t)sample->receiver_age_ms;
    } else {
        delay_ms = controller->config.nominal_camera_delay_ms +
                   (uint32_t)sample->receiver_age_ms;
    }
    if (delay_ms < controller->config.delay_min_ms) {
        delay_ms = controller->config.delay_min_ms;
    }
    if (delay_ms > controller->config.delay_max_ms) {
        delay_ms = controller->config.delay_max_ms;
    }
    return delay_ms;
}

static uint8_t bbv2_sample_basic_valid(const BBV2_Controller_t *controller,
                                       const BBV2_VisionSample_t *sample)
{
    if (!sample || !sample->valid || sample->status == 0U) return 0U;
    if (sample->confidence < controller->config.min_confidence) return 0U;
    if (sample->position_cm != sample->position_cm) return 0U;
    if (bbv2_absf(sample->position_cm) >
        controller->config.position_limit_cm) return 0U;
    return 1U;
}

static void bbv2_replay_history(BBV2_Controller_t *controller,
                                uint8_t start_index,
                                uint32_t steps,
                                float dt_s)
{
    float position = controller->history[start_index].position_cm;
    float velocity = controller->history[start_index].velocity_cm_s;
    uint8_t index = start_index;
    uint32_t step;

    for (step = 0U; step < steps; ++step) {
        index = (uint8_t)((index + 1U) % BBV2_HISTORY_LENGTH);
        bbv2_predict(controller, &position, &velocity,
                     controller->history[index].actual_angle_deg, dt_s);
        controller->history[index].position_cm = position;
        controller->history[index].velocity_cm_s = velocity;
    }
    controller->estimated_position_cm = position;
    controller->estimated_velocity_cm_s = velocity;
}

static void bbv2_accept_first_measurement(
    BBV2_Controller_t *controller,
    const BBV2_VisionSample_t *sample,
    uint32_t delay_ms)
{
    uint8_t i;
    controller->estimated_position_cm = sample->position_cm;
    controller->estimated_velocity_cm_s = 0.0f;
    controller->filtered_measured_velocity_cm_s = 0.0f;
    controller->previous_filtered_velocity_cm_s = 0.0f;
    controller->last_measurement_cm = sample->position_cm;
    controller->last_capture_time_ms = controller->time_ms - delay_ms;
    controller->has_measurement = 1U;
    controller->vision_age_ms = sample->receiver_age_ms;
    controller->last_delay_ms = delay_ms;
    controller->accepted_frames++;
    controller->reacquire_count = 1U;
    controller->vision_valid =
        controller->config.reacquire_frames <= 1U ? 1U : 0U;

    for (i = 0U; i < controller->history_count; ++i) {
        controller->history[i].position_cm = sample->position_cm;
        controller->history[i].velocity_cm_s = 0.0f;
    }
}

static void bbv2_handle_measurement(BBV2_Controller_t *controller,
                                    const BBV2_VisionSample_t *sample,
                                    uint32_t elapsed_ms)
{
    uint32_t delay_ms;
    uint32_t steps_ago;
    uint32_t capture_time_ms;
    uint32_t sample_interval_ms;
    uint8_t past_index;
    float dt_s;
    float raw_velocity;
    float innovation;
    float previous_filtered_velocity;
    float acceleration_observed;
    float relative_angle;
    float observed_gain;
    uint8_t allow_reacquire;

    if (!bbv2_sample_basic_valid(controller, sample)) return;
    if (sample->frame_seq != 0U && controller->has_last_frame_seq &&
        sample->frame_seq == controller->last_frame_seq) return;

    if (sample->frame_seq != 0U) {
        controller->last_frame_seq = sample->frame_seq;
        controller->has_last_frame_seq = 1U;
    }

    delay_ms = bbv2_sample_delay_ms(controller, sample);
    if (!controller->has_measurement) {
        bbv2_accept_first_measurement(controller, sample, delay_ms);
        return;
    }

    if (elapsed_ms == 0U) elapsed_ms = controller->config.control_period_ms;
    steps_ago = (delay_ms + elapsed_ms / 2U) / elapsed_ms;
    if (controller->history_count > 0U &&
        steps_ago >= controller->history_count) {
        steps_ago = (uint32_t)controller->history_count - 1U;
    }
    past_index = bbv2_history_index_ago(controller, steps_ago);
    capture_time_ms = controller->time_ms - steps_ago * elapsed_ms;
    sample_interval_ms = capture_time_ms - controller->last_capture_time_ms;

    if (sample_interval_ms >= 20U && sample_interval_ms <= 250U) {
        raw_velocity =
            (sample->position_cm - controller->last_measurement_cm) *
            (1000.0f / (float)sample_interval_ms);
        raw_velocity = bbv2_clampf(
            raw_velocity,
            -controller->config.max_measured_speed_cm_s,
            controller->config.max_measured_speed_cm_s);
    } else {
        raw_velocity = controller->filtered_measured_velocity_cm_s;
    }

    previous_filtered_velocity =
        controller->filtered_measured_velocity_cm_s;
    controller->filtered_measured_velocity_cm_s =
        (1.0f - controller->config.velocity_measurement_weight) *
            controller->filtered_measured_velocity_cm_s +
        controller->config.velocity_measurement_weight * raw_velocity;

    innovation = sample->position_cm -
                 controller->history[past_index].position_cm;
    allow_reacquire = (!controller->vision_valid ||
                       controller->vision_age_ms >
                           controller->config.vision_timeout_ms) ? 1U : 0U;
    if (!allow_reacquire &&
        bbv2_absf(innovation) > controller->config.innovation_limit_cm) {
        controller->filtered_measured_velocity_cm_s =
            previous_filtered_velocity;
        controller->rejected_frames++;
        return;
    }

    controller->history[past_index].position_cm +=
        controller->config.position_correction_weight * innovation;
    controller->history[past_index].velocity_cm_s =
        (1.0f - controller->config.velocity_measurement_weight) *
            controller->history[past_index].velocity_cm_s +
        controller->config.velocity_measurement_weight *
            controller->filtered_measured_velocity_cm_s;

    if (sample_interval_ms >= 30U && sample_interval_ms <= 150U) {
        dt_s = (float)sample_interval_ms / 1000.0f;
        acceleration_observed =
            (controller->filtered_measured_velocity_cm_s -
             previous_filtered_velocity) / dt_s;
        relative_angle = controller->history[past_index].actual_angle_deg -
                         controller->config.theta_trim_deg;
        if (bbv2_absf(relative_angle) >= 0.5f &&
            bbv2_absf(controller->filtered_measured_velocity_cm_s) <=
                controller->config.max_measured_speed_cm_s) {
            observed_gain =
                -(acceleration_observed +
                  controller->config.velocity_damping_per_s *
                      controller->filtered_measured_velocity_cm_s) /
                relative_angle;
            if (observed_gain >=
                    controller->config.plant_gain_min_cm_s2_per_deg &&
                observed_gain <=
                    controller->config.plant_gain_max_cm_s2_per_deg) {
                controller->plant_gain_cm_s2_per_deg +=
                    controller->config.plant_adapt_rate *
                    (observed_gain -
                     controller->plant_gain_cm_s2_per_deg);
            }
        }
    }

    bbv2_replay_history(controller, past_index, steps_ago,
                        (float)elapsed_ms / 1000.0f);
    controller->last_measurement_cm = sample->position_cm;
    controller->last_capture_time_ms = capture_time_ms;
    controller->previous_filtered_velocity_cm_s =
        previous_filtered_velocity;
    controller->vision_age_ms = sample->receiver_age_ms;
    controller->last_delay_ms = delay_ms;
    controller->accepted_frames++;
    if (!controller->vision_valid) {
        if (controller->reacquire_count < 255U) {
            controller->reacquire_count++;
        }
        if (controller->reacquire_count >=
            controller->config.reacquire_frames) {
            controller->vision_valid = 1U;
            controller->integral_cm_s = 0.0f;
            controller->stable_elapsed_ms = 0U;
            controller->stiction_elapsed_ms = 0U;
        }
    }
}

static float bbv2_rate_limit(float target, float previous,
                             float maximum_rate_per_s, float dt_s)
{
    float maximum_change = maximum_rate_per_s * dt_s;
    return bbv2_clampf(target, previous - maximum_change,
                       previous + maximum_change);
}

static void bbv2_fill_output(BBV2_Controller_t *controller,
                             BBV2_Mode_t mode,
                             uint8_t target_enabled,
                             uint8_t brake_active,
                             float error,
                             float feedforward,
                             float feedback,
                             float integral_angle)
{
    BBV2_Output_t *output = &controller->output;
    output->mode = mode;
    output->requested = controller->requested;
    output->vision_valid = controller->vision_valid;
    output->actuator_safe = controller->actuator_safe;
    output->target_enabled = target_enabled;
    output->stable =
        controller->stable_elapsed_ms >=
        controller->config.stable_time_ms ? 1U : 0U;
    output->brake_active = brake_active;
    output->target_angle_deg = controller->angle_command_deg;
    output->estimated_position_cm = controller->estimated_position_cm;
    output->estimated_velocity_cm_s = controller->estimated_velocity_cm_s;
    output->position_error_cm = error;
    output->velocity_reference_cm_s =
        controller->velocity_reference_cm_s;
    output->feedforward_angle_deg = feedforward;
    output->feedback_angle_deg = feedback;
    output->integral_angle_deg = integral_angle;
    output->plant_gain_cm_s2_per_deg =
        controller->plant_gain_cm_s2_per_deg;
    output->vision_age_ms = controller->vision_age_ms;
    output->estimated_delay_ms = controller->last_delay_ms;
    output->accepted_frames = controller->accepted_frames;
    output->rejected_frames = controller->rejected_frames;
}

void BBV2_DefaultConfig(BBV2_Config_t *config)
{
    if (!config) return;
    memset(config, 0, sizeof(*config));
    config->control_period_ms = 5U;
    config->vision_timeout_ms = 150U;
    config->nominal_camera_delay_ms = 100U;
    config->camera_extra_delay_ms = 50U;
    config->delay_min_ms = 50U;
    config->delay_max_ms = 180U;
    config->stable_time_ms = 500U;
    config->stiction_wait_ms = 400U;
    config->min_confidence = 50U;
    config->reacquire_frames = 2U;

    config->setpoint_cm = -5.0f;
    config->position_limit_cm = 9.0f;
    config->innovation_limit_cm = 2.5f;
    config->max_measured_speed_cm_s = 20.0f;
    config->position_correction_weight = 0.65f;
    config->velocity_measurement_weight = 0.65f;

    config->position_to_speed_per_s = 0.35f;
    config->max_speed_cm_s = 2.5f;
    config->max_reference_accel_cm_s2 = 2.0f;
    config->velocity_gain_deg_per_cm_s = 0.55f;

    config->integral_gain_deg_per_cm_s = 0.03f;
    config->integral_limit_cm_s = 3.0f;
    config->integral_enable_error_cm = 1.0f;
    config->integral_enable_speed_cm_s = 0.6f;

    config->stiction_error_cm = 0.35f;
    config->stiction_speed_cm_s = 0.20f;
    config->stiction_comp_deg = 0.30f;

    config->normal_angle_limit_deg = 1.5f;
    config->brake_angle_limit_deg = 2.0f;
    config->feedforward_limit_deg = 0.60f;
    config->angle_rate_limit_deg_s = 20.0f;
    config->brake_margin_cm = 0.35f;

    config->theta_trim_deg = 0.0f;
    config->plant_gain_cm_s2_per_deg = 2.4f;
    config->plant_gain_min_cm_s2_per_deg = 1.2f;
    config->plant_gain_max_cm_s2_per_deg = 6.0f;
    config->plant_adapt_rate = 0.005f;
    config->velocity_damping_per_s = 0.20f;

    config->stable_error_cm = 0.50f;
    config->stable_speed_cm_s = 0.50f;
}

void BBV2_Init(BBV2_Controller_t *controller,
               const BBV2_Config_t *config)
{
    BBV2_Config_t defaults;
    if (!controller) return;
    memset(controller, 0, sizeof(*controller));
    if (config) {
        controller->config = *config;
    } else {
        BBV2_DefaultConfig(&defaults);
        controller->config = defaults;
    }
    controller->angle_command_deg = controller->config.theta_trim_deg;
    controller->plant_gain_cm_s2_per_deg =
        controller->config.plant_gain_cm_s2_per_deg;
    controller->vision_age_ms = 0xFFFFFFFFU;
    controller->actuator_safe = 1U;
    bbv2_fill_output(controller, BBV2_MODE_IDLE, 1U, 0U,
                     0.0f, 0.0f, 0.0f, 0.0f);
}

void BBV2_Start(BBV2_Controller_t *controller)
{
    if (!controller) return;
    controller->requested = 1U;
    controller->integral_cm_s = 0.0f;
    controller->stable_elapsed_ms = 0U;
    controller->stiction_elapsed_ms = 0U;
    controller->velocity_reference_cm_s = 0.0f;
}

void BBV2_Pause(BBV2_Controller_t *controller)
{
    if (!controller) return;
    controller->requested = 0U;
    controller->integral_cm_s = 0.0f;
    controller->stable_elapsed_ms = 0U;
    controller->stiction_elapsed_ms = 0U;
    controller->velocity_reference_cm_s = 0.0f;
}

void BBV2_EmergencyStop(BBV2_Controller_t *controller)
{
    if (!controller) return;
    controller->emergency_stop = 1U;
    controller->stable_elapsed_ms = 0U;
}

void BBV2_SetSetpoint(BBV2_Controller_t *controller, float setpoint_cm)
{
    if (!controller || setpoint_cm != setpoint_cm) return;
    controller->config.setpoint_cm = bbv2_clampf(
        setpoint_cm, -controller->config.position_limit_cm,
        controller->config.position_limit_cm);
    controller->integral_cm_s = 0.0f;
    controller->stable_elapsed_ms = 0U;
    controller->stiction_elapsed_ms = 0U;
}

void BBV2_SetTrim(BBV2_Controller_t *controller, float trim_deg)
{
    if (!controller || trim_deg != trim_deg) return;
    controller->config.theta_trim_deg = trim_deg;
}

void BBV2_Tick(BBV2_Controller_t *controller,
               uint32_t elapsed_ms,
               float actual_angle_deg,
               uint8_t actuator_safe,
               const BBV2_VisionSample_t *new_sample)
{
    const BBV2_Config_t *config;
    float dt_s;
    float error = 0.0f;
    float raw_velocity_reference;
    float previous_velocity_reference;
    float reference_acceleration;
    float feedforward = 0.0f;
    float feedback = 0.0f;
    float integral_angle = 0.0f;
    float stiction_angle = 0.0f;
    float desired_angle;
    float normal_unsaturated_angle;
    float stopping_distance;
    float brake_acceleration;
    uint8_t target_enabled;
    uint8_t brake_active = 0U;
    BBV2_Mode_t mode;

    if (!controller) return;
    config = &controller->config;
    if (elapsed_ms == 0U) elapsed_ms = config->control_period_ms;
    dt_s = (float)elapsed_ms / 1000.0f;
    controller->time_ms = bbv2_add_u32_sat(controller->time_ms,
                                           elapsed_ms);
    controller->vision_age_ms = bbv2_add_u32_sat(
        controller->vision_age_ms, elapsed_ms);
    controller->actuator_safe = actuator_safe ? 1U : 0U;

    bbv2_predict(controller, &controller->estimated_position_cm,
                 &controller->estimated_velocity_cm_s,
                 actual_angle_deg, dt_s);
    bbv2_push_history(controller, actual_angle_deg);

    if (new_sample) {
        bbv2_handle_measurement(controller, new_sample, elapsed_ms);
    }
    if (controller->vision_age_ms > config->vision_timeout_ms) {
        controller->vision_valid = 0U;
        controller->reacquire_count = 0U;
        controller->stable_elapsed_ms = 0U;
    }

    if (controller->emergency_stop) {
        controller->velocity_reference_cm_s = 0.0f;
        controller->stable_elapsed_ms = 0U;
        bbv2_fill_output(controller, BBV2_MODE_EMERGENCY_STOP,
                         0U, 0U, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    if (!controller->actuator_safe) {
        controller->velocity_reference_cm_s = 0.0f;
        controller->stable_elapsed_ms = 0U;
        bbv2_fill_output(controller, BBV2_MODE_SAFETY_INHIBIT,
                         0U, 0U, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    target_enabled = 1U;
    if (!controller->requested) {
        controller->velocity_reference_cm_s = bbv2_rate_limit(
            0.0f, controller->velocity_reference_cm_s,
            config->max_reference_accel_cm_s2, dt_s);
        controller->integral_cm_s = 0.0f;
        controller->stable_elapsed_ms = 0U;
        desired_angle = config->theta_trim_deg;
        controller->angle_command_deg = bbv2_rate_limit(
            desired_angle, controller->angle_command_deg,
            config->angle_rate_limit_deg_s, dt_s);
        bbv2_fill_output(controller, BBV2_MODE_IDLE,
                         target_enabled, 0U, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    if (!controller->vision_valid) {
        controller->velocity_reference_cm_s = bbv2_rate_limit(
            0.0f, controller->velocity_reference_cm_s,
            config->max_reference_accel_cm_s2, dt_s);
        controller->integral_cm_s = 0.0f;
        controller->stable_elapsed_ms = 0U;
        controller->stiction_elapsed_ms = 0U;
        desired_angle = config->theta_trim_deg;
        controller->angle_command_deg = bbv2_rate_limit(
            desired_angle, controller->angle_command_deg,
            config->angle_rate_limit_deg_s, dt_s);
        mode = controller->has_measurement ? BBV2_MODE_LEVEL_HOLD :
                                            BBV2_MODE_WAIT_VISION;
        bbv2_fill_output(controller, mode, target_enabled, 0U,
                         0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    error = controller->estimated_position_cm - config->setpoint_cm;
    raw_velocity_reference = bbv2_clampf(
        -config->position_to_speed_per_s * error,
        -config->max_speed_cm_s, config->max_speed_cm_s);
    previous_velocity_reference = controller->velocity_reference_cm_s;
    controller->velocity_reference_cm_s = bbv2_rate_limit(
        raw_velocity_reference, previous_velocity_reference,
        config->max_reference_accel_cm_s2, dt_s);
    reference_acceleration =
        (controller->velocity_reference_cm_s -
         previous_velocity_reference) / dt_s;

    if (controller->plant_gain_cm_s2_per_deg > 0.01f) {
        feedforward = -reference_acceleration /
                      controller->plant_gain_cm_s2_per_deg;
    }
    feedforward = bbv2_clampf(feedforward,
                              -config->feedforward_limit_deg,
                              config->feedforward_limit_deg);
    feedback = config->velocity_gain_deg_per_cm_s *
               (controller->estimated_velocity_cm_s -
                controller->velocity_reference_cm_s);

    normal_unsaturated_angle = config->theta_trim_deg +
                               feedforward + feedback +
                               config->integral_gain_deg_per_cm_s *
                                   controller->integral_cm_s;
    if (bbv2_absf(error) <= config->integral_enable_error_cm &&
        bbv2_absf(controller->estimated_velocity_cm_s) <=
            config->integral_enable_speed_cm_s &&
        !controller->last_output_saturated &&
        bbv2_absf(normal_unsaturated_angle - config->theta_trim_deg) <
            config->normal_angle_limit_deg) {
        controller->integral_cm_s = bbv2_clampf(
            controller->integral_cm_s + error * dt_s,
            -config->integral_limit_cm_s,
            config->integral_limit_cm_s);
    } else if (bbv2_absf(error) > config->integral_enable_error_cm) {
        controller->integral_cm_s = 0.0f;
    }
    integral_angle = config->integral_gain_deg_per_cm_s *
                     controller->integral_cm_s;

    if (bbv2_absf(error) >= config->stiction_error_cm &&
        bbv2_absf(controller->estimated_velocity_cm_s) <=
            config->stiction_speed_cm_s) {
        controller->stiction_elapsed_ms = bbv2_add_u32_sat(
            controller->stiction_elapsed_ms, elapsed_ms);
        if (controller->stiction_elapsed_ms >= config->stiction_wait_ms) {
            stiction_angle = bbv2_signf(error) *
                             config->stiction_comp_deg;
        }
    } else {
        controller->stiction_elapsed_ms = 0U;
    }

    desired_angle = config->theta_trim_deg + feedforward + feedback +
                    integral_angle + stiction_angle;

    brake_acceleration = controller->plant_gain_cm_s2_per_deg *
                         config->brake_angle_limit_deg;
    if (brake_acceleration > 0.01f) {
        stopping_distance =
            controller->estimated_velocity_cm_s *
            controller->estimated_velocity_cm_s /
            (2.0f * brake_acceleration);
    } else {
        stopping_distance = 0.0f;
    }
    if (error * controller->estimated_velocity_cm_s < 0.0f &&
        bbv2_absf(controller->estimated_velocity_cm_s) >
            config->stable_speed_cm_s &&
        bbv2_absf(error) <= stopping_distance + config->brake_margin_cm) {
        desired_angle = config->theta_trim_deg +
                        bbv2_signf(controller->estimated_velocity_cm_s) *
                            config->brake_angle_limit_deg;
        brake_active = 1U;
    }

    if ((controller->estimated_position_cm >=
             config->position_limit_cm - config->brake_margin_cm &&
         controller->estimated_velocity_cm_s > 0.0f) ||
        (controller->estimated_position_cm <=
             -config->position_limit_cm + config->brake_margin_cm &&
         controller->estimated_velocity_cm_s < 0.0f)) {
        desired_angle = config->theta_trim_deg +
                        bbv2_signf(controller->estimated_velocity_cm_s) *
                            config->brake_angle_limit_deg;
        brake_active = 1U;
    }

    if (brake_active) {
        desired_angle = bbv2_clampf(
            desired_angle,
            config->theta_trim_deg - config->brake_angle_limit_deg,
            config->theta_trim_deg + config->brake_angle_limit_deg);
    } else {
        desired_angle = bbv2_clampf(
            desired_angle,
            config->theta_trim_deg - config->normal_angle_limit_deg,
            config->theta_trim_deg + config->normal_angle_limit_deg);
    }
    controller->last_output_saturated =
        bbv2_absf(normal_unsaturated_angle - desired_angle) > 0.001f ?
        1U : 0U;
    controller->angle_command_deg = bbv2_rate_limit(
        desired_angle, controller->angle_command_deg,
        config->angle_rate_limit_deg_s, dt_s);

    if (bbv2_absf(error) <= config->stable_error_cm &&
        bbv2_absf(controller->estimated_velocity_cm_s) <=
            config->stable_speed_cm_s) {
        controller->stable_elapsed_ms = bbv2_add_u32_sat(
            controller->stable_elapsed_ms, elapsed_ms);
    } else {
        controller->stable_elapsed_ms = 0U;
    }

    bbv2_fill_output(controller, BBV2_MODE_TRACKING,
                     target_enabled, brake_active, error,
                     feedforward, feedback, integral_angle);
}

const BBV2_Output_t *BBV2_GetOutput(const BBV2_Controller_t *controller)
{
    return controller ? &controller->output : NULL;
}
