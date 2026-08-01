#include "native_v3_core.h"

#include <stddef.h>

#define NATIVE_V3_TICK_MS 5U

static float absf(float value) { return value < 0.0f ? -value : value; }

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

static uint32_t saturating_add_u32(uint32_t value, uint32_t increment)
{
    if (value > 0xFFFFFFFFU - increment) return 0xFFFFFFFFU;
    return value + increment;
}

static void clear_history(NativeV3_Controller_t *controller)
{
    controller->history_count = 0U;
    controller->history_head = 0U;
    controller->last_sample_ms = controller->now_ms;
    controller->status.velocity_cm_s = 0.0f;
}

static void reset_dynamics(NativeV3_Controller_t *controller)
{
    controller->integral = 0.0f;
    controller->stationary_ms = 0U;
    controller->settled_ms = 0U;
    controller->breakaway_ms = 0U;
    controller->status.p_deg = 0.0f;
    controller->status.i_deg = 0.0f;
    controller->status.d_deg = 0.0f;
    controller->status.brake_scale = 0.0f;
    controller->status.desired_correction_deg = 0.0f;
    clear_history(controller);
}

static uint32_t newest_history_index(const NativeV3_Controller_t *controller)
{
    return (uint32_t)((controller->history_head + 2U) % 3U);
}

static uint32_t oldest_history_index(const NativeV3_Controller_t *controller)
{
    if (controller->history_count < 3U) return 0U;
    return (uint32_t)controller->history_head;
}

static void append_history(NativeV3_Controller_t *controller,
                           float position_cm)
{
    controller->history_position_cm[controller->history_head] = position_cm;
    controller->history_time_ms[controller->history_head] = controller->now_ms;
    controller->history_head = (uint8_t)((controller->history_head + 1U) % 3U);
    if (controller->history_count < 3U) controller->history_count++;
}

static float estimate_velocity(NativeV3_Controller_t *controller)
{
    uint32_t oldest, newest, elapsed_ms;
    float raw_velocity;

    if (controller->history_count < 2U) return 0.0f;
    newest = newest_history_index(controller);
    if (controller->history_count == 2U) {
        oldest = (uint32_t)((controller->history_head + 1U) % 3U);
    } else {
        oldest = oldest_history_index(controller);
    }
    elapsed_ms = controller->history_time_ms[newest] -
                 controller->history_time_ms[oldest];
    if (elapsed_ms < 10U || elapsed_ms > 300U) return 0.0f;  /* 60Hz视觉=16.7ms */

    raw_velocity = (controller->history_position_cm[newest] -
                    controller->history_position_cm[oldest]) *
                   (1000.0f / (float)elapsed_ms);
    return controller->cfg.velocity_old_weight *
               controller->status.velocity_cm_s +
           (1.0f - controller->cfg.velocity_old_weight) * raw_velocity;
}

static float brake_scale(const NativeV3_Controller_t *controller,
                         float error_cm, float velocity_cm_s)
{
    float distance, span, ratio;
    uint8_t approaching;

    approaching = (error_cm * velocity_cm_s < 0.0f) ? 1U : 0U;
    if (!approaching) return 1.0f;

    distance = absf(error_cm);
    if (distance <= controller->cfg.brake_near_cm) return 1.0f;
    if (distance >= controller->cfg.brake_far_cm) {
        return controller->cfg.brake_far_scale;
    }
    span = controller->cfg.brake_far_cm - controller->cfg.brake_near_cm;
    if (span <= 0.001f) return 1.0f;
    ratio = (controller->cfg.brake_far_cm - distance) / span;
    return controller->cfg.brake_far_scale +
           (1.0f - controller->cfg.brake_far_scale) * ratio;
}

static float conditional_integral(NativeV3_Controller_t *controller,
                                  float error_cm, float velocity_cm_s,
                                  float sample_dt_s, float p_deg,
                                  float d_deg)
{
    float candidate, old_term, new_term, old_unsaturated, new_unsaturated;
    uint8_t eligible;

    eligible = (absf(error_cm) <= controller->cfg.integral_position_cm &&
                absf(velocity_cm_s) <=
                    controller->cfg.integral_velocity_cm_s) ? 1U : 0U;
    if (!eligible || sample_dt_s <= 0.0f || sample_dt_s > 0.20f ||
        controller->status.mode == NATIVE_V3_BREAKAWAY ||
        controller->status.mode == NATIVE_V3_SETTLED) {
        return controller->integral;
    }

    candidate = clampf(controller->integral + error_cm * sample_dt_s,
                       -controller->cfg.integral_limit,
                       controller->cfg.integral_limit);
    old_term = controller->cfg.ki * controller->integral;
    new_term = controller->cfg.ki * candidate;
    old_unsaturated = p_deg + d_deg + old_term;
    new_unsaturated = p_deg + d_deg + new_term;

    if ((old_unsaturated >= controller->cfg.output_max_deg &&
         new_unsaturated > old_unsaturated) ||
        (old_unsaturated <= controller->cfg.output_min_deg &&
         new_unsaturated < old_unsaturated)) {
        return controller->integral;
    }
    return candidate;
}

static void calculate_new_command(NativeV3_Controller_t *controller,
                                  float sample_dt_s)
{
    float error, predicted_shift, predicted_error;
    float position_for_p, p_deg, d_deg, i_deg, output;
    float velocity = controller->status.velocity_cm_s;
    float scale;
    uint8_t toward_target, moved_toward_target;

    error = controller->status.position_cm - controller->status.setpoint_cm;
    predicted_shift = clampf(velocity * controller->cfg.prediction_s,
                             -controller->cfg.prediction_limit_cm,
                             controller->cfg.prediction_limit_cm);
    controller->status.predicted_position_cm =
        controller->status.position_cm + predicted_shift;
    predicted_error = error + predicted_shift;
    controller->status.position_error_cm = error;

    scale = brake_scale(controller, error, velocity);
    controller->status.brake_scale = scale;
    position_for_p = predicted_error;
    if (absf(error) <= controller->cfg.position_deadband_cm &&
        absf(velocity) <= controller->cfg.settled_velocity_cm_s) {
        position_for_p = 0.0f;
    }

    p_deg = controller->cfg.kp * position_for_p;
    d_deg = controller->cfg.kd * velocity * scale;
    controller->integral = conditional_integral(controller, error, velocity,
                                                sample_dt_s, p_deg, d_deg);
    i_deg = controller->cfg.ki * controller->integral;
    output = p_deg + i_deg + d_deg;

    toward_target = (error * velocity < 0.0f) ? 1U : 0U;
    moved_toward_target =
        ((controller->status.position_cm -
          controller->breakaway_start_position_cm) * error < 0.0f &&
         absf(controller->status.position_cm -
              controller->breakaway_start_position_cm) >=
             controller->cfg.breakaway_release_cm) ? 1U : 0U;
    if (absf(error) <= controller->cfg.settled_position_cm &&
        absf(velocity) <= controller->cfg.settled_velocity_cm_s) {
        controller->settled_ms = saturating_add_u32(
            controller->settled_ms,
            (uint32_t)(sample_dt_s * 1000.0f + 0.5f));
    } else {
        controller->settled_ms = 0U;
    }

    if (controller->status.mode == NATIVE_V3_SETTLED) {
        if (absf(error) <= controller->cfg.settled_exit_position_cm &&
            absf(velocity) <= controller->cfg.settled_exit_velocity_cm_s) {
            output = 0.0f;
            controller->integral = 0.0f;
        } else {
            controller->status.mode = NATIVE_V3_ROLLING;
            controller->settled_ms = 0U;
        }
    }
    if (controller->settled_ms >= controller->cfg.settled_confirm_ms) {
        controller->status.mode = NATIVE_V3_SETTLED;
        controller->integral = 0.0f;
        output = 0.0f;
    } else if (controller->status.mode != NATIVE_V3_SETTLED) {
        if (absf(error) >= controller->cfg.stiction_error_cm &&
            absf(velocity) <= controller->cfg.stiction_velocity_cm_s) {
            controller->stationary_ms = saturating_add_u32(
                controller->stationary_ms,
                (uint32_t)(sample_dt_s * 1000.0f + 0.5f));
        } else {
            controller->stationary_ms = 0U;
        }

        if (controller->status.mode == NATIVE_V3_BREAKAWAY) {
            controller->breakaway_ms = saturating_add_u32(
                controller->breakaway_ms,
                (uint32_t)(sample_dt_s * 1000.0f + 0.5f));
            if ((toward_target &&
                 absf(velocity) >= controller->cfg.moving_velocity_cm_s) ||
                moved_toward_target ||
                controller->breakaway_ms >= controller->cfg.breakaway_max_ms ||
                absf(error) < controller->cfg.stiction_error_cm) {
                controller->status.mode = NATIVE_V3_ROLLING;
                controller->stationary_ms = 0U;
                controller->breakaway_ms = 0U;
            }
        } else if (controller->stationary_ms >=
                   controller->cfg.stiction_confirm_ms) {
            controller->status.mode = NATIVE_V3_BREAKAWAY;
            controller->breakaway_ms = 0U;
            controller->breakaway_start_position_cm =
                controller->status.position_cm;
        } else {
            controller->status.mode = NATIVE_V3_ROLLING;
        }

        if (controller->status.mode == NATIVE_V3_BREAKAWAY) {
            if (error > 0.0f && output <
                controller->cfg.breakaway_positive_deg) {
                output = controller->cfg.breakaway_positive_deg;
            } else if (error < 0.0f && output >
                       controller->cfg.breakaway_negative_deg) {
                output = controller->cfg.breakaway_negative_deg;
            }
        }
    }

    output = clampf(output, controller->cfg.output_min_deg,
                    controller->cfg.output_max_deg);
    controller->status.p_deg = p_deg;
    controller->status.i_deg = controller->cfg.ki * controller->integral;
    controller->status.d_deg = d_deg;
    controller->status.desired_correction_deg = output;
}

void NativeV3_DefaultConfig(NativeV3_Config_t *cfg)
{
    if (cfg == NULL) return;

    /* Values are derived only from this repository's measurements/history. */
    cfg->kp = 2.35f;   /* 实机标定 */
    cfg->kd = 0.50f;
    cfg->ki = 0.02f;
    cfg->integral_limit = 3.0f;
    cfg->output_min_deg = -6.0f;
    cfg->output_max_deg = 6.0f;
    cfg->setpoint_limit_cm = 9.0f;

    cfg->velocity_old_weight = 0.50f;
    cfg->prediction_s = 0.05f;      /* 高KP下预测超前要保守, 防过冲 */
    cfg->prediction_limit_cm = 0.5f; /* 预测限幅减半 */
    cfg->brake_near_cm = 1.0f;
    cfg->brake_far_cm = 3.0f;
    cfg->brake_far_scale = 0.35f;

    cfg->position_deadband_cm = 0.20f;
    cfg->settled_position_cm = 0.50f;   /* 锁定范围放宽: 0.5cm内即锁定, 避免脱困爬行 */
    cfg->settled_exit_position_cm = 0.80f;  /* 滞回出口也放宽 */
    cfg->settled_velocity_cm_s = 0.60f;
    cfg->settled_exit_velocity_cm_s = 1.20f;
    cfg->settled_confirm_ms = 500U;

    cfg->integral_position_cm = 1.0f;
    cfg->integral_velocity_cm_s = 1.0f;

    cfg->stiction_error_cm = 0.70f;    /* 审查: 0.35太敏感, 静差被推成摆动 */
    cfg->stiction_velocity_cm_s = 0.30f;
    cfg->moving_velocity_cm_s = 0.40f;
    cfg->stiction_confirm_ms = 400U;   /* 审查: 确认时间加长 */
    cfg->breakaway_positive_deg = 6.00f;
    cfg->breakaway_negative_deg = -6.00f;
    cfg->breakaway_release_cm = 0.25f;
    cfg->breakaway_max_ms = 200U;      /* 审查: 最长脱困缩短, 避免猛推 */

    cfg->vision_timeout_ms = 200U;
    cfg->recovery_frames = 2U;
    cfg->minimum_confidence = 40U;
}

void NativeV3_Init(NativeV3_Controller_t *controller,
                   const NativeV3_Config_t *cfg)
{
    NativeV3_Config_t defaults;
    uint8_t i;

    if (controller == NULL) return;
    NativeV3_DefaultConfig(&defaults);
    controller->cfg = cfg != NULL ? *cfg : defaults;

    controller->now_ms = 0U;
    controller->last_sample_ms = 0U;
    controller->stationary_ms = 0U;
    controller->settled_ms = 0U;
    controller->breakaway_ms = 0U;
    controller->integral = 0.0f;
    controller->breakaway_start_position_cm = 0.0f;
    controller->history_count = 0U;
    controller->history_head = 0U;
    controller->recovery_count = 0U;
    for (i = 0U; i < 3U; ++i) {
        controller->history_position_cm[i] = 0.0f;
        controller->history_time_ms[i] = 0U;
    }

    controller->status.requested = 0U;
    controller->status.vision_valid = 0U;
    controller->status.emergency = 0U;
    controller->status.mode = NATIVE_V3_PAUSED;
    controller->status.setpoint_cm = 0.0f;
    controller->status.position_cm = 0.0f;
    controller->status.predicted_position_cm = 0.0f;
    controller->status.velocity_cm_s = 0.0f;
    controller->status.position_error_cm = 0.0f;
    controller->status.p_deg = 0.0f;
    controller->status.i_deg = 0.0f;
    controller->status.d_deg = 0.0f;
    controller->status.brake_scale = 0.0f;
    controller->status.desired_correction_deg = 0.0f;
    controller->status.vision_age_ms = 0xFFFFFFFFU;
    controller->status.accepted_frames = 0U;
    controller->status.rejected_frames = 0U;
}

void NativeV3_Start(NativeV3_Controller_t *controller)
{
    if (controller == NULL || controller->status.emergency) return;
    controller->status.requested = 1U;
    controller->status.vision_valid = 0U;
    controller->status.mode = NATIVE_V3_VISION_HOLD;
    controller->recovery_count = 0U;
    controller->status.vision_age_ms = 0xFFFFFFFFU;
    reset_dynamics(controller);
}

void NativeV3_Pause(NativeV3_Controller_t *controller)
{
    if (controller == NULL) return;
    controller->status.requested = 0U;
    controller->status.vision_valid = 0U;
    controller->status.mode = NATIVE_V3_PAUSED;
    controller->recovery_count = 0U;
    reset_dynamics(controller);
}

void NativeV3_EmergencyStop(NativeV3_Controller_t *controller)
{
    if (controller == NULL) return;
    /* Keep requested semantics; the independent fault latch blocks output. */
    controller->status.emergency = 1U;
    controller->status.vision_valid = 0U;
    controller->status.mode = NATIVE_V3_EMERGENCY;
    controller->recovery_count = 0U;
    reset_dynamics(controller);
}

uint8_t NativeV3_SetSetpoint(NativeV3_Controller_t *controller, float cm,
                             uint8_t reset_controller)
{
    if (controller == NULL || !finitef(cm) ||
        absf(cm) > controller->cfg.setpoint_limit_cm) return 0U;
    controller->status.setpoint_cm = cm;
    if (reset_controller) reset_dynamics(controller);
    return 1U;
}

void NativeV3_SetGains(NativeV3_Controller_t *controller, float kp,
                       float ki, float kd)
{
    if (controller == NULL) return;
    if (finitef(kp) && kp >= 0.20f && kp <= 8.0f) controller->cfg.kp = kp;
    if (finitef(kd) && kd >= 0.0f) controller->cfg.kd = kd;  /* 无上限 */
    if (finitef(ki) && ki >= 0.0f && ki <= 0.08f) controller->cfg.ki = ki;
    controller->integral = 0.0f;
}

void NativeV3_SetOutputLimits(NativeV3_Controller_t *controller,
                              float minimum_deg, float maximum_deg)
{
    if (controller == NULL || !finitef(minimum_deg) ||
        !finitef(maximum_deg)) return;
    if (minimum_deg < -10.0f || minimum_deg > -1.0f ||
        maximum_deg < 1.0f || maximum_deg > 10.0f ||
        minimum_deg >= maximum_deg) return;
    controller->cfg.output_min_deg = minimum_deg;
    controller->cfg.output_max_deg = maximum_deg;
    controller->status.desired_correction_deg = clampf(
        controller->status.desired_correction_deg, minimum_deg, maximum_deg);
}

uint8_t NativeV3_ObserveDirect(NativeV3_Controller_t *controller,
                               float position_cm, uint8_t confidence)
{
    uint32_t sample_ms;
    float sample_dt_s;

    if (controller == NULL || !finitef(position_cm) ||
        absf(position_cm) > controller->cfg.setpoint_limit_cm ||
        confidence < controller->cfg.minimum_confidence) {
        if (controller != NULL) controller->status.rejected_frames++;
        return 0U;
    }

    sample_ms = controller->now_ms - controller->last_sample_ms;
    if (controller->history_count != 0U &&
        (sample_ms < 10U || sample_ms > 200U)) {  /* 60Hz视觉=16.7ms */
        clear_history(controller);
        sample_ms = 50U;
    }
    if (controller->history_count == 0U) sample_ms = 50U;
    sample_dt_s = (float)sample_ms / 1000.0f;
    controller->last_sample_ms = controller->now_ms;

    append_history(controller, position_cm);
    controller->status.position_cm = position_cm;
    controller->status.velocity_cm_s = estimate_velocity(controller);
    controller->status.vision_age_ms = 0U;
    controller->status.accepted_frames++;

    if (controller->recovery_count < controller->cfg.recovery_frames) {
        controller->recovery_count++;
    }
    if (controller->recovery_count < controller->cfg.recovery_frames ||
        !controller->status.requested || controller->status.emergency) {
        controller->status.vision_valid = 0U;
        controller->status.desired_correction_deg = 0.0f;
        controller->status.mode = controller->status.emergency ?
            NATIVE_V3_EMERGENCY :
            (controller->status.requested ? NATIVE_V3_VISION_HOLD :
                                            NATIVE_V3_PAUSED);
        return 1U;
    }

    controller->status.vision_valid = 1U;
    calculate_new_command(controller, sample_dt_s);
    return 1U;
}

void NativeV3_MarkVisionMissing(NativeV3_Controller_t *controller)
{
    if (controller == NULL) return;
    if (controller->status.vision_valid || controller->recovery_count != 0U) {
        controller->status.vision_valid = 0U;
        controller->recovery_count = 0U;
        controller->status.desired_correction_deg = 0.0f;
        controller->integral = 0.0f;
        controller->stationary_ms = 0U;
        controller->settled_ms = 0U;
        clear_history(controller);
    }
    if (!controller->status.emergency) {
        controller->status.mode = controller->status.requested ?
            NATIVE_V3_VISION_HOLD : NATIVE_V3_PAUSED;
    }
}

float NativeV3_Tick5ms(NativeV3_Controller_t *controller)
{
    if (controller == NULL) return 0.0f;
    controller->now_ms = saturating_add_u32(controller->now_ms,
                                             NATIVE_V3_TICK_MS);
    controller->status.vision_age_ms = saturating_add_u32(
        controller->status.vision_age_ms, NATIVE_V3_TICK_MS);

    if (controller->status.vision_valid &&
        controller->status.vision_age_ms > controller->cfg.vision_timeout_ms) {
        NativeV3_MarkVisionMissing(controller);
    }
    if (controller->status.emergency) {
        controller->status.mode = NATIVE_V3_EMERGENCY;
        return 0.0f;
    }
    if (!controller->status.requested) {
        controller->status.mode = NATIVE_V3_PAUSED;
        controller->status.desired_correction_deg = 0.0f;
        return 0.0f;
    }
    if (!controller->status.vision_valid) {
        controller->status.mode = NATIVE_V3_VISION_HOLD;
        controller->status.desired_correction_deg = 0.0f;
        return 0.0f;
    }
    return controller->status.desired_correction_deg;
}

void NativeV3_GetStatus(const NativeV3_Controller_t *controller,
                        NativeV3_Status_t *status)
{
    if (controller != NULL && status != NULL) *status = controller->status;
}
