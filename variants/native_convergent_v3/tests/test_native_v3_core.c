#include "native_v3_core.h"

#include <assert.h>

static void advance_ms(NativeV3_Controller_t *controller, uint32_t ms)
{
    uint32_t elapsed;
    for (elapsed = 0U; elapsed < ms; elapsed += 5U) {
        (void)NativeV3_Tick5ms(controller);
    }
}

static void observe_after(NativeV3_Controller_t *controller, uint32_t ms,
                          float position_cm)
{
    advance_ms(controller, ms);
    assert(NativeV3_ObserveDirect(controller, position_cm, 90U));
}

static void test_defaults_and_latch(void)
{
    NativeV3_Controller_t controller;
    NativeV3_Status_t status;

    NativeV3_Init(&controller, 0);
    NativeV3_GetStatus(&controller, &status);
    assert(status.requested == 0U);
    assert(controller.cfg.kp == 0.80f);
    assert(controller.cfg.kd == 0.50f);
    assert(controller.cfg.ki == 0.02f);
    assert(controller.cfg.output_min_deg == -6.0f);
    assert(controller.cfg.output_max_deg == 6.0f);
    assert(controller.cfg.breakaway_positive_deg == 3.20f);
    assert(controller.cfg.breakaway_negative_deg == -5.20f);

    NativeV3_Start(&controller);
    observe_after(&controller, 50U, 2.0f);
    observe_after(&controller, 50U, 2.0f);
    NativeV3_GetStatus(&controller, &status);
    assert(status.vision_valid == 1U);

    NativeV3_MarkVisionMissing(&controller);
    NativeV3_GetStatus(&controller, &status);
    assert(status.requested == 1U);
    assert(status.vision_valid == 0U);
    assert(status.mode == NATIVE_V3_VISION_HOLD);
    assert(NativeV3_Tick5ms(&controller) == 0.0f);

    observe_after(&controller, 50U, 1.8f);
    NativeV3_GetStatus(&controller, &status);
    assert(status.vision_valid == 0U);
    observe_after(&controller, 50U, 1.7f);
    NativeV3_GetStatus(&controller, &status);
    assert(status.vision_valid == 1U);

    NativeV3_Pause(&controller);
    NativeV3_GetStatus(&controller, &status);
    assert(status.requested == 0U);
    assert(status.mode == NATIVE_V3_PAUSED);
}

static void test_asymmetric_breakaway(void)
{
    NativeV3_Controller_t positive;
    NativeV3_Controller_t negative;
    NativeV3_Status_t status;
    uint8_t i;

    NativeV3_Init(&positive, 0);
    NativeV3_Start(&positive);
    for (i = 0U; i < 8U; ++i) observe_after(&positive, 50U, 2.0f);
    NativeV3_GetStatus(&positive, &status);
    assert(status.mode == NATIVE_V3_BREAKAWAY);
    assert(status.desired_correction_deg >= 3.20f);

    NativeV3_Init(&negative, 0);
    NativeV3_Start(&negative);
    for (i = 0U; i < 8U; ++i) observe_after(&negative, 50U, -2.0f);
    NativeV3_GetStatus(&negative, &status);
    assert(status.mode == NATIVE_V3_BREAKAWAY);
    assert(status.desired_correction_deg <= -5.20f);
}

static void test_new_frame_semantics_and_limits(void)
{
    NativeV3_Controller_t controller;
    NativeV3_Status_t before, after;

    NativeV3_Init(&controller, 0);
    NativeV3_Start(&controller);
    assert(NativeV3_SetSetpoint(&controller, 9.0f, 1U));
    assert(!NativeV3_SetSetpoint(&controller, 9.01f, 1U));
    assert(!NativeV3_ObserveDirect(&controller, 9.1f, 90U));
    assert(!NativeV3_ObserveDirect(&controller, 0.0f, 39U));

    assert(NativeV3_SetSetpoint(&controller, 0.0f, 1U));
    observe_after(&controller, 50U, 2.0f);
    observe_after(&controller, 50U, 1.8f);
    NativeV3_GetStatus(&controller, &before);
    advance_ms(&controller, 100U);
    NativeV3_GetStatus(&controller, &after);
    assert(after.desired_correction_deg == before.desired_correction_deg);
    assert(after.accepted_frames == before.accepted_frames);

    advance_ms(&controller, 105U);
    NativeV3_GetStatus(&controller, &after);
    assert(after.vision_valid == 0U);
    assert(after.requested == 1U);
    assert(after.mode == NATIVE_V3_VISION_HOLD);
}

static void test_emergency_keeps_request_but_blocks_output(void)
{
    NativeV3_Controller_t controller;
    NativeV3_Status_t status;

    NativeV3_Init(&controller, 0);
    NativeV3_Start(&controller);
    NativeV3_EmergencyStop(&controller);
    NativeV3_GetStatus(&controller, &status);
    assert(status.requested == 1U);
    assert(status.emergency == 1U);
    assert(status.mode == NATIVE_V3_EMERGENCY);
    assert(NativeV3_Tick5ms(&controller) == 0.0f);
    NativeV3_Start(&controller);
    NativeV3_GetStatus(&controller, &status);
    assert(status.emergency == 1U);
}

int main(void)
{
    test_defaults_and_latch();
    test_asymmetric_breakaway();
    test_new_frame_semantics_and_limits();
    test_emergency_keeps_request_but_blocks_output();
    return 0;
}
