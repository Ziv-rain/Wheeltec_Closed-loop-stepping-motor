#include "ball_beam_v2.h"
#include "proto_rx_v2.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_DT_MS 5U
#define TEST_HISTORY 64U

static float test_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static uint16_t test_crc(uint8_t type, uint8_t length,
                         const uint8_t *data)
{
    uint16_t crc = 0U;
    uint8_t values[2] = {type, length};
    uint8_t i;
    uint8_t bit;
    for (i = 0U; i < 2U; ++i) {
        crc ^= values[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                               (uint16_t)(crc >> 1U);
        }
    }
    for (i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                               (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static void test_protocol_v2(void)
{
    uint8_t payload[8];
    uint8_t packet[14];
    uint16_t crc;
    uint8_t i;
    ProtoRxV2_Ball_t ball;

    payload[0] = 0x85U;
    payload[1] = 0xFFU; /* -123 = -1.23 cm */
    payload[2] = 90U;
    payload[3] = 2U;
    payload[4] = 42U;
    payload[5] = 0U;
    payload[6] = 37U;
    payload[7] = 0U;
    crc = test_crc(0x30U, 8U, payload);
    packet[0] = 0xAAU;
    packet[1] = 0x55U;
    packet[2] = 0x30U;
    packet[3] = 8U;
    memcpy(&packet[4], payload, sizeof(payload));
    packet[12] = (uint8_t)(crc & 0xFFU);
    packet[13] = (uint8_t)(crc >> 8U);

    ProtoRxV2_Init();
    for (i = 0U; i < sizeof(packet); ++i) {
        ProtoRxV2_ProcessByte(packet[i]);
    }
    assert(ProtoRxV2_GetSnapshot(&ball));
    assert(ball.position_centi_cm == -123);
    assert(ball.confidence == 90U);
    assert(ball.status == 2U);
    assert(ball.frame_seq == 42U);
    assert(ball.processing_delay_ms == 37U);
    ProtoRxV2_Tick(15U);
    assert(ProtoRxV2_GetSnapshot(&ball));
    assert(ball.age_ms == 15U);
}

static void send_sample(BBV2_Controller_t *controller, uint16_t sequence,
                        float position_cm, uint16_t processing_ms)
{
    BBV2_VisionSample_t sample;
    sample.position_cm = position_cm;
    sample.confidence = 90U;
    sample.status = 2U;
    sample.valid = 1U;
    sample.has_timing = 1U;
    sample.frame_seq = sequence;
    sample.processing_delay_ms = processing_ms;
    sample.receiver_age_ms = 0U;
    BBV2_Tick(controller, TEST_DT_MS, 0.0f, 1U, &sample);
}

static void tick_without_sample(BBV2_Controller_t *controller,
                                uint32_t duration_ms)
{
    uint32_t elapsed;
    for (elapsed = 0U; elapsed < duration_ms; elapsed += TEST_DT_MS) {
        BBV2_Tick(controller, TEST_DT_MS, 0.0f, 1U, 0);
    }
}

static void test_latch_loss_resume_and_pause(void)
{
    BBV2_Controller_t controller;
    BBV2_Config_t config;
    const BBV2_Output_t *output;

    BBV2_DefaultConfig(&config);
    config.setpoint_cm = 0.0f;
    BBV2_Init(&controller, &config);
    BBV2_Start(&controller);
    BBV2_Tick(&controller, TEST_DT_MS, 0.0f, 1U, 0);
    output = BBV2_GetOutput(&controller);
    assert(output->requested == 1U);
    assert(output->mode == BBV2_MODE_WAIT_VISION);

    send_sample(&controller, 1U, 5.0f, 50U);
    tick_without_sample(&controller, 45U);
    send_sample(&controller, 2U, 5.0f, 50U);
    output = BBV2_GetOutput(&controller);
    assert(output->vision_valid == 1U);
    assert(output->mode == BBV2_MODE_TRACKING);
    assert(output->velocity_reference_cm_s < 0.0f);
    assert(output->target_angle_deg > config.theta_trim_deg);

    tick_without_sample(&controller, config.vision_timeout_ms + 10U);
    output = BBV2_GetOutput(&controller);
    assert(output->requested == 1U);
    assert(output->vision_valid == 0U);
    assert(output->mode == BBV2_MODE_LEVEL_HOLD);

    send_sample(&controller, 3U, 4.0f, 50U);
    tick_without_sample(&controller, 45U);
    send_sample(&controller, 4U, 3.9f, 50U);
    output = BBV2_GetOutput(&controller);
    assert(output->requested == 1U);
    assert(output->vision_valid == 1U);

    BBV2_Pause(&controller);
    BBV2_Tick(&controller, TEST_DT_MS, 0.0f, 1U, 0);
    output = BBV2_GetOutput(&controller);
    assert(output->requested == 0U);
    assert(output->mode == BBV2_MODE_IDLE);
}

static void test_outlier_and_emergency_semantics(void)
{
    BBV2_Controller_t controller;
    BBV2_Config_t config;
    const BBV2_Output_t *output;
    uint32_t rejected_before;

    BBV2_DefaultConfig(&config);
    config.setpoint_cm = 0.0f;
    BBV2_Init(&controller, &config);
    BBV2_Start(&controller);
    send_sample(&controller, 1U, 0.0f, 50U);
    tick_without_sample(&controller, 45U);
    send_sample(&controller, 2U, 0.1f, 50U);
    rejected_before = BBV2_GetOutput(&controller)->rejected_frames;
    tick_without_sample(&controller, 45U);
    send_sample(&controller, 3U, 7.5f, 50U);
    output = BBV2_GetOutput(&controller);
    assert(output->rejected_frames == rejected_before + 1U);

    BBV2_EmergencyStop(&controller);
    BBV2_Tick(&controller, TEST_DT_MS, 0.0f, 1U, 0);
    output = BBV2_GetOutput(&controller);
    assert(output->requested == 1U);
    assert(output->target_enabled == 0U);
    assert(output->mode == BBV2_MODE_EMERGENCY_STOP);
}

typedef struct {
    float position_cm;
    float velocity_cm_s;
    float angle_deg;
    float position_history[TEST_HISTORY];
    uint32_t history_head;
} TestPlant_t;

static void plant_init(TestPlant_t *plant, float initial_position_cm)
{
    uint32_t i;
    memset(plant, 0, sizeof(*plant));
    plant->position_cm = initial_position_cm;
    for (i = 0U; i < TEST_HISTORY; ++i) {
        plant->position_history[i] = initial_position_cm;
    }
}

static void plant_step(TestPlant_t *plant, float target_angle_deg,
                       float plant_gain, float dt_s)
{
    float acceleration;
    plant->angle_deg += (target_angle_deg - plant->angle_deg) *
                        dt_s / 0.060f;
    acceleration = -plant_gain * plant->angle_deg -
                   0.20f * plant->velocity_cm_s;
    plant->velocity_cm_s += acceleration * dt_s;
    plant->position_cm += plant->velocity_cm_s * dt_s;
    plant->history_head = (plant->history_head + 1U) % TEST_HISTORY;
    plant->position_history[plant->history_head] = plant->position_cm;
}

static float plant_delayed_position(const TestPlant_t *plant,
                                    uint32_t delay_ms)
{
    uint32_t steps = delay_ms / TEST_DT_MS;
    uint32_t index;
    if (steps >= TEST_HISTORY) steps = TEST_HISTORY - 1U;
    index = (plant->history_head + TEST_HISTORY - steps) % TEST_HISTORY;
    return plant->position_history[index];
}

static void run_closed_loop_case(float physical_gain, uint32_t delay_ms)
{
    BBV2_Controller_t controller;
    BBV2_Config_t config;
    BBV2_VisionSample_t sample;
    const BBV2_Output_t *output;
    TestPlant_t plant;
    uint32_t step;
    uint32_t total_steps = 18000U / TEST_DT_MS;
    uint32_t camera_steps = 50U / TEST_DT_MS;
    uint16_t sequence = 0U;
    float target_angle = 0.0f;
    float maximum_abs_position = 0.0f;
    float tail_min = 1000.0f;
    float tail_max = -1000.0f;
    float noise;

    BBV2_DefaultConfig(&config);
    config.setpoint_cm = 0.0f;
    config.plant_adapt_rate = 0.0f;
    BBV2_Init(&controller, &config);
    BBV2_Start(&controller);
    plant_init(&plant, 8.0f);

    for (step = 0U; step < total_steps; ++step) {
        BBV2_VisionSample_t *sample_pointer = 0;
        if ((step % camera_steps) == 0U) {
            sequence++;
            noise = 0.05f * sinf((float)step * 0.071f);
            sample.position_cm =
                plant_delayed_position(&plant, delay_ms) + noise;
            sample.confidence = 92U;
            sample.status = 2U;
            sample.valid = 1U;
            sample.has_timing = 1U;
            sample.frame_seq = sequence;
            sample.processing_delay_ms = (uint16_t)(
                delay_ms > config.camera_extra_delay_ms ?
                delay_ms - config.camera_extra_delay_ms : 1U);
            sample.receiver_age_ms = 0U;
            sample_pointer = &sample;
        }
        BBV2_Tick(&controller, TEST_DT_MS, plant.angle_deg, 1U,
                  sample_pointer);
        output = BBV2_GetOutput(&controller);
        if (output->target_enabled) target_angle = output->target_angle_deg;
        plant_step(&plant, target_angle, physical_gain,
                   (float)TEST_DT_MS / 1000.0f);
        if (test_absf(plant.position_cm) > maximum_abs_position) {
            maximum_abs_position = test_absf(plant.position_cm);
        }
        if (step >= 12000U / TEST_DT_MS) {
            if (plant.position_cm < tail_min) tail_min = plant.position_cm;
            if (plant.position_cm > tail_max) tail_max = plant.position_cm;
        }
    }

    printf("gain=%4.1f delay=%3lu ms final=%+.3f tail_amp=%.3f max=%.3f\n",
           physical_gain, (unsigned long)delay_ms, plant.position_cm,
           0.5f * (tail_max - tail_min), maximum_abs_position);
    assert(maximum_abs_position < 9.0f);
    assert(test_absf(plant.position_cm) < 0.80f);
    assert(0.5f * (tail_max - tail_min) < 0.80f);
}

static void test_closed_loop_sweep(void)
{
    const float gains[] = {1.2f, 2.4f, 6.0f};
    const uint32_t delays[] = {50U, 100U, 150U};
    uint32_t gain_index;
    uint32_t delay_index;
    for (gain_index = 0U;
         gain_index < sizeof(gains) / sizeof(gains[0]);
         ++gain_index) {
        for (delay_index = 0U;
             delay_index < sizeof(delays) / sizeof(delays[0]);
             ++delay_index) {
            run_closed_loop_case(gains[gain_index], delays[delay_index]);
        }
    }
}

int main(void)
{
    test_protocol_v2();
    test_latch_loss_resume_and_pause();
    test_outlier_and_emergency_semantics();
    test_closed_loop_sweep();
    puts("all convergent_v2 tests passed");
    return 0;
}
