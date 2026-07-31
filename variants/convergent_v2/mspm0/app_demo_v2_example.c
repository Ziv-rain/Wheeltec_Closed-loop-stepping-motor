/*
 * Integration example only. This file deliberately does not replace
 * Control/app_demo.c or Hardware/proto_rx.c.
 *
 * Call BBV2_Example_ProcessVisionByte() from the UART2 RX ISR, call
 * BBV2_Example_Tick5ms() after homing is ready, and map UART0 V/W/X/N
 * commands to the functions below in a dedicated V2 build.
 */
#include "ball_beam_v2_runtime.h"
#include "proto_rx_v2.h"

static uint32_t s_example_last_receive_id;

void BBV2_Example_Init(void)
{
    BBV2_Config_t config;
    BBV2_DefaultConfig(&config);
    BBV2_Runtime_Init(&config);
    ProtoRxV2_Init();
    s_example_last_receive_id = 0U;
}

void BBV2_Example_ProcessVisionByte(uint8_t byte)
{
    ProtoRxV2_ProcessByte(byte);
}

void BBV2_Example_StartV(void)
{
    BBV2_Runtime_Start();
}

void BBV2_Example_PauseW(void)
{
    BBV2_Runtime_Pause();
}

void BBV2_Example_EmergencyX(void)
{
    BBV2_Runtime_EmergencyStop();
}

void BBV2_Example_SetTargetN(float target_cm)
{
    BBV2_Runtime_SetSetpoint(target_cm);
}

void BBV2_Example_Tick5ms(void)
{
    ProtoRxV2_Ball_t ball;
    BBV2_VisionSample_t sample;
    BBV2_VisionSample_t *sample_pointer = 0;

    ProtoRxV2_Tick(5U);
    if (ProtoRxV2_GetSnapshot(&ball) &&
        ball.receive_id != s_example_last_receive_id) {
        s_example_last_receive_id = ball.receive_id;
        sample.position_cm = (float)ball.position_centi_cm / 100.0f;
        sample.confidence = ball.confidence;
        sample.status = ball.status;
        sample.valid = ball.status != 0U ? 1U : 0U;
        sample.has_timing = ball.processing_delay_ms > 0U ? 1U : 0U;
        sample.frame_seq = ball.frame_seq;
        sample.processing_delay_ms = ball.processing_delay_ms;
        sample.receiver_age_ms = (uint16_t)(
            ball.age_ms > 65535U ? 65535U : ball.age_ms);
        sample_pointer = &sample;
    }
    BBV2_Runtime_Tick5ms(sample_pointer);
}
