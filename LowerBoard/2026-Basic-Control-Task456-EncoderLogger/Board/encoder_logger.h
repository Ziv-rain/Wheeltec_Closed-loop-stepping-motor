#ifndef ENCODER_LOGGER_H
#define ENCODER_LOGGER_H

#include <stdint.h>

/*
 * Record-only logger for repeatable Task 4/5/6 runs.
 *
 * Sampling is done in RAM at 20 Hz. No flash write and no UART transmission
 * takes place while the car is moving. After EncoderLogger_Stop(), the main
 * loop exports the frozen run over the existing AA55 link.
 */
#define ENCODER_LOG_SAMPLE_MS       50U
#define ENCODER_LOG_CAPACITY      1536U  /* 76.8 seconds at 20 Hz */

void EncoderLogger_Init(void);
void EncoderLogger_Start(uint8_t task_id, uint32_t tick_ms);
void EncoderLogger_Stop(uint32_t tick_ms);
uint8_t EncoderLogger_RequestDump(void);
void EncoderLogger_Tick1ms(uint32_t tick_ms);
void EncoderLogger_Process(void);

uint8_t EncoderLogger_IsRecording(void);
uint8_t EncoderLogger_IsDumping(void);

#endif /* ENCODER_LOGGER_H */
