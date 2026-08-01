#ifndef HISTORY_LOGGER_H
#define HISTORY_LOGGER_H

#include <stdint.h>
#include "task_ctrl.h"

/*
 * One-run RAM recorder for the upper controller.
 *
 * Samples are captured only when a fresh 20 Hz wheel telemetry frame arrives.
 * Nothing is printed and no flash is written while recording. After the run
 * has stopped, HistoryLogger_RequestDump() enables incremental CSV output from
 * the main loop.
 */
#define HISTORY_LOG_CAPACITY 768U  /* 38.4 seconds at 20 Hz, 15,360 bytes */

void HistoryLogger_Init(void);
void HistoryLogger_Start(uint8_t task_id);
void HistoryLogger_Stop(void);
void HistoryLogger_Capture(const WheelTelemetry_t *wheel);
uint8_t HistoryLogger_RequestDump(void);
void HistoryLogger_Process(void);
uint8_t HistoryLogger_IsRecording(void);
uint8_t HistoryLogger_IsDumping(void);

#endif /* HISTORY_LOGGER_H */
