#ifndef BALL_BEAM_V2_RUNTIME_H
#define BALL_BEAM_V2_RUNTIME_H

#include "ball_beam_v2.h"

void BBV2_Runtime_Init(const BBV2_Config_t *config);
void BBV2_Runtime_Start(void);
void BBV2_Runtime_Pause(void);
void BBV2_Runtime_EmergencyStop(void);
void BBV2_Runtime_SetSetpoint(float setpoint_cm);
void BBV2_Runtime_SetTrim(float trim_deg);
void BBV2_Runtime_Tick5ms(const BBV2_VisionSample_t *new_sample);
const BBV2_Output_t *BBV2_Runtime_GetOutput(void);

#endif
