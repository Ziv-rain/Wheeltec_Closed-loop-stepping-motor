#ifndef BALL_CONTROL_H
#define BALL_CONTROL_H
#include <stdint.h>

void BallControl_Init(void);
void BallControl_Tick5ms(void);
uint8_t BallControl_IsHomingReady(void);
uint8_t BallControl_HasFault(void);
void BallControl_EmergencyStop(void);
#endif
