#ifndef BALL_CONTROL_H
#define BALL_CONTROL_H
#include <stdint.h>

void BallControl_Init(void);
void BallControl_Tick5ms(void);
uint8_t BallControl_IsHomingReady(void);
uint8_t BallControl_HasFault(void);
void BallControl_EmergencyStop(void);
/* 强制PID模式: V键启动后无论如何不退出, 直到收到T键停止 */
void BallControl_ForceStart(void);
void BallControl_ForceStop(void);
#endif
