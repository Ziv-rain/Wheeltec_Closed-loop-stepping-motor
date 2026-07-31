#ifndef PROTO_RX_H
#define PROTO_RX_H
#include <stdint.h>
typedef struct {
    int16_t position_centi_cm;
    uint8_t confidence, status;
    uint32_t age_ms;
    uint32_t frame_id; /* 每收到一帧合法视觉数据递增 */
} BallData_t;
void ProtoRx_Init(void);
void ProtoRx_ProcessByte(uint8_t byte);
void ProtoRx_Tick(uint32_t elapsed_ms);
uint8_t ProtoRx_GetBall(BallData_t *ball);
uint8_t ProtoRx_GetTouchTarget(float *target_cm); /* 触摸目标: 返回1=有新目标 */
void UART2_IRQHandler(void);
#endif
