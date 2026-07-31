#ifndef PROTO_RX_V2_H
#define PROTO_RX_V2_H

#include <stdint.h>

typedef struct {
    int16_t position_centi_cm;
    uint8_t confidence;
    uint8_t status;
    uint16_t frame_seq;
    uint16_t processing_delay_ms;
    uint32_t age_ms;
    uint32_t receive_id;
} ProtoRxV2_Ball_t;

void ProtoRxV2_Init(void);
void ProtoRxV2_ProcessByte(uint8_t byte);
void ProtoRxV2_Tick(uint32_t elapsed_ms);
uint8_t ProtoRxV2_GetSnapshot(ProtoRxV2_Ball_t *ball);

#endif
