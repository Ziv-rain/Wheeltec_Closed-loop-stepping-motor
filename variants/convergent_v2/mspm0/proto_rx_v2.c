#include "proto_rx_v2.h"

#include <string.h>

#define PROTO_V2_SYNC0 0xAAU
#define PROTO_V2_SYNC1 0x55U
#define PROTO_V2_TYPE_BALL 0x30U
#define PROTO_V2_PAYLOAD_LENGTH 8U
#define PROTO_V2_MAX_PAYLOAD 64U

typedef struct {
    uint8_t state;
    uint8_t type;
    uint8_t length;
    uint8_t index;
    uint8_t data[PROTO_V2_MAX_PAYLOAD];
    uint16_t received_crc;
    ProtoRxV2_Ball_t balls[2];
    volatile uint8_t published_index;
    volatile uint8_t has_ball;
    volatile uint32_t age_ms;
    uint32_t next_receive_id;
} ProtoRxV2_State_t;

static ProtoRxV2_State_t s_proto_v2;

static uint16_t proto_v2_crc(uint8_t type, uint8_t length,
                             const uint8_t *data)
{
    uint16_t crc = 0U;
    uint8_t i;
    uint8_t bit;
    uint8_t values[2];
    values[0] = type;
    values[1] = length;

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

static void proto_v2_accept_ball(void)
{
    const uint8_t *data = s_proto_v2.data;
    uint8_t next_index = (uint8_t)(s_proto_v2.published_index ^ 1U);
    ProtoRxV2_Ball_t *ball = &s_proto_v2.balls[next_index];

    ball->position_centi_cm = (int16_t)(
        (uint16_t)data[0] | ((uint16_t)data[1] << 8U));
    ball->confidence = data[2];
    ball->status = data[3];
    ball->frame_seq = (uint16_t)(
        (uint16_t)data[4] | ((uint16_t)data[5] << 8U));
    ball->processing_delay_ms = (uint16_t)(
        (uint16_t)data[6] | ((uint16_t)data[7] << 8U));
    s_proto_v2.next_receive_id++;
    if (s_proto_v2.next_receive_id == 0U) {
        s_proto_v2.next_receive_id = 1U;
    }
    ball->receive_id = s_proto_v2.next_receive_id;
    ball->age_ms = 0U;

    /* Publish only after the inactive snapshot has been fully written. */
    s_proto_v2.age_ms = 0U;
    s_proto_v2.published_index = next_index;
    s_proto_v2.has_ball = 1U;
}

void ProtoRxV2_Init(void)
{
    memset(&s_proto_v2, 0, sizeof(s_proto_v2));
    s_proto_v2.age_ms = 0xFFFFFFFFU;
}

void ProtoRxV2_ProcessByte(uint8_t byte)
{
    uint16_t calculated_crc;
    switch (s_proto_v2.state) {
    case 0U:
        if (byte == PROTO_V2_SYNC0) s_proto_v2.state = 1U;
        break;
    case 1U:
        if (byte == PROTO_V2_SYNC1) {
            s_proto_v2.state = 2U;
        } else {
            s_proto_v2.state = byte == PROTO_V2_SYNC0 ? 1U : 0U;
        }
        break;
    case 2U:
        s_proto_v2.type = byte;
        s_proto_v2.state = 3U;
        break;
    case 3U:
        s_proto_v2.length = byte;
        s_proto_v2.index = 0U;
        if (s_proto_v2.length > PROTO_V2_MAX_PAYLOAD) {
            s_proto_v2.state = 0U;
        } else {
            s_proto_v2.state = s_proto_v2.length > 0U ? 4U : 5U;
        }
        break;
    case 4U:
        s_proto_v2.data[s_proto_v2.index++] = byte;
        if (s_proto_v2.index >= s_proto_v2.length) {
            s_proto_v2.state = 5U;
        }
        break;
    case 5U:
        s_proto_v2.received_crc = byte;
        s_proto_v2.state = 6U;
        break;
    default:
        s_proto_v2.received_crc |= (uint16_t)byte << 8U;
        calculated_crc = proto_v2_crc(s_proto_v2.type,
                                      s_proto_v2.length,
                                      s_proto_v2.data);
        if (calculated_crc == s_proto_v2.received_crc &&
            s_proto_v2.type == PROTO_V2_TYPE_BALL &&
            s_proto_v2.length == PROTO_V2_PAYLOAD_LENGTH) {
            proto_v2_accept_ball();
        }
        s_proto_v2.state = 0U;
        break;
    }
}

void ProtoRxV2_Tick(uint32_t elapsed_ms)
{
    uint32_t age_ms = s_proto_v2.age_ms;
    if (age_ms == 0xFFFFFFFFU) return;
    if (elapsed_ms > 0xFFFFFFFFU - age_ms) {
        s_proto_v2.age_ms = 0xFFFFFFFFU;
    } else {
        s_proto_v2.age_ms = age_ms + elapsed_ms;
    }
}

uint8_t ProtoRxV2_GetSnapshot(ProtoRxV2_Ball_t *ball)
{
    uint8_t published_index;
    if (!ball || !s_proto_v2.has_ball) return 0U;
    published_index = s_proto_v2.published_index;
    *ball = s_proto_v2.balls[published_index];
    ball->age_ms = s_proto_v2.age_ms;
    return 1U;
}
