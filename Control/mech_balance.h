#ifndef MECH_BALANCE_H
#define MECH_BALANCE_H

#include <stdint.h>

#define MECH_SEQ_STEP_COUNT       7U
#define MECH_ANGLE_MIN_DEG       (-30.0f)
#define MECH_ANGLE_MAX_DEG       45.0f
#define MECH_SEQ_TIME_MIN_MS      5U
#define MECH_SEQ_TIME_MAX_MS      10000U

typedef struct {
    float gravity;
    float accel_gain_fwd;
    float accel_gain_brake;
    float accel_bias;
    float theta_trim_deg;
    float theta_rate_limit;
    float pitch_deg;
} MechParams_t;

typedef enum {
    MECH_STATUS_ARMED = 0,
    MECH_STATUS_RUNNING,
    MECH_STATUS_FINISHED,
    MECH_STATUS_STOPPED,
    MECH_STATUS_FAULT_PWM,
    MECH_STATUS_FAULT_LIMIT,
    MECH_STATUS_FAULT_DRIVER
} MechStatus_t;

void MechBalance_Init(void);
void MechBalance_Tick5ms(void);
void MechBalance_EmergencyStop(void);

uint8_t MechBalance_SetAccel(float ax_mps2);
uint8_t MechBalance_SetParam(uint8_t id, float value);
const MechParams_t *MechBalance_GetParams(void);

uint8_t MechBalance_SetDirectAngle(float deg);
void MechBalance_ExitDirect(void);
uint8_t MechBalance_IsDirect(void);

uint8_t MechBalance_SetSeqAngle(uint8_t index, float deg);
uint8_t MechBalance_SetSeqTime(uint8_t index, uint32_t ms);
float MechBalance_GetSeqAngle(uint8_t index);
uint32_t MechBalance_GetSeqTime(uint8_t index);
uint8_t MechBalance_StartSeq(uint8_t run_mode);
void MechBalance_StopSeq(void);
uint8_t MechBalance_IsSeqActive(void);
uint8_t MechBalance_GetSeqStep(void);
uint32_t MechBalance_GetSeqElapsed(void);
MechStatus_t MechBalance_GetStatus(void);

#define MP_GRAVITY      0U
#define MP_GAIN_FWD     1U
#define MP_GAIN_BRAKE   2U
#define MP_ACCEL_BIAS   3U
#define MP_TRIM         4U
#define MP_RATE_LIMIT   5U
#define MP_PITCH        6U

#endif
