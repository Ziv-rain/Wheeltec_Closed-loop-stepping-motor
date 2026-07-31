#ifndef MECH_BALANCE_H
#define MECH_BALANCE_H

#include <stdint.h>

typedef struct {
    float gravity;
    float accel_gain_fwd;
    float accel_gain_brake;
    float accel_bias;
    float theta_trim_deg;
    float theta_min_deg;
    float theta_max_deg;
    float theta_rate_limit;
    float pitch_deg;
} MechParams_t;

typedef enum {
    MECH_STATUS_ARMED = 0,
    MECH_STATUS_RUNNING,
    MECH_STATUS_STOPPED,
    MECH_STATUS_FAULT_PWM,
    MECH_STATUS_FAULT_LIMIT,
    MECH_STATUS_FAULT_DRIVER
} MechStatus_t;

void MechBalance_Init(void);
void MechBalance_Tick5ms(void);
void MechBalance_EmergencyStop(void);
uint8_t MechBalance_SetAccel(float ax_mps2);
uint8_t MechBalance_SetParam(uint8_t id, float v);
uint8_t MechBalance_SetDirectAngle(float deg);
void MechBalance_ExitDirect(void);
uint8_t MechBalance_IsDirect(void);
const MechParams_t *MechBalance_GetParams(void);
MechStatus_t MechBalance_GetStatus(void);

#define MP_GRAVITY      0U
#define MP_GAIN_FWD     1U
#define MP_GAIN_BRAKE   2U
#define MP_ACCEL_BIAS   3U
#define MP_TRIM         4U
#define MP_MIN_DEG      5U
#define MP_MAX_DEG      6U
#define MP_RATE_LIMIT   7U
#define MP_PITCH        8U

#endif
