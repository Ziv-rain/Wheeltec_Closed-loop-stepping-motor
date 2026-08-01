/**
 * accel_history_profile.h - Conservative Task-5 acceleration preview.
 *
 * The profile is the per-phase median of nine complete Task-5 runs collected
 * on the current vehicle.  Every entry is the measured vehicle acceleration
 * 50 ms after the indexed phase, stored in milli-m/s^2 to save flash.
 *
 * Rollback switch: set REAL_PERCENT to 100U and HISTORY_PERCENT to 0U.
 * No PID, motor, trajectory, angle-limit or feed-forward gain is changed here.
 */
#ifndef ACCEL_HISTORY_PROFILE_H
#define ACCEL_HISTORY_PROFILE_H

#include <stdint.h>

#ifndef ACCEL_HISTORY_REAL_PERCENT
#define ACCEL_HISTORY_REAL_PERCENT      70U
#endif
#ifndef ACCEL_HISTORY_HISTORY_PERCENT
#define ACCEL_HISTORY_HISTORY_PERCENT   30U
#endif
#define ACCEL_HISTORY_SAMPLE_MS         50U
#define ACCEL_HISTORY_TASK5_COUNT       507U

#if (ACCEL_HISTORY_REAL_PERCENT + ACCEL_HISTORY_HISTORY_PERCENT) != 100U
#error "Acceleration-history weights must add up to 100 percent"
#endif

#if ACCEL_HISTORY_HISTORY_PERCENT > 0U
static const int16_t s_task5_future50_milli_mps2[ACCEL_HISTORY_TASK5_COUNT] = {
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,  157,  471,  314,  157,  209,  314,  419,  367,  367,
     367,  314,  314,  314,  262,  262,  209,  209,  157,  157,  157,  209,
      52,  105,    0,    0,   52,   52,    0,    0,    0,    0,   52,    0,
      52,   52,    0,    0,    0,    0,    0,    0,    0,    0,   52,    0,
       0,    0,  -52,    0,    0,  -52,  -52,    0,    0,    0,    0,    0,
       0,   52,    0,    0,    0,   52,    0,    0,    0,  -52,    0,    0,
     -52,  -52,  -52,  -52,  -52,    0,    0,   52,    0,    0,   52,    0,
      52,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,   52,   52,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,  -52, -105, -157, -262, -262, -157, -262, -209, -105, -105,
       0,    0,    0,   52,  105,  157,  105,  157,  105,   52,   52,   52,
      52,    0,    0,    0, -105,  -52, -157, -105,  -52,  -52,  -52,    0,
       0,   52,   52,   52,  105,    0,   52,   52,   52,    0,    0,    0,
       0,    0,  -52,    0,    0,  -52,    0,  -52,  -52,  -52,    0,    0,
      52,  105,   52,   52,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      52,   52,    0,    0,   52,   52,   52,  105,  105,   52,   52,   52,
      52,   52,  157,   52,   52,   52,    0,    0,    0,    0, -105,  -52,
     -52,  -52,  -52, -105,    0,  -52,  -52,    0,    0,    0,   52,    0,
      52,    0,    0,    0,    0,    0,    0,   52,   52,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,  -52,    0,    0,    0,   52,   52,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,  -52,    0,    0,
       0,    0,    0,   52,   52,   52,   52,   52,   52,    0,    0,    0,
       0,    0,  -52,  -52, -157, -262, -209, -157, -314, -157,  -52,  -52,
       0,    0,    0,    0,  157,  105,   52,  157,   52,   52,   52,    0,
       0,    0,    0,  -52,  -52,  -52, -209, -105, -157, -105,  -52,    0,
       0,    0,   52,   52,   52,  105,   52,    0,   52,   52,    0,    0,
       0,    0,    0,    0,  -52,  -52,  -52, -105,  -52, -105,    0,    0,
       0,  105,    0,   52,  105,   52,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   52,
      52,   52,    0,   52,   52,   52,   52,  105,   52,   52,   52,   52,
      52,    0,   52,    0,    0,  -52,  -52,  -52, -105, -157, -157, -157,
    -262, -314, -314, -262, -367, -367, -262, -314, -262, -262, -262, -262,
    -262, -209, -314, -157, -157,  -52,    0,    0,    0,    0,    0,    0,
       0,    0,    0
};

/* Nearest 50 ms phase. Returns 0 outside the learned Task-5 duration. */
static inline uint8_t AccelHistory_GetTask5Future50(uint32_t elapsed_ms,
                                                    float *accel_mps2)
{
    uint32_t index;
    if (accel_mps2 == 0) return 0U;
    index = (elapsed_ms + (ACCEL_HISTORY_SAMPLE_MS / 2U)) /
            ACCEL_HISTORY_SAMPLE_MS;
    if (index >= ACCEL_HISTORY_TASK5_COUNT) return 0U;
    *accel_mps2 = (float)s_task5_future50_milli_mps2[index] * 0.001f;
    return 1U;
}

static inline float AccelHistory_Blend(float real_accel_mps2,
                                       float history_accel_mps2)
{
    return (float)ACCEL_HISTORY_REAL_PERCENT * 0.01f * real_accel_mps2 +
           (float)ACCEL_HISTORY_HISTORY_PERCENT * 0.01f * history_accel_mps2;
}
#endif

#endif
