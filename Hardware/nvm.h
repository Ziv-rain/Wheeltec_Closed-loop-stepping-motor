/**
 * nvm.h - Flash non-volatile storage for motor settings
 *
 * Uses last flash sector (0x1FC00) to store:
 *   - PWM absolute angle at zero position (float, 0~360 deg)
 *   - Positive direction level (uint8_t)
 *
 * On MSPM0G3507: 128KB flash, 512B pages, flash is at 0x00000000-0x00020000
 */
#ifndef NVM_H
#define NVM_H

#include <stdint.h>

/* Settings stored in flash */
typedef struct {
    uint32_t magic;        /* Magic number to validate data */
    float    pwm_at_zero;  /* PWM absolute angle (deg) at zero position */
    uint8_t  pos_dir_level;/* AXIS_X_POSITIVE_DIR_LEVEL */
    uint8_t  reserved[3];  /* Padding */
} NVM_Settings_t;

/* Load settings from flash. Returns 1 if valid data found, 0 if empty. */
uint8_t NVM_Load(NVM_Settings_t *settings);

/* Save settings to flash (disabled on main bring-up branch). */
uint8_t NVM_Save(const NVM_Settings_t *settings);

#endif
