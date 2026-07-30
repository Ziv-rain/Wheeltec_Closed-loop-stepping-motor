/**
 * nvm.h - Flash non-volatile storage for motor settings
 *
 * Uses last flash page (0x1FE00, page 255 of 256) to store:
 *   - Encoder zero offset (int32_t)
 *   - Positive direction level (uint8_t)
 *
 * On MSPM0G3507: 128KB flash, 512B pages, flash is at 0x00000000-0x00020000
 */
#ifndef NVM_H
#define NVM_H

#include <stdint.h>

/* Settings stored in flash */
typedef struct {
    uint32_t magic;       /* Magic number to validate data */
    int32_t  encoder_zero; /* Encoder zero offset (counts) */
    uint8_t  pos_dir_level;/* AXIS_X_POSITIVE_DIR_LEVEL */
    uint8_t  reserved[3];  /* Padding */
} NVM_Settings_t;

/* Load settings from flash. Returns 1 if valid data found, 0 if empty. */
uint8_t NVM_Load(NVM_Settings_t *settings);

/* Save settings to flash. */
void NVM_Save(const NVM_Settings_t *settings);

#endif
