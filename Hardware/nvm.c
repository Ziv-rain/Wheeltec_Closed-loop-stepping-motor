/**
 * nvm.c - Flash non-volatile storage
 *
 * MSPM0G3507: 128KB flash, 1KB sectors (128 sectors total)
 * Uses last sector (sector 127) at address 0x0001FC00
 */
#include "nvm.h"
#include <stddef.h>
#include "ti_msp_dl_config.h"

/* Last 1KB sector */
#define NVM_SECTOR_ADDR     0x0001FC00U
#define NVM_MAGIC           0xA5A5F0F0U

uint8_t NVM_Load(NVM_Settings_t *settings)
{
    const NVM_Settings_t *flash_data = (const NVM_Settings_t *)NVM_SECTOR_ADDR;

    if (settings == NULL) return 0U;

    if (flash_data->magic == NVM_MAGIC) {
        settings->magic         = flash_data->magic;
        settings->encoder_zero  = flash_data->encoder_zero;
        settings->pos_dir_level = flash_data->pos_dir_level;
        return 1U;
    }

    settings->magic = 0U;
    return 0U;
}

void NVM_Save(const NVM_Settings_t *settings)
{
    uint32_t buf[4];
    uint32_t i;
    DL_FLASHCTL_COMMAND_STATUS status;

    if (settings == NULL) return;

    buf[0] = NVM_MAGIC;
    buf[1] = (uint32_t)settings->encoder_zero;
    buf[2] = (uint32_t)settings->pos_dir_level;
    buf[3] = 0U;

    /* Unprotect sector */
    DL_FlashCTL_unprotectSector(FLASHCTL, NVM_SECTOR_ADDR,
                                 DL_FLASHCTL_REGION_SELECT_MAIN);

    /* Erase 1KB sector (void API, must wait for completion) */
    DL_FlashCTL_eraseMemory(FLASHCTL, NVM_SECTOR_ADDR,
                             DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    DL_FlashCTL_waitForCmdDone(FLASHCTL);

    /* Program 4 words one-by-one using RAM-based API (returns status) */
    for (i = 0U; i < 4U; i++) {
        status = DL_FlashCTL_programMemoryFromRAM32(
            FLASHCTL, NVM_SECTOR_ADDR + i * 4U, &buf[i]);
        if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) break;
    }

    /* Re-protect */
    DL_FlashCTL_protectSector(FLASHCTL, NVM_SECTOR_ADDR,
                               DL_FLASHCTL_REGION_SELECT_MAIN);
}
