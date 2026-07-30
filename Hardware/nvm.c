#include "nvm.h"
uint8_t NVM_Save(const NVM_Settings_t *settings) { (void)settings; return 0U; }
uint8_t NVM_Load(NVM_Settings_t *settings) { if (settings) settings->magic=0U; return 0U; }
