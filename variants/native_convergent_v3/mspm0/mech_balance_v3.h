#ifndef MECH_BALANCE_V3_H
#define MECH_BALANCE_V3_H

/* Keep the original application-facing API unchanged. */
#include "mech_balance.h"
#include "native_v3_core.h"

/* Optional extended telemetry for bench logging; the old app need not use it. */
void MechBalanceV3_GetDetailedStatus(NativeV3_Status_t *status);

#endif
