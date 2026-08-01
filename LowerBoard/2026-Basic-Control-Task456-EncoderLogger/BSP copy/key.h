#ifndef _KEY_H_
#define _KEY_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ================================================================
 *   天猛星 MSPM0G3507 64Pin 开发板
 *   依据硬件网表验证, 4 个引脚均仅连接到排针:
 *     KEY1 - PB2  → IOMUX_PINCM15
 *     KEY2 - PB3  → IOMUX_PINCM16
 *     KEY3 - PB5  → IOMUX_PINCM18
 *     KEY4 - PB7  → IOMUX_PINCM22
 * ================================================================ */

#define KEY1_IOMUX          (IOMUX_PINCM15)   /* PB2 */
#define KEY2_IOMUX          (IOMUX_PINCM16)   /* PB3 */
#define KEY3_IOMUX          (IOMUX_PINCM18)   /* PB5 */
#define KEY4_IOMUX          (IOMUX_PINCM22)   /* PB7 */

#define KEY_PORT            (GPIOB)
#define KEY1_PIN            (DL_GPIO_PIN_2)
#define KEY2_PIN            (DL_GPIO_PIN_3)
#define KEY3_PIN            (DL_GPIO_PIN_5)
#define KEY4_PIN            (DL_GPIO_PIN_7)

#define KEY_NONE            0
#define KEY1_PRESSED        1
#define KEY2_PRESSED        2
#define KEY3_PRESSED        3
#define KEY4_PRESSED        4

#define KEY_DEBOUNCE_MS     20

void Key_Init(void);
uint8_t Key_Scan(void);

#endif /* _KEY_H_ */
