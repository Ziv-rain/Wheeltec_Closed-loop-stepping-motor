#include "key.h"
#include "delay.h"

void Key_Init(void)
{
    DL_GPIO_initDigitalInputFeatures(KEY1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 强制切换方向为输入 — 解决 SYSCFG_DL_init 将未使用引脚初始化为输出的问题 */
    DL_GPIO_clearPins(KEY_PORT,
        KEY1_PIN | KEY2_PIN | KEY3_PIN | KEY4_PIN);
    DL_GPIO_disableOutput(KEY_PORT,
        KEY1_PIN | KEY2_PIN | KEY3_PIN | KEY4_PIN);
}

static uint8_t Key_ReadPin(uint32_t pin)
{
    uint32_t val = DL_GPIO_readPins(KEY_PORT, pin);
    return (val != 0) ? 1 : 0;
}

uint8_t Key_Scan(void)
{
    if (Key_ReadPin(KEY1_PIN) == 0)
    {
        delay_ms(KEY_DEBOUNCE_MS);
        if (Key_ReadPin(KEY1_PIN) == 0)
        {
            while (Key_ReadPin(KEY1_PIN) == 0);
            return KEY1_PRESSED;
        }
    }

    if (Key_ReadPin(KEY2_PIN) == 0)
    {
        delay_ms(KEY_DEBOUNCE_MS);
        if (Key_ReadPin(KEY2_PIN) == 0)
        {
            while (Key_ReadPin(KEY2_PIN) == 0);
            return KEY2_PRESSED;
        }
    }

    if (Key_ReadPin(KEY3_PIN) == 0)
    {
        delay_ms(KEY_DEBOUNCE_MS);
        if (Key_ReadPin(KEY3_PIN) == 0)
        {
            while (Key_ReadPin(KEY3_PIN) == 0);
            return KEY3_PRESSED;
        }
    }

    if (Key_ReadPin(KEY4_PIN) == 0)
    {
        delay_ms(KEY_DEBOUNCE_MS);
        if (Key_ReadPin(KEY4_PIN) == 0)
        {
            while (Key_ReadPin(KEY4_PIN) == 0);
            return KEY4_PRESSED;
        }
    }

    return KEY_NONE;
}
