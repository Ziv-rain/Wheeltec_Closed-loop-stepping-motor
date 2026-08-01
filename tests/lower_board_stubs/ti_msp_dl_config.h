#ifndef TEST_TI_MSP_DL_CONFIG_H
#define TEST_TI_MSP_DL_CONFIG_H

#include <stdint.h>
#include <stddef.h>

#define UART_3_INST 3U
#define UART_3_INST_INT_IRQN 3U
#define DL_UART_MAIN_INTERRUPT_RX 1U
#define GPIO_Drive_A_PORT 1U
#define GPIO_Drive_B_PORT 2U
#define GPIO_Drive_A_PINA_16_PIN (1U << 16)
#define GPIO_Drive_A_PINA_17_PIN (1U << 17)
#define GPIO_Drive_B_PINB_1_PIN  (1U << 1)
#define GPIO_Drive_B_PINB_4_PIN  (1U << 4)
#define PWM_A_INST 1U
#define PWM_B_INST 2U
#define GPIO_PWM_A_C0_IDX 0U
#define GPIO_PWM_B_C1_IDX 1U
#define GPIO_H_PINB_22_PORT 1U
#define GPIO_H_PINB_22_PIN (1U << 22)
#define GPIO_H_PINA_12_PORT 2U
#define GPIO_H_PINA_12_PIN (1U << 12)
#define GPIO_H_GPIOA_INT_IRQN 1U
#define GPIO_H_GPIOB_INT_IRQN 2U
#define GPIO_KEY_PINB_0_PORT 3U
#define GPIO_KEY_PINB_0_PIN (1U << 0)
#define GPIO_KEY_PINA_2_PORT 4U
#define GPIO_KEY_PINA_2_PIN (1U << 2)
#define GPIO_KEY_PINB_16_PORT 5U
#define GPIO_KEY_PINB_16_PIN (1U << 16)
#define TIMER_0_INST_INT_IRQN 4U
#define SysTick_CTRL_CLKSOURCE_Msk (1U << 2)
#define SysTick_CTRL_TICKINT_Msk (1U << 1)
#define SysTick_CTRL_ENABLE_Msk (1U << 0)

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
} TestSysTick_t;
extern TestSysTick_t *SysTick;

static inline uint32_t test_get_primask(void) { return 0U; }
static inline void test_disable_irq(void) {}
static inline void test_enable_irq(void) {}
#define __get_PRIMASK() test_get_primask()
#define __disable_irq() test_disable_irq()
#define __enable_irq() test_enable_irq()

uint8_t DL_UART_Main_isTXFIFOFull(uint32_t instance);
uint8_t DL_UART_Main_isRXFIFOEmpty(uint32_t instance);
uint32_t DL_UART_Main_receiveData(uint32_t instance);
void DL_UART_Main_transmitData(uint32_t instance, uint8_t data);
void DL_UART_Main_enableInterrupt(uint32_t instance, uint32_t mask);
void DL_GPIO_setPins(uint32_t port, uint32_t pins);
void DL_GPIO_clearPins(uint32_t port, uint32_t pins);
uint32_t DL_GPIO_getEnabledInterruptStatus(uint32_t port, uint32_t pins);
void DL_GPIO_clearInterruptStatus(uint32_t port, uint32_t pins);
uint32_t DL_GPIO_readPins(uint32_t port, uint32_t pins);
void DL_Timer_setCaptureCompareValue(uint32_t timer, uint32_t value, uint32_t index);
void NVIC_EnableIRQ(uint32_t interrupt);
void NVIC_ClearPendingIRQ(uint32_t interrupt);
void SYSCFG_DL_init(void);

#endif /* TEST_TI_MSP_DL_CONFIG_H */
