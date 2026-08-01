#ifndef TASK_CTRL_COMPILE_STUBS_H
#define TASK_CTRL_COMPILE_STUBS_H

#include <stdint.h>

#define ti_msp_dl_config_h
#define _BOARD_H_
#define UART_1_INST 1U
#define DL_UART_MAIN_INTERRUPT_RX 1U

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

void uart_puts(const char *text);
void uart_putf(float value, uint8_t decimals);
void uart_putu(uint32_t value);
void uart_puti(int32_t value);
void uart_putc(char value);

#endif /* TASK_CTRL_COMPILE_STUBS_H */
