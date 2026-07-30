/**
 * board.c - �弶�������� / Board-level utilities
 * SysTick��ʱ������printf�ض����
 * Delay functions, printf retarget to UART
 */
#include "ti_msp_dl_config.h"
#include "board.h"

volatile unsigned long tick_ms;
volatile uint32_t start_time;

/* 天猛星板载 LED 初始化: PB22, 推挽输出, 初始熄灭 */
void Board_LED_Init(void)
{
    DL_GPIO_initDigitalOutput(IOMUX_PINCM50);
    DL_GPIO_enableOutput(LED_PORT, LED_PIN);
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
}

void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

// ��ȡSysTick��ǰ����ֵ / Read current SysTick count
uint32_t Systick_getTick(void)
{
	return (SysTick->VAL);
}


// ms��������ʱ / Blocking delay in ms
void delay_ms(uint32_t ms)
{
	// ����������ʱ��Χ��ض� / Clamp to max possible delay
	//if( ms > SysTickMAX_COUNT/(SysTickFre/1000) ) ms = SysTickMAX_COUNT/(SysTickFre/1000);
	for(int i=0;i<1000;i++)
	{
		delay_us(ms);
	}
}


// us��������ʱ / Blocking delay in us
void delay_us(uint32_t us)
{
	// �ضϳ���Χֵ / Clamp if exceeds max
	if( us > SysTickMAX_COUNT/(SysTickFre/1000000) ) us = SysTickMAX_COUNT/(SysTickFre/1000000);

	us = us*(SysTickFre/1000000); // ��λת��Ϊ����ֵ / Convert to tick count

	// ��¼�Ѿ��߹���ʱ�� / Accumulated elapsed ticks
	uint32_t runningtime = 0;

	// ��¼��ʼʱ�̵ļ���ֵ / Capture starting tick
	uint32_t InserTick = Systick_getTick();

	// ��ѯ��ʵʱˢ�� / Live tick during polling
	uint32_t tick = 0;

	uint8_t countflag = 0;
	// �ȴ���ʱ���� / Wait for delay to expire
	while(1)
	{
		tick = Systick_getTick();// ˢ�µ�ǰ����ֵ / Refresh current tick

		// �������緭ת���л����㷽ʽ / Handle wrap-around
		if( tick > InserTick ) countflag = 1;

		if( countflag ) runningtime = InserTick + SysTickMAX_COUNT - tick;
		else runningtime = InserTick - tick;

		if( runningtime>=us ) break;
	}

}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

#if !defined(__MICROLIB)
// δʹ��΢��ʱ��Ҫ�ֶ���ȫ�ײ㺯�� / Needed when not using microlib
#if (__ARMCLIB_VERSION <= 6000000)
// AC5��������Ҫ����FILE�ṹ�� / AC5 compiler needs FILE struct
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

// ���ð�����ģʽ / Disable semihosting
void _sys_exit(int x)
{
	(void)x;
}
#endif

// printf�ض��򵽴���0 / Redirect printf to UART0
int fputc(int ch, FILE *stream)
{
	// æ��ֱ�����ڿ����ٷ��� / Wait until UART is ready
	while( DL_UART_isBusy(UART_0_INST) == true );

	DL_UART_Main_transmitData(UART_0_INST, ch);

	return ch;
}

/* ---- Simple UART output functions (no heap, no printf dependency) ---- */
void uart_putc(char c)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)c);
}

void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

void uart_putu(uint32_t n)
{
    char buf[12];
    int i = 0;
    if (n == 0) { uart_putc('0'); return; }
    while (n > 0) { buf[i++] = '0' + (n % 10); n /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

void uart_puti(int32_t n)
{
    if (n < 0) { uart_putc('-'); n = -n; }
    uart_putu((uint32_t)n);
}

void uart_putf(float f, int decimals)
{
    int32_t ipart;
    uint32_t fpart;
    int i;
    if (f < 0.0f) { uart_putc('-'); f = -f; }
    ipart = (int32_t)f;
    uart_puti(ipart);
    if (decimals > 0) {
        uart_putc('.');
        f -= (float)ipart;
        for (i = 0; i < decimals; i++) f *= 10.0f;
        fpart = (uint32_t)(f + 0.5f);
        { uint32_t div = 1; for (i = 1; i < decimals; i++) div *= 10;
          for (i = 0; i < decimals - 1; i++) { if (fpart < div) uart_putc('0'); div /= 10; } }
        uart_putu(fpart);
    }
}
/* fputs removed - libc.a provides it (calls our fputc internally) */
#if 0
/*
 * TI���п��printf����fputc�����ͨ�ַ���%s��
 * ��fputs����Ѿ�ת����ɵ��������������͸������ַ�����
 * �����ӿڱ����ض���ͬһ��UART�����������������������ֶ�ʧ��
 */
int fputs(const char *text, FILE *stream)
{
    while (*text != '\0') {
        if (fputc((unsigned char)*text++, stream) == EOF) return EOF;
    }
    return 0;
}
#endif
