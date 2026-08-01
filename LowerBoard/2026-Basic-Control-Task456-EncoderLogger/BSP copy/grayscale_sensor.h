#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "delay.h"

#define SENSOR_AD0_PORT         GPIO_trace_PINA_18_PORT
#define SENSOR_AD0_PIN          GPIO_trace_PINA_18_PIN

#define SENSOR_AD1_PORT         GPIO_trace_PINA_24_PORT
#define SENSOR_AD1_PIN          GPIO_trace_PINA_24_PIN

#define SENSOR_AD2_PORT         GPIO_trace_PINA_15_PORT
#define SENSOR_AD2_PIN          GPIO_trace_PINA_15_PIN

#define GrayS_OUT_PORT          GPIO_trace_PINB_18_PORT
#define GrayS_OUT_PIN           GPIO_trace_PINB_18_PIN

#define GRAYSCALE_PIN_WRITE(port, pin, state) do { \
    if(state) DL_GPIO_setPins(port, pin); \
    else DL_GPIO_clearPins(port, pin); \
} while(0)

#define SENSOR_AD0_WRITE(state)  GRAYSCALE_PIN_WRITE(GPIO_trace_PINA_18_PORT, GPIO_trace_PINA_18_PIN, state)
#define SENSOR_AD1_WRITE(state)  GRAYSCALE_PIN_WRITE(GPIO_trace_PINA_24_PORT, GPIO_trace_PINA_24_PIN, state)
#define SENSOR_AD2_WRITE(state)  GRAYSCALE_PIN_WRITE(GPIO_trace_PINA_15_PORT, GPIO_trace_PINA_15_PIN, state)

#define SENSOR_OUT_READ()        (!!(DL_GPIO_readPins(GPIO_trace_PINB_18_PORT, GPIO_trace_PINB_18_PIN)))

#define GRAYSCALE_SENSOR_CHANNELS   8

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(void);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);
void Sensor_Logic_Control(void);

#define BASE_SPEED 1600
#define TURN_SPEED 800
#define STOP_SPEED 0
extern uint8_t sensor_data[GRAYSCALE_SENSOR_CHANNELS];
#endif // __GRAYSCALE_SENSOR_H
