#include "grayscale_sensor.h"
#include "Drive.h"

static void _delay_us(volatile uint32_t us)
{
    delay_us(us);
}

static void _select_channel(uint8_t channel)
{
    SENSOR_AD0_WRITE((channel >> 0) & 0x01);
    SENSOR_AD1_WRITE((channel >> 1) & 0x01);
    SENSOR_AD2_WRITE((channel >> 2) & 0x01);
}

static uint16_t Read_OUT_value(void)
{
    return SENSOR_OUT_READ();
}

void Grayscale_Sensor_Init(void)
{
    for (int i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        sensor_data[i] = 0;
    }
}

void Sensor_Logic_Control(void)
{
    Grayscale_Sensor_Read_All();
}

void Grayscale_Sensor_Read_All(void)
{
    uint8_t a;
    for (a=0; a<8; a++)
    {
        _select_channel(a);
        _delay_us(50);
        sensor_data[a] = Read_OUT_value();
    }
}

uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
    if (channel >= GRAYSCALE_SENSOR_CHANNELS)
    {
        return 0;
    }
    _select_channel(channel);
    _delay_us(50);
    return Read_OUT_value();
}
