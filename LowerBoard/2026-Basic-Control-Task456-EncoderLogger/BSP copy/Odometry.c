#include "Odometry.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_leftTravelCounts;
static volatile uint32_t g_rightTravelCounts;

/*
 * PB22: left encoder A phase, PB23: left encoder B phase
 * PA12: right encoder A phase, PA13: right encoder B phase
 *
 * A-phase interrupts are configured on both edges. Each valid interrupt is
 * one travelled count. Counting travelled pulses (instead of signed position)
 * makes the finishing distance independent of the two motors' mounting
 * direction.
 */
void GROUP1_IRQHandler(void)
{
    uint32_t leftStatus;
    uint32_t rightStatus;

    leftStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIO_H_PINB_22_PORT, GPIO_H_PINB_22_PIN);
    rightStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIO_H_PINA_12_PORT, GPIO_H_PINA_12_PIN);

    if ((leftStatus & GPIO_H_PINB_22_PIN) != 0u) {
        g_leftTravelCounts++;
        DL_GPIO_clearInterruptStatus(
            GPIO_H_PINB_22_PORT, GPIO_H_PINB_22_PIN);
    }

    if ((rightStatus & GPIO_H_PINA_12_PIN) != 0u) {
        g_rightTravelCounts++;
        DL_GPIO_clearInterruptStatus(
            GPIO_H_PINA_12_PORT, GPIO_H_PINA_12_PIN);
    }
}

void Odometry_Init(void)
{
    Odometry_Reset();

    DL_GPIO_clearInterruptStatus(
        GPIO_H_PINB_22_PORT, GPIO_H_PINB_22_PIN);
    DL_GPIO_clearInterruptStatus(
        GPIO_H_PINA_12_PORT, GPIO_H_PINA_12_PIN);
    NVIC_EnableIRQ(GPIO_H_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_H_GPIOB_INT_IRQN);
}

void Odometry_Reset(void)
{
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    g_leftTravelCounts = 0u;
    g_rightTravelCounts = 0u;
    if (interruptState == 0u) {
        __enable_irq();
    }
}

uint32_t Odometry_GetLeftCounts(void)
{
    return g_leftTravelCounts;
}

uint32_t Odometry_GetRightCounts(void)
{
    return g_rightTravelCounts;
}

uint32_t Odometry_GetAverageCounts(void)
{
    uint32_t left = g_leftTravelCounts;
    uint32_t right = g_rightTravelCounts;

    return (uint32_t)(((uint64_t)left + (uint64_t)right) / 2u);
}

void Odometry_GetSnapshot(uint32_t *leftCounts, uint32_t *rightCounts)
{
    uint32_t interruptState;

    if (leftCounts == 0 || rightCounts == 0) {
        return;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    *leftCounts = g_leftTravelCounts;
    *rightCounts = g_rightTravelCounts;
    if (interruptState == 0u) {
        __enable_irq();
    }
}

uint8_t Odometry_TargetReached(uint32_t targetAverageCounts)
{
    uint32_t left = g_leftTravelCounts;
    uint32_t right = g_rightTravelCounts;

    return (((uint64_t)left + (uint64_t)right) >=
            ((uint64_t)targetAverageCounts * 2u)) ? 1u : 0u;
}
