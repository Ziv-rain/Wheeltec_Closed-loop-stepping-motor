#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdint.h>

/*
 * MG513P30 + 65 mm wheel
 * Encoder: 13 Hall pulses / motor revolution
 * Gearbox: 1:30
 * Counting: A phase rising and falling edges (x2)
 */
#define ODOMETRY_WHEEL_DIAMETER_MM       65u
#define ODOMETRY_ENCODER_PPR             13u
#define ODOMETRY_GEAR_RATIO              30u
#define ODOMETRY_EDGE_MULTIPLIER          2u
#define ODOMETRY_COUNTS_PER_WHEEL_REV   780u

/*
 * Track length = (3 + pi) m.
 * Target = (3 + pi) / (pi * 0.065) * 780 = 23459.16 counts.
 * Task 2 length = full length - 0.31 m: 22275.04 counts.
 * Task 4/5/6 length = full length - 0.39 m: 21969.46 counts.
 * The comparison uses the mean travel count of the two wheels.
 */
#define ODOMETRY_TARGET_AVERAGE_COUNTS 23841u           /* +10 cm */
#define ODOMETRY_TASK2_TARGET_AVERAGE_COUNTS 22657u      /* +10 cm */
#define ODOMETRY_TASK456_TARGET_AVERAGE_COUNTS 22924u    /* +10 cm, prev.+15 cm */

void Odometry_Init(void);
void Odometry_Reset(void);

uint32_t Odometry_GetLeftCounts(void);
uint32_t Odometry_GetRightCounts(void);
uint32_t Odometry_GetAverageCounts(void);
void Odometry_GetSnapshot(uint32_t *leftCounts, uint32_t *rightCounts);
uint8_t Odometry_TargetReached(uint32_t targetAverageCounts);

#endif /* ODOMETRY_H_ */
