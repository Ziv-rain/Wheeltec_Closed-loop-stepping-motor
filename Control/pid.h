#ifndef PID_H
#define PID_H
#include <stdint.h>
typedef struct{float kp,ki,kd,integral,prev_error,integral_limit,output_min,output_max;uint8_t initialized;}PID_t;
void PID_Init(PID_t*p,float kp,float ki,float kd,float ilim,float omin,float omax);
void PID_Reset(PID_t*p);
float PID_Update(PID_t*p,float error,float dt);
float PID_UpdateWithDerivative(PID_t*p,float error,float derivative,float dt);
#endif
