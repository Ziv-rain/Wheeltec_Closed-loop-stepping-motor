#ifndef _PIDSPEED_H
#define _PIDSPEED_H
#include "Drive.h"
#include "grayscale_sensor.h"
#include "Define.h"

/* ============ Turn parameters ============ */
#define TRACK_STEER_GAIN       1.50f
#define SENSOR_FILTER_ALPHA    0.55f
#define SENSOR_DEADBAND        0.30f
#define MIN_CRUISE_RATIO       0.55f
#define LINE_LOST_CONFIRM      14u
#define LOST_SPEED_RATIO       0.45f
#define LOST_STEER_RATIO       0.35f

/* 1 ms actuator rates, in PWM percentage points per update. */
#define SPEED_ACCEL_STEP       0.06f
#define SPEED_DECEL_STEP       0.12f
#define STEER_SLEW_STEP        0.25f

struct tPid
{
    double Kp;
    double Ki;
    double Kd;
    double target_val;
    double actual_val;
    double err;
    double err_last;
    double err_sum;
    double output;
};

void Pid_Init(void);
void FollowLine_Reset(void);
void Motor_Smooth_Update(void);
void PID_caculate (struct tPid * pid,double actual_val,double target_val);
void I_limit (struct tPid * pid, double low, double high);
float get_sensor_actual(void);
void xunji_pid(void);
void TIMER_0_INST_IRQHandler(void);
extern float None_flag;
extern struct tPid EncoderLPid;
extern struct tPid EncoderRPid;
extern struct tPid GxPid;
extern struct tPid TurnErrorPid;
extern struct tPid jiaoduPid;
extern float Pre;
static float last_valid_bias = 0;
uint8_t target_yaw,start_flag,keyquan,biansu_flag,yizhi_flag;
uint8_t keynum,keycnt,clear_flag,work_1,work_flag,xunji_flag,kaishi_flag,quanshu,far_flag,m0,xia_flag,baohu_flag;
uint8_t base_speed;
extern int Left,Right,count;

extern uint8_t g_in_pivot;
#endif
