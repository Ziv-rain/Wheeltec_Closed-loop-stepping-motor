#include "ball_control.h"
#include "proto_rx.h"
#include "pid.h"
#include "closed_loop.h"
#include "demo_config.h"
#include "encoder.h"
#include "motor.h"
#include "task_ctrl.h"

/* 目标角度斜坡限制: 每5ms最多变化 TARGET_SLEW 度, 让电机匀速平滑过渡 */
#define TARGET_SLEW 1.5f

typedef enum{HC_INIT=0,HC_WAIT,HC_SEEK,HC_READY,HC_FAULT}HC_t;
static PID_t sp;static HC_t hc;static uint32_t hm,pmm,npm;static float lherr;static uint8_t act;
static float smoothed_target=0.0f;static uint8_t sm_init=0;
static float absf(float v){return v<0?-v:v;}
static float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
static uint8_t sane(float p){return p>=PWM_LIMIT_LOW-PWM_LIMIT_MARGIN&&p<=PWM_LIMIT_HIGH+PWM_LIMIT_MARGIN?1:0;}
static void stp(void){if(act){CL_Stop(MOTOR_AXIS_X);act=0;}PID_Reset(&sp);sm_init=0;}
static void hfail(void){Motor_Stop(MOTOR_AXIS_X);CL_Stop(MOTOR_AXIS_X);PID_Reset(&sp);act=0;hc=HC_FAULT;TaskCtrl_ReportFault();}
static void hfin(void){Motor_Stop(MOTOR_AXIS_X);CL_SetZero(MOTOR_AXIS_X);PID_Reset(&sp);act=0;hc=HC_READY;sm_init=0;}
static void hstart(float p){uint8_t d=p<PWM_HORIZONTAL_REF?AXIS_X_POSITIVE_DIR_LEVEL:(uint8_t)!AXIS_X_POSITIVE_DIR_LEVEL;Motor_StartContinuous(HOMING_FREQ_HZ,d);if(!Motor_IsBusy(MOTOR_AXIS_X)){hfail();return;}hc=HC_SEEK;hm=0;pmm=0;npm=0;lherr=absf(p-PWM_HORIZONTAL_REF);}
static void htik(void){float p,e;if(hc==HC_READY||hc==HC_FAULT)return;if(hc==HC_INIT||hc==HC_WAIT){hm+=CL_PERIOD_MS;if(!Encoder_GetPwmAngle(ENCODER_AXIS_X,&p)){if(hm>=HOMING_WAIT_TIMEOUT_MS)hfail();return;}if(!sane(p)){hfail();return;}if(absf(p-PWM_HORIZONTAL_REF)<=PWM_HORIZONTAL_TOL){hfin();}else{hstart(p);}return;}hm+=CL_PERIOD_MS;if(hm>=HOMING_MOVE_TIMEOUT_MS){hfail();return;}if(!Encoder_GetPwmAngle(ENCODER_AXIS_X,&p)){pmm+=CL_PERIOD_MS;if(pmm>=HOMING_PWM_LOSS_TIMEOUT_MS)hfail();return;}pmm=0;if(!sane(p)){hfail();return;}e=absf(p-PWM_HORIZONTAL_REF);if(e<=PWM_HORIZONTAL_TOL){hfin();return;}if(e+0.05f<lherr){lherr=e;npm=0;}else{npm+=CL_PERIOD_MS;if(npm>=HOMING_NO_PROGRESS_MS){hfail();return;}}}

void BallControl_Init(void){PID_Init(&sp,BALL_KP,BALL_KI,BALL_KD,PID_INTEGRAL_LIMIT,-MOTOR_MAX_ANGLE_NEG,MOTOR_MAX_ANGLE_POS);hc=HC_INIT;hm=0;pmm=0;npm=0;lherr=0;act=0;sm_init=0;}
void BallControl_Tick5ms(void){
    BallData_t b;TaskInfo_t ti;float bp,out,pw,ca;
    htik();
    if(hc==HC_FAULT){TaskCtrl_ReportFault();return;}
    if(hc!=HC_READY)return;
    TaskCtrl_GetInfo(&ti);
    if(ti.state!=STATE_RUNNING){stp();return;}
    if(!ProtoRx_GetBall(&b)){TaskCtrl_ReportBallInvalid();stp();return;}
    if(!Encoder_GetPwmAngle(ENCODER_AXIS_X,&pw)||!sane(pw)){stp();TaskCtrl_ReportFault();return;}
    bp=(float)b.position_centi_cm/100.0f;
    TaskCtrl_ReportBallPos(bp);
    out=PID_Update(&sp,bp-ti.setpoint_cm,0.005f);
    ca=CL_GetCurrentAngle(MOTOR_AXIS_X);
    if(pw>=PWM_LIMIT_HIGH-PWM_LIMIT_MARGIN&&out>ca)out=ca;
    if(pw<=PWM_LIMIT_LOW+PWM_LIMIT_MARGIN&&out<ca)out=ca;
    /* 斜坡限制: 第一次初始化为当前角度, 之后每周期最多变化 TARGET_SLEW 度 */
    if(!sm_init){smoothed_target=ca;sm_init=1;}
    smoothed_target=clampf(out,smoothed_target-TARGET_SLEW,smoothed_target+TARGET_SLEW);
    if(CL_SetTargetAngle(MOTOR_AXIS_X,smoothed_target)==MOTOR_OK)act=1;
    else{stp();TaskCtrl_ReportFault();}
}
