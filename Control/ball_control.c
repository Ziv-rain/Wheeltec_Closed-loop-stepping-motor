/**
 * ball_control.c - 平衡球PID控制 + 强制PID模式(V键锁定)
 *
 * 强制PID模式: 发V键启动后, 无论如何不退出PID, 直到收到T键停止。
 * 期间忽略所有异常: 球数据丢失→用最后有效值, PWM丢失→跳过限位,
 * 归零失败→自动重试, CL故障→自动清除, 任务状态变化→忽略。
 */
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
static uint8_t force_pid;     /* 强制PID模式: V键置1, T键清0, 期间绝不退出 */
static float last_ball_cm;    /* 球最后有效位置(cm), 视觉丢失时用此值保持PID */
static float absf(float v){return v<0?-v:v;}
static float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
static uint8_t sane(float p){return p>=PWM_LIMIT_LOW-PWM_LIMIT_MARGIN&&p<=PWM_LIMIT_HIGH+PWM_LIMIT_MARGIN?1:0;}
static void stp(void){if(act){CL_Stop(MOTOR_AXIS_X);act=0;}PID_Reset(&sp);sm_init=0;}
static void hfail(void){Motor_Stop(MOTOR_AXIS_X);CL_Stop(MOTOR_AXIS_X);PID_Reset(&sp);act=0;hc=HC_FAULT;TaskCtrl_ReportFault();}
static void hfin(void){Motor_Stop(MOTOR_AXIS_X);CL_SetZero(MOTOR_AXIS_X);PID_Reset(&sp);act=0;hc=HC_READY;sm_init=0;}
static void hstart(float p){uint8_t d=p<PWM_HORIZONTAL_REF?AXIS_X_POSITIVE_DIR_LEVEL:(uint8_t)!AXIS_X_POSITIVE_DIR_LEVEL;Motor_StartContinuous(HOMING_FREQ_HZ,d);if(!Motor_IsBusy(MOTOR_AXIS_X)){hfail();return;}hc=HC_SEEK;hm=0;pmm=0;npm=0;lherr=absf(p-PWM_HORIZONTAL_REF);}
static void htik(void){float p,e;if(hc==HC_READY||hc==HC_FAULT)return;if(hc==HC_INIT||hc==HC_WAIT){hm+=CL_PERIOD_MS;if(!Encoder_GetPwmAngle(ENCODER_AXIS_X,&p)){if(hm>=HOMING_WAIT_TIMEOUT_MS)hfail();return;}if(!sane(p)){hfail();return;}if(absf(p-PWM_HORIZONTAL_REF)<=PWM_HORIZONTAL_TOL){hfin();}else{hstart(p);}return;}hm+=CL_PERIOD_MS;if(hm>=HOMING_MOVE_TIMEOUT_MS){hfail();return;}if(!Encoder_GetPwmAngle(ENCODER_AXIS_X,&p)){pmm+=CL_PERIOD_MS;if(pmm>=HOMING_PWM_LOSS_TIMEOUT_MS)hfail();return;}pmm=0;if(!sane(p)){hfail();return;}e=absf(p-PWM_HORIZONTAL_REF);if(e<=PWM_HORIZONTAL_TOL){hfin();return;}if(e+0.05f<lherr){lherr=e;npm=0;}else{npm+=CL_PERIOD_MS;if(npm>=HOMING_NO_PROGRESS_MS){hfail();return;}}}

void BallControl_Init(void){PID_Init(&sp,BALL_KP,BALL_KI,BALL_KD,PID_INTEGRAL_LIMIT,-MOTOR_MAX_ANGLE_NEG,MOTOR_MAX_ANGLE_POS);hc=HC_INIT;hm=0;pmm=0;npm=0;lherr=0;act=0;sm_init=0;force_pid=0;last_ball_cm=0;}
uint8_t BallControl_IsHomingReady(void){return hc==HC_READY?1U:0U;}
uint8_t BallControl_HasFault(void){return hc==HC_FAULT?1U:0U;}
void BallControl_EmergencyStop(void){hfail();}

/* V键: 强制启动PID, 如果尚未归零则自动归零后进入PID */
void BallControl_ForceStart(void){force_pid=1;last_ball_cm=0;}
/* T键: 强制停止PID, 电机停转, 退出强制模式 */
void BallControl_ForceStop(void){force_pid=0;stp();last_ball_cm=0;}

void BallControl_Tick5ms(void){
    BallData_t b;TaskInfo_t ti;float bp,out,pw,ca;

    /* ========== 强制PID模式 ==========
     * V键启动后走这条路径, 绕过以下所有退出条件:
     *   - 归零失败 → 自动重置重试(永不永久退出)
     *   - 球数据超时 → 用last_ball_cm保持PID
     *   - PWM角度无效 → 跳过机械限位检查
     *   - PWM硬限位 → 仅钳位输出, 不报故障
     *   - CL层故障 → 自动清除并重试
     *   - 任务状态变化 → 完全忽略
     *   - 链路超时 → 完全忽略
     *   - 自动DONE → 完全忽略
     * 唯一退出方式: T键调用BallControl_ForceStop() */
    if(force_pid){
        htik();
        /* 归零失败时重置状态并重试, 永不永久退出 */
        if(hc==HC_FAULT){
            Motor_Stop(MOTOR_AXIS_X);
            CL_Stop(MOTOR_AXIS_X);
            hc=HC_INIT;
            hm=0;pmm=0;npm=0;lherr=0;
            return;
        }
        /* 归零进行中, 等待完成 */
        if(hc!=HC_READY)return;

        /* 获取球位置: 有效数据则更新缓存, 超时/无效则用缓存值继续PID */
        if(ProtoRx_GetBall(&b)){
            bp=(float)b.position_centi_cm/100.0f;
            last_ball_cm=bp;
        }else{
            bp=last_ball_cm;
        }
        TaskCtrl_ReportBallPos(bp);

        /* 获取setpoint(任务系统), 无任务时默认0cm(球在中心) */
        if(!TaskCtrl_GetInfo(&ti)){ti.setpoint_cm=0;}

        /* PID计算 */
        out=PID_Update(&sp,bp-ti.setpoint_cm,0.005f);

        /* PWM机械限位: 有效时做软限位钳制, 无效时跳过(电机编码器仍然保护) */
        if(Encoder_GetPwmAngle(ENCODER_AXIS_X,&pw)&&sane(pw)){
            ca=CL_GetCurrentAngle(MOTOR_AXIS_X);
            if(pw>=PWM_LIMIT_HIGH-PWM_LIMIT_MARGIN&&out>ca)out=ca;
            if(pw<=PWM_LIMIT_LOW+PWM_LIMIT_MARGIN&&out<ca)out=ca;
        }

        /* 斜坡限制: 安全平滑过渡(初始化时从当前角度起步) */
        ca=CL_GetCurrentAngle(MOTOR_AXIS_X);
        if(!sm_init){smoothed_target=ca;sm_init=1;}
        smoothed_target=clampf(out,smoothed_target-TARGET_SLEW,smoothed_target+TARGET_SLEW);

        /* 发送目标角度: CL故障时自动清除并重试, 绝不停止PID循环 */
        if(CL_SetTargetAngle(MOTOR_AXIS_X,smoothed_target)!=MOTOR_OK){
            CL_ClearFault(MOTOR_AXIS_X);
            CL_SetTargetAngle(MOTOR_AXIS_X,smoothed_target);
        }
        act=1;
        return;
    }

    /* ========== 原始任务模式(非强制PID) ========== */
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
