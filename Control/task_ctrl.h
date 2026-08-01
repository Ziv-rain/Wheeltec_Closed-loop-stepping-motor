#ifndef TASK_CTRL_H
#define TASK_CTRL_H
#include <stdint.h>
#define TASK_IDLE 0
#define TASK_2 2
#define TASK_3 3
#define TASK_4 4
#define TASK_5 5
#define TASK_6 6
#define STATE_IDLE 0
#define STATE_RUNNING 1
#define STATE_DONE 2
#define STATE_FAULT 3
typedef struct{uint8_t task_id;uint8_t state;float setpoint_cm;uint32_t run_time_ms;}TaskInfo_t;
void TaskCtrl_Init(void);
void TaskCtrl_Tick5ms(void);
void TaskCtrl_Process(void);
void TaskCtrl_FeedByte(uint8_t byte);
void TaskCtrl_ReportBallPos(float cm);
void TaskCtrl_ReportBallInvalid(void);
uint8_t TaskCtrl_GetWheelAccel(float *ax_mps2); /* 车轮加速度: 0=暂无数据 */
uint32_t TaskCtrl_GetWheelFrame(void);           /* 有效编码器帧计数(判断新帧) */
void TaskCtrl_ReportFault(void);
uint8_t TaskCtrl_GetInfo(TaskInfo_t *info);
/* 第3题参数设置 (UART0调试命令调用) */
void TaskCtrl_SetT3Target1(float cm);
void TaskCtrl_SetT3Target2(float cm);
void TaskCtrl_SetT3Mid(float cm);
void TaskCtrl_SetT3Ramp(uint32_t ms);
void TaskCtrl_SetT3Tol(float cm);
void TaskCtrl_SetT3Brake(float deg);
/* 第3题一键启动 + 停止 + 参数查询 */
void TaskCtrl_StartTask3(void);
void TaskCtrl_StopTask(void);
float TaskCtrl_GetT3Target1(void);
float TaskCtrl_GetT3Target2(void);
/* UART0赛题控制 (等同UART1 0x40指令) */
void TaskCtrl_SelectTask(uint8_t task); /* 等同 cmd=0x01 */
void TaskCtrl_StartTask(void);          /* 等同 cmd=0x02 */
#endif
