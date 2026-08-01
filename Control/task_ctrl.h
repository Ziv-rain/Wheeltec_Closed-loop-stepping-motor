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
typedef struct {
    uint16_t left_counts;
    uint16_t right_counts;
    uint32_t source_tick_ms;
    uint32_t frame;
} WheelTelemetry_t;
void TaskCtrl_Init(void);
void TaskCtrl_Tick5ms(void);
void TaskCtrl_Process(void);
void TaskCtrl_FeedByte(uint8_t byte);
void TaskCtrl_ReportBallPos(float cm);
void TaskCtrl_ReportBallInvalid(void);
uint8_t TaskCtrl_GetWheelAccel(float *ax_mps2); /* 车轮加速度: 0=暂无数据 */
uint32_t TaskCtrl_GetWheelFrame(void);           /* 有效编码器帧计数(判断新帧) */
uint8_t TaskCtrl_GetWheelTelemetry(WheelTelemetry_t *telemetry);
uint8_t TaskCtrl_RequestHistoryDump(void);
void TaskCtrl_ReportFault(void);
uint8_t TaskCtrl_GetInfo(TaskInfo_t *info);
#endif
