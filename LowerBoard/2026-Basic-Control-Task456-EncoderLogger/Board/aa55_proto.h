/**
 * aa55_proto.h - AA55 串口通信协议 / AA55 Serial Protocol
 *
 * 协议帧格式 (与轮趣 Wheeltec 主控协议完全兼容):
 *   AA 55 TYPE LEN DATA[LEN] CRC_L CRC_H
 *   CRC-16-IBM: 多项式 0xA001, 初始 0x0000, 覆盖 TYPE+LEN+DATA
 *
 * === 主控 → 本机 (TYPE 0x40 TASK_CMD, LEN=4) ===
 *   指令格式: AA 55 40 04 [cmd] [task] [paramL] [paramH] CRC_L CRC_H
 *
 *   cmd=0x01 切换赛题     task=2/3/4/5/6
 *   cmd=0x02 开始执行
 *   cmd=0x03 停止
 *   cmd=0x04 设目标位置(T6) param = int16, cm×100
 *
 * === 本机 → 主控 (TYPE 0x41 TASK_STATE, LEN=6) ===
 *   每 100ms 自动上报:
 *   AA 55 41 06 [task] [state] [posL] [posH] [angL] [angH] CRC_L CRC_H
 *
 *   task:  当前赛题 (0=空闲, 2/3/4/5/6)
 *   state: 0=IDLE 1=RUNNING 2=DONE 3=FAULT
 *   pos:   里程位置 × 100, int16 LE (cm)
 *   ang:   保留, int16 LE
 *
 * === 心跳包 (双向, 1Hz) ===
 *   AA 55 FF 01 00 CRC_L CRC_H
 *
 * 使用 UART3 (PA25 RX, PA26 TX), 9600-8-N-1
 */

#ifndef AA55_PROTO_H
#define AA55_PROTO_H

#include <stdint.h>

/* ---- AA55 帧类型 / Frame Types ---- */
#define AA55_TYPE_TASK_CMD   0x40U  /* 主控→本机: 任务指令 */
#define AA55_TYPE_TASK_STATE 0x41U  /* 本机→主控: 任务状态 */
#define AA55_TYPE_ENCODER    0x42U  /* 本机→主控: 编码器遥测 */
#define AA55_TYPE_LOG_META   0x43U  /* 本机→主控: 记录开始/结束 */
#define AA55_TYPE_LOG_TIME   0x44U  /* 本机→主控: run/index/timestamp */
#define AA55_TYPE_LOG_COUNTS 0x45U  /* 本机→主控: 左右轮50ms增量 */
#define AA55_TYPE_LOG_DRIVE  0x46U  /* 本机→主控: PWM/base/循迹位图 */
#define AA55_TYPE_HEARTBEAT  0xFFU  /* 双向: 心跳包 */

/* ---- 任务编号 / Task IDs ---- */
#define AA55_TASK_IDLE  0U
#define AA55_TASK_2     2U
#define AA55_TASK_3     3U
#define AA55_TASK_4     4U
#define AA55_TASK_5     5U
#define AA55_TASK_6     6U

/* ---- 状态 / States ---- */
#define AA55_STATE_IDLE    0U
#define AA55_STATE_RUNNING 1U
#define AA55_STATE_DONE    2U
#define AA55_STATE_FAULT   3U

/* ---- 指令码 (TYPE 0x40 载荷第0字节) / Commands ---- */
#define AA55_CMD_SWITCH_TASK  0x01U
#define AA55_CMD_START        0x02U
#define AA55_CMD_STOP         0x03U
#define AA55_CMD_SET_TARGET   0x04U
#define AA55_CMD_DUMP_LOG     0x05U

/* ---- 接收到的指令结构 / Received command struct ---- */
typedef struct {
    uint8_t  cmd;       /* 指令码 */
    uint8_t  task;      /* 赛题编号 */
    int16_t  param;     /* 参数 (设目标位置时为 cm*100) */
} AA55_Cmd_t;

/* ---- 指令回调类型 / Command callback type ---- */
typedef void (*AA55_CmdCallback)(const AA55_Cmd_t *cmd);

/* ============ API 函数 ============ */

/**
 * @brief 初始化 AA55 协议模块
 *        使能 UART3 接收中断，初始化内部状态
 */
void AA55_Init(void);

/**
 * @brief 主循环处理 (在 while(1) 中持续调用)
 *        消费指令队列、发送周期性状态和心跳
 */
void AA55_Process(void);

/**
 * @brief 5ms 定时回调 (在定时器 ISR 或主循环中按 5ms 调用)
 *        用于链路超时检测和 100ms 状态上报的计时
 */
void AA55_Tick5ms(void);

/**
 * @brief 发送任务状态帧 (TYPE 0x41)
 * @param task     当前赛题 (0=空闲, 2/3/4/5/6)
 * @param state    当前状态 (0=IDLE, 1=RUNNING, 2=DONE, 3=FAULT)
 * @param pos_cm   里程位置 (cm, float)
 * @param angle_deg 保留字段 (deg, float)
 */
void AA55_SendState(uint8_t task, uint8_t state, float pos_cm, float angle_deg);

/**
 * @brief 发送心跳帧 (TYPE 0xFF)
 */
void AA55_SendHeartbeat(void);

/**
 * @brief 注册指令回调函数
 *        收到有效 TASK_CMD 帧时，在 AA55_Process() 中回调
 * @param cb  回调函数指针, 传 NULL 取消回调
 */
void AA55_SetCmdCallback(AA55_CmdCallback cb);

/**
 * @brief 上报故障 (发送 STATE_FAULT 状态)
 */
void AA55_ReportFault(void);

/**
 * @brief 更新任务状态 (仅设置内部变量, 不立即发送)
 *        由上层状态机在每次状态变化后调用
 *        实际的帧发送由 AA55_Tick5ms 每 100ms 自动完成
 * @param task     当前赛题 (0=空闲, 2/3/4/5/6)
 * @param state    当前状态 (0=IDLE, 1=RUNNING, 2=DONE, 3=FAULT)
 * @param pos_cm   里程位置 (cm)
 * @param angle_deg 保留字段
 */
void AA55_UpdateTaskState(uint8_t task, uint8_t state, float pos_cm, float angle_deg);

/**
 * @brief 更新编码器遥测数据 (仅存储, 不立即发送)
 *        实际的帧发送由 AA55_Tick5ms 每 50ms 自动完成
 * @param left_counts   左轮编码器计数
 * @param right_counts  右轮编码器计数
 */
void AA55_UpdateEncoderData(uint16_t left_counts, uint16_t right_counts);
void AA55_UpdateEncoderSnapshot(
    uint16_t left_counts, uint16_t right_counts, uint32_t tick_ms);

/* Main-loop-only raw frame sender used by the stopped-car log exporter. */
void AA55_SendFrame(uint8_t type, const uint8_t *data, uint8_t len);

/**
 * @brief 获取链路状态 (最近是否收到过主控数据)
 * @return 1=链路正常, 0=链路超时
 */
uint8_t AA55_IsLinkAlive(void);

/* ============ UART3 中断处理 (需在中断向量中调用) ============ */
void UART3_IRQHandler(void);

#endif /* AA55_PROTO_H */
