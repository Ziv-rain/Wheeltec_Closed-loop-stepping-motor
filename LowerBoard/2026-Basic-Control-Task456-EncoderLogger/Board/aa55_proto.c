/**
 * aa55_proto.c - AA55 串口通信协议实现 / AA55 Protocol Implementation
 *
 * 严格遵循轮趣 Wheeltec AA55 主控协议:
 *   帧结构: AA 55 TYPE LEN DATA[LEN] CRC_L CRC_H
 *   CRC-16-IBM: 多项式 0xA001, 初始 0x0000, 覆盖 TYPE+LEN+DATA
 *
 * 使用 UART3 (PA25 RX, PA26 TX, 9600-8-N-1)
 * 指令队列深度 8, ISR 安全入队, 主循环出队
 * 链路超时 2.5s 无数据自动进入 FAULT
 */

#include "aa55_proto.h"
#include "ti_msp_dl_config.h"

/* ======================== 常量 ======================== */

#define RX_BUF_MAX       8U    /* AA55 帧载荷最大字节数 */
#define CMD_QUEUE_SIZE   8U    /* 指令队列深度 (2的幂) */
#define CMD_QUEUE_MASK   (CMD_QUEUE_SIZE - 1U)

#define HEARTBEAT_MS     1000U /* 心跳间隔 */
#define STATE_MS         100U  /* 状态上报间隔 */
#define ENCODER_MS        50U  /* 编码器遥测间隔 */
#define LINK_TIMEOUT_MS  2500U /* 链路超时 */

#define TX_PENDING_STATE   (1U << 0)
#define TX_PENDING_HB      (1U << 1)
#define TX_PENDING_ENCODER (1U << 2)

/* ======================== AA55 帧解析器状态机 ======================== */

typedef struct {
    uint8_t  state;          /* 解析状态: 0~6 */
    uint8_t  type;           /* 帧类型 */
    uint8_t  len;            /* 载荷长度 */
    uint8_t  idx;            /* 当前载荷写入位置 */
    uint8_t  data[RX_BUF_MAX]; /* 载荷数据 */
    uint16_t crc;            /* 接收的 CRC (低字节在前) */
} AA55_Rx_t;

/* ======================== 指令队列 ======================== */

typedef struct {
    uint8_t  cmd;
    uint8_t  task;
    int16_t  param;
} CmdEntry_t;

/* ======================== 全局状态 (模块私有) ======================== */

static AA55_Rx_t      g_rx;
static CmdEntry_t     g_cmdQueue[CMD_QUEUE_SIZE];
static volatile uint8_t g_qHead;       /* ISR 写入位置 */
static volatile uint8_t g_qTail;       /* 主循环读取位置 */
static volatile uint8_t g_emergencyStop; /* 紧急停止标志 */
static volatile uint8_t g_overflow;    /* 队列溢出标志 */

static AA55_CmdCallback g_callback;    /* 用户回调 */

static struct {
    uint8_t  task_id;      /* 当前赛题 */
    uint8_t  state;        /* 当前状态 */
    float    pos_cm;       /* 位置 cm */
    float    angle_deg;    /* 角度 deg (保留) */
    uint16_t enc_left;     /* 左轮编码器计数 */
    uint16_t enc_right;    /* 右轮编码器计数 */
    uint32_t enc_tick;     /* 与编码器计数同一时刻锁存的时间戳 */
    uint8_t  link_alive;   /* 链路存活标志 */
    volatile uint8_t pending;  /* 待发送标志位 */
    uint32_t ms_now;       /* 当前时间 ms (由 AA55_Tick5ms 累加) */
    uint32_t ms_last_rx;   /* 上次收到数据的时间 */
    uint32_t ms_last_hb;   /* 上次心跳发送时间 */
    uint32_t ms_last_state;/* 上次状态发送时间 */
    uint32_t ms_last_enc;  /* 上次编码器发送时间 */
} g_ts;

/* ======================== CRC-16-IBM ======================== */
/* 多项式 0xA001, 初始值 0x0000 — 与轮趣协议完全一致 */

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t i;
    crc ^= byte;
    for (i = 0U; i < 8U; i++) {
        if (crc & 1U) {
            crc = (crc >> 1U) ^ 0xA001U;
        } else {
            crc = crc >> 1U;
        }
    }
    return crc;
}

static uint16_t crc16_frame(uint8_t type, uint8_t len, const uint8_t *data)
{
    uint16_t crc = 0U;
    uint8_t i;
    crc = crc16_update(crc, type);
    crc = crc16_update(crc, len);
    for (i = 0U; i < len; i++) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

/* ======================== AA55 帧发送 ======================== */

static void uart3_send_byte(uint8_t byte)
{
    while (DL_UART_Main_isTXFIFOFull(UART_3_INST)) {}
    DL_UART_Main_transmitData(UART_3_INST, byte);
}

/**
 * @brief 发送一帧 AA55 数据
 * @param type   帧类型
 * @param data   载荷数据指针
 * @param len    载荷长度 (≤8)
 *
 * 帧格式: AA 55 TYPE LEN DATA[0..LEN-1] CRC_L CRC_H
 * 最大发送缓冲: 2 + 1 + 1 + 8 + 2 = 14 字节
 */
static void aa55_send_frame(uint8_t type, const uint8_t *data, uint8_t len)
{
    uint16_t crc;
    uint8_t i;

    if (len > RX_BUF_MAX) return;

    /* 帧头 */
    uart3_send_byte(0xAAU);
    uart3_send_byte(0x55U);

    /* TYPE + LEN */
    uart3_send_byte(type);
    uart3_send_byte(len);

    /* DATA */
    for (i = 0U; i < len; i++) {
        uart3_send_byte(data[i]);
    }

    /* CRC (小端, 低字节在前) */
    crc = crc16_frame(type, len, data);
    uart3_send_byte((uint8_t)(crc & 0xFFU));
    uart3_send_byte((uint8_t)((crc >> 8U) & 0xFFU));
}

void AA55_SendFrame(uint8_t type, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len > RX_BUF_MAX) {
        return;
    }
    aa55_send_frame(type, data, len);
}

/* ======================== 单字节 AA55 帧解析器 ======================== */

/**
 * @brief 处理一个字节, 检测完整 AA55 帧
 * @param b      输入字节
 * @param type   输出: 帧类型 (仅返回值=1时有效)
 * @param len    输出: 载荷长度 (仅返回值=1时有效)
 * @param data   输出: 载荷数据指针 (仅返回值=1时有效)
 * @return 1=收到有效帧, 0=继续接收
 *
 * 解析状态机 (与轮趣 task_ctrl.c 完全一致):
 *   0 → 等待 0xAA
 *   1 → 等待 0x55 (或回退到 0/1)
 *   2 → 读 TYPE
 *   3 → 读 LEN (若 LEN>8 则回 0)
 *   4 → 读 DATA[0..LEN-1]
 *   5 → 读 CRC 低字节
 *   6 → 读 CRC 高字节, 校验, 回 0
 */
static uint8_t aa55_parse_byte(AA55_Rx_t *rx, uint8_t b,
                                uint8_t *type, uint8_t *len, uint8_t **data)
{
    uint16_t calc_crc;

    switch (rx->state) {
    case 0:  /* 等待帧头 AA */
        if (b == 0xAAU) {
            rx->state = 1U;
        }
        break;

    case 1:  /* 等待帧头 55 */
        if (b == 0x55U) {
            rx->state = 2U;
        } else {
            /* 非 0x55: 若是 0xAA 则保持在状态1, 否则回状态0 */
            rx->state = (b == 0xAAU) ? 1U : 0U;
        }
        break;

    case 2:  /* TYPE */
        rx->type  = b;
        rx->state = 3U;
        break;

    case 3:  /* LEN */
        rx->len = b;
        rx->idx = 0U;
        if (rx->len > RX_BUF_MAX) {
            rx->state = 0U;  /* 非法长度, 丢弃 */
            break;
        }
        rx->state = (rx->len > 0U) ? 4U : 5U;  /* 无载荷直接跳到 CRC */
        break;

    case 4:  /* DATA */
        rx->data[rx->idx++] = b;
        if (rx->idx >= rx->len) {
            rx->state = 5U;
        }
        break;

    case 5:  /* CRC 低字节 */
        rx->crc = (uint16_t)b;
        rx->state = 6U;
        break;

    default: /* 6: CRC 高字节 + 校验 */
        rx->crc |= ((uint16_t)b << 8U);
        rx->state = 0U;

        /* CRC 校验通过 */
        calc_crc = crc16_frame(rx->type, rx->len, rx->data);
        if (rx->crc == calc_crc) {
            *type = rx->type;
            *len  = rx->len;
            *data = rx->data;
            return 1U;
        }
        break;
    }

    return 0U;
}

/* ======================== 指令队列 (ISR 安全) ======================== */

static uint8_t cmd_queue_pop(CmdEntry_t *entry)
{
    uint32_t primask;
    uint8_t  tail;

    primask = __get_PRIMASK();
    __disable_irq();

    tail = g_qTail;
    if (tail == g_qHead) {
        /* 队列空 */
        if (!primask) __enable_irq();
        return 0U;
    }

    entry->cmd   = g_cmdQueue[tail].cmd;
    entry->task  = g_cmdQueue[tail].task;
    entry->param = g_cmdQueue[tail].param;

    g_qTail = (uint8_t)((tail + 1U) & CMD_QUEUE_MASK);

    if (!primask) __enable_irq();
    return 1U;
}

static void cmd_queue_push(uint8_t cmd, uint8_t task, int16_t param)
{
    uint32_t primask;
    uint8_t  head, next;

    if (cmd < 1U || cmd > AA55_CMD_DUMP_LOG) return;  /* 非法指令码 */

    /* cmd=0x03 (STOP) 是紧急指令: 清空队列并立即生效 */
    if (cmd == AA55_CMD_STOP) {
        g_emergencyStop = 1U;
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    head = g_qHead;
    next = (uint8_t)((head + 1U) & CMD_QUEUE_MASK);

    if (next == g_qTail) {
        /* 队列满: 标记溢出 */
        g_overflow = 1U;
    } else {
        g_cmdQueue[head].cmd   = cmd;
        g_cmdQueue[head].task  = task;
        g_cmdQueue[head].param = param;
        g_qHead = next;
    }

    if (!primask) __enable_irq();
}

/* ======================== 指令执行 ======================== */

static void aa55_exec_cmd(const CmdEntry_t *entry)
{
    /* 通过回调通知上层应用 */
    if (g_callback) {
        AA55_Cmd_t cmd;
        cmd.cmd   = entry->cmd;
        cmd.task  = entry->task;
        cmd.param = entry->param;
        g_callback(&cmd);
    }

    /* 更新内部任务状态 */
    switch (entry->cmd) {
    case AA55_CMD_SWITCH_TASK:  /* 0x01 */
        if (entry->task >= AA55_TASK_2 && entry->task <= AA55_TASK_6) {
            g_ts.task_id = entry->task;
            g_ts.state   = AA55_STATE_IDLE;
            g_ts.pos_cm  = 0.0f;
        }
        g_ts.pending |= TX_PENDING_STATE;
        break;

    case AA55_CMD_START:  /* 0x02 */
        g_ts.state    = AA55_STATE_RUNNING;
        g_ts.pending  |= TX_PENDING_STATE;
        break;

    case AA55_CMD_STOP:  /* 0x03 */
        g_ts.state    = AA55_STATE_IDLE;
        g_ts.pos_cm   = 0.0f;
        g_ts.pending  |= TX_PENDING_STATE;
        break;

    case AA55_CMD_SET_TARGET:  /* 0x04 */
        /* 目标位置由上层应用处理, 这里只记录并通知 */
        g_ts.pending |= TX_PENDING_STATE;
        break;

    default:
        break;
    }
}

static void aa55_process_queue(void)
{
    uint32_t primask;
    uint8_t  es, ov;
    CmdEntry_t entry;

    /* 原子读取紧急标志 */
    primask = __get_PRIMASK();
    __disable_irq();
    es = g_emergencyStop;
    ov = g_overflow;
    g_emergencyStop = 0U;
    g_overflow = 0U;

    /* 紧急停止: 清空队列 */
    if (es || ov) {
        g_qTail = g_qHead;
    }

    if (!primask) __enable_irq();

    /* 处理紧急停止 */
    if (es) {
        entry.cmd   = AA55_CMD_STOP;
        entry.task  = g_ts.task_id;
        entry.param = 0;
        aa55_exec_cmd(&entry);
        return;
    }

    /* 溢出: 上报故障 */
    if (ov) {
        AA55_ReportFault();
        return;
    }

    /* 消费队列中所有指令 */
    while (cmd_queue_pop(&entry)) {
        aa55_exec_cmd(&entry);
    }
}

/* ======================== UART3 中断处理 ======================== */

void UART3_IRQHandler(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_3_INST)) {
        uint8_t  byte;
        uint8_t  type, len;
        uint8_t *data;

        byte = (uint8_t)DL_UART_Main_receiveData(UART_3_INST);

        if (aa55_parse_byte(&g_rx, byte, &type, &len, &data)) {
            /* 有效帧: 更新时间戳并标记链路存活 */
            g_ts.ms_last_rx = g_ts.ms_now;
            g_ts.link_alive = 1U;

            /* 仅处理 TASK_CMD (0x40) 帧 */
            if (type == AA55_TYPE_TASK_CMD && len == 4U) {
                cmd_queue_push(data[0], data[1],
                    (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8U)));
            }
            /* 心跳帧 (0xFF): 标记链路存活即可 */
        }
    }
}

/* ======================== API 实现 ======================== */

void AA55_Init(void)
{
    /* 重置解析器 */
    g_rx.state = 0U;

    /* 重置队列 */
    g_qHead         = 0U;
    g_qTail         = 0U;
    g_emergencyStop = 0U;
    g_overflow      = 0U;

    /* 重置状态 */
    g_ts.task_id      = AA55_TASK_IDLE;
    g_ts.state        = AA55_STATE_IDLE;
    g_ts.pos_cm       = 0.0f;
    g_ts.angle_deg    = 0.0f;
    g_ts.link_alive   = 0U;
    g_ts.pending      = 0U;
    g_ts.ms_now       = 0U;
    g_ts.ms_last_rx   = 0U;
    g_ts.ms_last_hb   = 0U;
    g_ts.ms_last_state = 0U;
    g_ts.ms_last_enc   = 0U;
    g_ts.enc_left      = 0U;
    g_ts.enc_right     = 0U;
    g_ts.enc_tick      = 0U;

    /* 默认无回调 */
    g_callback = NULL;

    /* 清除 UART3 中断并启用接收 */
    NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_3_INST, DL_UART_MAIN_INTERRUPT_RX);
}

/* ---- 编码器遥测帧发送 (TYPE 0x42) ---- */

static void aa55_send_encoder(uint16_t left, uint16_t right, uint32_t tick_ms)
{
    uint8_t d[8];
    d[0] = (uint8_t)(left & 0xFFU);
    d[1] = (uint8_t)((left >> 8U) & 0xFFU);
    d[2] = (uint8_t)(right & 0xFFU);
    d[3] = (uint8_t)((right >> 8U) & 0xFFU);
    d[4] = (uint8_t)(tick_ms & 0xFFU);
    d[5] = (uint8_t)((tick_ms >> 8U) & 0xFFU);
    d[6] = (uint8_t)((tick_ms >> 16U) & 0xFFU);
    d[7] = (uint8_t)((tick_ms >> 24U) & 0xFFU);
    aa55_send_frame(AA55_TYPE_ENCODER, d, 8U);
}

void AA55_UpdateEncoderData(uint16_t left_counts, uint16_t right_counts)
{
    AA55_UpdateEncoderSnapshot(left_counts, right_counts, g_ts.ms_now);
}

void AA55_UpdateEncoderSnapshot(
    uint16_t left_counts, uint16_t right_counts, uint32_t tick_ms)
{
    uint32_t primask;
    primask = __get_PRIMASK();
    __disable_irq();
    g_ts.enc_left  = left_counts;
    g_ts.enc_right = right_counts;
    g_ts.enc_tick  = tick_ms;
    if (!primask) __enable_irq();
}

void AA55_Process(void)
{
    uint32_t primask;
    uint8_t  pending;
    uint8_t  tid, st;
    float    pos, ang;

    /* 1. 消费指令队列 */
    aa55_process_queue();

    /* 2. 原子读取并清除 pending 标志 */
    primask = __get_PRIMASK();
    __disable_irq();
    pending = g_ts.pending;
    g_ts.pending = 0U;
    tid  = g_ts.task_id;
    st   = g_ts.state;
    pos  = g_ts.pos_cm;
    ang  = g_ts.angle_deg;
    if (!primask) __enable_irq();

    /* 3. 发送待发送帧 */
    if (pending & TX_PENDING_STATE) {
        AA55_SendState(tid, st, pos, ang);
    }
    if (pending & TX_PENDING_HB) {
        AA55_SendHeartbeat();
    }
    if (pending & TX_PENDING_ENCODER) {
        uint16_t el, er;
        uint32_t now;
        primask = __get_PRIMASK();
        __disable_irq();
        el  = g_ts.enc_left;
        er  = g_ts.enc_right;
        now = g_ts.enc_tick;
        if (!primask) __enable_irq();
        aa55_send_encoder(el, er, now);
    }
}

void AA55_Tick5ms(void)
{
    /* 时间累计 (5ms 步进) */
    g_ts.ms_now += 5U;

    /* 链路超时检测: 仅在 RUNNING 状态下检测 */
    if (g_ts.state == AA55_STATE_RUNNING &&
        g_ts.link_alive &&
        (g_ts.ms_now - g_ts.ms_last_rx) > LINK_TIMEOUT_MS) {
        g_ts.state    = AA55_STATE_FAULT;
        g_ts.pending  |= TX_PENDING_STATE;
    }

    /* 心跳: 每 1000ms */
    if ((g_ts.ms_now - g_ts.ms_last_hb) >= HEARTBEAT_MS) {
        g_ts.pending      |= TX_PENDING_HB;
        g_ts.ms_last_hb    = g_ts.ms_now;
    }

    /* 状态上报: 每 100ms */
    if ((g_ts.ms_now - g_ts.ms_last_state) >= STATE_MS) {
        g_ts.pending       |= TX_PENDING_STATE;
        g_ts.ms_last_state  = g_ts.ms_now;
    }

    /* 编码器遥测: 每 50ms */
    if ((g_ts.ms_now - g_ts.ms_last_enc) >= ENCODER_MS) {
        g_ts.pending      |= TX_PENDING_ENCODER;
        g_ts.ms_last_enc   = g_ts.ms_now;
    }
}

void AA55_SendState(uint8_t task, uint8_t state, float pos_cm, float angle_deg)
{
    uint8_t d[6];
    float   bc;
    int16_t pos_int16, ang_int16;

    /* 更新全局副本, 供 Process() 中的异步上报使用 */
    g_ts.task_id   = task;
    g_ts.state     = state;
    g_ts.pos_cm    = pos_cm;
    g_ts.angle_deg = angle_deg;

    /* pos_cm → cm*100 int16 */
    bc = pos_cm * 100.0f;
    if (bc > 32767.0f)  bc = 32767.0f;
    if (bc < -32768.0f) bc = -32768.0f;
    pos_int16 = (int16_t)bc;

    /* angle_deg → deg*100 int16 */
    bc = angle_deg * 100.0f;
    if (bc > 32767.0f)  bc = 32767.0f;
    if (bc < -32768.0f) bc = -32768.0f;
    ang_int16 = (int16_t)bc;

    d[0] = task;
    d[1] = state;
    d[2] = (uint8_t)((uint16_t)pos_int16 & 0xFFU);
    d[3] = (uint8_t)(((uint16_t)pos_int16 >> 8U) & 0xFFU);
    d[4] = (uint8_t)((uint16_t)ang_int16 & 0xFFU);
    d[5] = (uint8_t)(((uint16_t)ang_int16 >> 8U) & 0xFFU);

    aa55_send_frame(AA55_TYPE_TASK_STATE, d, 6U);
}

void AA55_SendHeartbeat(void)
{
    uint8_t d = 0x00U;
    aa55_send_frame(AA55_TYPE_HEARTBEAT, &d, 1U);
}

void AA55_SetCmdCallback(AA55_CmdCallback cb)
{
    g_callback = cb;
}

void AA55_ReportFault(void)
{
    if (g_ts.state != AA55_STATE_FAULT) {
        g_ts.state    = AA55_STATE_FAULT;
        g_ts.pending  |= TX_PENDING_STATE;
    }
}

void AA55_UpdateTaskState(uint8_t task, uint8_t state, float pos_cm, float angle_deg)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    g_ts.task_id   = task;
    g_ts.state     = state;
    g_ts.pos_cm    = pos_cm;
    g_ts.angle_deg = angle_deg;

    if (!primask) __enable_irq();
}

uint8_t AA55_IsLinkAlive(void)
{
    return g_ts.link_alive;
}
