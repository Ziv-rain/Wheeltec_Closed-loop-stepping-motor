/*
 * H题 车载平衡滚球运动控制系统 — 三按键状态机
 *
 * 按键 (SysTick ISR 每 10ms 扫描):
 *   SW1 (PB0)  = 启动键 (Start)
 *   SW2 (PA2)  = 切题键 (Switch, T2→T3→...→T6→T2)
 *   SW3 (PB16) = 结束键 (Stop, 回到 T2)
 *
 * 状态: TASK_SELECT → TASK_RUNNING → TASK_DONE → TASK_SELECT
 */

#include "ti_msp_dl_config.h"
#include "Board/board.h"
#include "oled.h"
#include "Drive.h"
#include "grayscale_sensor.h"
#include "PID_speed.h"
#include "Odometry.h"
#include "Board/aa55_proto.h"
#include "Board/encoder_logger.h"
#include <stdio.h>
#include <string.h>

/* 引用 OLED 驱动内部的显存，用于直接清零（避免 OLED_Clear 刷屏） */
extern u8 OLED_GRAM[144][8];

/* ======================== 状态定义 ======================== */

typedef enum { TASK_SELECT, TASK_RUNNING, TASK_DONE } state_t;

#define TASK_MIN  2
#define TASK_MAX  6

/* ======================== 末段减速与制动参数 ======================== */

#define FINISH_SLOW_START_COUNTS  1910u  /* 约 50 cm */
#define FINISH_CRAWL_START_COUNTS  764u  /* 约 20 cm */
#define FINISH_IR_WINDOW_COUNTS    573u  /* 终点前约 15 cm */
#define FINISH_MAX_BRAKE_COUNTS    191u  /* 提前制动最多约 5 cm */
#define FINISH_MIN_BRAKE_COUNTS      8u
#define FINISH_CRAWL_SPEED            9u
#define FINISH_FINAL_SPEED            8u
#define FINISH_SPEED_SAMPLE_MS       20u
#define FINISH_CONTROL_DELAY_MS       5u

/* Task4/5/6 only: 2-second quintic S-curve start and stop. */
#define TASK456_RAMP_TIME_MS        2000u
#define TASK456_SCURVE_SCALE       65535u
#define TASK456_STOP_DISTANCE_FACTOR  50u
#define TASK456_MIN_STOP_COUNTS      573u  /* about 15 cm */

/* ======================== 按键事件 ======================== */

#define KEY_NONE    0
#define KEY_START   1
#define KEY_SWITCH  2
#define KEY_STOP    3

/* ======================== 全局变量 ======================== */

static volatile uint32_t g_sysTick = 0;        /* 1ms 系统时钟 */
static volatile uint8_t  g_keyEvent = KEY_NONE; /* 按键事件（ISR 写，主循环读并清除） */

static state_t  g_state       = TASK_SELECT;
static int      g_curTask     = 2;              /* T2 ~ T6 */
static uint32_t g_taskStartMs;                  /* 任务开始时的 g_sysTick */
static uint32_t g_doneStartMs;                  /* 进入 DONE 时的 g_sysTick */
static uint32_t g_lastUpdateMs;                 /* 上次刷新 TIME 的时刻 */

typedef enum {
    FINISH_IDLE,
    FINISH_DRIVING,
    FINISH_COMPLETE
} finish_state_t;

static volatile finish_state_t g_finishState = FINISH_IDLE;
static volatile uint8_t  g_finishStopTriggered;
static volatile uint32_t g_finishStopTick;
static uint32_t g_finishTargetCounts;
static uint32_t g_finishLastCounts;
static uint8_t  g_finishSampleMs;
static uint8_t  g_finishCruiseSpeed;
static uint8_t  g_finishUseSlowdown;
static uint16_t g_finishSpeedCounts20Ms;

typedef enum {
    TASK456_MOTION_IDLE,
    TASK456_MOTION_STARTING,
    TASK456_MOTION_RUNNING,
    TASK456_MOTION_STOPPING
} task456_motion_state_t;

static volatile task456_motion_state_t g_task456MotionState = TASK456_MOTION_IDLE;
static uint16_t g_task456RampMs;
static uint8_t  g_task456CruiseSpeed;
static uint8_t  g_task456StopStartSpeed;

static void FinishController_Stop(uint32_t tick);

static uint32_t Task456_SCurveQ16(uint32_t elapsedMs)
{
    uint32_t x;
    uint32_t x2;
    uint32_t x3;
    uint32_t x4;
    uint32_t x5;
    int64_t y;

    if (elapsedMs >= TASK456_RAMP_TIME_MS) {
        return TASK456_SCURVE_SCALE;
    }

    x = (elapsedMs * TASK456_SCURVE_SCALE) / TASK456_RAMP_TIME_MS;
    x2 = (uint32_t)(((uint64_t)x * x) / TASK456_SCURVE_SCALE);
    x3 = (uint32_t)(((uint64_t)x2 * x) / TASK456_SCURVE_SCALE);
    x4 = (uint32_t)(((uint64_t)x3 * x) / TASK456_SCURVE_SCALE);
    x5 = (uint32_t)(((uint64_t)x4 * x) / TASK456_SCURVE_SCALE);

    /* Quintic smootherstep: 6x^5 - 15x^4 + 10x^3. */
    y = 6LL * x5 - 15LL * x4 + 10LL * x3;
    if (y < 0) {
        y = 0;
    }
    if (y > (int64_t)TASK456_SCURVE_SCALE) {
        y = TASK456_SCURVE_SCALE;
    }
    return (uint32_t)y;
}

static void Task456Motion_Cancel(void)
{
    g_task456MotionState = TASK456_MOTION_IDLE;
    g_task456RampMs = 0u;
}

static void Task456Motion_Start(uint8_t cruiseSpeed)
{
    g_task456CruiseSpeed = cruiseSpeed;
    g_task456StopStartSpeed = 0u;
    g_task456RampMs = 0u;
    g_task456MotionState = TASK456_MOTION_STARTING;
    base_speed = 0u;
}

static void Task456Motion_RequestStop(void)
{
    if (g_task456MotionState == TASK456_MOTION_STOPPING ||
        g_task456MotionState == TASK456_MOTION_IDLE) {
        return;
    }

    g_task456StopStartSpeed = base_speed;
    g_task456RampMs = 0u;
    g_task456MotionState = TASK456_MOTION_STOPPING;
}

static void Task456Motion_Update1ms(uint32_t tick)
{
    uint32_t curve;
    uint32_t speed;

    if (g_task456MotionState == TASK456_MOTION_STARTING) {
        if (g_task456RampMs < TASK456_RAMP_TIME_MS) {
            g_task456RampMs++;
        }
        curve = Task456_SCurveQ16(g_task456RampMs);
        speed = ((uint32_t)g_task456CruiseSpeed * curve +
                 TASK456_SCURVE_SCALE / 2u) / TASK456_SCURVE_SCALE;
        base_speed = (uint8_t)speed;

        if (g_task456RampMs >= TASK456_RAMP_TIME_MS) {
            base_speed = g_task456CruiseSpeed;
            g_task456MotionState = TASK456_MOTION_RUNNING;
        }
    } else if (g_task456MotionState == TASK456_MOTION_STOPPING) {
        if (g_task456RampMs < TASK456_RAMP_TIME_MS) {
            g_task456RampMs++;
        }
        curve = Task456_SCurveQ16(g_task456RampMs);
        speed = ((uint32_t)g_task456StopStartSpeed *
                 (TASK456_SCURVE_SCALE - curve) +
                 TASK456_SCURVE_SCALE / 2u) / TASK456_SCURVE_SCALE;
        base_speed = (uint8_t)speed;

        if (g_task456RampMs >= TASK456_RAMP_TIME_MS) {
            FinishController_Stop(tick);
        }
    }
}

/* ======================== AA55 串口通信 ======================== */

static uint8_t g_aa55TickCnt = 0U;       /* 5ms 分频计数器 (SysTick 1ms 累加) */
static float   g_aa55PosCm   = 0.0f;     /* 当前里程位置 (cm), 供 AA55 上报 */

/* 里程计数值转厘米 (轮径 65mm, 780 counts/rev) */
static float OdometryCountsToCm(uint32_t counts)
{
    /* cm = counts * π * 65mm / (780 * 10) */
    return (float)counts * 3.1415927f * 65.0f / 7800.0f;
}

static uint32_t Task_TargetCounts(void)
{
    if (g_curTask == 2) {
        return ODOMETRY_TASK2_TARGET_AVERAGE_COUNTS;
    }
    if (g_curTask >= 4) {
        return ODOMETRY_TASK456_TARGET_AVERAGE_COUNTS;
    }
    return ODOMETRY_TARGET_AVERAGE_COUNTS;
}

static uint8_t Task_CruiseSpeed(void)
{
    return (g_curTask >= 4) ? 30u : 43u;
}

static void FinishController_Cancel(void)
{
    g_finishState = FINISH_IDLE;
    g_finishStopTriggered = 0u;
    Task456Motion_Cancel();
}

static void FinishController_Start(
    uint32_t targetCounts, uint8_t cruiseSpeed, uint8_t useSlowdown)
{
    Odometry_Reset();
    g_finishTargetCounts = targetCounts;
    g_finishLastCounts = 0u;
    g_finishSpeedCounts20Ms = 0u;
    g_finishSampleMs = 0u;
    g_finishCruiseSpeed = cruiseSpeed;
    g_finishUseSlowdown = useSlowdown;
    g_finishStopTriggered = 0u;
    g_finishStopTick = 0u;
    g_finishState = FINISH_DRIVING;
    base_speed = cruiseSpeed;
}

static void FinishController_Stop(uint32_t tick)
{
    /*
     * Keep following the line right up to the target.  At the target, remove
     * PWM from both motors before changing either direction input, so neither
     * wheel can receive an asymmetric braking or reverse pulse.
     */
    xunji_flag = 0u;
    set_motor_speed(0.0f, 0.0f);
    motor_left_stop();
    motor_right_stop();
    Task456Motion_Cancel();

    g_finishStopTick = tick;
    g_finishStopTriggered = 1u;
    g_finishState = FINISH_COMPLETE;
}

static void FinishController_Update1ms(uint32_t tick)
{
    uint32_t counts;
    uint32_t remaining;
    uint32_t brakeLead;
    uint32_t speed20;
    uint32_t desiredSpeed;

    if (g_finishState != FINISH_DRIVING) {
        return;
    }

    counts = Odometry_GetAverageCounts();

    /* Estimate wheel speed as average encoder counts per 20 ms. */
    if (++g_finishSampleMs >= FINISH_SPEED_SAMPLE_MS) {
        uint32_t delta = counts - g_finishLastCounts;
        g_finishLastCounts = counts;
        g_finishSampleMs = 0u;

        /* 1/4 low-pass update suppresses individual encoder edge jitter. */
        g_finishSpeedCounts20Ms =
            (uint16_t)(((uint32_t)g_finishSpeedCounts20Ms * 3u + delta + 2u) / 4u);
    }

    /*
     * Infrared finish check for every line-following task.  It is armed only
     * in the final 15 cm.  Test this before the odometer limit so the real
     * track marker has priority.
     */
    if (counts < g_finishTargetCounts &&
        (g_finishTargetCounts - counts) <= FINISH_IR_WINDOW_COUNTS) {
        uint8_t middleOn = sensor_data[2] + sensor_data[3] +
                           sensor_data[4] + sensor_data[5];
        if (middleOn >= 3u) {
            if (g_curTask >= 4) {
                Task456Motion_RequestStop();
            } else {
                FinishController_Stop(tick);
            }
            return;
        }
    }

    if (counts >= g_finishTargetCounts) {
        if (g_curTask >= 4) {
            Task456Motion_RequestStop();
        } else {
            FinishController_Stop(tick);
        }
        return;
    }
    remaining = g_finishTargetCounts - counts;

    /* Task4/5/6: cruise at fixed speed; stop is triggered by IR detection only. */
    if (!g_finishUseSlowdown) {
        return;
    }

    /*
     * More than 50 cm: cruise.
     * 50..20 cm: linear deceleration.
     * Last 20 cm: crawl at a stable low speed.
     *
     * Task2 delays its soft-stop entry by 5 cm so the car carries speed
     * further into the finish zone before decelerating.
     */
    {
        uint32_t slowRemaining = remaining;
        if (g_curTask == 2) {
            slowRemaining += 191u;  /* delay all thresholds by ~5 cm */
        }
        if (slowRemaining > FINISH_SLOW_START_COUNTS) {
            desiredSpeed = g_finishCruiseSpeed;
        } else if (slowRemaining > FINISH_CRAWL_START_COUNTS) {
            desiredSpeed = FINISH_CRAWL_SPEED +
                ((uint32_t)(g_finishCruiseSpeed - FINISH_CRAWL_SPEED) *
                 (slowRemaining - FINISH_CRAWL_START_COUNTS)) /
                (FINISH_SLOW_START_COUNTS - FINISH_CRAWL_START_COUNTS);
        } else if (slowRemaining > FINISH_MAX_BRAKE_COUNTS) {
            desiredSpeed = FINISH_CRAWL_SPEED;
        } else {
            desiredSpeed = FINISH_FINAL_SPEED;
        }
    }
    /*
     * Task2: slowdown always controls base_speed.
     * Task4/5/6: slowdown only controls during RUNNING; STARTING and
     * STOPPING are handled by the S-curve motion profile exclusively.
     */
    if (g_curTask < 4 || g_task456MotionState == TASK456_MOTION_RUNNING) {
        base_speed = (uint8_t)desiredSpeed;
    }

    /*
     * Predict stopping distance from measured speed:
     *   distance = speed * control_delay + speed^2 / (2 * deceleration)
     *
     * speed20 is counts/20 ms.  The integer divisor 6 represents the measured
     * starting estimate of braking deceleration and is intentionally bounded
     * to the final 5 cm.  It can be tuned after straight-line brake tests.
     */
    speed20 = g_finishSpeedCounts20Ms;
    brakeLead = (speed20 * FINISH_CONTROL_DELAY_MS + 19u) / 20u;
    brakeLead += (speed20 * speed20 + 3u) / 6u;

    if (brakeLead < FINISH_MIN_BRAKE_COUNTS) {
        brakeLead = FINISH_MIN_BRAKE_COUNTS;
    }
    if (brakeLead > FINISH_MAX_BRAKE_COUNTS) {
        brakeLead = FINISH_MAX_BRAKE_COUNTS;
    }

    /*
     * A higher measured speed enters the final speed earlier, but does not
     * disable tracking.  Actual stopping occurs only at the target count.
     */
    if (remaining <= brakeLead) {
        if (g_curTask >= 4 && g_task456MotionState == TASK456_MOTION_RUNNING) {
            Task456Motion_RequestStop();
        } else {
            base_speed = FINISH_FINAL_SPEED;
        }
    }
}

/* ======================== SysTick ISR (1kHz) ======================== */

void SysTick_Handler(void)
{
    uint32_t tick = ++g_sysTick;

    /* Task4/5/6 motion profile is isolated from Task2 and all communication. */
    Task456Motion_Update1ms(tick);

    /* 里程、减速和停车均在固定 1 ms 中断中完成，不受 OLED 刷新影响。 */
    FinishController_Update1ms(tick);

    /* AA55 协议 5ms 定时处理 (链路检测 + 心跳 + 状态上报) */
    if (++g_aa55TickCnt >= 5U) {
        g_aa55TickCnt = 0U;
        AA55_Tick5ms();
    }

    /* 20 Hz atomic RAM snapshot; UART export happens only after stopping. */
    EncoderLogger_Tick1ms(tick);

    /* 每 10ms 扫描一次按键 */
    if ((tick % 10) == 0)
    {
        static uint8_t  deb_pb0  = 0, conf_pb0  = 0, last_pb0  = 1;
        static uint8_t  deb_pa2  = 0, conf_pa2  = 0, last_pa2  = 1;
        static uint8_t  deb_pb16 = 0, conf_pb16 = 0, last_pb16 = 1;

        uint8_t pb0  = (DL_GPIO_readPins(GPIO_KEY_PINB_0_PORT,  GPIO_KEY_PINB_0_PIN)  == 0) ? 0 : 1;
        uint8_t pa2  = (DL_GPIO_readPins(GPIO_KEY_PINA_2_PORT,  GPIO_KEY_PINA_2_PIN)  == 0) ? 0 : 1;
        uint8_t pb16 = (DL_GPIO_readPins(GPIO_KEY_PINB_16_PORT, GPIO_KEY_PINB_16_PIN) == 0) ? 0 : 1;

        /* ---- PB0 ---- */
        if (!conf_pb0) {
            deb_pb0 = (pb0 == 0) ? deb_pb0 + 1 : 0;
            if (deb_pb0 >= 3) { conf_pb0 = 1; }
        } else {
            if (pb0 == 1 && last_pb0 == 0)
                g_keyEvent = KEY_START;
        }
        last_pb0 = pb0;

        /* ---- PA2 ---- */
        if (!conf_pa2) {
            deb_pa2 = (pa2 == 0) ? deb_pa2 + 1 : 0;
            if (deb_pa2 >= 3) { conf_pa2 = 1; }
        } else {
            if (pa2 == 1 && last_pa2 == 0)
                g_keyEvent = KEY_SWITCH;
        }
        last_pa2 = pa2;

        /* ---- PB16 ---- */
        if (!conf_pb16) {
            deb_pb16 = (pb16 == 0) ? deb_pb16 + 1 : 0;
            if (deb_pb16 >= 3) { conf_pb16 = 1; }
        } else {
            if (pb16 == 1 && last_pb16 == 0)
                g_keyEvent = KEY_STOP;
        }
        last_pb16 = pb16;

        if (pb0  == 1 && conf_pb0)  { conf_pb0  = 0; deb_pb0  = 0; }
        if (pa2  == 1 && conf_pa2)  { conf_pa2  = 0; deb_pa2  = 0; }
        if (pb16 == 1 && conf_pb16) { conf_pb16 = 0; deb_pb16 = 0; }
    }
}

/* ======================== OLED 显示 ======================== */

static void OLED_Show_Select(void)
{
    char s[20];
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    sprintf(s, "H-Task    H-%d", g_curTask);
    OLED_ShowString(0,  0, (u8 *)s,             16, 1);
    OLED_ShowString(0, 20, (u8 *)"SW1: Start",  12, 1);
    OLED_ShowString(0, 34, (u8 *)"SW2: Switch", 12, 1);
    OLED_ShowString(0, 48, (u8 *)"SW3: Reset",  12, 1);
    OLED_Refresh();
}

static void OLED_Update_Running(void)
{
    char s[20];
    uint32_t elapsed = g_sysTick - g_taskStartMs;
    uint16_t sec = elapsed / 1000;
    sprintf(s, "Time: %us", sec);
    OLED_ShowString(0, 20, (u8 *)s, 16, 1);
    OLED_Refresh();
}

static void OLED_Show_Running(void)
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    char s[20];
    sprintf(s, "RUNNING  H-%d", g_curTask);
    OLED_ShowString(0,  0, (u8 *)s, 16, 1);
    OLED_Update_Running();
    OLED_ShowString(0, 50, (u8 *)"SW3: Stop", 12, 1);
    OLED_Refresh();
}

static void OLED_Show_Done(void)
{
    char s[20];
    uint32_t elapsed = g_doneStartMs - g_taskStartMs;
    uint16_t sec = elapsed / 1000;
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    sprintf(s, "H-%d   DONE", g_curTask);
    OLED_ShowString(0,  0, (u8 *)s, 16, 1);
    sprintf(s, "Time: %us", sec);
    OLED_ShowString(0, 20, (u8 *)s, 16, 1);
    OLED_ShowString(0, 50, (u8 *)"SW1:Re SW3:Home", 12, 1);
    OLED_Refresh();
}

/* ======================== 主状态机 ======================== */

static void StateMachine_Run(void)
{
    uint8_t key = g_keyEvent;
    g_keyEvent = KEY_NONE;

    switch (g_state) {

    case TASK_SELECT:
        if (key == KEY_START) {
            g_taskStartMs  = g_sysTick;
            g_lastUpdateMs = g_sysTick;
            base_speed = Task_CruiseSpeed();
            if (g_curTask != 3) {
                FinishController_Start(
                    Task_TargetCounts(), base_speed, (g_curTask == 2) ? 1u : 0u);
                if (g_curTask >= 4) {
                    Task456Motion_Start(base_speed);
                }
            } else {
                FinishController_Cancel();
                Odometry_Reset();
            }
            EncoderLogger_Start((uint8_t)g_curTask, g_sysTick);
            g_state = TASK_RUNNING;
            OLED_Show_Running();
            if (g_curTask != 3) {
                FollowLine_Reset();
                xunji_flag = 1;
            }
        } else if (key == KEY_SWITCH) {
            g_curTask = (g_curTask >= TASK_MAX) ? TASK_MIN : g_curTask + 1;
            OLED_Show_Select();
        } else if (key == KEY_STOP) {
            g_curTask = TASK_MIN;
            OLED_Show_Select();
        }
        break;

    case TASK_RUNNING:
        /* 每秒刷新 TIME */
        if ((g_sysTick - g_lastUpdateMs) >= 1000) {
            g_lastUpdateMs = g_sysTick;
            OLED_Update_Running();
        }
        if (key == KEY_STOP) {
            FinishController_Cancel();
            xunji_flag = 0;
            motor_left_stop();
            motor_right_stop();
            EncoderLogger_Stop(g_sysTick);
            g_doneStartMs = g_sysTick;
            g_state = TASK_DONE;
            OLED_Show_Done();
        } else if (g_curTask != 3 && g_finishStopTriggered) {
            xunji_flag = 0;
            EncoderLogger_Stop(g_finishStopTick);
            g_doneStartMs = g_finishStopTick;
            g_state = TASK_DONE;
            OLED_Show_Done();
        }
        break;

    case TASK_DONE:
        if (key == KEY_START) {
            g_taskStartMs  = g_sysTick;
            g_lastUpdateMs = g_sysTick;
            base_speed = Task_CruiseSpeed();
            if (g_curTask != 3) {
                FinishController_Start(
                    Task_TargetCounts(), base_speed, (g_curTask == 2) ? 1u : 0u);
                if (g_curTask >= 4) {
                    Task456Motion_Start(base_speed);
                }
            } else {
                FinishController_Cancel();
                Odometry_Reset();
            }
            EncoderLogger_Start((uint8_t)g_curTask, g_sysTick);
            g_state = TASK_RUNNING;
            OLED_Show_Running();
            if (g_curTask != 3) {
                FollowLine_Reset();
                xunji_flag = 1;
            }
        } else if (key == KEY_SWITCH) {
            g_curTask = (g_curTask >= TASK_MAX) ? TASK_MIN : g_curTask + 1;
            g_state = TASK_SELECT;
            OLED_Show_Select();
        } else if (key == KEY_STOP) {
            g_curTask = TASK_MIN;
            g_state = TASK_SELECT;
            OLED_Show_Select();
        }
        break;
    }
}

/* ======================== AA55 指令回调 ======================== */

/* 接收主控发来的任务指令 (定义在此处以保证所有被调函数已声明) */
static void AA55_CmdHandler(const AA55_Cmd_t *cmd)
{
    switch (cmd->cmd) {

    case AA55_CMD_SWITCH_TASK:  /* 0x01 切换赛题 */
        if (g_state == TASK_RUNNING) {
            EncoderLogger_Stop(g_sysTick);
        }
        if (cmd->task >= (uint8_t)TASK_MIN && cmd->task <= (uint8_t)TASK_MAX) {
            g_curTask = (int)cmd->task;
            g_state   = TASK_SELECT;
            FinishController_Cancel();
            xunji_flag = 0U;
            motor_left_stop();
            motor_right_stop();
            OLED_Show_Select();
        }
        break;

    case AA55_CMD_START:  /* 0x02 开始执行 */
        if (g_state == TASK_SELECT || g_state == TASK_DONE) {
            g_keyEvent = KEY_START;
        }
        break;

    case AA55_CMD_STOP:  /* 0x03 停止 */
        g_keyEvent = KEY_STOP;
        break;

    case AA55_CMD_SET_TARGET:  /* 0x04 设目标位置 (T6) */
        (void)cmd->param;
        break;

    case AA55_CMD_DUMP_LOG:
        (void)EncoderLogger_RequestDump();
        break;

    default:
        break;
    }
}

/* ======================== 主函数 ======================== */

int main(void)
{
    SYSCFG_DL_init();

    /* 循迹模块初始化 */
    Grayscale_Sensor_Init();
    Pid_Init();
    Odometry_Init();
    EncoderLogger_Init();
    base_speed = 40;
    xunji_flag = 0;
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* SysTick: 1kHz 中断 */
    SysTick->LOAD  = 32000 - 1;
    SysTick->VAL   = 0;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk    |
                     SysTick_CTRL_ENABLE_Msk;

    /* OLED */
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Show_Select();

    /* AA55 串口通信初始化 (UART3: PA25 RX, PA26 TX) */
    AA55_Init();
    AA55_SetCmdCallback(AA55_CmdHandler);

    while (1) {
        /* AA55 协议处理: 消费指令队列 + 发送状态/心跳 */
        AA55_Process();

        /* Incremental post-stop export; never transmits while recording. */
        EncoderLogger_Process();

        /* 更新 AA55 位置信息 (里程 → cm) */
        g_aa55PosCm = OdometryCountsToCm(Odometry_GetAverageCounts());

        StateMachine_Run();

        /* 同步 AA55 任务状态 (供 100ms 周期上报) */
        {
            uint8_t s;
            switch (g_state) {
            case TASK_SELECT:  s = AA55_STATE_IDLE;    break;
            case TASK_RUNNING: s = AA55_STATE_RUNNING; break;
            case TASK_DONE:    s = AA55_STATE_DONE;    break;
            default:           s = AA55_STATE_IDLE;    break;
            }
            AA55_UpdateTaskState((uint8_t)g_curTask, s, g_aa55PosCm, 0.0f);
        }
    }
}
