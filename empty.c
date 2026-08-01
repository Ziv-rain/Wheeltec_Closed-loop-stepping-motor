/**
 * ============================================================================
 * 轮趣 MS42CG + D36A 闭环步进电机 — 天猛星 MSPM0G3507
 * ============================================================================
 *
 * 接线指南 (引脚 100% 兼容天猛星, 无需修改):
 *
 *   [天猛星]                          [D36A 驱动器]
 *   PA24 (A0_3)  ──────────────────→  ST1   (STEP 脉冲)
 *   PA13         ──────────────────→  DIR1  (方向)
 *   PA12         ──────────────────→  EN1   (使能, 高电平锁轴)
 *   GND          ──────────────────→  GND   (⚠️ 必须共地)
 *
 *   [天猛星]                          [MS42CG 编码器]  6芯排线顺序:
 *   3.3V         ──────────────────→  VCC   (① ⚠️ 必须3.3V, 接5V烧MCU!)
 *   PA1          ──────────────────→  A     (② 编码器A相)
 *   PA0          ──────────────────→  B     (③ 编码器B相)
 *   PB20 (A0_6)  ──────────────────→  PWM   (④ 绝对角度)
 *   PA25 (A0_2)  ──────────────────→  Z     (⑤ 索引信号)
 *   GND          ──────────────────→  GND   (⑥)
 *
 *   [D36A]                           [MS42CG 电机]
 *   A+/A-  ────────────────────────→  电机 A+ A-
 *   B+/B-  ────────────────────────→  电机 B+ B-
 *
 *   供电:
 *   12V DC + → D36A VIN    12V DC - → D36A GND
 *   天猛星 USB-C 供电 (或 5V)
 *
 *   D36A 拨码开关 (默认): MS1/2/3=OFF(16细分), CUR1/2/3=OFF(1.44A)
 *   代码 D36A_MICROSTEP=16 必须与拨码一致!
 *
 *   串口0（调试）: PA28(TX)/PA31(RX) -> USB-TTL -> PC, 115200-8-N-1
 *   串口1（主控）: PA17(TX)/PA18(RX) <-> 主控 RX/TX, 115200-8-N-1
 *   串口1接线: PA17 -> 主控RX, PA18 <- 主控TX, GND <-> GND
 *   串口2（视觉）: PA21(TX)/PA22(RX) <-> MaixCam A18(RX)/A19(TX), 115200-8-N-1
 *   串口2接线: PA21 -> A18, PA22 <- A19, GND <-> GND
 *   主控通过 AA55 协议(TASK_CMD 0x40)切换赛题/开始/停止/设目标位置
 *   本机通过 AA55 协议(TASK_STATE 0x41)回报状态
 *   所有串口必须 TX/RX 交叉连接；所有设备必须共地
 *
 *   上电后测试模式(1~4)可发送 A1 5；平衡球模式(5)等待视觉数据
 *
 * ⚠️ 注意事项:
 *   1. 编码器 VCC 必须 3.3V, 5V 会烧 MCU
 *   2. 严禁带电插拔电机功率线 (A+/A-/B+/B-)
 *   3. 12V 电源不要接反
 *   4. 首次上电用小角度测试: A1 5
 *
 * ============================================================================
 * 实验模式 (demo_config.h DEMO_SELECT):
 *   1 = 开环转动   2 = 编码器读取
 *   5 = 平衡球控制（UART2视觉输入 + PID + 电机位置闭环）
 * ============================================================================
 *
 * 主控通信协议 (串口1, AA55帧格式):
 *   电脑/主控 → 本机:  波特率 115200-8-N-1, 必须共地
 *   本机       → 电脑:  波特率 115200-8-N-1, 同上
 *
 *   帧结构: AA 55 TYPE LEN DATA[LEN] CRC_L CRC_H
 *   CRC-16-IBM: 多项式 0xA001, 初始 0x0000, 覆盖 TYPE+LEN+DATA
 *
 *   === 主控 → 本机 (TYPE 0x40 TASK_CMD, LEN=4) ===
 *   指令格式: AA 55 40 04 [cmd] [task] [paramL] [paramH] CRC_L CRC_H
 *
 *   详细命令列表 (cmd 字节):
 *
 *   cmd=0x01 切换赛题     task=3/4/5/6
 *     AA 55 40 04 01 03 00 00 0E FC → 切换到第3题 (轨迹跟踪 O->+5cm->-5cm)
 *     AA 55 40 04 01 04 00 00 BF 3D → 切换到第4题 (行驶,球稳定在O点)
 *     AA 55 40 04 01 05 00 00 EE FD → 切换到第5题 (行驶一圈,球稳定在O点)
 *     AA 55 40 04 01 06 00 00 1E FD → 切换到第6题 (行驶一圈,球稳定在指定位置)
 *
 *   cmd=0x02 开始执行
 *     AA 55 40 04 02 00 00 00 FE B8 → 开始执行当前赛题
 *     第3题开始轨迹: O→+5→折返→-5
 *     第4/5/6题开始 PID 平衡控制
 *
 *   cmd=0x03 停止
 *     AA 55 40 04 03 00 00 00 FF 44 → 停止, 回到IDLE
 *
 *   cmd=0x04 设目标位置(第6题)  param = int16, cm×100
 *     AA 55 40 04 04 00 00 00 FE 30 → 设目标位置  0cm (O点中心)
 *     AA 55 40 04 04 00 F4 01 79 30 → 设目标位置 +5cm
 *     AA 55 40 04 04 00 D8 F0 A4 74 → 设目标位置 -10cm
 *
 *   === 本机 → 主控 (TYPE 0x41 TASK_STATE, LEN=6) ===
 *   自动每100ms上报一次:
 *   AA 55 41 06 [task] [state] [posL] [posH] [angL] [angH] CRC_L CRC_H
 *
 *   task:  当前赛题 (0=空闲, 3/4/5/6)
 *   state: 0=IDLE 1=RUNNING 2=DONE 3=FAULT
 *   pos:   ball_pos × 100, int16 小端 (cm)
 *   ang:   电机角度 × 100, int16 小端 (deg)
 *
 *   === 心跳包 (双向, 1Hz) ===
 *     AA 55 FF 01 00 CRC_L CRC_H
 *
 *   === 各赛题流程 ===
 *   第3题: 切换(0x01,task=3) → 可选设参(0x04设tgt1, 0x05设tgt2) → 开始(0x02)
 *          → setpoint 从 0cm 跳变到 target1(默认+5cm, UART0: H命令)
 *          → 球稳定在target1±tol达settle或超时 → setpoint跳到target2(默认-5cm, K命令)
 *          → 球稳定在target2±tol达settle或超时 → 自动 DONE
 *          → 停止(0x03)
 *          UART0一键启动: @键  参数: H/K/Q/C命令
 *          cmd=0x05: 设T3参数 task=索引(1..6) param=值
 *
 *   第4题: 切换(0x01,task=4) → 开始(0x02)
 *          → PID 平衡控制(setpoint=0cm)
 *          → 小车跑完AB段(8秒) → 主控发 停止(0x03)
 *
 *   第5题: 切换(0x01,task=5) → 开始(0x02)
 *          → PID 平衡控制(setpoint=0cm)
 *          → 小车跑完一圈(30秒) → 主控发 停止(0x03)
 *
 *   第6题: 切换(0x01,task=6) → 设目标(0x04,param) → 开始(0x02)
 *          → PID 平衡控制(setpoint=指定位置)
 *          → 小车跑完一圈(30秒) → 主控发 停止(0x03)
 * ============================================================================
 */

#include "board.h"
#include "app_demo.h"
#include "encoder.h"
#include "motor.h"

int main(void)
{
    SYSCFG_DL_init();
    Board_LED_Init();
    Motor_Init();
    Encoder_Init();
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    Demo_Init();

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) Demo_Process();
}

/**
 * TIMG0 5ms 周期中断
 * 编码器采样 + 闭环控制与任务状态更新
 */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        Demo_Tick5ms();
    }
}
