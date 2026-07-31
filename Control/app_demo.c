/**
 * app_demo.c - Demo experiments and serial command handler
 * Uses uart_put*() for numeric output (printf %d/%u/%f broken in TI libc)
 *
 * DEMO_SELECT: 1=open-loop  2=encoder read  3=closed-loop auto  4=serial cmd
 */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "app_demo.h"
#include "closed_loop.h"
#include "demo_config.h"
#include "encoder.h"
#include "motor.h"
#include "proto_rx.h"
#include "ball_control.h"
#include "task_ctrl.h"
#include "mech_balance.h"

static volatile uint32_t s_ms;
#if (DEMO_SELECT == 4) || (DEMO_SELECT == 7) || (DEMO_SELECT == 8)
static char s_line[40];
static uint8_t s_line_len;
#endif
static uint32_t s_last_action;
static uint32_t s_last_output;
static uint8_t s_demo1_state;
static uint8_t s_demo3_state;
static uint8_t s_demo2_state;

#if (DEMO_SELECT == 3) || (DEMO_SELECT == 4)
static const char *Demo_FaultName(CL_Fault_t fault)
{
    if (fault == CL_FAULT_NONE)        return "OK";
    if (fault == CL_FAULT_NO_ENCODER)  return "NO_ENCODER";
    if (fault == CL_FAULT_DIRECTION)   return "DIR_REVERSED";
    return "DRIVER";
}
#endif

/* ---- Experiment 4: Serial Commands ---- */
#if (DEMO_SELECT == 4)
static void Demo4_PrintHelp(void)
{
    uart_puts("\r\n========== Exp4 Serial Commands ==========\r\n");
    uart_puts("  A1 <angle>  Goto angle, e.g. A1 90 or A1 -45.5\r\n");
    uart_puts("  S           Query axis-1 status\r\n");
    uart_puts("  Z           Set current position as zero\r\n");
    uart_puts("  X           Emergency stop\r\n");
    uart_puts("  C1          Clear fault\r\n");
    uart_puts("  D1          Toggle positive direction\r\n");
    uart_puts("  H           Print this help\r\n");
    uart_puts("==========================================\r\n\r\n");
}

static void Demo4_PrintAxis(void)
{
    CL_Snapshot_t snap;
    float pwm_angle = 0.0f;
    uint8_t pwm_ok;
    int32_t zcnt;
    CL_GetSnapshot(MOTOR_AXIS_X, &snap);
    pwm_ok = Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle);
    zcnt = Encoder_GetZCount(ENCODER_AXIS_X);

    uart_puts("AX1: Tgt=");  uart_putf(snap.target_angle_deg, 2);
    uart_puts("  Act=");     uart_putf(snap.current_angle_deg, 2);
    uart_puts("  Err=");     uart_putf(snap.target_angle_deg - snap.current_angle_deg, 2);
    uart_puts("  [");
    uart_puts((snap.reached != 0U) ? "IN_POS" : ((snap.active != 0U) ? "MOVING" : "IDLE"));
    uart_puts("]  Fault:");  uart_puts(Demo_FaultName(snap.fault));
    uart_puts("  PWM=");
    if (pwm_ok != 0U) uart_putf(pwm_angle, 2);
    else uart_puts("N/A");
    uart_puts("  Z=");       uart_puti(zcnt);
    uart_puts("\r\n");
}

static void Demo4_HandleCommand(char *line)
{
    char *p;
    float angle;
    for (p = line; *p != '\0'; p++) {
        if (*p >= 'a' && *p <= 'z') *p = (char)(*p - 'a' + 'A');
    }
    if (sscanf(line, "A1 %f", &angle) == 1) {
        if (CL_SetTargetAngle(MOTOR_AXIS_X, angle) == MOTOR_OK) {
            uart_puts("OK: Target="); uart_putf(angle, 2); uart_puts("\r\n");
        } else {
            uart_puts("ERR: Motor has fault, send C1 to clear\r\n");
        }
    } else if (strcmp(line, "S") == 0) {
        Demo4_PrintAxis();
    } else if (strcmp(line, "Z") == 0) {
        CL_SetZeroAll();
        uart_puts("OK: Zero set\r\n");
    } else if (strcmp(line, "X") == 0) {
        CL_StopAll();
        uart_puts("OK: Stopped\r\n");
    } else if (strcmp(line, "C1") == 0) {
        CL_ClearFault(MOTOR_AXIS_X);
        uart_puts("OK: Fault cleared\r\n");
    } else if (strcmp(line, "D1") == 0) {
        if (CL_TogglePositiveDirLevel(MOTOR_AXIS_X) == MOTOR_OK)
            uart_puts("OK: Direction toggled\r\n");
        else
            uart_puts("ERR: Cannot toggle while moving\r\n");
    } else if (strcmp(line, "H") == 0 || strcmp(line, "?") == 0) {
        Demo4_PrintHelp();
    } else if (line[0] != '\0') {
        uart_puts("Unknown: "); uart_puts(line); uart_puts(" (send H)\r\n");
    }
}
#endif

/* ---- Experiment 2: Encoder Read ---- */
#if (DEMO_SELECT == 2)
static void Demo2_SetMotion(uint8_t motion)
{
    if (motion == 0U) {
        Motor_StopAll();
        s_demo2_state = 0U;
        uart_puts("[Exp2] Stopped\r\n");
    } else if (motion == s_demo2_state) {
        uart_puts("[Exp2] Already running\r\n");
    } else {
        Motor_StartContinuous(DEMO2_FREQ_HZ, motion == 1U ? 0U : 1U);
        s_demo2_state = motion;
        uart_puts("[Exp2] Start "); uart_puts(motion == 1U ? "CW" : "CCW");
        uart_puts(" freq="); uart_putu(DEMO2_FREQ_HZ); uart_puts(" Hz, send 0 to stop\r\n");
    }
}
#endif

static void Demo_PollUart(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
        char ch = (char)DL_UART_Main_receiveData(UART_0_INST);
#if (DEMO_SELECT == 2)
        if (ch == '0') Demo2_SetMotion(0U);
        else if (ch == '1') Demo2_SetMotion(1U);
        else if (ch == '2') Demo2_SetMotion(2U);
        else if (ch != '\r' && ch != '\n' && ch != ' ' && ch != '\t') {
            uart_puts("[Exp2] Unknown key, send 1/2/0\r\n");
        }
#elif (DEMO_SELECT == 4)
        if (ch == '\r' || ch == '\n') {
            if (s_line_len != 0U) {
                s_line[s_line_len] = '\0';
                Demo4_HandleCommand(s_line);
                s_line_len = 0U;
            }
        } else if (s_line_len < sizeof(s_line) - 1U) {
            s_line[s_line_len++] = ch;
        } else {
            s_line_len = 0U;
        }
#else
#if (DEMO_SELECT == 7) || (DEMO_SELECT == 8)
        /* 模式7/8: 力学补偿调参 + 视觉PID */
        if (ch == 'X' || ch == 'x') {
            s_line_len = 0U;
            MechBalance_EmergencyStop();
            BallControl_EmergencyStop();
            uart_puts("OK emergency stop latched; reboot required\r\n");
        } else if (ch == 'S' || ch == 's') {
            CL_Snapshot_t snap; float pwm=0; uint8_t ok;
            CL_GetSnapshot(MOTOR_AXIS_X, &snap);
            ok = Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm);
            uart_puts("Tgt="); uart_putf(snap.target_angle_deg, 2);
            uart_puts(" Act="); uart_putf(snap.current_angle_deg, 2);
            uart_puts(" PWM="); if(ok) uart_putf(pwm,2); else uart_puts("N/A");
#if (DEMO_SELECT == 8)
            {
                VisionStatus_t vs;
                MechBalance_GetVisionStatus(&vs);
                uart_puts(" | VIS:auto="); uart_putu(MechBalance_GetSeqAutoVision());
                uart_puts(" act="); uart_putu(vs.active);
                uart_puts(" valid="); uart_putu(vs.valid);
                uart_puts(" fault="); uart_putu(vs.fault);
                uart_puts(" sp="); uart_putf(vs.setpoint_cm, 2);
                uart_puts(" ball="); uart_putf(vs.ball_pos_cm, 2);
                uart_puts(" vel="); uart_putf(vs.ball_velocity_cm_s, 2);
                uart_puts(" vref="); uart_putf(vs.velocity_ref_cm_s, 2);
                uart_puts(" out="); uart_putf(vs.pid_out_deg, 2);
                uart_puts(" seq="); uart_putu(vs.seq_step);
                uart_puts(" stable="); uart_putu(vs.stable);
            }
#endif
            uart_puts("\r\n");
        } else if (ch == 'Z' || ch == 'z') {
            uart_puts("ERR zero is owned by automatic homing\r\n");
        } else if (ch == 'P' || ch == 'p') {
            const MechParams_t *m = MechBalance_GetParams();
            uart_puts("g="); uart_putf(m->gravity,2);
            uart_puts(" fwd="); uart_putf(m->accel_gain_fwd,2);
            uart_puts(" brk="); uart_putf(m->accel_gain_brake,2);
            uart_puts(" trim="); uart_putf(m->theta_trim_deg,2);
            uart_puts(" rate="); uart_putf(m->theta_rate_limit,0);
#if (DEMO_SELECT == 8)
            {
                VisionStatus_t vs;
                MechBalance_GetVisionStatus(&vs);
                uart_puts(" auto_vis="); uart_putu(MechBalance_GetSeqAutoVision());
            }
#endif
            uart_puts("\r\n");
#if (DEMO_SELECT == 8)
        } else if (ch == 'V' || ch == 'v') {
            MechBalance_StartVision();
            uart_puts("Vision PID started\r\n");
        } else if (ch == 'W' || ch == 'w') {
            MechBalance_StopVision();
            uart_puts("Vision PID stopped\r\n");
        } else if (ch == 'U' || ch == 'u') {
            uint8_t en = MechBalance_ToggleSeqAutoVision();
            uart_puts(en ? "Auto-vision ON\r\n" : "Auto-vision OFF\r\n");
#endif
        }
        /* 回车优先处理: 触发命令解析 (必须在缓冲分支之前) */
        else if (ch == '\r' || ch == '\n') {
            if (s_line_len > 0U) {
                char c = s_line[0];
                float v = 0.0f;
                uint8_t i, neg = 0, has_digit = 0, idx = 0;
                /* 手动解析浮点数 (TI libc 的 sscanf %f 不可用) */
                if (c>='a'&&c<='z') c -= 32;
                /* K: 单字符命令, 执行角度序列 */
                if (c == 'K' && s_line_len == 1U) {
                    if (BallControl_IsHomingReady()) {
                        MechBalance_StartSeq();
                        uart_puts("OK seq start\r\n");
                    } else {
                        uart_puts("ERR homing not ready\r\n");
                    }
                    s_line_len = 0U;
                    continue;
                }
                /* Q/E: 数字从 s_line[2] 开始, s_line[1] 是步索引 1-4 */
                i = (c=='Q'||c=='E') ? 2U : 1U;
                if (c=='Q'||c=='E') {
                    if (s_line[1] >= '1' && s_line[1] <= '4') idx = (uint8_t)(s_line[1]-'1');
                    else { uart_puts("ERR idx\r\n"); s_line_len = 0U; continue; }
                }
                while (i < s_line_len && s_line[i] == ' ') i++;  /* 跳过前导空格 */
                if (i < s_line_len && s_line[i] == '-') { neg = 1; i++; }
                else if (i < s_line_len && s_line[i] == '+') { i++; }
                while (i < s_line_len && s_line[i] >= '0' && s_line[i] <= '9') {
                    v = v * 10.0f + (float)(s_line[i] - '0');
                    has_digit = 1; i++;
                }
                if (i < s_line_len && s_line[i] == '.') {
                    float frac = 0.1f;
                    i++;
                    while (i < s_line_len && s_line[i] >= '0' && s_line[i] <= '9') {
                        v += (float)(s_line[i] - '0') * frac;
                        frac *= 0.1f; i++; has_digit = 1;
                    }
                }
                if (neg) v = -v;
                if (has_digit) {
                    switch(c) {
                    case 'A': MechBalance_ExitDirect(); MechBalance_SetAccel(v); uart_puts("OK accel="); uart_putf(v,3); break;
                    case 'D': MechBalance_SetDirectAngle(v); uart_puts("OK dir="); uart_putf(v,2); break;
                    case 'T': MechBalance_SetParam(MP_TRIM, v); uart_puts("OK trim="); uart_putf(v,2); break;
                    case 'G': MechBalance_SetParam(MP_GAIN_FWD, v); uart_puts("OK fwd="); uart_putf(v,2); break;
                    case 'B': MechBalance_SetParam(MP_GAIN_BRAKE, v); uart_puts("OK brk="); uart_putf(v,2); break;
                    case 'L': MechBalance_SetParam(MP_RATE_LIMIT, v); uart_puts("OK rate="); uart_putf(v,0); break;
                    case 'R': MechBalance_SetParam(MP_RATE_LIMIT, v); uart_puts("OK rate="); uart_putf(v,0); break;
                    case 'Q': MechBalance_SetSeqAngle(idx, v); uart_puts("OK seqA"); uart_putu((uint32_t)(idx+1)); uart_puts("="); uart_putf(v,2); break;
                    case 'E': MechBalance_SetSeqTime(idx, (uint32_t)v); uart_puts("OK seqT"); uart_putu((uint32_t)(idx+1)); uart_puts("="); uart_putu((uint32_t)v); break;
                    case 'N':
                        if (MechBalance_SetVisionSetpoint(v)) {
                            uart_puts("OK vis_sp="); uart_putf(v,2);
                        } else {
                            uart_puts("ERR vis_sp range");
                        }
                        break;
                    case 'J': MechBalance_SetVisKp(v); uart_puts("OK vis_KP="); uart_putf(v,2); break;
                    case 'M': MechBalance_SetVisKd(v); uart_puts("OK vis_KD="); uart_putf(v,2); break;
                    case 'I': MechBalance_SetVisKi(v); uart_puts("OK vis_KI="); uart_putf(v,2); break;
                    default: uart_puts("ERR cmd"); break;
                    }
                    uart_puts("\r\n");
                } else {
                    uart_puts("ERR no number\r\n");
                }
                s_line_len = 0U;
            }
        }
        /* A/T/G/B/L/R/D/Q/E/K/N/J/M/I 命令开头 */
        else if (ch == 'A'||ch=='a'||ch=='T'||ch=='t'||ch=='G'||ch=='g'||ch=='B'||ch=='b'||ch=='L'||ch=='l'||ch=='R'||ch=='r'||ch=='D'||ch=='d'||ch=='Q'||ch=='q'||ch=='E'||ch=='e'||ch=='K'||ch=='k'||ch=='N'||ch=='n'||ch=='J'||ch=='j'||ch=='M'||ch=='m'||ch=='I'||ch=='i') {
            if (s_line_len < sizeof(s_line) - 1U) {
                s_line[s_line_len++] = ch;
            }
        }
        /* 命令进行中: 数字、小数点、负号全部进缓冲 */
        else if (s_line_len > 0U) {
            if (s_line_len < sizeof(s_line) - 1U) {
                s_line[s_line_len++] = ch;
            }
        }
#else
        /* 模式5(平衡球)下调试串口入口 */
        TaskCtrl_FeedByte((uint8_t)ch);
        if (ch == 'X' || ch == 'x') {
            CL_StopAll();
            TaskCtrl_ReportFault();
            uart_puts("ERR: Emergency stop\r\n");
        } else if (ch == 'S' || ch == 's') {
            float pwm_angle = 0.0f; uint8_t pwm_ok;
            CL_Snapshot_t snap; CL_GetSnapshot(MOTOR_AXIS_X, &snap);
            pwm_ok = Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle);
            uart_puts("Tgt="); uart_putf(snap.target_angle_deg, 2);
            uart_puts(" Act="); uart_putf(snap.current_angle_deg, 2);
            uart_puts(" Err="); uart_putf(snap.target_angle_deg - snap.current_angle_deg, 2);
            uart_puts("  PWM="); if(pwm_ok) uart_putf(pwm_angle, 2); else uart_puts("N/A");
            uart_puts(" Z="); uart_puti(Encoder_GetZCount(ENCODER_AXIS_X));
            uart_puts("\r\n");
        } else if (ch == 'Z' || ch == 'z') {
            CL_SetZero(MOTOR_AXIS_X);
            uart_puts("Zero set\r\n");
        }
#endif
#endif
    }
}

/* ---- Init ---- */
void Demo_Init(void)
{
    ProtoRx_Init();
    TaskCtrl_Init();
    s_ms = 0U;
    s_last_action = 0U;
    s_last_output = 0U;
    s_demo1_state = 0U;
    s_demo2_state = 0U;
    s_demo3_state = 0U;
#if (DEMO_SELECT == 3) || (DEMO_SELECT == 4) || (DEMO_SELECT == 5) || (DEMO_SELECT == 7) || (DEMO_SELECT == 8)
    CL_Init();
#endif
#if (DEMO_SELECT == 5)
    BallControl_Init();
#endif
#if (DEMO_SELECT == 7) || (DEMO_SELECT == 8)
    MechBalance_Init();
    BallControl_Init();  /* 复用自动回零状态机 */
#endif
#if (DEMO_SELECT == 8)
    MechBalance_SetSeqAutoVision(1); /* 序列结束后自动启动视觉PID */
#endif
    uart_puts("\r\n****************************************************\r\n");
    uart_puts("*  MS42CG + D36A Closed-loop Stepper Demo          *\r\n");
    uart_puts("*  MCU: MSPM0G3507  UART: 115200-8-N-1             *\r\n");
    uart_puts("****************************************************\r\n");
    uart_puts("MICROSTEP = "); uart_putu(D36A_MICROSTEP);
    uart_puts(" (must match DIP switch!)\r\n");
    uart_puts("MOTOR_COUNT = 1\r\n");
    uart_puts("Exp"); uart_puti(DEMO_SELECT); uart_puts(" ");
#if (DEMO_SELECT == 1)
    uart_puts("- Open-loop\r\n");
#elif (DEMO_SELECT == 2)
    uart_puts("- Encoder readout. Send 1=CW 2=CCW 0=Stop\r\n");
#elif (DEMO_SELECT == 3)
    uart_puts("- Closed-loop auto: 0 <-> "); uart_putf(DEMO3_TARGET_DEG, 1);
    uart_puts(" deg\r\n");
#elif (DEMO_SELECT == 4)
    uart_puts("- Serial command mode\r\n");
    Demo4_PrintHelp();
#elif (DEMO_SELECT == 7)
    uart_puts("- Mechanical balance (pure feedforward)\r\n");
    uart_puts("  A<val>  set accel m/s2\r\n");
    uart_puts("  T<val>  set trim deg\r\n");
    uart_puts("  G<val>  set fwd gain\r\n");
    uart_puts("  B<val>  set brake gain\r\n");
    uart_puts("  L<val>  set angle limit deg\r\n");
    uart_puts("  R<val>  set rate limit deg/s\r\n");
    uart_puts("  P       print params\r\n");
    uart_puts("  S       print status\r\n");
#elif (DEMO_SELECT == 8)
    uart_puts("- Mechanical balance + vision PID\r\n");
    uart_puts("  K        4-step seq (auto vision after)\r\n");
    uart_puts("  V        start vision PID\r\n");
    uart_puts("  W        stop vision PID\r\n");
    uart_puts("  U        toggle auto-vision after seq\r\n");
    uart_puts("  J<val>   set vision KP\r\n");
    uart_puts("  M<val>   set vision KD\r\n");
    uart_puts("  I<val>   set vision KI\r\n");
    uart_puts("  N<val>   set vision target cm\r\n");
    uart_puts("  Q<n><v>  set seq angle n=1..4\r\n");
    uart_puts("  E<n><v>  set seq time n=1..4 ms\r\n");
    uart_puts("  A/T/G/B/R/D  mech params (same as DEMO7)\r\n");
    uart_puts("  S        status (ball/vision)\r\n");
    uart_puts("  P        print params\r\n");
    uart_puts("  X        immediate latched emergency stop\r\n");
#else
    uart_puts("- Ball control mode (UART2 vision + PID)\r\n");
#endif
}

/* ---- 5ms Tick ---- */
void Demo_Tick5ms(void)
{
    s_ms += CL_PERIOD_MS;
    Encoder_Tick(CL_PERIOD_MS);
#if (DEMO_SELECT == 5)
    ProtoRx_Tick(CL_PERIOD_MS);
    TaskCtrl_Tick5ms();
    BallControl_Tick5ms();
    CL_Process(); CL_Process(); CL_Process(); CL_Process();
#elif (DEMO_SELECT == 7)
    BallControl_Tick5ms();  /* 上电自动回零; 回零完成后因无RUNNING状态不会启动PID */
    MechBalance_Tick5ms();
    CL_Process(); CL_Process(); CL_Process(); CL_Process();
#elif (DEMO_SELECT == 8)
    ProtoRx_Tick(CL_PERIOD_MS);  /* 关键: 更新视觉数据age, 否则超时保护失效 */
    BallControl_Tick5ms();
    if (BallControl_IsHomingReady()) {
        MechBalance_Tick5ms();
    } else if (BallControl_HasFault()) {
        MechBalance_EmergencyStop();
    }
    CL_Process();
#elif (DEMO_SELECT == 3) || (DEMO_SELECT == 4)
    CL_Process();
#endif
}

/* ---- Main Loop ---- */
void Demo_Process(void)
{
    Demo_PollUart();
#if (DEMO_SELECT == 5)
    TaskCtrl_Process();
#endif

#if (DEMO_SELECT == 1)
    if (Motor_IsBusy(MOTOR_AXIS_X) == 0U && (s_ms - s_last_action) >= 1000U) {
        uint8_t direction = (s_demo1_state == 0U) ? 0U : 1U;
        uart_puts("[Exp1] "); uart_puts(s_demo1_state == 0U ? "CW " : "CCW ");
        uart_putu(DEMO1_STEPS); uart_puts(" steps, ");
        uart_putu(DEMO1_FREQ_HZ); uart_puts(" Hz\r\n");
        (void)Motor_SetDirection(MOTOR_AXIS_X, direction);
        (void)Motor_Start(MOTOR_AXIS_X, DEMO1_STEPS, DEMO1_FREQ_HZ);
        s_demo1_state = (uint8_t)!s_demo1_state;
        s_last_action = s_ms;
    }
#elif (DEMO_SELECT == 2)
    if ((s_ms - s_last_output) >= DEMO2_PRINT_MS) {
        float pwm_angle = 0.0f;
        uint8_t pwm_ok = Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle);
        uart_puts(s_demo2_state == 1U ? "CW " : (s_demo2_state == 2U ? "CCW " : "STOP "));
        uart_puts("Cnt="); uart_puti(Encoder_GetCount(ENCODER_AXIS_X));
        uart_puts("  Ang="); uart_putf(Encoder_GetAngle(ENCODER_AXIS_X), 2);
        uart_puts("  Spd="); uart_putf(Encoder_CalcSpeedDps(ENCODER_AXIS_X, DEMO2_PRINT_MS), 1);
        uart_puts("  PWM=");
        if (pwm_ok) uart_putf(pwm_angle, 2); else uart_puts("N/A");
        uart_puts("  Z="); uart_puti(Encoder_GetZCount(ENCODER_AXIS_X));
        uart_puts("\r\n");
        s_last_output = s_ms;
    }
#elif (DEMO_SELECT == 3)
    if ((s_ms - s_last_action) >= DEMO3_SWITCH_MS) {
        float target;
        if (CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE) {
            uart_puts("[Exp3] Fault: ");
            uart_puts(Demo_FaultName(CL_GetFault(MOTOR_AXIS_X)));
            uart_puts("\r\n");
        }
        if (s_demo3_state == 0U) target = DEMO3_TARGET_DEG;
        else if (s_demo3_state == 1U) target = 0.0f;
        else if (s_demo3_state == 2U) target = -DEMO3_TARGET_DEG;
        else target = 0.0f;
        (void)CL_SetTargetAngle(MOTOR_AXIS_X, target);
        s_demo3_state = (uint8_t)((s_demo3_state + 1U) % 4U);
        s_last_action = s_ms;
    }
    if ((s_ms - s_last_output) >= DEMO3_OUTPUT_MS) {
        CL_Snapshot_t snap;
        CL_GetSnapshot(MOTOR_AXIS_X, &snap);
#if (DEMO3_OUTPUT_MODE == 1)
        uart_puts("{B"); uart_putf(snap.target_angle_deg, 2);
        uart_putc(':');  uart_putf(snap.current_angle_deg, 2);
        uart_putc(':');  uart_putf(snap.target_angle_deg - snap.current_angle_deg, 2);
        uart_puts("}$");
#else
        static uint8_t s_div;
        if (++s_div >= 10U) {
            s_div = 0U;
            uart_puts("AX1: Tgt="); uart_putf(snap.target_angle_deg, 2);
            uart_puts("  Act=");    uart_putf(snap.current_angle_deg, 2);
            uart_puts("  Err=");    uart_putf(snap.target_angle_deg - snap.current_angle_deg, 2);
            uart_puts("  ");        uart_puts(snap.reached != 0U ? "IN_POS" : "MOVING");
            uart_puts("\r\n");
        }
#endif
        s_last_output = s_ms;
    }
#endif
}
