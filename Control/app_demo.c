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

static volatile uint32_t s_ms;
static char s_line[40];
static uint8_t s_line_len;
static uint32_t s_last_action;
static uint32_t s_last_output;
static uint8_t s_demo1_state;
static uint8_t s_demo3_state;
static uint8_t s_demo2_state;

static const char *Demo_FaultName(CL_Fault_t fault)
{
    if (fault == CL_FAULT_NONE)        return "OK";
    if (fault == CL_FAULT_NO_ENCODER)  return "NO_ENCODER";
    if (fault == CL_FAULT_DIRECTION)   return "DIR_REVERSED";
    return "DRIVER";
}

/* ---- Experiment 4: Serial Commands ---- */
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

/* ---- Experiment 2: Encoder Read ---- */
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
        (void)ch;
#endif
    }
}

/* ---- Init ---- */
void Demo_Init(void)
{
    s_ms = 0U;
    s_last_action = 0U;
    s_last_output = 0U;
    s_demo1_state = 0U;
    s_demo2_state = 0U;
    s_demo3_state = 0U;
#if (DEMO_SELECT == 3) || (DEMO_SELECT == 4)
    CL_Init();
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
#else
    uart_puts("- Serial command mode\r\n");
    Demo4_PrintHelp();
#endif
}

/* ---- 5ms Tick ---- */
void Demo_Tick5ms(void)
{
    s_ms += CL_PERIOD_MS;
    Encoder_Tick(CL_PERIOD_MS);
#if (DEMO_SELECT == 3) || (DEMO_SELECT == 4)
    CL_Process();
#endif
}

/* ---- Main Loop ---- */
void Demo_Process(void)
{
    Demo_PollUart();

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
