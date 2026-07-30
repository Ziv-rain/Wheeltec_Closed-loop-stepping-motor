/**
  ******************************************************************************
  * @file    demo_config.h
  * @brief   单轴MS42CG + D36A例程的用户参数集中配置
  ******************************************************************************
  * 修改本文件后需要重新编译下载。D36A_MICROSTEP定义在motor.h中，必须和
  * 驱动器拨码一致；编码器A/B每圈4000计数，闭环以A/B增量计数为主反馈。
  ******************************************************************************
  */
#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

/*
 * 实验选择：
 * 1=开环定步往返；2=串口1/2/0控制连续旋转并监视编码器；
 * 3=闭环自动往返；4=串口角度指令闭环控制。
 */
#define DEMO_SELECT                 4
/* 本例程只使用轴1，电机数量固定为1。 */
#define MOTOR_COUNT                 1

/* MS42CG的A/B为1000线正交信号，硬件4倍频后每圈得到4000计数。 */
#define ENCODER_COUNTS_PER_REV      4000U
/* 若正向转动时计数减小，将该符号改成-1；只影响软件对外的正负方向。 */
#define ENCODER_AXIS_X_SIGN         1
/* DIR输出该电平时，期望编码器计数增加；方向故障时可修改或用D1临时翻转。 */
#define AXIS_X_POSITIVE_DIR_LEVEL   1U
#define AXIS_Y_POSITIVE_DIR_LEVEL   1U

/* 闭环周期和到位容差：2计数约等于0.18度，连续稳定3次才判定到位。 */
#define CL_PERIOD_MS                5U
#define CL_TOLERANCE_COUNTS         2U

/* 各实验参数。首次带机构测试时应使用小角度、低频率，防止撞机械限位。 */
#define DEMO1_STEPS                 (MOTOR_STEPS_PER_REV / 4U)
#define DEMO1_FREQ_HZ               800U
#define DEMO2_FREQ_HZ               800U
#define DEMO2_PRINT_MS              200U
#define DEMO3_TARGET_DEG            5.0f
#define DEMO3_SWITCH_MS             4000U
/* 模式3输出：0=普通中文文本；1={B目标:实际:误差}$上位机波形帧。 */
#define DEMO3_OUTPUT_MODE           0
#define DEMO3_OUTPUT_MS             20U

#if (DEMO_SELECT < 1) || (DEMO_SELECT > 4)
#error "DEMO_SELECT must be 1..4"
#endif
#if (MOTOR_COUNT < 1) || (MOTOR_COUNT > 2)
#error "MOTOR_COUNT must be 1 or 2"
#endif

#endif
