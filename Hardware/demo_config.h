/**
  ******************************************************************************
  * @file    demo_config.h
  * @brief   ����MS42CG + D36A���̵��û�������������
  ******************************************************************************
  * �޸ı��ļ�����Ҫ���±������ء�D36A_MICROSTEP������motor.h�У������
  * ����������һ�£�������A/BÿȦ4000�������ջ���A/B��������Ϊ��������
  ******************************************************************************
  */
#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

/*
 * ʵ��ѡ��
 * 1=��������������2=����1/2/0����������ת�����ӱ�������
 * 3=�ջ��Զ�������4=���ڽǶ�ָ��ջ����ơ�
 */
#define DEMO_SELECT                 8
#define MOTOR_MAX_ANGLE_POS         50.0f
#define MOTOR_MAX_ANGLE_NEG         35.0f
#define BALL_KP                     4.0f
#define BALL_KI                     0.0f
#define BALL_KD                     5.0f
#define PID_INTEGRAL_LIMIT          10.0f

/* ---- 视觉PID精调参数 (DEMO8) ---- */
#define VIS_KP                      10.0f   /* 高比例: 响应快 */
#define VIS_KD                      8.0f    /* 高微分: 抑制超调 */
#define VIS_KI                      0.0f
#define VIS_INTEGRAL_LIMIT          10.0f
#define VIS_OUTPUT_LIMIT_DEG        50.0f   /* PID输出宽限幅±50°, 安全由PWM限位钳制 */
#define VIS_SETPOINT_CM             0.0f    /* 默认视觉目标位置(cm), 运行时可N命令修改 */

/* PWM 绝对角度机械限位保护 (电机固定区间不跨0°/360°, 实测值) */
#define PWM_LIMIT_HIGH              190.0f   /* 正极限 PWM 角度 (+50° 电机位置) */
#define PWM_LIMIT_LOW               105.0f   /* 负极限 PWM 角度 (-35° 电机位置) */
#define PWM_LIMIT_MARGIN            2.0f     /* 限位保护预留余量 (°) */

/* 上电自动零点标定 (赛前调水平后读 PWM 填入, 比赛现场无需电脑) */
#define PWM_HORIZONTAL_REF          143.0f   /* 实测: 摆杆水平时 PWM = 143° */
#define PWM_HORIZONTAL_TOL          1.0f     /* 判断水平的容差 (°) */
#define HOMING_FREQ_HZ              100U
#define HOMING_WAIT_TIMEOUT_MS      3000U    /* 等待绝对角度信号的最长时间 */
#define HOMING_MOVE_TIMEOUT_MS      15000U   /* 连续回零运动的最长时间 */
#define HOMING_PWM_LOSS_TIMEOUT_MS  150U     /* 回零途中PWM丢失容忍时间 */
#define HOMING_NO_PROGRESS_MS       1500U    /* 未向水平点靠近时的停机时间 */

/* MS42CG��A/BΪ1000�������źţ�Ӳ��4��Ƶ��ÿȦ�õ�4000������ */
#define ENCODER_COUNTS_PER_REV      4000U
/* ������ת��ʱ������С�����÷��Ÿĳ�-1��ֻӰ������������������� */
#define ENCODER_AXIS_X_SIGN         1
/* DIR����õ�ƽʱ�������������������ӣ��������ʱ���޸Ļ���D1��ʱ��ת�� */
#define AXIS_X_POSITIVE_DIR_LEVEL   1U
#define AXIS_Y_POSITIVE_DIR_LEVEL   1U

/* �ջ����ں͵�λ�ݲ2����Լ����0.18�ȣ������ȶ�3�β��ж���λ�� */
#define CL_PERIOD_MS                5U
#define CL_TOLERANCE_COUNTS         2U

/* ��ʵ��������״δ���������ʱӦʹ��С�Ƕȡ���Ƶ�ʣ���ֹײ��е��λ�� */
#define DEMO1_STEPS                 (MOTOR_STEPS_PER_REV / 4U)
#define DEMO1_FREQ_HZ               800U
#define DEMO2_FREQ_HZ               800U
#define DEMO2_PRINT_MS              200U
#define DEMO3_TARGET_DEG            5.0f
#define DEMO3_SWITCH_MS             4000U
/* ģʽ3�����0=��ͨ�����ı���1={BĿ��:ʵ��:���}$��λ������֡�� */
#define DEMO3_OUTPUT_MODE           0
#define DEMO3_OUTPUT_MS             20U

#if (DEMO_SELECT < 1) || (DEMO_SELECT > 8)
#error "DEMO_SELECT must be 1..8"
#endif
#define MOTOR_COUNT                 1
#if (MOTOR_COUNT < 1) || (MOTOR_COUNT > 2)
#error "MOTOR_COUNT must be 1 or 2"
#endif

#endif
