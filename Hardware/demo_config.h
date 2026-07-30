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
#define DEMO_SELECT                 6
/* ������ֻʹ����1����������̶�Ϊ1�� */
#define MOTOR_COUNT                 1

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

/* 训练数据采集: 自动缓慢摆球 (电机在 -20°~+20° 间三角波扫描) */
#define SWEEP_ANGLE_MIN             -20.0f
#define SWEEP_ANGLE_MAX             20.0f
#define SWEEP_PERIOD_MS             6000U
#define SWEEP_UPDATE_MS             20U

#if (DEMO_SELECT < 1) || (DEMO_SELECT > 6) || (DEMO_SELECT == 5)
#error "DEMO_SELECT must be 1, 2, 3, 4, or 6"
#endif
#if (MOTOR_COUNT < 1) || (MOTOR_COUNT > 2)
#error "MOTOR_COUNT must be 1 or 2"
#endif
#if (SWEEP_PERIOD_MS < (2U * CL_PERIOD_MS)) || ((SWEEP_PERIOD_MS % 2U) != 0U)
#error "SWEEP_PERIOD_MS must be even and at least two control periods"
#endif
#if (SWEEP_UPDATE_MS < CL_PERIOD_MS)
#error "SWEEP_UPDATE_MS must not be shorter than CL_PERIOD_MS"
#endif

#endif
