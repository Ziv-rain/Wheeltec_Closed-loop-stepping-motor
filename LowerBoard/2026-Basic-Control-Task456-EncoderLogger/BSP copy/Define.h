#ifndef _DEFINE_H_
#define _DEFINE_H_
#include "ti_msp_dl_config.h"
typedef struct{
		uint8_t encoder;
        uint8_t encoder_flag;
		uint16_t mpu;
        uint8_t mpu_flag;
		uint8_t encoderpid;
        uint8_t encoderpid_flag;
		uint8_t gxpid;
        uint8_t gxpid_flag;
		uint8_t jiaodupid;
        uint8_t jiaodu_flag;
		uint8_t turnerrorpid;
        uint8_t turnerrorpid_flag;
		uint8_t motor;
        uint8_t motor_flag;
}_st_tick;

typedef struct{
		int16_t left;
		int16_t right;
}_st_encoder;

typedef struct{
		double left;
		double right;
		double run_Lspeed;
		double run_Rspeed;
		double turn_error;
		double track_l;
		double track_r;
		uint8_t button;
		int16_t Lbuchang;
		int16_t Rbuchang;
}_st_motor;

typedef struct{
        short gyro[3];
		int16_t gx;
		int16_t gy;
		int16_t gz;
		float pitch;
		float roll;
		float yaw;
		float chazhi_pitch;
		float chazhi_roll;
		float chazhi_yaw;
		float CAL_pitch;
		float CAL_roll;
		float CAL_yaw;
}_st_mpu;

extern _st_tick Tick;
extern _st_encoder Encoder;
extern _st_motor Motor;
extern _st_mpu Mpu;
float new_yaw;
uint8_t oled_buffer[32];

int16_t Tick_work;
_st_tick Tick;
_st_encoder Encoder;
_st_motor Motor;
_st_mpu Mpu;

int data;
void Timer_work(void);
int32_t L_PWM,R_PWM,jiaodu,tick_jiaodu,time_2s,t_200ms,t_3000ms,time_3s;
unsigned char L2,L1,M0,R1,R2;

#endif  /* #ifndef _MAIN_H_ */