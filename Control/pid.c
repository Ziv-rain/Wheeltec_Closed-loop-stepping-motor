#include "pid.h"
static float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
void PID_Init(PID_t*p,float kp,float ki,float kd,float il,float omi,float oma){p->kp=kp;p->ki=ki;p->kd=kd;p->integral=0;p->prev_error=0;p->integral_limit=il;p->output_min=omi;p->output_max=oma;p->initialized=0;}
void PID_Reset(PID_t*p){p->integral=0;p->prev_error=0;p->initialized=0;}
float PID_Update(PID_t*p,float error,float dt){float d,o;if(dt<=0)dt=0.005f;if(!p->initialized){p->prev_error=error;p->initialized=1;}d=(error-p->prev_error)/dt;p->integral=clampf(p->integral+error*dt,-p->integral_limit,p->integral_limit);o=p->kp*error+p->ki*p->integral+p->kd*d;p->prev_error=error;return clampf(o,p->output_min,p->output_max);}
