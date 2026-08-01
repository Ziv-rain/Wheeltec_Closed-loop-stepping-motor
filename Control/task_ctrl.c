/** task_ctrl.c - 赛题状态机 + UART1/AA55协议 + 第3题两段离散setpoint */
#include "task_ctrl.h"
#include "ti_msp_dl_config.h"
#include "board.h"
#include "mech_balance.h"

#define TARGET_SETTLE_MS 500U
#define TARGET_SETTLE_TOL 1.0f
#define HB_INTERVAL 1000U
#define STATE_INTERVAL 100U
#define LINK_TIMEOUT_MS 2500U
#define TX_PENDING_STATE (1U<<0)
#define TX_PENDING_HB (1U<<1)
#define CMD_QUEUE_SIZE 8U
#define CMD_QUEUE_MASK (CMD_QUEUE_SIZE-1U)
typedef struct{uint8_t cmd,task;int16_t param;}TCmd_t;
typedef struct{uint8_t s,t,l,i,d[8];uint16_t c;}AA55_t;static AA55_t rx,rx0;
static volatile TCmd_t q[CMD_QUEUE_SIZE];static volatile uint8_t qh,qt;
static struct{uint8_t tid,st;float sp;uint32_t run;float bp;uint8_t bv;volatile uint8_t esp,ovf,link,pend;uint32_t sms,lrx;}ts;
static volatile uint32_t tms;static uint32_t lhb,lst;
/* === 第3题可配置参数 === */
static float t3_tgt1=4.0f,t3_tgt2=-5.0f;        /* 初始目标 / 终值 (cm) */
static float t3_tol=0.8f;                        /* 到位容差 (cm) */
static uint32_t t3_stl=500U;                     /* 到位稳定时间 (ms) */
static float t3_mid=1.5f;                        /* 摆荡中间目标 (cm) */
static uint32_t t3_ramp=750U;                    /* 终段ramp时长 (ms) */
static float t3_brake=2.5f;                      /* 刹车角度 (deg) */
static uint8_t t3_seg=0;                         /* 0=第一段, 1=摆荡, 2=ramp */
static uint32_t t3_seg_start=0U;
/* 车轮编码器: TYPE 0x42 解析 + 加速度估算 */
#define WENC_SCALE 0.0002618f /* 65mm轮780脉冲/圈: pi*0.065/780, 审查实测 */
static volatile uint32_t w_enc_l,w_enc_r,w_enc_ts;    /* 最新一帧 */
static volatile uint32_t w_enc_lp,w_enc_rp,w_enc_tsp;  /* 上一帧 */
static volatile float w_speed_ms,w_accel_ms2;           /* 速度(m/s) + 加速度(m/s²) */
static volatile uint8_t w_valid;                        /* 是否有足够数据计算加速度 */
static volatile uint32_t w_frame;                       /* 有效0x42帧计数(新帧判断) */
static volatile uint32_t w_age_ms;                      /* 距上次有效0x42帧的时间 */
/* 诊断输出暂存: exec在5ms中断里只置标志, 由主循环TaskCtrl_Process输出, 避免阻塞中断 */
static volatile uint8_t diag,dt;static volatile float dsp;
/* RX诊断: 收到0x40帧立即打印cmd/task/param, 验证指令是否到达 */
static volatile uint8_t rx_diag;static volatile uint8_t rx_cmd,rx_task;static volatile int16_t rx_param;
#define DIAG_TASK_OK 1U
#define DIAG_TASK_ERR 2U
#define DIAG_START 3U
#define DIAG_START_T4 4U
#define DIAG_NO_TASK 5U
#define DIAG_STOP 6U
#define DIAG_SP 7U
#define DIAG_NOT_T6 8U
#define DIAG_T3_SEG 9U    /* 第3题段切换 */
#define DIAG_T3_START 10U  /* 第3题启动 */
#define DIAG_T3_DONE 11U   /* 第3题自动完成 */

static uint16_t crc16_update(uint16_t c,uint8_t b){c^=b;for(uint8_t i=0;i<8;i++)c=(c&1)?(c>>1)^0xA001:c>>1;return c;}
static uint16_t crc16_frame(uint8_t t,uint8_t l,const uint8_t*d){uint16_t c=0;c=crc16_update(c,t);c=crc16_update(c,l);for(uint8_t i=0;i<l;i++)c=crc16_update(c,d[i]);return c;}
/* 单一AA55解析器, 消除rx/rx0重复; 收到完整有D:\MyProject\TI\2026\Wheeltec_Closed-loop-stepping-motor\力学模型效帧返回1并通过out参数输出 */
static uint8_t aa55(AA55_t*st,uint8_t b,uint8_t*t,uint8_t*l,uint8_t**d){uint16_t c;switch(st->s){case 0:if(b==0xAA)st->s=1;break;case 1:if(b==0x55)st->s=2;else st->s=(b==0xAA)?1:0;break;case 2:st->t=b;st->s=3;break;case 3:st->l=b;st->i=0;if(st->l>8){st->s=0;break;}st->s=st->l?4:5;break;case 4:st->d[st->i++]=b;if(st->i>=st->l)st->s=5;break;case 5:st->c=b;st->s=6;break;default:c=(uint16_t)st->c|((uint16_t)b<<8);st->s=0;if(c==crc16_frame(st->t,st->l,st->d)){*t=st->t;*l=st->l;*d=st->d;return 1;}break;}return 0;}
static void u1tx(const uint8_t*d,uint16_t n){for(uint16_t i=0;i<n;i++){while(DL_UART_Main_isTXFIFOFull(UART_1_INST)){}DL_UART_Main_transmitData(UART_1_INST,d[i]);}}
static void sframe(uint8_t t,const uint8_t*d,uint8_t l){uint8_t b[16];uint16_t c;if(l>8)return;b[0]=0xAA;b[1]=0x55;b[2]=t;b[3]=l;for(uint8_t i=0;i<l;i++)b[4+i]=d[i];c=crc16_frame(t,l,d);b[4+l]=(uint8_t)c;b[5+l]=(uint8_t)(c>>8);u1tx(b,l+6);}
static void sstate(uint8_t tid,uint8_t st,float bp){uint8_t d[6];float bc=bp*100.0f;if(bc>32767)bc=32767;if(bc<-32768)bc=-32768;int16_t b=(int16_t)bc;d[0]=tid;d[1]=st;d[2]=(uint8_t)(uint16_t)b;d[3]=(uint8_t)((uint16_t)b>>8);d[4]=0;d[5]=0;sframe(0x41,d,6);}

static uint8_t popq(TCmd_t*c){uint32_t pk;uint8_t t;pk=__get_PRIMASK();__disable_irq();t=qt;if(t==qh){if(!pk)__enable_irq();return 0;}c->cmd=q[t].cmd;c->task=q[t].task;c->param=q[t].param;qt=(uint8_t)((t+1)&CMD_QUEUE_MASK);if(!pk)__enable_irq();return 1;}
static void pushq(uint8_t cmd,uint8_t task,int16_t param){uint32_t pk;uint8_t h,n;if(cmd<1||cmd>5)return;pk=__get_PRIMASK();__disable_irq();if(cmd==3){ts.esp=1;if(!pk)__enable_irq();return;}h=qh;n=(uint8_t)((h+1)&CMD_QUEUE_MASK);if(n==qt){ts.ovf=1;}else{q[h].cmd=cmd;q[h].task=task;q[h].param=param;qh=n;}if(!pk)__enable_irq();}

static void exec(const TCmd_t*c){switch(c->cmd){case 1:if(c->task>=TASK_2&&c->task<=6){ts.tid=c->task;ts.st=STATE_IDLE;ts.sp=0;ts.run=0;ts.sms=0;ts.bv=0;if(c->task==TASK_3){t3_seg=0;t3_seg_start=0;}diag=DIAG_TASK_OK;dt=c->task;}else diag=DIAG_TASK_ERR;break;case 2:if(ts.tid==TASK_3){ts.st=STATE_RUNNING;ts.run=0;ts.sms=0;t3_seg=0;t3_seg_start=0;diag=DIAG_T3_START;}else if(ts.tid==TASK_4||ts.tid==TASK_5){ts.st=STATE_RUNNING;ts.run=0;ts.sms=0;diag=DIAG_START_T4;dt=ts.tid;}else if(ts.tid==TASK_6){ts.st=STATE_RUNNING;ts.run=0;ts.sms=0;diag=DIAG_START;}else diag=DIAG_NO_TASK;break;case 3:ts.st=STATE_IDLE;ts.sp=0;ts.run=0;ts.sms=0;ts.bv=0;diag=DIAG_STOP;break;case 4:{float v4=(float)c->param/100.0f;if(ts.tid==TASK_3){t3_tgt1=v4;diag=DIAG_SP;dsp=v4;}else if(ts.tid>=TASK_4&&ts.tid<=TASK_6){ts.sp=v4;ts.sms=0;diag=DIAG_SP;dsp=ts.sp;}else diag=DIAG_NOT_T6;}break;case 5:if(ts.tid==TASK_3){t3_tgt2=(float)c->param/100.0f;diag=DIAG_SP;dsp=t3_tgt2;}break;}ts.pend|=TX_PENDING_STATE;}

static void execq(void){TCmd_t c;uint32_t pk;uint8_t es,ov;pk=__get_PRIMASK();__disable_irq();es=ts.esp;ov=ts.ovf;ts.esp=0;ts.ovf=0;if(es||ov){qt=qh;}if(!pk)__enable_irq();if(es){c.cmd=3;c.task=ts.tid;c.param=0;exec(&c);return;}if(ov){TaskCtrl_ReportFault();return;}while(popq(&c))exec(&c);}

static void ftchk(void){float e=ts.bp-ts.sp;if(e<0)e=-e;if(ts.bv&&e<TARGET_SETTLE_TOL){ts.sms+=5;if(ts.sms>=TARGET_SETTLE_MS){ts.st=STATE_DONE;}}else ts.sms=0;}
/* 第3题: 三段式 — target1→球过峰→target2(摆荡)→终值→DONE */
/* 第3题: target1→过4.5cm→mid→ramp→过-4.5cm刹车→DONE */
static void t3tick(void){float tgt;uint8_t sw=0;uint32_t el=ts.run-t3_seg_start;
    if(t3_seg==0){tgt=t3_tgt1;
        if(ts.bv&&ts.bp>=4.5f){sw=1;}}                /* 球过4.5cm → mid */
    else if(t3_seg==1){tgt=t3_mid;
        if(ts.bv&&ts.bp<=t3_mid+0.5f)sw=2;             /* 球到mid附近 → ramp */
        else if(el>=2000)sw=2;}                         /* 或2s超时 */
    else if(t3_seg==2){float p=(float)el/(float)t3_ramp;if(p>1.0f)p=1.0f;
        tgt=t3_mid+(t3_tgt2-t3_mid)*p;                  /* ramp: mid→final */
        if(ts.bv&&ts.bp<=-4.5f&&p>0.3f)sw=3;}           /* 球过-4.5cm → 刹车 */
    else{MechBalance_SetDirectAngle(t3_brake);           /* 刹车: 锁电机角度 */
        if(el>=500)sw=4;}                                /* 500ms后 → DONE */
    if(sw){t3_seg=sw;t3_seg_start=ts.run;ts.sms=0;
        if(sw==1){diag=DIAG_T3_SEG;dt=1;dsp=t3_mid;}
        else if(sw==2){diag=DIAG_T3_SEG;dt=2;dsp=t3_tgt2;}
        else if(sw==3){diag=DIAG_T3_SEG;dt=3;dsp=t3_brake;}
        else{ts.st=STATE_DONE;diag=DIAG_T3_DONE;}ts.pend|=TX_PENDING_STATE;}
    if(t3_seg<3)ts.sp=tgt;}

void TaskCtrl_Init(void){rx.s=0;rx0.s=0;ts.tid=0;ts.st=STATE_IDLE;ts.sp=0;ts.run=0;ts.bp=0;ts.sms=0;ts.bv=0;ts.esp=0;ts.ovf=0;ts.link=0;ts.lrx=0;ts.pend=0;qh=0;qt=0;tms=0;lhb=0;lst=0;DL_UART_Main_enableInterrupt(UART_1_INST,DL_UART_MAIN_INTERRUPT_RX);}
void TaskCtrl_Tick5ms(void){tms+=5;if(w_age_ms<=0xFFFFFFFFU-5U)w_age_ms+=5U;execq();if(ts.st==STATE_RUNNING&&ts.link&&(tms-ts.lrx)>LINK_TIMEOUT_MS){ts.st=STATE_FAULT;ts.sms=0;ts.pend|=TX_PENDING_STATE;}if(ts.st==STATE_RUNNING)ts.run+=5;if(ts.st==STATE_RUNNING&&ts.tid==TASK_3){t3tick();}else if(ts.st==STATE_RUNNING&&ts.tid==TASK_6){ftchk(); /* 仅第6题自动判定DONE, T4/T5由主控发停止 */}if(tms-lhb>=HB_INTERVAL){ts.pend|=TX_PENDING_HB;lhb=tms;}if(tms-lst>=STATE_INTERVAL){ts.pend|=TX_PENDING_STATE;lst=tms;}}
void TaskCtrl_Process(void){uint32_t pk;uint8_t pe,tid,st;float bp;pk=__get_PRIMASK();__disable_irq();pe=ts.pend;ts.pend=0;tid=ts.tid;st=ts.st;bp=ts.bp;if(!pk)__enable_irq();if(pe&TX_PENDING_STATE)sstate(tid,st,bp);if(pe&TX_PENDING_HB){uint8_t d=0;sframe(0xFF,&d,1);}if(rx_diag){rx_diag=0;uart_puts("[RX] cmd=");uart_putu(rx_cmd);uart_puts(" task=");uart_putu(rx_task);uart_puts(" param=");uart_puti(rx_param);uart_puts("\r\n");}if(diag){uint8_t dc=diag;diag=0;switch(dc){case DIAG_TASK_OK:uart_puts("[OK] Task ");uart_putu(dt);uart_puts("\r\n");break;case DIAG_TASK_ERR:uart_puts("[ERR] Bad task\r\n");break;case DIAG_T3_START:uart_puts("[T3] START sp=");uart_putf(t3_tgt1,1);uart_puts(" -> ball>=5 -> ");uart_putf(t3_mid,1);uart_puts(" -> ");uart_putf(t3_tgt2,1);uart_puts("\r\n");break;case DIAG_START:uart_puts("[OK] Start\r\n");break;case DIAG_START_T4:uart_puts("[OK] Start T");uart_putu(dt);uart_puts(" (sp=0)\r\n");break;case DIAG_NO_TASK:uart_puts("[ERR] No task\r\n");break;case DIAG_STOP:uart_puts("[OK] Stop\r\n");break;case DIAG_SP:uart_puts("[OK] SP=");uart_putf(dsp,2);uart_puts("\r\n");break;case DIAG_NOT_T6:uart_puts("[ERR] Not T6\r\n");break;case DIAG_T3_DONE:uart_puts("[T3] DONE\r\n");break;case DIAG_T3_SEG:uart_puts("[T3] SEG");uart_putu(dt);uart_puts(" sp=");uart_putf(dsp,1);uart_puts("\r\n");break;}}}
void TaskCtrl_FeedByte(uint8_t b){uint8_t t,l,*d;if(aa55(&rx0,b,&t,&l,&d)){if(t==0x40&&l==4)pushq(d[0],d[1],(int16_t)((uint16_t)d[2]|((uint16_t)d[3]<<8)));}}
void TaskCtrl_ReportBallPos(float cm){ts.bp=cm;ts.bv=1;}
void TaskCtrl_ReportBallInvalid(void){ts.bv=0;ts.sms=0;}
void TaskCtrl_ReportFault(void){if(ts.st!=STATE_FAULT){ts.st=STATE_FAULT;ts.sms=0;ts.pend|=TX_PENDING_STATE;}}
uint8_t TaskCtrl_GetInfo(TaskInfo_t*inf){if(!inf)return 0;inf->task_id=ts.tid;inf->state=ts.st;inf->setpoint_cm=ts.sp;inf->run_time_ms=ts.run;return 1;}
void UART1_IRQHandler(void){while(!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)){uint8_t b=(uint8_t)DL_UART_Main_receiveData(UART_1_INST),t,l,*d;if(aa55(&rx,b,&t,&l,&d)){ts.lrx=tms;ts.link=1;if(t==0x40&&l==4){rx_cmd=d[0];rx_task=d[1];rx_param=(int16_t)((uint16_t)d[2]|((uint16_t)d[3]<<8));rx_diag=1U;pushq(d[0],d[1],(int16_t)((uint16_t)d[2]|((uint16_t)d[3]<<8)));}else if(t==0x42&&l==8){uint32_t pk=__get_PRIMASK();__disable_irq();w_enc_lp=w_enc_l;w_enc_rp=w_enc_r;w_enc_tsp=w_enc_ts;w_enc_l=(uint32_t)((uint16_t)d[0]|((uint16_t)d[1]<<8));w_enc_r=(uint32_t)((uint16_t)d[2]|((uint16_t)d[3]<<8));w_enc_ts=(uint32_t)d[4]|((uint32_t)d[5]<<8)|((uint32_t)d[6]<<16)|((uint32_t)d[7]<<24);if(w_enc_tsp&&w_enc_ts>w_enc_tsp){float dt=(float)(w_enc_ts-w_enc_tsp)/1000.0f;/* 16位模运算: 65535->0得+1, 消除回绕错误 */int32_t dl32=(int32_t)(int16_t)((uint16_t)w_enc_l-(uint16_t)w_enc_lp);int32_t dr32=(int32_t)(int16_t)((uint16_t)w_enc_r-(uint16_t)w_enc_rp);if(dt>=0.01f&&dt<=0.5f&&dl32>-500&&dl32<500&&dr32>-500&&dr32<500){float dl=(float)dl32*WENC_SCALE;float dr=(float)dr32*WENC_SCALE;float sp=(dl+dr)*0.5f/dt;float a=(sp-w_speed_ms)/dt;/* 速度基准总是更新, 异常帧不发布加速度但基准保持新鲜 */w_speed_ms=sp;w_age_ms=0U;if(a>-3.0f&&a<3.0f){w_accel_ms2=a;w_valid=1U;w_frame++;}}else{/* 增量/时间戳异常帧: 不更新基准, 等待下帧重建 */}}if(!pk)__enable_irq();}}}}
uint32_t TaskCtrl_GetWheelFrame(void){return w_frame;}
uint8_t TaskCtrl_GetWheelAccel(float*ax_mps2){if(!ax_mps2||!w_valid)return 0U;if(w_age_ms>500U)return 0U;*ax_mps2=w_accel_ms2;return 1U;}
/* 第3题参数设置 + 一键启动 */
void TaskCtrl_SetT3Target1(float cm){t3_tgt1=cm;}
void TaskCtrl_SetT3Target2(float cm){t3_tgt2=cm;}
void TaskCtrl_SetT3Mid(float cm){t3_mid=cm;}
void TaskCtrl_SetT3Ramp(uint32_t ms){if(ms>=200&&ms<=5000)t3_ramp=ms;}
void TaskCtrl_SetT3Tol(float cm){if(cm>0.1f&&cm<=5.0f)t3_tol=cm;}
void TaskCtrl_SetT3Brake(float deg){t3_brake=deg;}
void TaskCtrl_StartTask3(void){ts.tid=TASK_3;ts.st=STATE_RUNNING;ts.sp=0;ts.run=0;ts.sms=0;ts.bv=0;t3_seg=0;t3_seg_start=0;diag=DIAG_T3_START;ts.pend|=TX_PENDING_STATE;}
void TaskCtrl_StopTask(void){ts.st=STATE_IDLE;ts.sp=0;ts.run=0;ts.sms=0;ts.bv=0;diag=DIAG_STOP;ts.pend|=TX_PENDING_STATE;}
float TaskCtrl_GetT3Target1(void){return t3_tgt1;}
float TaskCtrl_GetT3Target2(void){return t3_tgt2;}
