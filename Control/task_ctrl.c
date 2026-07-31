/** task_ctrl.c - 赛题状态机 + UART1/AA55协议 + 第3题轨迹 */
#include "task_ctrl.h"
#include "ti_msp_dl_config.h"
#include "board.h"

#define TASK3_SETTLE_MS 500U
#define TASK3_SETTLE_TOL 1.0f
#define TARGET_SETTLE_MS 500U
#define TARGET_SETTLE_TOL 1.0f
#define HB_INTERVAL 1000U
#define STATE_INTERVAL 100U
#define TASK3_TOTAL_MS 5000U
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
/* 诊断输出暂存: exec在5ms中断里只置标志, 由主循环TaskCtrl_Process输出, 避免阻塞中断 */
static volatile uint8_t diag,dt;static volatile float dsp;
#define DIAG_TASK_OK 1U
#define DIAG_TASK_ERR 2U
#define DIAG_START 3U
#define DIAG_START_T4 4U
#define DIAG_NO_TASK 5U
#define DIAG_STOP 6U
#define DIAG_SP 7U
#define DIAG_NOT_T6 8U

static uint16_t crc16_update(uint16_t c,uint8_t b){c^=b;for(uint8_t i=0;i<8;i++)c=(c&1)?(c>>1)^0xA001:c>>1;return c;}
static uint16_t crc16_frame(uint8_t t,uint8_t l,const uint8_t*d){uint16_t c=0;c=crc16_update(c,t);c=crc16_update(c,l);for(uint8_t i=0;i<l;i++)c=crc16_update(c,d[i]);return c;}
/* 单一AA55解析器, 消除rx/rx0重复; 收到完整有效帧返回1并通过out参数输出 */
static uint8_t aa55(AA55_t*st,uint8_t b,uint8_t*t,uint8_t*l,uint8_t**d){uint16_t c;switch(st->s){case 0:if(b==0xAA)st->s=1;break;case 1:if(b==0x55)st->s=2;else st->s=(b==0xAA)?1:0;break;case 2:st->t=b;st->s=3;break;case 3:st->l=b;st->i=0;if(st->l>8){st->s=0;break;}st->s=st->l?4:5;break;case 4:st->d[st->i++]=b;if(st->i>=st->l)st->s=5;break;case 5:st->c=b;st->s=6;break;default:c=(uint16_t)st->c|((uint16_t)b<<8);st->s=0;if(c==crc16_frame(st->t,st->l,st->d)){*t=st->t;*l=st->l;*d=st->d;return 1;}break;}return 0;}
static void u1tx(const uint8_t*d,uint16_t n){for(uint16_t i=0;i<n;i++){while(DL_UART_Main_isTXFIFOFull(UART_1_INST)){}DL_UART_Main_transmitData(UART_1_INST,d[i]);}}
static void sframe(uint8_t t,const uint8_t*d,uint8_t l){uint8_t b[16];uint16_t c;if(l>8)return;b[0]=0xAA;b[1]=0x55;b[2]=t;b[3]=l;for(uint8_t i=0;i<l;i++)b[4+i]=d[i];c=crc16_frame(t,l,d);b[4+l]=(uint8_t)c;b[5+l]=(uint8_t)(c>>8);u1tx(b,l+6);}
static void sstate(uint8_t tid,uint8_t st,float bp){uint8_t d[6];float bc=bp*100.0f;if(bc>32767)bc=32767;if(bc<-32768)bc=-32768;int16_t b=(int16_t)bc;d[0]=tid;d[1]=st;d[2]=(uint8_t)(uint16_t)b;d[3]=(uint8_t)((uint16_t)b>>8);d[4]=0;d[5]=0;sframe(0x41,d,6);}

static uint8_t popq(TCmd_t*c){uint32_t pk;uint8_t t;pk=__get_PRIMASK();__disable_irq();t=qt;if(t==qh){if(!pk)__enable_irq();return 0;}c->cmd=q[t].cmd;c->task=q[t].task;c->param=q[t].param;qt=(uint8_t)((t+1)&CMD_QUEUE_MASK);if(!pk)__enable_irq();return 1;}
static void pushq(uint8_t cmd,uint8_t task,int16_t param){uint32_t pk;uint8_t h,n;if(cmd<1||cmd>4)return;pk=__get_PRIMASK();__disable_irq();if(cmd==3){ts.esp=1;if(!pk)__enable_irq();return;}h=qh;n=(uint8_t)((h+1)&CMD_QUEUE_MASK);if(n==qt){ts.ovf=1;}else{q[h].cmd=cmd;q[h].task=task;q[h].param=param;qh=n;}if(!pk)__enable_irq();}

static void exec(const TCmd_t*c){switch(c->cmd){case 1:if(c->task>=3&&c->task<=6){ts.tid=c->task;ts.st=STATE_IDLE;ts.sp=0;ts.run=0;ts.sms=0;ts.bv=0;diag=DIAG_TASK_OK;dt=c->task;}else diag=DIAG_TASK_ERR;break;case 2:if(ts.tid==TASK_3||ts.tid==TASK_6){ts.st=STATE_RUNNING;ts.run=0;ts.sms=0;diag=DIAG_START;}else if(ts.tid==TASK_4||ts.tid==TASK_5){ts.sp=0;ts.st=STATE_RUNNING;ts.run=0;ts.sms=0;diag=DIAG_START_T4;dt=ts.tid;}else diag=DIAG_NO_TASK;break;case 3:ts.st=STATE_IDLE;ts.sp=0;ts.run=0;ts.sms=0;ts.bv=0;diag=DIAG_STOP;break;case 4:if(ts.tid==TASK_6){ts.sp=(float)c->param/100.0f;ts.sms=0;diag=DIAG_SP;dsp=ts.sp;}else diag=DIAG_NOT_T6;break;}ts.pend|=TX_PENDING_STATE;}

static void execq(void){TCmd_t c;uint32_t pk;uint8_t es,ov;pk=__get_PRIMASK();__disable_irq();es=ts.esp;ov=ts.ovf;ts.esp=0;ts.ovf=0;if(es||ov){qt=qh;}if(!pk)__enable_irq();if(es){c.cmd=3;c.task=ts.tid;c.param=0;exec(&c);return;}if(ov){TaskCtrl_ReportFault();return;}while(popq(&c))exec(&c);}

static float t3traj(uint32_t ms){float p;if(ms>=TASK3_TOTAL_MS)return -5.0f;if(ms<2000)p=5.0f*(float)ms/2000.0f;else if(ms<2500)p=5.0f-5.0f*(float)(ms-2000)/500.0f;else if(ms<4500)p=-5.0f*(float)(ms-2500)/2000.0f;else p=-5.0f;return p;}
static void t3chk(void){if(ts.run<4500)return;float e=ts.bp-(-5.0f);if(e<0)e=-e;if(ts.bv&&e<TASK3_SETTLE_TOL){ts.sms+=5;if(ts.sms>=TASK3_SETTLE_MS){ts.st=STATE_DONE;}}else ts.sms=0;}
static void ftchk(void){float e=ts.bp-ts.sp;if(e<0)e=-e;if(ts.bv&&e<TARGET_SETTLE_TOL){ts.sms+=5;if(ts.sms>=TARGET_SETTLE_MS){ts.st=STATE_DONE;}}else ts.sms=0;}

void TaskCtrl_Init(void){rx.s=0;rx0.s=0;ts.tid=0;ts.st=STATE_IDLE;ts.sp=0;ts.run=0;ts.bp=0;ts.sms=0;ts.bv=0;ts.esp=0;ts.ovf=0;ts.link=0;ts.lrx=0;ts.pend=0;qh=0;qt=0;tms=0;lhb=0;lst=0;DL_UART_Main_enableInterrupt(UART_1_INST,DL_UART_MAIN_INTERRUPT_RX);}
void TaskCtrl_Tick5ms(void){tms+=5;execq();if(ts.st==STATE_RUNNING&&ts.link&&(tms-ts.lrx)>LINK_TIMEOUT_MS){ts.st=STATE_FAULT;ts.sms=0;ts.pend|=TX_PENDING_STATE;}if(ts.st==STATE_RUNNING)ts.run+=5;if(ts.st==STATE_RUNNING&&ts.tid==TASK_3){ts.sp=t3traj(ts.run);t3chk();}else if(ts.st==STATE_RUNNING&&ts.tid==TASK_6){ftchk();}if(tms-lhb>=HB_INTERVAL){ts.pend|=TX_PENDING_HB;lhb=tms;}if(tms-lst>=STATE_INTERVAL){ts.pend|=TX_PENDING_STATE;lst=tms;}}
void TaskCtrl_Process(void){uint32_t pk;uint8_t pe,tid,st;float bp;pk=__get_PRIMASK();__disable_irq();pe=ts.pend;ts.pend=0;tid=ts.tid;st=ts.st;bp=ts.bp;if(!pk)__enable_irq();if(pe&TX_PENDING_STATE)sstate(tid,st,bp);if(pe&TX_PENDING_HB){uint8_t d=0;sframe(0xFF,&d,1);}if(diag){uint8_t dc=diag;diag=0;switch(dc){case DIAG_TASK_OK:uart_puts("[OK] Task ");uart_putu(dt);uart_puts("\r\n");break;case DIAG_TASK_ERR:uart_puts("[ERR] Bad task\r\n");break;case DIAG_START:uart_puts("[OK] Start\r\n");break;case DIAG_START_T4:uart_puts("[OK] Start T");uart_putu(dt);uart_puts(" (sp=0)\r\n");break;case DIAG_NO_TASK:uart_puts("[ERR] No task\r\n");break;case DIAG_STOP:uart_puts("[OK] Stop\r\n");break;case DIAG_SP:uart_puts("[OK] SP=");uart_putf(dsp,2);uart_puts("\r\n");break;case DIAG_NOT_T6:uart_puts("[ERR] Not T6\r\n");break;}}}
void TaskCtrl_FeedByte(uint8_t b){uint8_t t,l,*d;if(aa55(&rx0,b,&t,&l,&d)){if(t==0x40&&l==4)pushq(d[0],d[1],(int16_t)((uint16_t)d[2]|((uint16_t)d[3]<<8)));}}
void TaskCtrl_ReportBallPos(float cm){ts.bp=cm;ts.bv=1;}
void TaskCtrl_ReportBallInvalid(void){ts.bv=0;ts.sms=0;}
void TaskCtrl_ReportFault(void){if(ts.st!=STATE_FAULT){ts.st=STATE_FAULT;ts.sms=0;ts.pend|=TX_PENDING_STATE;}}
uint8_t TaskCtrl_GetInfo(TaskInfo_t*inf){if(!inf)return 0;inf->task_id=ts.tid;inf->state=ts.st;inf->setpoint_cm=ts.sp;inf->run_time_ms=ts.run;return 1;}
void UART1_IRQHandler(void){while(!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)){uint8_t b=(uint8_t)DL_UART_Main_receiveData(UART_1_INST),t,l,*d;if(aa55(&rx,b,&t,&l,&d)){ts.lrx=tms;ts.link=1;if(t==0x40&&l==4)pushq(d[0],d[1],(int16_t)((uint16_t)d[2]|((uint16_t)d[3]<<8)));}}}
