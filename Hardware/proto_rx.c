#include "proto_rx.h"
#include "ti_msp_dl_config.h"
#define RX_MAX 64U
#define BALL_DATA_TIMEOUT_MS 100U
typedef struct{uint8_t s,t,l,i,d[RX_MAX];uint16_t c;volatile BallData_t b;}PR_t;
static PR_t r;
static uint16_t crc16f(uint8_t t,uint8_t l,const uint8_t*d){uint16_t c=0;uint8_t i,j;c^=t;for(j=0;j<8;j++)c=(c&1)?(c>>1)^0xA001:c>>1;c^=l;for(j=0;j<8;j++)c=(c&1)?(c>>1)^0xA001:c>>1;for(i=0;i<l;i++){c^=d[i];for(j=0;j<8;j++)c=(c&1)?(c>>1)^0xA001:c>>1;}return c;}
void ProtoRx_Init(void){r.s=0;r.b.age_ms=0xFFFFFFFFU;r.b.frame_id=0U;DL_UART_Main_enableInterrupt(UART_2_INST,DL_UART_MAIN_INTERRUPT_RX);}
void ProtoRx_ProcessByte(uint8_t b){uint16_t c;switch(r.s){case 0:if(b==0xAA)r.s=1;break;case 1:if(b==0x55)r.s=2;else r.s=(b==0xAA)?1:0;break;case 2:r.t=b;r.s=3;break;case 3:r.l=b;r.i=0;if(r.l>RX_MAX){r.s=0;break;}r.s=r.l?4:5;break;case 4:r.d[r.i++]=b;if(r.i>=r.l)r.s=5;break;case 5:r.c=b;r.s=6;break;default:c=(uint16_t)r.c|((uint16_t)b<<8);if(c==crc16f(r.t,r.l,r.d)&&r.t==0x30&&r.l==8){uint32_t pk=__get_PRIMASK();__disable_irq();r.b.position_centi_cm=(int16_t)((uint16_t)r.d[0]|((uint16_t)r.d[1]<<8));r.b.confidence=r.d[2];r.b.status=r.d[3];r.b.age_ms=0;r.b.frame_id++;if(r.b.frame_id==0U)r.b.frame_id=1U;if(!pk)__enable_irq();}r.s=0;break;}}
void ProtoRx_Tick(uint32_t m){uint32_t pk=__get_PRIMASK();__disable_irq();if(r.b.age_ms!=0xFFFFFFFFU){r.b.age_ms=(m>0xFFFFFFFFU-r.b.age_ms)?0xFFFFFFFFU:(r.b.age_ms+m);}if(!pk)__enable_irq();}
uint8_t ProtoRx_GetBall(BallData_t*b){BallData_t s;uint32_t pk=__get_PRIMASK();__disable_irq();s=r.b;if(!pk)__enable_irq();if(!b)return 0;if(s.age_ms>BALL_DATA_TIMEOUT_MS||s.status==0)return 0;*b=s;return 1;}
void UART2_IRQHandler(void){while(!DL_UART_Main_isRXFIFOEmpty(UART_2_INST))ProtoRx_ProcessByte((uint8_t)DL_UART_Main_receiveData(UART_2_INST));}
