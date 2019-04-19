#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_
#include "stm32f4xx.h"
u8 PWM_Out_Init(uint16_t hz);
u8 PWM_AUX_Out_Init(uint16_t hz);//50Hz
void Set_DJ_PWM(void);
void LEG_POWER(u8 sel);
extern int dj_out[12];
typedef struct 
{ u16 pwm_tem[2];
	int flag[2];
	u16 init[2];
	u16 max[2];
	u16 min[2];
	float att[2],att_ctrl[2],att_off[2];
	float pwm_per_dig[2];
	float ero[2],ero_reg[2];
}AUX_S;
extern AUX_S aux;

#endif

