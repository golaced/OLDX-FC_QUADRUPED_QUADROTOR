#ifndef _PWM_IN_H_
#define _PWM_IN_H_

#include "stm32f4xx.h"

void PWM_IN_Init(void);
void _TIM3_IRQHandler(void);
void _TIM5_IRQHandler(void);

extern u32 Rc_Pwm_In_mine[8],Rc_Pwm_Inr_mine[8],Rc_Pwm_Out_mine[8];
typedef struct 
{
	u32 max;
	u32 min;
	u32 T;
	u32 duty;
	u8 CALIBRATE;
	u8 sel,sel_in;
	u8 cal_cycle;
	u16 hz;
}PWMIN;

extern PWMIN pwmin;
void PWIN_CAL(void);
#define DEAD_PWM 20
#endif
