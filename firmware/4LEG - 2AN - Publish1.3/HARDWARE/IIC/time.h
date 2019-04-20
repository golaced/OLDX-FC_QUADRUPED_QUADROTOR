#ifndef _TIME_H_
#define _TIME_H_

#include "stm32f4xx.h"

void TIM_INIT(void);
void sys_time(void);

u16 Get_Time(u8,u16,u16);

float Get_Cycle_T(u8 );

void Cycle_Time_Init(void);

extern volatile uint32_t sysTickUptime;
extern int time_1h,time_1m,time_1s,time_1ms;

void Delay_us(uint32_t);
void Delay_ms(uint32_t);
void SysTick_Configuration(void);
uint32_t GetSysTime_us(void);


void Initial_Timer_SYS(void);
uint32_t micros(void);


#define GET_T_BRAIN 0
#define GET_T_LEG1 1
#define GET_T_LEG2 2
#define GET_T_LEG3 3
#define GET_T_LEG4 4
#define GET_T_TRIG 5
#define GET_T_FUSHION 6
#endif
