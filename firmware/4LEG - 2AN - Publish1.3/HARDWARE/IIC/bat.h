#ifndef __ADC_H
#define __ADC_H	
#include "stm32f4xx.h" 
 							   
void Adc_Init(void); 				//ADC通道初始化
float Get_Adc(u8 ch); 				//获得某个通道值 
u16 Get_Adc_Average(u8 ch,u8 times);//得到某个通道给定次数采样的平均值  
void ADC_Configuration(void);
typedef struct 
{ 
 float origin;
 float average;
 u8 bat_s;
 u8 low_bat;
 u16 low_bat_cnt;
 float percent;
 float protect_percent;
 float full;
}BAT;

extern BAT bat; 
void Bat_protect(float dt);
extern float press_leg_end[5],spd_ground[5];
extern char ground_v[5],ground_vad[5];
#endif 















