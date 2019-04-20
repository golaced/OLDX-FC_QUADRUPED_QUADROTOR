
#include "time.h"
#include "include.h"

volatile uint32_t sysTickUptime = 0;

#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)

void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

uint32_t GetSysTime_us(void) 
{
//	register uint32_t ms;
//	u32 value;
//	ms = sysTickUptime;
//	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return micros();
}

void Delay_us(uint32_t us)
{
 delay_us(1);
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
        Delay_us(1000);
}

int time_1h,time_1m,time_1s,time_1ms;


volatile float Cycle_T[GET_TIME_NUM][3];

enum
{
	NOW = 0,
	OLD,
	NEW,
};

float Get_Cycle_T(u8 item)	
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
	Cycle_T[item][NOW] = (float)micros()/1000000.0f; //GetSysTime_us()/1000000.0f; //本次的时间
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//间隔的时间（周期）
	return Cycle_T[item][NEW];
}

void Cycle_Time_Init()
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}

}


#define TIME_SYS TIM5
#define TIME_SYS_RCC RCC_APB1Periph_TIM5
/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim2  Tim3 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
*******************************************************************************/
void Initial_Timer_SYS(void)
{ RCC_APB1PeriphClockCmd(TIME_SYS_RCC, ENABLE);
 // RCC->APB1ENR |= 0x0008;	//使能TIM5时钟
	TIME_SYS->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIME_SYS->CR2 = 0x0000;
	TIME_SYS->CNT = 0x0000;
	TIME_SYS->ARR = 0xFFFFFFFF;
	TIME_SYS->PSC = 84 - 1;	//分出 1M 的时钟 保证每个周期为1us
	TIME_SYS->EGR = 0x0001;
	TIME_SYS->CR1 |= 0x0001; //启动定时器           
}


/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIME_SYS->CNT;
 	return temp;
}
