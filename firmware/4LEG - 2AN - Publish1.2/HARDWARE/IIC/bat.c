#include "bat.h"
#include "include.h"		 
#include "my_math.h"	
#include "vmc.h"
//初始化ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PA5 通道5;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
#if defined(LEG_USE_AD)
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;//PA5 通道5;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
#endif
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
 	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
 
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	

}				  
#define ADC1_DR_Address    ((uint32_t)0x4001204C) 
uint16_t ADC_Value[5]={0};//AD??
/*??????? ????*/
void ADC_Configuration(void)
{

 ADC_InitTypeDef       ADC_InitStructure; 
  ADC_CommonInitTypeDef ADC_CommonInitStructure; 
  DMA_InitTypeDef       DMA_InitStructure; 
  GPIO_InitTypeDef      GPIO_InitStructure; 

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 

 DMA_DeInit(DMA2_Stream0); 
  
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;   
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address; 
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value; 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; 
  DMA_InitStructure.DMA_BufferSize = 5; 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;          
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
  DMA_Init(DMA2_Stream0, &DMA_InitStructure); 
  DMA_Cmd(DMA2_Stream0, ENABLE); 


  /* Configure ADC1 Channel6 pin as analog input ******************************/ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  /* ADC Common Init **********************************************************/ 
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; 
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; 
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
  ADC_CommonInit(&ADC_CommonInitStructure); 

  /* ADC1 Init ****************************************************************/ 
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; 
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None ; 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
  ADC_InitStructure.ADC_NbrOfConversion =5; 
  ADC_Init(ADC1, &ADC_InitStructure); 

/* ADC1 regular channe6 configuration *************************************/ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_3Cycles); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_3Cycles); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 4, ADC_SampleTime_3Cycles); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 5, ADC_SampleTime_15Cycles); 
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); 
	ADC_DMACmd(ADC1, ENABLE); 
	ADC_Cmd(ADC1, ENABLE); 
  ADC_SoftwareStartConv(ADC1);	
}
//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
#if USE_VER_6
float k_ad=0.00875;
#else
float k_ad=12.38/1370;
#endif
float Get_Adc(u8 ch)   
{ 
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_112Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1)*k_ad;	//返回最近一次ADC1规则组的转换结果
}

float press_leg_end[5];
ESO_X flt_ground[5];
char ground_v[5],ground_vad[5];
float ground_press_max[5]={0,  0.1,  0.1,  4, 7.6};
float ground_press_min[5]={0,  0.1,  0.1,  2.4, 1.9};
float k_check_force=0.28;
float k_check_dforce=0.23;
void get_leg_press(float dt)
{   static float press_leg_endr[5];
    char i;
	  static char init;
	  if(!init){init=1;flt_ground[1].r0=420;}
		DigitalLPF(Get_Adc(1), &press_leg_end[1], 66, dt);
		DigitalLPF(Get_Adc(4), &press_leg_end[2], 66, dt);
		DigitalLPF(Get_Adc(5), &press_leg_end[3], 66, dt);
		DigitalLPF(Get_Adc(6), &press_leg_end[4], 66, dt);
//		press_leg_end[1]=Get_Adc(1);
//		press_leg_end[2]=Get_Adc(4);
//		press_leg_end[3]=Get_Adc(5);
//		press_leg_end[4]=Get_Adc(6);
		
		for(i=0;i<4;i++)
			vmc[i].ground_force[0]=flt_ground[i+1].v1;
		
	  for(i=2;i<5;i++){
			flt_ground[i].h0=flt_ground[1].h0=dt;
			flt_ground[i].r0=flt_ground[1].r0;
	  }
	  for(i=1;i<5;i++){
			OLDX_SMOOTH_IN_ESOX(&flt_ground[i],
			LIMIT(press_leg_end[i]-vmc_all.param.ground_force[i-1][0],0,ABS(vmc_all.param.ground_force[i-1][1]-vmc_all.param.ground_force[i-1][0]))
			/ABS(vmc_all.param.ground_force[i-1][1]-vmc_all.param.ground_force[i-1][0]));
			vmc[i-1].ground_force[1]=flt_ground[i].v2/10;
			if((vmc[i-1].ground_force[1]>k_check_dforce&&0)||vmc[i-1].ground_force[0]>k_check_force)
				ground_vad[i]=1;
			else
				ground_vad[i]=0;
		}
		
		press_leg_endr[1]=press_leg_end[1];
		press_leg_endr[2]=press_leg_end[2];
		press_leg_endr[3]=press_leg_end[3];
		press_leg_endr[4]=press_leg_end[4];
}	

//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		Delay_ms(5);
	}
	return temp_val/times;
} 
	 
BAT bat;
void Bat_protect(float dt)
{
static u8 state;
static u16 cnt[5];	
static  float temp,temp1;
	#if defined(LEG_USE_AD)
	 get_leg_press(dt);
	#endif
	switch(state)
	{
		case 0:
			  if(Get_Adc(7)!=0){
			  temp+=Get_Adc(7);
				cnt[0]++;	
				}
				if(cnt[0]>20){
					state=1;
					temp1=temp/cnt[0];
					if(temp1>11.1-1&&temp1<12.6+1)
					  bat.bat_s=3;
					else if(temp1>14.8-1&&temp1<16.4+1)
						bat.bat_s=4;
				  else if(temp1>3.7*2-1&&temp1<4.2*2+1)
						bat.bat_s=2;
					else
						bat.bat_s=2;
					bat.full=4.25*bat.bat_s;
					temp=0;
				}	  
		break;
	  case 1:
			 if(Get_Adc(7)!=0){
			  bat.origin=Get_Adc(7);
				temp+=bat.origin;
				cnt[1]++;	
				}
				if(cnt[1]>2/dt){		
					bat.average=temp/cnt[1];cnt[1]=0;
					bat.percent=LIMIT((bat.average-bat.bat_s*3.12)/(bat.full-bat.bat_s*3.12),0,0.99);

          temp=0;		
          switch(bat.bat_s){
            case 3:
						 	if(bat.percent<0.03)//(bat.average<11.1-0.6||bat.average<11.1-0.6+(12.6-11.1)*bat.protect_percent)
							bat.low_bat=1;	
							else			
							bat.low_bat=0;
						break;
						case 4:
						if(bat.percent<0.6)//bat.average<14.8-0.6||bat.average<14.8-0.6+(16.4-14.8)*bat.protect_percent)
							bat.low_bat_cnt++;
						else
							bat.low_bat_cnt=0;
						 if(bat.low_bat_cnt>2/0.05)
						  bat.low_bat=1;	
							else			
							bat.low_bat=0;
						break;;
 					}
				}	  
		break;
	
	}
}