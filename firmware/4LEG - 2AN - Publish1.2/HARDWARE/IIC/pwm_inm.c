#include "led_fc.h"
#include "pwm_in.h"
#include "rc.h"
#include "include.h"
#include "pwm_out.h"
#include "filter.h"
#include "circle.h"
u32 Rc_Pwm_Inr_mine[8];
u32 Rc_Pwm_In_mine[8],Rc_Pwm_Out_mine[8]={1500,1500,1500,1500,1500,1500,1500,1500};
PWMIN pwmin;
u8 PER_SCAL=2;
void PWM_IN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM5, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOD, ENABLE);
	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_P;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_S;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_P;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_S;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);	 
	
//////////////////////////////////////////////////////////////////////////////////////////////
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;//|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
  
	TIM3->PSC = (168/PER_SCAL)-1;
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	//TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
/////////////////////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
  
	TIM5->PSC = (168/PER_SCAL)-1;
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM5, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);

}


void PWM_IN_Init_FOR_CYCLE(void)
{
  TIM_ICInitTypeDef  TIM_ICInitStructure;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  
}

void PWM_IN_Init_FOR_DUTY(void)
{
  TIM_ICInitTypeDef  TIM_ICInitStructure;

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//  TIM_ICInit(TIM3, &TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  
}
u32 cnt_sample1,now_dj[8],lastUpdate_dj[8];
void TIM3_IRQHandler(void)	
{
	static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	

	
	if(TIM3->SR & TIM_IT_CC1) 
	{
		TIM3->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM3->SR = ~TIM_FLAG_CC1OF;
		if(GPIOA->IDR & GPIO_Pin_6)
		{
			 lastUpdate_dj[4] =  GetSysTime_us();		//temp_cnt2 = TIM_GetCapture2(TIM5);//temp_cnt1 = TIM_GetCapture1(TIM3);
		}
		else
		{
			
			 now_dj[4] = GetSysTime_us();  //读取时间
				 if( now_dj[4] <lastUpdate_dj[4]){Rc_Pwm_In_mine[4] =  ( now_dj[4]  + (0xffff- lastUpdate_dj[4])) ;}
				 else	{ Rc_Pwm_In_mine[4] =  ( now_dj[4]  - lastUpdate_dj[4]); }
//			temp_cnt1_2 = TIM_GetCapture1(TIM3);
//			if(temp_cnt1_2>=temp_cnt1)
//				Rc_Pwm_In[4] = temp_cnt1_2-temp_cnt1;
//			else
//				Rc_Pwm_In[4] = 0xffff-temp_cnt1+temp_cnt1_2+1;
		}
	}
//	if(TIM3->SR & TIM_IT_CC2) 
//	{
//		TIM3->SR = ~TIM_IT_CC2;
//		TIM3->SR = ~TIM_FLAG_CC2OF;
//		if(GPIOA->IDR & GPIO_Pin_7)
//		{
//			 lastUpdate_dj[5] =  GetSysTime_us();		//temp_cnt2 = TIM_GetCapture2(TIM5);//temp_cnt2 = TIM_GetCapture2(TIM3);
//		}
//		else
//		{
////			temp_cnt2_2 = TIM_GetCapture2(TIM3);
////			if(temp_cnt2_2>=temp_cnt2)
////				Rc_Pwm_In[5] = temp_cnt2_2-temp_cnt2;
////			else
////				Rc_Pwm_In[5] = 0xffff-temp_cnt2+temp_cnt2_2+1;
//			 now_dj[5] = GetSysTime_us();  //读取时间
//				 if( now_dj[5] <lastUpdate_dj[5]){Rc_Pwm_In_mine[5] =  (( now_dj[5]  + (0xffff- lastUpdate_dj[5])) );}
//				 else	{ Rc_Pwm_In_mine[5] =  (( now_dj[5]  - lastUpdate_dj[5])); }
//		}
//	}
	if(TIM3->SR & TIM_IT_CC3) 
	{
		TIM3->SR = ~TIM_IT_CC3;
		TIM3->SR = ~TIM_FLAG_CC3OF;
		if(GPIOB->IDR & GPIO_Pin_0)
		{
			 lastUpdate_dj[6] =  GetSysTime_us();		//temp_cnt2 = TIM_GetCapture2(TIM5);//temp_cnt3 = TIM_GetCapture3(TIM3);
		}
		else
		{
//			temp_cnt3_2 = TIM_GetCapture3(TIM3);
//			if(temp_cnt3_2>=temp_cnt3)
//				Rc_Pwm_In_mine[6] = temp_cnt3_2-temp_cnt3;
//			else
//				Rc_Pwm_In_mine[6] = 0xffff-temp_cnt3+temp_cnt3_2+1;
			 now_dj[6] = GetSysTime_us();  //读取时间
				 if( now_dj[6] <lastUpdate_dj[6]){Rc_Pwm_In_mine[6] =  (( now_dj[6]  + (0xffff- lastUpdate_dj[6])) );}
				 else	{ Rc_Pwm_In_mine[6] =  (( now_dj[6]  - lastUpdate_dj[6])); }
		}
	}
	if(TIM3->SR & TIM_IT_CC4) 
	{
		TIM3->SR = ~TIM_IT_CC4;
		TIM3->SR = ~TIM_FLAG_CC4OF;
		if(GPIOB->IDR & GPIO_Pin_1)
		{
		 lastUpdate_dj[7] =  GetSysTime_us();		//temp_cnt2 = TIM_GetCapture2(TIM5);//	temp_cnt4 = TIM_GetCapture4(TIM3);
		}
		else
		{
//			temp_cnt4_2 = TIM_GetCapture4(TIM3);
//			if(temp_cnt4_2>=temp_cnt4)
//				Rc_Pwm_In_mine[7] = temp_cnt4_2-temp_cnt4;
//			else
//				Rc_Pwm_In_mine[7] = 0xffff-temp_cnt4+temp_cnt4_2+1;
			 now_dj[7] = GetSysTime_us();  //读取时间
				 if( now_dj[7] <lastUpdate_dj[7]){Rc_Pwm_In_mine[7] =  (( now_dj[7]  + (0xffff- lastUpdate_dj[7])) );}
				 else	{ Rc_Pwm_In_mine[7] =  (( now_dj[7]  - lastUpdate_dj[7])); }
		}
	}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void TIM5_IRQHandler(void)		
{
	static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	

	if(pwmin.cal_cycle){
		
			if(TIM5->SR & TIM_IT_CC1) 
	{
		TIM5->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM5->SR = ~TIM_FLAG_CC1OF;
		if(GPIOA->IDR & GPIO_Pin_0)
		{
			
			
			//	temp_cnt1 = TIM_GetCapture1(TIM5);
			  
				 now_dj[0] = GetSysTime_us();  //读取时间
				 if( now_dj[0] <lastUpdate_dj[0]){Rc_Pwm_In_mine[0] =  (( now_dj[0]  + (0xffff- lastUpdate_dj[0])) );}
				 else	{ Rc_Pwm_In_mine[0] =  (( now_dj[0]  - lastUpdate_dj[0])); }
			   lastUpdate_dj[0] = now_dj[0];	
		}
	}	
	}//-------------------------------------------
	else{
	if(TIM5->SR & TIM_IT_CC1) 
	{
		TIM5->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM5->SR = ~TIM_FLAG_CC1OF;
		if(GPIOA->IDR & GPIO_Pin_0)
		{
			 //temp_cnt1 = TIM_GetCapture1(TIM5);
			 lastUpdate_dj[0] =  GetSysTime_us();		 
		}
		else
		{
				 now_dj[0] = GetSysTime_us();  //读取时间
				 if( now_dj[0] <lastUpdate_dj[0]){Rc_Pwm_In_mine[0] =  (( now_dj[0]  + (0xffff- lastUpdate_dj[0])) );}
				 else	{ Rc_Pwm_In_mine[0] =  (( now_dj[0]  - lastUpdate_dj[0])); }
//			temp_cnt1_2 = TIM_GetCapture1(TIM5);
//			if(temp_cnt1_2>=temp_cnt1)
//				Rc_Pwm_In_mine[0] = temp_cnt1_2-temp_cnt1;
//			else
//				Rc_Pwm_In_mine[0] = 0xffff-temp_cnt1+temp_cnt1_2+1;
		}
	}
}
	

	if(TIM5->SR & TIM_IT_CC2) 
	{
		TIM5->SR = ~TIM_IT_CC2;
		TIM5->SR = ~TIM_FLAG_CC2OF;
		if(GPIOA->IDR & GPIO_Pin_1)
		{
			 lastUpdate_dj[1] =  GetSysTime_us();		//temp_cnt2 = TIM_GetCapture2(TIM5);
		}
		else
		{
			   now_dj[1] = GetSysTime_us();  //读取时间
				 if( now_dj[1] <lastUpdate_dj[1]){Rc_Pwm_In_mine[1] =  (( now_dj[1]  + (0xffff- lastUpdate_dj[1])) );}
				 else	{ Rc_Pwm_In_mine[1] =  (( now_dj[1]  - lastUpdate_dj[1])); }
//			temp_cnt2_2 = TIM_GetCapture2(TIM5);
//			if(temp_cnt2_2>=temp_cnt2)
//				Rc_Pwm_In_mine[1] = temp_cnt2_2-temp_cnt2;
//			else
//				Rc_Pwm_In_mine[1] = 0xffff-temp_cnt2+temp_cnt2_2+1;
		}
	}
	if(TIM5->SR & TIM_IT_CC3) 
	{
		TIM5->SR = ~TIM_IT_CC3;
		TIM5->SR = ~TIM_FLAG_CC3OF;
		if(GPIOA->IDR & GPIO_Pin_2)
		{
			lastUpdate_dj[2] =  GetSysTime_us();	//temp_cnt3 = TIM_GetCapture3(TIM5);
		}
		else
		{
			 now_dj[2] = GetSysTime_us();  //读取时间
				 if( now_dj[2] <lastUpdate_dj[2]){Rc_Pwm_In_mine[2] =  (( now_dj[2]  + (0xffff- lastUpdate_dj[2])) );}
				 else	{ Rc_Pwm_In_mine[2] =  (( now_dj[2]  - lastUpdate_dj[2])); }
//			temp_cnt3_2 = TIM_GetCapture3(TIM5);
//			if(temp_cnt3_2>=temp_cnt3)
//				Rc_Pwm_In_mine[2] = temp_cnt3_2-temp_cnt3;
//			else
//				Rc_Pwm_In_mine[2] = 0xffff-temp_cnt3+temp_cnt3_2+1;
				 
				 
		}
	}
	if(TIM5->SR & TIM_IT_CC4) 
	{
		TIM5->SR = ~TIM_IT_CC4;
		TIM5->SR = ~TIM_FLAG_CC4OF;
		if(GPIOA->IDR & GPIO_Pin_3)
		{
		lastUpdate_dj[3] =  GetSysTime_us();	//	temp_cnt4 = TIM_GetCapture4(TIM5);
		}
		else
		{
			 now_dj[3] = GetSysTime_us();  //读取时间
				 if( now_dj[3] <lastUpdate_dj[3]){Rc_Pwm_In_mine[3] =  (( now_dj[3]  + (0xffff- lastUpdate_dj[3])) );}
				 else	{ Rc_Pwm_In_mine[3] =  (( now_dj[3]  - lastUpdate_dj[3])); }
//			temp_cnt4_2 = TIM_GetCapture4(TIM5);
//			if(temp_cnt4_2>=temp_cnt4)
//				Rc_Pwm_In_mine[3] = temp_cnt4_2-temp_cnt4;
//			else
//				Rc_Pwm_In_mine[3] = 0xffff-temp_cnt4+temp_cnt4_2+1;
		}
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}
 u16	PWMMAX =  1500, PWMMIN =  1500 ,PWMT=1500;
#define CALIBRATING_PWMIN_CYCLES              100  //校准时间持续2s
#define CALIBRATING_PWMIN_MAXMIN              500  //校准时间持续20s
u16 led_cnt=10;
void PWIN_CAL(void){

}	


