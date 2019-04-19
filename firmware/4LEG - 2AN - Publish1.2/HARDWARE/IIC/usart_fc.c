#include "include.h"
#include "usart_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "rc.h"
#include "sbus.h"
#include "vmc.h"
#include "my_math.h"
_ODOMETER flow;
_PI pi;

void Usart1_Init(u32 br_num)//-------Radio
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART1, ENABLE); 
}

void Usart2_Init(u32 br_num)//--GPS
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	USART_ClearFlag(USART2, USART_FLAG_TC);
	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
}

void Usart3_Init(u32 br_num)//-------PI
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
//	//使能发送（进入移位）中断
}

void Usart4_Init(u32 br_num)//-------FLOW
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(UART4, ENABLE);  //使能串口1 
	USART_ClearFlag(UART4, USART_FLAG_TC);
	//使能USART2接收中断
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(UART4, ENABLE); 
}


void Uart5_Init(u32 br_num)//-----Sbus
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode =  USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
}

void UsartSend1(uint8_t ch)
{
while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); 
}

void UsartSend2(uint8_t ch)
{
while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}
void UsartSend3(uint8_t ch)
{
while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}
void UsartSend4(uint8_t ch)
{
while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch); 
}



//图像控制命令
void Data_LEG_CMD(u8 *data_buf,u8 num)
{ static u8 cnt[4];
	u8 id;
	vs16 rc_value_temp;
	u8 sum = 0,sum_rx=0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	sum_rx=*(data_buf+num-1);
	if(!(sum==*(data_buf+num-1))&&*(data_buf+2)!=MODE_FACE)		return;		//判断sum
	//if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==MODE_CUBE)//Cube
  { 
	  pi.connect=MODE_CUBE;
		pi.lost_cnt=0;
		pi.cube.check=*(data_buf+4);
	  pi.cube.x=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))-320/2;
		pi.cube.y=-(((int16_t)(*(data_buf+7)<<8)|*(data_buf+8))-240/2);
		pi.cube.s=((int16_t)(*(data_buf+9)<<8)|*(data_buf+10));
		
		pi.cube.pos[Xr]=(float)((int16_t)(*(data_buf+11)<<8)|*(data_buf+12))/100.;
		pi.cube.pos[Yr]=(float)((int16_t)(*(data_buf+13)<<8)|*(data_buf+14))/100.;
		pi.cube.pos[Zr]=(float)((int16_t)(*(data_buf+15)<<8)|*(data_buf+16))/100.;
		
		pi.cube.att[Xr]=(float)((int16_t)(*(data_buf+17)<<8)|*(data_buf+18))/100.;
		pi.cube.att[Yr]=(float)((int16_t)(*(data_buf+19)<<8)|*(data_buf+20))/100.;
		pi.cube.att[Zr]=(float)((int16_t)(*(data_buf+21)<<8)|*(data_buf+22))/100.;
	}		
	else if(*(data_buf+2)==MODE_FACE)//Face
  { 
	  pi.connect=MODE_FACE;
		pi.lost_cnt=0;
		pi.face.check=*(data_buf+4);
	  pi.face.x=*(data_buf+5)-256/2;
		pi.face.y=-(*(data_buf+6)-192/2);
		pi.face.s=*(data_buf+7);
	}	
	else if(*(data_buf+2)==MODE_ROS)//ROS
  { 
	  pi.connect=MODE_ROS;
		pi.lost_cnt=0;
	
		vmc_all.param.tar_spd_on.x=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/100.;
		vmc_all.param.tar_spd_on.y=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100.;
		vmc_all.param.tar_spd_on.z=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100.;
		
		vmc_all.param.tar_spd_on.x=LIMIT(vmc_all.tar_spd.x,-MAX_SPD,MAX_SPD);
		vmc_all.param.tar_spd_on.y=LIMIT(vmc_all.tar_spd.y,-MAX_SPD,MAX_SPD);
		vmc_all.param.tar_spd_on.z=LIMIT(vmc_all.tar_spd.z,-MAX_SPD_RAD,MAX_SPD_RAD);
		
	}			
}

//数传控制命令
void Anal_Outter_CMD(u8 *data_buf,u8 num)
{ static u8 cnt[4];
	u8 id;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==1)//
  { 
	  vmc_all.param.rc_mode[0]=1;
		vmc_all.param.rc_mode[1]=0;
	  vmc_all.param.tar_spd_on.x=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/100.;
		vmc_all.param.tar_spd_on.y=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100.;
		vmc_all.param.tar_spd_on.z=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100.;
		
		vmc_all.param.tar_spd_on.x=LIMIT(vmc_all.tar_spd.x,-MAX_SPD,MAX_SPD);
		vmc_all.param.tar_spd_on.y=LIMIT(vmc_all.tar_spd.y,-MAX_SPD,MAX_SPD);
		vmc_all.param.tar_spd_on.z=LIMIT(vmc_all.tar_spd.z,-MAX_SPD_RAD,MAX_SPD_RAD);
	}		
}

u8 TxBuffer1[256];
u8 TxCounter1=0;
u8 count1=0; 
u8 RxBuffer1[50];
u8 RxState1 = 0;
u8 RxBufferNum1 = 0;
u8 RxBufferCnt1 = 0;
u8 RxLen1 = 0;
static u8 _data_len1 = 0,_data_cnt1 = 0;
void USART1_IRQHandler(void)//Radio
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART1->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART1->DR;
		if(RxState1==0&&com_data==0xAA)
		{
			RxState1=1;
			RxBuffer1[0]=com_data;
		}
		else if(RxState1==1&&com_data==0xAF)
		{
			RxState1=2;
			RxBuffer1[1]=com_data;
		}
		else if(RxState1==2&&com_data>0&&com_data<0XF1)
		{
			RxState1=3;
			RxBuffer1[2]=com_data;
		}
		else if(RxState1==3&&com_data<50)
		{
			RxState1= 4;
			RxBuffer1[3]=com_data;
			_data_len1 = com_data;
			_data_cnt1 = 0;
		}
		else if(RxState1==4&&_data_len1>0)
		{
			_data_len1--;
			RxBuffer1[4+_data_cnt1++]=com_data;
			if(_data_len1==0)
				RxState1= 5;
		}
		else if(RxState1==5)
		{
			RxState1 = 0;
			RxBuffer1[4+_data_cnt1]=com_data;
      Anal_Outter_CMD(RxBuffer1,_data_cnt1+5);
		}
		else
			RxState1 = 0;
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART1->DR = TxBuffer1[TxCounter1++]; //写DR清除中断标志          
		if(TxCounter1 == count1)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}

   OSIntExit(); 
}


u8 TxBuffer2[256];
u8 TxCounter2=0;
u8 count2=0; 
u8 RxBuffer2[80];
u8 RxState2 = 0;
u8 RxBufferNum2 = 0;
u8 RxBufferCnt2 = 0;
u8 RxLen2 = 0;
static u8 _data_len2 = 0,_data_cnt2 = 0;
void USART2_IRQHandler(void)//GPS
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
		if(RxState2==0&&com_data==0xAA)
		{
			RxState2=1;
			RxBuffer2[0]=com_data;
		}
		else if(RxState2==1&&com_data==0xAF)
		{
			RxState2=2;
			RxBuffer2[1]=com_data;
		}
		else if(RxState2==2&&com_data>0&&com_data<0XF1)
		{
			RxState2=3;
			RxBuffer2[2]=com_data;
		}
		else if(RxState2==3&&com_data<80)
		{
			RxState2 = 4;
			RxBuffer2[3]=com_data;
			_data_len2 = com_data;
			_data_cnt2 = 0;
		}
		else if(RxState2==4&&_data_len2>0)
		{
			_data_len2--;
			RxBuffer2[4+_data_cnt2++]=com_data;
			if(_data_len2==0)
				RxState2= 5;
		}
		else if(RxState2==5)
		{
			RxState2 = 0;
			RxBuffer2[4+_data_cnt2]=com_data;
   
		}
		else
			RxState2 = 0;
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer2[TxCounter2++]; //写DR清除中断标志          
		if(TxCounter2 == count2)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}

   OSIntExit(); 
}


u8 TxBuffer3[256];
u8 TxCounter3=0;
u8 count3=0; 
u8 Rx_Buf3[256];	//串口接收缓存
u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)//PI
{  OSIntEnter();  
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
		
		if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			Data_LEG_CMD(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
	
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer2[TxCounter3++]; //写DR清除中断标志          
		if(TxCounter3 == count3)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}

 OSIntExit();        
}



u8 TxBuffer4[256];
u8 TxCounter4=0;
u8 count4=0; 
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4= 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void UART4_IRQHandler(void)//FLOW
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART4->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//清除中断标志

		com_data = UART4->DR;
		if(RxState4==0&&com_data==0xAA)
		{
			RxState4=1;
			RxBuffer4[0]=com_data;
		}
		else if(RxState4==1&&com_data==0xAF)
		{
			RxState4=2;
			RxBuffer4[1]=com_data;
		}
		else if(RxState4==2&&com_data>0&&com_data<0XF1)
		{
			RxState4=3;
			RxBuffer4[2]=com_data;
		}
		else if(RxState4==3&&com_data<50)
		{
			RxState4 = 4;
			RxBuffer4[3]=com_data;
			_data_len4 = com_data;
			_data_cnt4 = 0;
		}
		else if(RxState4==4&&_data_len4>0)
		{
			_data_len4--;
			RxBuffer4[4+_data_cnt4++]=com_data;
			if(_data_len4==0)
				RxState4 =5;
		}
		else if(RxState4==5)
		{
			RxState4 = 0;
			RxBuffer4[4+_data_cnt4]=com_data;

		}
		else
			RxState4 = 0;
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{
				
		UART4->DR = TxBuffer4[TxCounter4++]; //写DR清除中断标志          
		if(TxCounter4 == count4)
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
   OSIntExit(); 

}


RC_GETDATA Rc_Get_PWM,Rc_Get_SBUS;
static u8 _data_len5 = 0,_data_cnt5 = 0;
void UART5_IRQHandler(void)//SBUS
{ OSIntEnter(); 
	u8 com_data;
	 u16 temps;
	if(UART5->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART5->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
		Rc_Get_SBUS.lose_cnt=0;
		Rc_Get_SBUS.connect=1;
		oldx_sbus_rx(com_data);
		if(channels[16]==500||channels[16]==503){
		Rc_Get_SBUS.update=1;Rc_Get_SBUS.lose_cnt_rx=0;

				temps=((channels[0])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.ROLL=		 temps;
				temps=((channels[1])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.PITCH=		 temps;
				temps=((channels[2])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.THROTTLE=		 temps;
				temps=((channels[3])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.YAW=		 temps;
				temps=((channels[4])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX1=		 temps;
				temps=((channels[5])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX2=		 temps;
				temps=((channels[6])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX3=		 temps;
				temps=((channels[7])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX4=		 temps;

		}
		if(Rc_Get_SBUS.lose_cnt_rx++>100){
		Rc_Get_SBUS.update=0;}
	}
   OSIntExit(); 
}


u8 SendBuff1[SEND_BUF_SIZE1];
u8 SendBuff3[SEND_BUF_SIZE3];
int16_t BLE_DEBUG[16];
void data_per_uart_rc(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;

char ctemp;	
BLE_DEBUG[1]=ax;
BLE_DEBUG[2]=ay;
BLE_DEBUG[3]=az;	
BLE_DEBUG[4]=gx;
BLE_DEBUG[5]=gy;
BLE_DEBUG[6]=gz;
BLE_DEBUG[7]=hx;
BLE_DEBUG[8]=hy;
BLE_DEBUG[9]=hz;
BLE_DEBUG[10]=yaw;
BLE_DEBUG[11]=pitch;
BLE_DEBUG[12]=roll;
BLE_DEBUG[13]=alt;
BLE_DEBUG[13]=tempr;
BLE_DEBUG[14]=press;
BLE_DEBUG[15]=IMUpersec;
}

u16 leg_uart_cnt;
void GOL_LINK_TASK_DMA(void)//5ms
{
static u8 cnt[10];
static u8 flag[10];
u8 i;

end_gol_link1:;
}

void clear_leg_uart(void)
{u16 i;
leg_uart_cnt=0;
for(i=0;i<SEND_BUF_SIZE1;i++)
SendBuff1[i]=0;
}


u16 nrf_uart_cnt1;
void data_per_uart1(void)
{
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  
	cnt_reg=nrf_uart_cnt1;
	SendBuff1[nrf_uart_cnt1++]=0xAA;
	SendBuff1[nrf_uart_cnt1++]=0xAF;
	SendBuff1[nrf_uart_cnt1++]=0x01;
	SendBuff1[nrf_uart_cnt1++]=0;
	_temp=vmc_all.att[0]*10;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
  _temp=vmc_all.att[1]*10;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	_temp=vmc_all.att[2]*10;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	
	for(i=0;i<4;i++){
	_temp=vmc[i].epos.x*10;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	_temp=vmc[i].epos.y*10;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	_temp=vmc[i].epos.z*10;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	}
	
	
	SendBuff1[cnt_reg+3] =(nrf_uart_cnt1-cnt_reg)-4;
		for( i=cnt_reg;i<nrf_uart_cnt1;i++)
	sum += SendBuff1[i];
	SendBuff1[nrf_uart_cnt1++] = sum;
}