#include "pressure.h"
#include "time.h"
 
void pressure_init(void)
{

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOD, ENABLE);//??GPIOA??

  //DOUT1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	
	//DOUT2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	
	//DOUT3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
	
	//DOUT4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_6);
	
	
  //SCK
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??????
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);
}
 
 
int Pres1=0;
int Pres2=0;
int Pres3=0;
int Pres4=0;

void Read_HX711(void)  
{ 
unsigned long val1 = 0; 
unsigned long val2 = 0; 
unsigned long val3 = 0; 
unsigned long val4 = 0; 
unsigned char i = 0; 

DOUT1=0;  
DOUT2=0;   
DOUT3=0;    
DOUT4=0;   
SCK=0;	   //SCK=0 
while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)); 
while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)); 
while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)); 
while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)); 
delay_us(1); 
for(i=0;i<24;i++) 
{ 
SCK=1;	   //SCK=1 
val1=val1<<1;
val2=val2<<1; 
val3=val3<<1; 
val4=val4<<1; 	
delay_us(1);  
SCK=0;	   //SCK=0 
if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1))   //DOUT1=1 
val1++; 
if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4))   //DOUT2=1 
val2++; 
if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5))   //DOUT3=1 
val3++; 
if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6))   //DOUT4=1 
val4++; 
delay_us(1); 
} 
SCK=1; 
val1 = val1^0x800000; 
val2 = val2^0x800000; 
val3 = val3^0x800000; 
val4 = val4^0x800000; 
delay_us(1); 
SCK=0; 
delay_us(1);  
Pres1=(int)(((int)(val1/1000.0)-8490)*2.5);  //???????
Pres2=(int)(((int)(val2/1000.0)-8490)*2.5);  //???????
Pres3=(int)(((int)(val3/1000.0)-8490)*2.5);  //???????
Pres4=(int)(((int)(val4/1000.0)-8490)*2.5);  //???????	
} 



