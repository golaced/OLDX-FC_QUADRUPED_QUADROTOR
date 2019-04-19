#include "led_fc.h"
#include "include.h"
#include "mems.h"
#include "beep.h"
#include "bat.h"
#include "usart_fc.h"
#include "vmc.h"
#include "my_math.h"
void POWER_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	leg_power_control(0);
}

void leg_power_control(u8 sel){
#if defined(HARDWARE_V1)
if(sel)
#else
if(!sel)
#endif
GPIO_ResetBits(GPIOC,GPIO_Pin_3);
else
GPIO_SetBits(GPIOC,GPIO_Pin_3);
}

void LEDRGB_COLOR(u8 color);
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 //SEL
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_12 ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	LEDRGB_COLOR(BLUE);
	Delay_ms(500);
	LEDRGB_COLOR(RED);
	Delay_ms(500);
	LEDRGB_COLOR(GREEN);
  Delay_ms(500);

}


void LEDRGB(u8 sel,u8 on)
{
switch(sel)
{
case BLUE:
if(!on)
GPIO_ResetBits(GPIOB,GPIO_Pin_12);
else
GPIO_SetBits(GPIOB,GPIO_Pin_12);
break;
case GREEN:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_0);
else
GPIO_SetBits(GPIOA,GPIO_Pin_0);
break;
case RED:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_11);
else
GPIO_SetBits(GPIOA,GPIO_Pin_11);
break;
}
}
u8 LED[3]={0};
void LEDRGB_COLOR(u8 color)
{
switch (color)
{
case RED:
LED[0]=1;
LED[1]=0;
LED[2]=0;
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case BLUE:
LED[0]=0;
LED[1]=1;
LED[2]=0;	
LEDRGB(RED,0);
LEDRGB(BLUE,1);
LEDRGB(GREEN,0);
break;
case GREEN:
LED[0]=0;
LED[1]=0;
LED[2]=1;	
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
case WHITE:
LED[0]=1;
LED[1]=1;
LED[2]=1;
LEDRGB(RED,1);
LEDRGB(BLUE,1);
LEDRGB(GREEN,1);
break;
case BLACK:
LED[0]=0;
LED[1]=0;
LED[2]=0;	
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case YELLOW:
LED[0]=1;
LED[1]=0;
LED[2]=1;	
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
}
}
#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2
#define CAL_DJ1 3
#define CAL_DJ2	4
#define CAL_DJ3 5
#define CAL_DJ4 6
#define CAL_DJ5 7
#define DELAY_S 11
int dj_sel=0;
void LEDRGB_STATE(float dt)
{
static u8 main_state;
static u8 mpu_state,m_state,idle_state,beep_state;
static u16 cnt,cnt_idle;
static u8 beep_sel;
static float cnt_beep;
u8 mode_control;
static float cnt_dj_cal[3];	
static float cnt_dj_beep;
char dj_sel_table1[8]={0,0, 1,1, 2,2 ,3,3};
char dj_sel_table2[8]={0,1, 0,1, 0,1 ,0,1};
switch(main_state)
{ 
	case IDLE:
	if(mems.Gyro_CALIBRATE)
	{idle_state=0;main_state=CAL_MPU;
	}
	
	if(KEY[0]&&KEY[1]==1)
	{main_state=CAL_DJ1;dj_sel=cnt_dj_cal[0]=cnt_dj_cal[1]=0;}
	
	if(KEY[4])
		cnt_dj_cal[0]+=dt;
	else
		cnt_dj_cal[0]=0;
	if(cnt_dj_cal[0]>6){
		cnt_dj_cal[0]=0;
	  main_state=CAL_MPU;
	}
	break;
	case CAL_MPU:
		cnt_dj_cal[0]+=dt;
	  Play_Music_Task(MEMS_GPS_RIGHT,dt);	
		if(cnt_dj_cal[0]>2)
		{cnt_dj_cal[0]=0;
		 main_state=DELAY_S;
		 vmc_all.param.cal_flag[1]=2;
		 Tone(0,0);}
  break;
		 
  case DELAY_S:
		cnt_dj_cal[0]+=dt;
		if(cnt_dj_cal[0]>2)
		{cnt_dj_cal[0]=0;
		 main_state=IDLE;}
	break;
	case CAL_M:
  break;
	
	case CAL_DJ1:
		cnt_dj_cal[0]+=dt;
		if(KEY[0]&&KEY[1])
		{
		   if(Rc_Get.THROTTLE>1800){
		      main_state=CAL_DJ2;
			    cnt_dj_cal[0]=0;
				  cnt_dj_cal[1]+=1;
			 }
		}	
		else
			main_state=IDLE;
		
		if(cnt_dj_cal[0]>3.5)
			main_state=IDLE;
		else if(cnt_dj_cal[1]>10)
			{main_state=CAL_DJ3;dj_sel=cnt_dj_cal[0]=cnt_dj_cal[1]=0;force_dj_off_reset=1;}
	break;
	case CAL_DJ2:
		cnt_dj_cal[0]+=dt;
		if(KEY[0]&&KEY[1])
		{
		   if(Rc_Get.THROTTLE<1200)
		    {
		      main_state=CAL_DJ1;
			    cnt_dj_cal[0]=0;
				  cnt_dj_cal[1]+=1;
				}
		}	
		else
			main_state=IDLE;
		
	if(cnt_dj_cal[0]>3.5)
			main_state=IDLE;
		else if(cnt_dj_cal[1]>10)
		{main_state=CAL_DJ3;dj_sel=cnt_dj_cal[0]=cnt_dj_cal[1]=0;force_dj_off_reset=1;vmc_all.sita_test[4]=2;}
	break;
	case CAL_DJ3:
		cnt_dj_cal[0]+=dt;
	  Play_Music_Task(MEMS_GPS_RIGHT,dt);
		if(cnt_dj_cal[0]>2)
		{cnt_dj_cal[0]=cnt_dj_cal[2]=cnt_dj_beep=0;main_state=CAL_DJ4;Tone(0,0);}
	  break;
	case CAL_DJ4:
		vmc_all.sita_test[4]=2;
	
		cnt_dj_beep+=dt;	
		if(cnt_dj_beep>2*dj_sel){cnt_dj_beep=0;;Tone(0,0);}	
		Play_Music_Task(BEEP_DJ_CAL1,dt);
		
		if(KEY[0]&&KEY[1])
		{
      if(Rc_Get.YAW<1150)
				cnt_dj_cal[0]+=dt;
			else
				cnt_dj_cal[0]=0;
			
			if(Rc_Get.YAW>1850)
				cnt_dj_cal[1]+=dt;
			else
				cnt_dj_cal[1]=0;
			
			if(cnt_dj_cal[0]>2)
			{cnt_dj_cal[0]=0;dj_sel--;}
			if(cnt_dj_cal[1]>2)
			{cnt_dj_cal[1]=0;dj_sel++;}
			dj_sel=LIMIT(dj_sel,0,7);
			
			if(KEY[4])
			  cnt_dj_cal[2]+=dt;
			else
				cnt_dj_cal[2]=0;
			if(cnt_dj_cal[2]>4){
				cnt_dj_cal[2]=0;
			  main_state=CAL_DJ5;
			}
			
			vmc[dj_sel_table1[dj_sel]].param.PWM_OFF[dj_sel_table2[dj_sel]]+=(float)my_deathzoom(Rc_Get.THROTTLE-1500,200)/100.;
		}	
		else
		 {main_state=IDLE;vmc_all.sita_test[4]=0;}
	break;
	case CAL_DJ5:
		cnt_dj_cal[0]+=dt;
	  Play_Music_Task(MEMS_GPS_RIGHT,dt);	
		if(cnt_dj_cal[0]>2)
		{cnt_dj_cal[0]=cnt_dj_cal[2]=0;
		 main_state=IDLE;vmc_all.sita_test[4]=0;
		 vmc_all.param.cal_flag[1]=1;Tone(0,0);}
	  break;
	break;
}


//   | | | |    | | | |   | | | |   | | | |   | | | |
//    ARM          GPS1     GPS2      GPS3      MODE  
#define RGB_DELAY 3
static u8 cnt_gps;
static u8 flag_cnt_gps;
if(cnt_gps++>1){cnt_gps=0;
	flag_cnt_gps=!flag_cnt_gps;
}

{
	//LED st
	switch(idle_state)
	{
		case 0:
			if(main_state==IDLE)
				{idle_state=1;cnt_idle=0;}
		break;
		case 1:
		if(Rc_Get_SBUS.update||module.nrf==2) 
			{LEDRGB_COLOR(BLUE);fc_state_beep[0]=BEEP_STATE1;} 	
			else
			{LEDRGB_COLOR(RED);fc_state_beep[0]=BEEP_STATE1;} 
		if(cnt_idle++>0.1/0.05)
		{idle_state=2;cnt_idle=0;}
		break;
		case 2:
			LEDRGB_COLOR(BLACK);	
		if(cnt_idle++>0.1/0.05)
		{idle_state=3;cnt_idle=0;}
		break;
		
		case 3:
			if(pi.connect) 
			{LEDRGB_COLOR(WHITE);fc_state_beep[1]=BEEP_STATE1;} 	
			else
			{LEDRGB_COLOR(RED);fc_state_beep[1]=BEEP_STATE1;} 
		if(cnt_idle++>0.1/0.05)
		{idle_state=4;cnt_idle=0;}
		break;
		case 4:
			LEDRGB_COLOR(BLACK);	
		if(cnt_idle++>0.1/0.05)
		{idle_state=5;cnt_idle=0;}
		break;
		
		case 5:
			 {LEDRGB_COLOR(RED);fc_state_beep[2]=BEEP_STATE1; }
		if(cnt_idle++>0.1/0.05)
		{idle_state=6;cnt_idle=0;}
		break;
		case 6:
			LEDRGB_COLOR(BLACK);	
		if(cnt_idle++>1.2/0.05)
		{idle_state=0;cnt_idle=0;}
		break;
	}
	
	//BEEP st
	static u8 mission_flag_reg;
	if(main_state==IDLE)
	switch(beep_state)
	{
	  case 0:
			if(bat.low_bat)
				Play_Music_Task(BAT_ERO_BEEP,dt);
			else
				Tone(0,0);
			if(mission_flag>0&&mission_flag_reg==0)
			{beep_state=1;cnt_beep=0;}
		break;
	  case 1:
			cnt_beep+=dt;
      if(cnt_beep>3||mission_flag==0)
				beep_state=cnt_beep=0;
      Play_Music_Task(MEMS_GPS_RIGHT,dt);
    break;		
	}
  mission_flag_reg=mission_flag;
}
}
