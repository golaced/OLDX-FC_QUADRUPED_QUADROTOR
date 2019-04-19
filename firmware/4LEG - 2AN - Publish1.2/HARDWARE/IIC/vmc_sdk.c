#include "vmc.h"
#include "usart_fc.h"
#include "my_math.h"
#include "filter.h"
int mission_sel_lock;
int mission_state;
u8 mission_flag;
float timer_sdk[10];

//demo
u8 Cube_Track(float dt);

//api
u8 delay_api(float time,float dt);
u8 set_spd_api(float x,float y,float z,char mode,float time,float dt);
u8 track_pixy_api(u8 mode,float dt);
//SDK开发
void smart_control(float dt)
{
	static float cnt;
	u8 mission_finish=0;
	//reset
	vmc_all.param.tar_spd_on.x=vmc_all.param.tar_spd_on.z=0;
	vmc_all.param.tar_att_on[0]=vmc_all.param.tar_att_on[1]=0;
	vmc_all.param.tar_att_on[2]=vmc_all.att_ctrl[2];
	vmc_all.param.tar_pos_on.z=vmc_all.pos.z;
	vmc_all.param.smart_control_mode[0]=vmc_all.param.smart_control_mode[1]=vmc_all.param.smart_control_mode[2]=0;
	switch(mission_flag)
	{
	  case 0:
			if(vmc_all.param.en_sdk&&vmc_all.power_state>2)
				cnt+=dt;
			else
				cnt=0;
			if(cnt>1)
			{cnt=0;mission_flag=1;mission_state=0;}
		break;
		case 1:
			switch(mission_sel_lock)
			{
				case 0:
					mission_finish=Cube_Track(dt);
				break;
			}	
			if(vmc_all.param.en_sdk)
				cnt+=dt;
			else
				cnt=0;
			if(cnt>2||mission_finish==1)
			{cnt=0;
			 vmc_all.param.smart_control_mode[0]=vmc_all.param.smart_control_mode[1]=vmc_all.param.smart_control_mode[2]=0;
			 mission_flag=0;}
		break;
	}
}

void timer_init(void)
{
	u8 i;
  for(i=0;i<10;i++)
    timer_sdk[i]=0;
}

u8 Cube_Track(float dt){
u8 flag=0,mission_finish=0;
	switch(mission_state){
	case 0:
		     if(vmc_all.power_state>2)
				   {flag=1;timer_init();}
		     break;	
  case 1:flag=set_spd_api(0.2,0,0,MODE_BODY, 3, dt);break;	
	case 2:flag=delay_api(2,dt);break;	
	case 3:flag=set_spd_api(-0.2,0,0,MODE_BODY, 3, dt);break;	
	case 4:flag=delay_api(2,dt);break;					 
	case 5:
		     if(KEY[1]==0) 
					mission_state=1;
		     flag=track_pixy_api(0,dt);
	      break;	
  case 6:mission_finish=1; 	
	default:break;
	}
	if(flag)
		mission_state++;
	
	if(mission_finish){
		mission_state=0;
	  return 1;}		
  else
    return 0;		
}	



//--------------------------------------API-----------------------------
u8 delay_api(float time,float dt)
{
	if(timer_sdk[0]>time)
	{
	  timer_sdk[0]=0;
		return 1;
	}else{
    timer_sdk[0]+=dt;
		return 0;
	}
}
// 								 俯仰  左右  前后  设定距离
float param_track[5]={35,0.4,0.035,22};
u8 track_pixy_api(u8 mode,float dt)
{
 float err[3];
 static float err_flt[3];
 static u8 hold;
 static float hold_timer,reset_timer;
	
 err[Yr]=LIMIT(pi.cube.y,-160,160);//俯仰
 err[Xr]=LIMIT(-pi.cube.x,-120,120);//左右
 err[Zr]=LIMIT(param_track[3]-pi.cube.s,-20,35);//前后
 if(pi.cube.check){
 Low_Fass_Filter(my_deathzoom(err[Yr],10),&err_flt[Yr], 1.5,  dt);//俯仰
 Low_Fass_Filter(my_deathzoom(err[Xr],10), &err_flt[Xr], 3,  dt);//左右 
 Low_Fass_Filter(my_deathzoom(err[Zr],5), &err_flt[Zr], 3,  dt);//前后
 }
 if(pi.cube.check){
	 hold_timer=0;hold=1;
	 reset_timer=0;
 }else
   reset_timer+=dt;
 if(hold)//pix
 {
	hold_timer+=dt; 
  vmc_all.gait_on=1;
	vmc_all.param.tar_spd_on.x=err_flt[Zr]*param_track[2];
  vmc_all.param.tar_spd_on.z=err_flt[Xr]*param_track[1];
	vmc_all.param.smart_control_mode[0]=MODE_SPD;
	 
	vmc_all.param.tar_att_on[PITr]+=err_flt[Yr]*param_track[0]*dt;
	vmc_all.param.tar_att_on[PITr]=LIMIT(vmc_all.param.tar_att_on[PITr],-12,25);
	vmc_all.param.smart_control_mode[2]=MODE_ATT;
 }
 if(reset_timer>3.5)
 {
	reset_timer=0; 
 	vmc_all.param.tar_spd_on.x=vmc_all.param.tar_spd_on.z=0;
	vmc_all.param.smart_control_mode[0]=MODE_SPD;
	 
	vmc_all.param.tar_att_on[PITr]=0;
	vmc_all.param.smart_control_mode[2]=MODE_ATT;
 }
 
 if(hold_timer>0.5)
	  hold_timer=hold=0;
 
 return 0;
}


u8 set_spd_api(float x,float y,float z,char mode,float time,float dt)
{
   if(mode==MODE_BODY)//body
	 {
	   vmc_all.param.tar_spd_on.x=x;
		 vmc_all.param.tar_spd_on.z=z;
		 vmc_all.param.smart_control_mode[0]=MODE_SPD;
	 }
	
   return delay_api(time,dt);
}


void set_pos_api(float x,float y,float z,char mode,float dt)
{


}