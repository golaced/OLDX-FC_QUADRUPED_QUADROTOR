#include "vmc.h"
#include "usart_fc.h"
#include "my_math.h"
#include "filter.h"
#include "nav.h"
int mission_sel_lock=0;//选择DEMO
int mission_state;
u8 mission_flag;
float timer_sdk[10];
char flag_sdk[10];
//demo测试程序
u8 Cube_Track(float dt);
u8 Trajectory_Test(float dt);
//api
u8 delay_api(float time,float dt);
u8 set_spd_api(float x,float y,float z,char mode,float time,float dt);
u8 set_pos_api(float x,float y,float z,char mode,float dt);
u8 track_pixy_api(u8 mode,float dt);

//SDK开发
void smart_control(float dt)
{
	static float cnt,timer_rst;
	u8 mission_finish=0;
	//reset
	timer_rst+=dt;
	if(timer_rst>0){
		timer_rst=0;
		vmc_all.param.tar_spd_on.x=vmc_all.param.tar_spd_on.z=0;
		vmc_all.param.tar_att_on[PITr]=vmc_all.param.tar_att_on[ROLr]=0;
		vmc_all.param.tar_att_on[YAWr]=vmc_all.att_ctrl[YAWr];
		vmc_all.param.tar_pos_on.x=vmc_all.pos_n.x;
		vmc_all.param.tar_pos_on.y=vmc_all.pos_n.y;
		vmc_all.param.tar_pos_on.z=vmc_all.pos.z;
		vmc_all.param.smart_control_mode[0]=vmc_all.param.smart_control_mode[1]=vmc_all.param.smart_control_mode[2]=0;
	}
	
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
		case 1://选择DEMO
			switch(mission_sel_lock)
			{
				case 0:
					mission_finish=Cube_Track(dt);
				break;
				case 1:
					mission_finish=Trajectory_Test(dt);
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
	
	if(mission_flag==1)
		vmc_all.param.control_mode_all=2;//sdk mode
	else
		vmc_all.param.control_mode_all=1;//man mode
}

void timer_init(void)
{
	u8 i;
  for(i=0;i<10;i++)
    timer_sdk[i]=flag_sdk[i]=0;
}

u8 Cube_Track(float dt){
u8 flag=0,mission_finish=0;
	switch(mission_state){
	case 0:
		     if(vmc_all.power_state>2)
				   {flag=1;timer_init();}
		     break;	
	case 1:flag=delay_api(2,dt);break;		 
	case 2:
		     track_pixy_api(0,dt);
	      break;	
  case 3:mission_finish=1; 	
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

u8 Trajectory_Test(float dt){
u8 flag=0,mission_finish=0;
	switch(mission_state){
	case 0:
		     if(vmc_all.power_state>2)
				   {flag=1;timer_init();}
		     break;	
	case 1:flag=delay_api(0.5,dt);break;		 
	case 2:
		     flag=set_pos_api(0,0,0,MODE_POS,dt);
	      break;	
	case 3:
		     flag=set_pos_api(0,1,0,MODE_POS,dt);
	      break;	
	case 4:
		     flag=set_pos_api(1,1,0,MODE_POS,dt);
	      break;	
	case 5:
		     flag=set_pos_api(1,0,0,MODE_POS,dt);
	      break;	
  case 6:mission_state=1; break;	
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
float param_track[5]={30,0.25,0.03,22};
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
 Low_Fass_Filter(my_deathzoom(err[Xr],10),&err_flt[Xr], 3,  dt);//左右 
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
	vmc_all.param.tar_att_on[PITr]=LIMIT(vmc_all.param.tar_att_on[PITr],-12,15);
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


float k_traj=0.368;
u8 traj_init_task(float ps[3],float pe[3],float T,u8 sel)
{		
			traj[sel].ps[Xr]=ps[0];//m
			traj[sel].ps[Yr]=ps[1];//m
			traj[sel].ps[Zr]=ps[2];//m
			traj[sel].vs[Xr]=MAX_SPD*sind(vmc_all.att_ctrl[YAWr]);//m
			traj[sel].vs[Yr]=MAX_SPD*cosd(vmc_all.att_ctrl[YAWr]);//m
			traj[sel].vs[Zr]=0;//m
			traj[sel].pe[Xr]=pe[0];//m
			traj[sel].pe[Yr]=pe[1];//m
			traj[sel].pe[Zr]=pe[2];//m
			traj[sel].time_now=0;
			traj[sel].Dis=my_sqrt(my_pow(traj[sel].ps[Xr]-traj[sel].pe[Xr])
													 +my_pow(traj[sel].ps[Yr]-traj[sel].pe[Yr]));

			traj[sel].Time=traj[sel].Dis/(MAX_SPD)*k_traj;
			traj[sel].defined[0]=1;traj[sel].defined[1]=1;traj[sel].defined[2]=0;
			if(T!=0)
				traj[sel].Time=T;//s
			plan_tra(&traj[sel]);
	    return 1;
}

//mode 1->pos  2->spd
u8 follow_traj(u8 sel,u8 mode,float dt)
{
	   if(mode==MODE_SPD){
				traj[sel].time_now+=dt;
				traj[sel].time_now=LIMIT(traj[sel].time_now,0,traj[sel].Time);
				get_tra(&traj[sel],traj[sel].time_now);
				float angle=fast_atan2(traj[sel].vt[0],traj[sel].vt[1])*57.3;
				vmc_all.param.smart_control_mode[0]=MODE_SPD;
				float body_spd[2];
				float yaw=vmc_all.att_ctrl[YAWr];
				body_spd[Yr]= traj[sel].vt[Yr]*cosd(yaw)+traj[sel].vt[Xr]*sind(yaw); 
				body_spd[Xr]=-traj[sel].vt[Yr]*sind(yaw)+traj[sel].vt[Xr]*cosd(yaw);
				vmc_all.param.tar_spd_on.x=LIMIT(body_spd[Yr],-MAX_SPD,MAX_SPD);
				vmc_all.param.tar_spd_on.z=0;
			 
				vmc_all.param.smart_control_mode[2]=MODE_ATT;
				vmc_all.param.tar_att_on[2]=angle;
					
				if(traj[sel].time_now>=traj[sel].Time)
					 return 1;
				else 
					 return 0;
			}else if(mode==MODE_POS){
				vmc_all.param.smart_control_mode[0]=MODE_POS;
			
				float dis=my_sqrt(my_pow(vmc_all.param.tar_pos_on.x-vmc_all.pos_n.x)
												 +my_pow(vmc_all.param.tar_pos_on.y-vmc_all.pos_n.y));	
				
				float dis_final=my_sqrt(my_pow(traj[sel].pe[Xr]-vmc_all.pos_n.x)
												       +my_pow(traj[sel].pe[Yr]-vmc_all.pos_n.y));	
				
				float Per_t=LIMIT(traj[sel].Dis/(vmc_all.H*1.75),0.025,0.1);
				traj[sel].traj_pre_d=Per_t;
				if(dis<vmc_all.H*3.5||flag_sdk[1]==0)
				{
					traj[sel].time_now+=traj[sel].Time*Per_t;
					traj[sel].time_now=LIMIT(traj[sel].time_now,0,traj[sel].Time);
					get_tra(&traj[sel],traj[sel].time_now);
					flag_sdk[1]=1;
				}
				vmc_all.param.tar_pos_on.x=traj[sel].pt[Xr];		
				vmc_all.param.tar_pos_on.y=traj[sel].pt[Yr];		
					
				 if(dis_final<POS_DEAD)//reach check
				 {flag_sdk[1]=0;return 1;}
				 else
					 return 0;
			}
}


u8 set_pos_api(float x,float y,float z,char mode,float dt)
{
	 float ps[3];
	 float pe[3];
	 char finish;
	 switch(flag_sdk[0]){
		 case 0:
		 ps[0]=nav.pos_n[Xr];ps[1]=nav.pos_n[Yr];
		 pe[0]=x;pe[1]=y;
		 traj_init_task(ps,pe,0,0);
		 flag_sdk[0]=1;
		 flag_sdk[1]=0;
		 break;
		 case 1:
		 finish=follow_traj(0,mode,dt);
		 if(finish){
			 flag_sdk[0]=flag_sdk[1]=0;
		   return 1;
			}	
		 break;
	 }
    return 0;
}
