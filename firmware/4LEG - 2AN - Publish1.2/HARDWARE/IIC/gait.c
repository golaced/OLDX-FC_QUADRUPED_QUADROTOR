#include "leg.h"
#include "vmc.h"
#include "my_math.h"
#include "math.h"
#include "bat.h"
#include "eso.h"
//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//
//------------------------------------状态机-------------------------------------//
//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//
//VMC Control Task
float test_force[3]={0};
//虚拟力输出测试  
void  VMC_Control_test1(float dt)
{

 static char init;	
 char i;
	if(!init)
	{
	  init=1;
		vmc_init();
	}
   estimate_end_state(&vmc[FL],dt);estimate_end_state(&vmc[BL],dt);
    
	 cal_jacobi(&vmc[FL]);cal_jacobi(&vmc[BL]);
    
	 //test force
	 vmc[FL].force[Xr]=vmc[BL].force[Xr]=test_force[Xr];//N
	 vmc[FL].force[Zr]=vmc[BL].force[Zr]=test_force[Zr];//N
	 cal_torque(&vmc[FL]);cal_torque(&vmc[BL]);
	 
	 //out 
	 vmc[FL].sita1+=vmc[FL].param.spd_dj[0]*dt;
	 vmc[FL].sita2+=vmc[FL].param.spd_dj[1]*dt;
	 vmc[BL].sita1+=vmc[BL].param.spd_dj[0]*dt;
	 vmc[BL].sita2+=vmc[BL].param.spd_dj[1]*dt;
	 LIMIT(vmc[FL].sita1,-45,90);LIMIT(vmc[FL].sita2,90,215);
	 LIMIT(vmc[BL].sita1,-45,90);LIMIT(vmc[BL].sita2,90,215);
	 
	 convert_mine_to_vmc(&vmc[FL]);convert_mine_to_vmc(&vmc[FL]);
	 convert_mine_to_vmc(&vmc[BL]);convert_mine_to_vmc(&vmc[BL]);
}

//跨腿测试
char trig_test=0;
float pos_test[4]={0,-0.12};
void  VMC_Control_test2(float dt)
{
 static char init,state;
 static u16 time,time1;
 float out[2];	
 char i;
	if(!init)
	{
	  init=1;
		vmc_init();
	}
	state_rst(dt);
	
	estimate_end_state(&vmc[FL1],dt);estimate_end_state(&vmc[BL1],dt);
  estimate_end_state(&vmc[FL2],dt);estimate_end_state(&vmc[BL2],dt);	
	switch(state)
	{
		case 0:
		if(trig_test)	
		{
		  state=1;
			for(i=0;i<4;i++){
			vmc[i].st_pos.x=vmc[i].epos.x;
			vmc[i].st_pos.z=vmc[i].epos.z;
			
			vmc[i].tar_epos.x=LIMIT(pos_test[0],MIN_X,MAX_X);
			vmc[i].tar_epos.z=LIMIT(pos_test[1],MAX_Z,MIN_Z);
			vmc[i].param.time_trig=0;time=time1=0;
			cal_curve_from_pos_new(&vmc[FL1],vmc[FL1].tar_pos, vmc_all.gait_time-vmc_all.stance_time);
			cal_curve_from_pos_new(&vmc[BL1],vmc[BL1].tar_pos, vmc_all.gait_time-vmc_all.stance_time);
			cal_curve_from_pos_new(&vmc[FL2],vmc[FL2].tar_pos, vmc_all.gait_time-vmc_all.stance_time);
		  cal_curve_from_pos_new(&vmc[BL2],vmc[BL2].tar_pos, vmc_all.gait_time-vmc_all.stance_time);
			}
		}
		break;
		case 1:
		trig_curve(&vmc[FL1],&vmc[FL1].param.tar_epos.x,&vmc[FL1].param.tar_epos.z,dt,vmc_all.trig_mode);
		inv_end_state(vmc[FL1].param.tar_epos.x,vmc[FL1].param.tar_epos.z,&vmc[FL1].sita1,&vmc[FL1].sita2);

		trig_curve(&vmc[FL2],&vmc[FL2].param.tar_epos.x,&vmc[FL2].param.tar_epos.z,dt,vmc_all.trig_mode);
		inv_end_state(vmc[FL2].param.tar_epos.x,vmc[FL2].param.tar_epos.z,&vmc[FL2].sita1,&vmc[FL2].sita2);
		
		trig_curve(&vmc[BL1],&vmc[BL1].param.tar_epos.x,&vmc[BL1].param.tar_epos.z,dt,vmc_all.trig_mode);
		inv_end_state(vmc[BL1].param.tar_epos.x,vmc[BL1].param.tar_epos.z,&vmc[BL1].sita1,&vmc[BL1].sita2);
		
		if(trig_curve(&vmc[BL2],&vmc[BL2].param.tar_epos.x,&vmc[BL2].param.tar_epos.z,dt,vmc_all.trig_mode))
			state=2;
		inv_end_state(vmc[BL2].param.tar_epos.x,vmc[BL2].param.tar_epos.z,&vmc[BL2].sita1,&vmc[BL2].sita2);
			if(time1++>0.1/dt)
				 time1=0 ;
		break;
		case 2:
			 if(time++>3/dt)
	        state=0;
		break;
	}
	//output
	  convert_mine_to_vmc(&vmc[FL1]);
		convert_mine_to_vmc(&vmc[BL1]);
		convert_mine_to_vmc(&vmc[FL2]);
		convert_mine_to_vmc(&vmc[BL2]);
}

//正逆解测试
float sita_inv[2];
void  VMC_Control_test3(float dt)
{
 static char init,state;
 static u16 time,time1;
 float out[2];	
 char i;
	if(!init)
	{
	  init=1;
		vmc_init();
	}
	state_rst(dt);
	estimate_end_state(&vmc[FL1],dt);
	inv_end_state(vmc[FL1].epos.x,vmc[FL1].epos.z,&sita_inv[0],&sita_inv[1]);
}



//基础步态测试  USE
void  VMC_Control_test4(float dt)
{
 static char init=0,state=0,switch_flag=0,cnt_time_change=0;
 static u16 time,time1;
 float out[2];	
 char i;
 static char front_leg,back_leg;
	if(!init)
	{
	  init=1;
		vmc_init();
	}
	state_rst(dt);
	//估计足末状态
	estimate_end_state(&vmc[FL1],dt);estimate_end_state(&vmc[BL2],dt);
	estimate_end_state(&vmc[FL2],dt);estimate_end_state(&vmc[BL1],dt);
	switch(state)
	{
		case 0:
		if(ABS(vmc_all.tar_spd.x)>MIN_SPD_ST||ABS(vmc_all.tar_spd.z)>MIN_SPD_ST||ABS(vmc_all.tar_spd.y)>MIN_SPD_ST||vmc_all.gait_on){
				cnt_time_change++;
			  if(cnt_time_change>=3){
			  vmc_all.stance_time=vmc_all.stance_time_auto;		
			  vmc_all.delay_time[2]=vmc_all.delay_time[1];			
				cnt_time_change=0;}
			//规划落足点
			if(switch_flag)	
			{
				cal_tar_end_pos(&vmc[FL1]);cal_tar_end_pos(&vmc[BL2]);
				vmc[FL1].st_pos.x=vmc[FL1].epos.x;
				vmc[FL1].st_pos.z=vmc[FL1].epos.z;
				vmc[FL1].tar_pos.x=vmc[FL1].tar_epos.x;
				vmc[FL1].tar_pos.z=vmc[FL1].tar_epos.z;
				vmc[FL1].param.time_trig=0;
				vmc[BL2].st_pos.x=vmc[BL2].epos.x;
				vmc[BL2].st_pos.z=vmc[BL2].epos.z;
				vmc[BL2].tar_pos.x=vmc[BL2].tar_epos.x;
				vmc[BL2].tar_pos.z=vmc[BL2].tar_epos.z;
				vmc[BL2].param.time_trig=0;
				vmc[BL2].param.trig_state=vmc[FL1].param.trig_state=0;
				vmc[BL2].param.ground_state=vmc[FL1].param.ground_state=0;
				vmc[FL2].force_deng[1]=vmc[BL1].force_deng[1]=0;
				vmc[FL2].force_deng[2]=vmc[BL1].force_deng[2]=1;
				vmc[FL1].param.delta_h_att_off=vmc_all.delta_h_att_off;
				vmc[BL2].param.delta_h_att_off=vmc_all.delta_h_att_off;
				
				cal_curve_from_pos_new(&vmc[FL1],vmc[FL1].tar_pos, vmc_all.gait_time-vmc_all.stance_time);
			  cal_curve_from_pos_new(&vmc[BL2],vmc[BL2].tar_pos, vmc_all.gait_time-vmc_all.stance_time);
			}
			else
			{
				cal_tar_end_pos(&vmc[FL2]);cal_tar_end_pos(&vmc[BL1]);
				vmc[FL2].st_pos.x=vmc[FL2].epos.x;
				vmc[FL2].st_pos.z=vmc[FL2].epos.z;
				vmc[FL2].tar_pos.x=vmc[FL2].tar_epos.x;
				vmc[FL2].tar_pos.z=vmc[FL2].tar_epos.z;
				vmc[FL2].param.time_trig=0;
				vmc[BL1].st_pos.x=vmc[BL1].epos.x;
				vmc[BL1].st_pos.z=vmc[BL1].epos.z;
				vmc[BL1].tar_pos.x=vmc[BL1].tar_epos.x;
				vmc[BL1].tar_pos.z=vmc[BL1].tar_epos.z;
				vmc[BL1].param.time_trig=0;
				vmc[BL1].param.trig_state=vmc[FL2].param.trig_state=0;
				vmc[BL1].param.ground_state=vmc[FL2].param.ground_state=0;
				vmc[FL1].force_deng[1]=vmc[BL2].force_deng[1]=0;
				vmc[FL1].force_deng[2]=vmc[BL2].force_deng[2]=1;
				vmc[FL2].param.delta_h_att_off=vmc_all.delta_h_att_off;
				vmc[BL1].param.delta_h_att_off=vmc_all.delta_h_att_off;
				
				cal_curve_from_pos_new(&vmc[FL2],vmc[FL2].tar_pos, vmc_all.gait_time-vmc_all.stance_time);
			  cal_curve_from_pos_new(&vmc[BL1],vmc[BL1].tar_pos, vmc_all.gait_time-vmc_all.stance_time);
			}	
			state++;
	  }
		break;
		case 1:
			//跨腿
			if(switch_flag)	
			{
				trig_curve(&vmc[FL1],&vmc[FL1].param.tar_epos.x,&vmc[FL1].param.tar_epos.z,dt,vmc_all.trig_mode);
        vmc[FL1].param.tar_epos.z=LIMIT(vmc[FL1].param.tar_epos.z,MAX_Z,MIN_Z);vmc[FL1].param.tar_epos.x=LIMIT(vmc[FL1].param.tar_epos.x,MIN_X,MAX_X);
				trig_curve(&vmc[BL2],&vmc[BL2].param.tar_epos.x,&vmc[BL2].param.tar_epos.z,dt,vmc_all.trig_mode);
				vmc[BL2].param.tar_epos.z=LIMIT(vmc[BL2].param.tar_epos.z,MAX_Z,MIN_Z);vmc[BL2].param.tar_epos.x=LIMIT(vmc[BL2].param.tar_epos.x,MIN_X,MAX_X);
				inv_end_state(vmc[FL1].param.tar_epos.x,vmc[FL1].param.tar_epos.z,&vmc[FL1].sita1,&vmc[FL1].sita2);
				inv_end_state(vmc[BL2].param.tar_epos.x,vmc[BL2].param.tar_epos.z,&vmc[BL2].sita1,&vmc[BL2].sita2);
				if(vmc[FL1].ground&&vmc[FL2].ground&&vmc[BL1].ground&&vmc[BL2].ground)
				{state++;vmc[FL1].param.trig_state=vmc[BL2].param.trig_state=0;
				 vmc[FL1].param.ground_state=vmc[BL2].param.ground_state=1;
				}
			}
			else
			{
				trig_curve(&vmc[FL2],&vmc[FL2].param.tar_epos.x,&vmc[FL2].param.tar_epos.z,dt,vmc_all.trig_mode);
				vmc[FL1].param.tar_epos.z=LIMIT(vmc[FL1].param.tar_epos.z,MAX_Z,MIN_Z);vmc[FL1].param.tar_epos.x=LIMIT(vmc[FL1].param.tar_epos.x,MIN_X,MAX_X);
				trig_curve(&vmc[BL1],&vmc[BL1].param.tar_epos.x,&vmc[BL1].param.tar_epos.z,dt,vmc_all.trig_mode);
				vmc[BL1].param.tar_epos.z=LIMIT(vmc[BL1].param.tar_epos.z,MAX_Z,MIN_Z);vmc[BL1].param.tar_epos.x=LIMIT(vmc[BL1].param.tar_epos.x,MIN_X,MAX_X);
				inv_end_state(vmc[FL2].param.tar_epos.x,vmc[FL2].param.tar_epos.z,&vmc[FL2].sita1,&vmc[FL2].sita2);
				inv_end_state(vmc[BL1].param.tar_epos.x,vmc[BL1].param.tar_epos.z,&vmc[BL1].sita1,&vmc[BL1].sita2);
				if(vmc[FL1].ground&&vmc[FL2].ground&&vmc[BL1].ground&&vmc[BL2].ground)
					 {state++;vmc[FL2].param.trig_state=vmc[BL1].param.trig_state=0;
					 vmc[FL2].param.ground_state=vmc[BL1].param.ground_state=1;
					 }
			}	
			if(time1++>0.1/dt)
				 time1=0 ;
		break;
		case 2:
			//四足着地延时
      if(time++>vmc_all.delay_time[2]/dt)
			  {time=0 ;state++;}
    break;		
		case 3:
			 switch_flag=!switch_flag;
		   state=0;
		   vmc_all.gait_on=0;
		break;
	}
	
	 //蹬腿
   for(i=0;i<4;i++){
		   //姿态控制
		 if(vmc_all.use_att==1&&vmc_all.power_state==3)
			 if(vmc_all.param.control_mode==0||(vmc_all.param.control_mode==2&&vmc[i].ground==0)){
				  attitude_control(&vmc[i],dt);
       }

		 if(vmc[i].ground){
				if(vmc_all.param.control_mode==1||vmc_all.param.control_mode==2){
		     attitude_control_newp(&vmc[i],dt);
			   attitude_control_newr(&vmc[i],dt);
		     height_ctrl(&vmc[i],dt);
			  }
				
				if(vmc_all.param.control_mode==3&&vmc_all.use_att==1&&vmc_all.power_state==3){
				  attitude_control(&vmc[i],dt);
       }
				 
			 //跨腿下蹬 
			 if(vmc[i].force_deng[2]==1){
			 vmc[i].force_deng[0]=-cosd(vmc[i].force_deng[1])*vmc_all.kp_deng[i]*0.4/(vmc_all.gait_time+0.0000001); 
			 vmc[i].force_deng[1]+=180/(vmc_all.stance_time+vmc_all.delay_time[2]+vmc_all.gait_delay_time)*dt;
			 if(vmc[i].force_deng[1]>179.8){
				 vmc[i].force_deng[2]=vmc[i].force_deng[1]=0;
				}
			 }else 
			 vmc[i].force_deng[1]=vmc[i].force_deng[0]=0;
			 
			 cal_jacobi(&vmc[i]);
				
			 //反馈控制
			 if(vmc_all.power_state==3&&!DEBUG_MODE)
			   cal_force_y(&vmc[i],dt);
			 cal_force_z(&vmc[i],dt);	
			 		 
			 trig_extend(&vmc[i],dt);
			 out_range_protect();
			 //力测试
			 if(test_force[Xr]!=0)
				vmc[i].force[Xr]=test_force[Xr];
			 if(test_force[Zr]!=0)
				vmc[i].force[Zr]=test_force[Zr];	
			 
			 if(fabs(vmc_all.tar_spd.x)>0.05||1)
			  vmc[i].force[Zr]+=vmc[i].force_deng[0];
			 
			 //力矩输出
			 cal_torque(&vmc[i]);
			 vmc[i].sita1+=vmc[i].param.spd_dj[0]*dt;
			 vmc[i].sita2+=vmc[i].param.spd_dj[1]*dt;
			 LIMIT(vmc[i].sita1,-40,90);LIMIT(vmc[i].sita2,90,210);
		 }
   }	 
	 //计算PWM
	 if(vmc_all.sita_test[4]){//强制角度测试
	 for(i=1;i<5;i++){
		 leg[i].sita[0]=vmc_all.sita_test[0];
		 leg[i].sita[1]=vmc_all.sita_test[1];
		 leg[i].sita[2]=vmc_all.sita_test[2];
		 leg[i].sita[3]=vmc_all.sita_test[3];
		 cal_pwm_from_sita(&leg[i]);//计算PWM由角度	
	 }
   }else{
		 convert_mine_to_vmc(&vmc[FL1]);convert_mine_to_vmc(&vmc[BL1]);
		 convert_mine_to_vmc(&vmc[FL2]);convert_mine_to_vmc(&vmc[BL2]);
   }		 
}


//基础步态测试2 跨腿延时 四腿独立分离
void  VMC_Control_test5(float dt)
{
 static char init=0,state=0,state_trig[4]={0},switch_flag=0,en_gait[4]={0},cnt_time_change;
 static u16 time,time1,time_gait[4],time_gait_delay[4];
 static float time_safe;
 float out[2];	
 char i;
 static char front_leg,back_leg,state_reg;
 
if(!init)
{
	init=1;
	vmc_init();
}
/**-----------------------------------------------------------------------------**/			
	//状态复位
	state_rst(dt);
	//估计足末状态
	estimate_end_state(&vmc[FL1],dt);estimate_end_state(&vmc[BL2],dt);
	estimate_end_state(&vmc[FL2],dt);estimate_end_state(&vmc[BL1],dt);
/**-----------------------------------------------------------------------------**/			
		//主状态机
		switch(state)
		{
			case 0:
			if((ABS(vmc_all.tar_spd.x)>0.01||ABS(vmc_all.tar_spd.y)>0.01||ABS(vmc_all.tar_spd.z)>0.01||vmc_all.gait_on)
				&&vmc_all.power_state==3){
				cnt_time_change++;
			  if(cnt_time_change>=3){
			  vmc_all.stance_time=vmc_all.stance_time_auto;	
        vmc_all.delay_time[2]=vmc_all.delay_time[1];					
				cnt_time_change=0;}
				//前后腿判断
				if(ABS(vmc_all.tar_spd.x)>0.01){
				 if(vmc_all.tar_spd.x>0)//forward
					if(switch_flag)
						front_leg=FL1;
					else
						front_leg=FL2;
				 else								   //backward
					if(switch_flag)
						front_leg=BL2;
					else
						front_leg=BL1; 
				}else if(ABS(vmc_all.tar_spd.z)>0.01){
					if(vmc_all.tar_spd.z>0)//right
					if(switch_flag)
						front_leg=FL1;
					else
						front_leg=FL2;
				 else								   //left
					if(switch_flag)
						front_leg=FL2;
					else
						front_leg=FL1; 
				}else {
					if(switch_flag)
						front_leg=FL1;
					else
						front_leg=FL2;}
				 
				//规划落足点 触发跨腿
				if(switch_flag)	//FL1 BL2
				{
					if(front_leg==FL1){
					trig_set(&vmc[FL1]);
					state_reg=11;
					en_gait[FL1]=1;	
					vmc[BL2].force_deng[1]=0;//额外着地腿
					vmc[BL2].force_deng[2]=1;	
					}
					else{
					trig_set(&vmc[BL2]);
					state_reg=12;
					en_gait[BL2]=1;	
					vmc[FL1].force_deng[1]=0;
					vmc[FL1].force_deng[2]=1;	
					}
					
					vmc[FL2].force_deng[1]=vmc[BL1].force_deng[1]=0;//clear angle
					vmc[FL2].force_deng[2]=vmc[BL1].force_deng[2]=1;//en deng
				}
				else  //FL2 BL1
				{
					if(front_leg==FL2){
					trig_set(&vmc[FL2]);
					state_reg=14;
					en_gait[FL2]=1;	
					vmc[BL1].force_deng[1]=0;
					vmc[BL1].force_deng[2]=1;	
					}else{
					trig_set(&vmc[BL1]);
					state_reg=15;
					en_gait[BL1]=1;	
					vmc[FL2].force_deng[1]=0;
					vmc[FL2].force_deng[2]=1;	
					}
					vmc[FL1].force_deng[1]=vmc[BL2].force_deng[1]=0;
					vmc[FL1].force_deng[2]=vmc[BL2].force_deng[2]=1;
				}	
				state=state_reg;
				for(i=0;i<4;i++)
					time_gait[i]=0;
				time_safe=time=0;
			}
			break;
			//---------switch phase 1
			case 11://FL1 first
				if(time_gait[BL2]++>vmc_all.gait_delay_time/dt)
				{trig_set(&vmc[BL2]);en_gait[BL2]=1;	
				 time_gait[BL2]=0;
				 state=97;
				 vmc[BL2].force_deng[2]=vmc[BL2].force_deng[1]=0;//停止下蹬
				}
			break;
			case 12://BL2 first
				if(time_gait[FL1]++>vmc_all.gait_delay_time/dt)
				{trig_set(&vmc[FL1]);en_gait[FL1]=1;	
				 time_gait[FL1]=0;
				 state=97;
				 vmc[FL1].force_deng[2]=vmc[FL1].force_deng[1]=0;
				}
			break;	
			//---------switch phase 0	
			case 14://FL2 first
				if(time_gait[BL1]++>vmc_all.gait_delay_time/dt)
				{trig_set(&vmc[BL1]);en_gait[BL1]=1;	
				 time_gait[BL1]=0;
				 state=97;
				 vmc[BL1].force_deng[2]=vmc[BL1].force_deng[1]=0;
				}
			break;
			case 15://BL1 first
				if(time_gait[FL2]++>vmc_all.gait_delay_time/dt)
				{trig_set(&vmc[FL2]);en_gait[FL2]=1;
				 time_gait[FL2]=0;
				 state=97;
				 vmc[FL2].force_deng[2]=vmc[FL2].force_deng[1]=0;
				}
			break;	
			case 97://safe delay
				  time_safe+=dt;
				  if(time_safe>vmc_all.gait_delay_time*0.3){
						time_safe=0;state=98;
				  }
		  break;		
			case 98://all ground check
				  time_safe+=dt;
					if(vmc[FL1].ground&&vmc[FL2].ground&&vmc[BL1].ground&&vmc[BL2].ground)
					{time_safe=0;state=99;}
					if(time_safe>vmc_all.gait_time+vmc_all.gait_delay_time+vmc_all.delay_time[2])
					{time_safe=0;state=99;}
		  break;
			case 99://切换组
				 switch_flag=!switch_flag;
				 state=0;
			break;
		}
		
/**-----------------------------------------------------------------------------**/
    if(vmc_all.power_state==3){//已经启动		
		//跨腿
		for(i=0;i<4;i++)
		{
			switch(state_trig[i])
			{
				case 0:
					if(en_gait[i])
					 state_trig[i]=1;
				break;
				case 1://follow trig
					trig_curve(&vmc[i],&vmc[i].param.tar_epos.x,&vmc[i].param.tar_epos.z,dt,vmc_all.trig_mode);
					inv_end_state(vmc[i].param.tar_epos.x,vmc[i].param.tar_epos.z,&vmc[i].sita1,&vmc[i].sita2);
					if(vmc[i].ground)
					{state_trig[i]=1;time_gait_delay[i]=0;}
				break;
				case 2://delay after trig finish
					if(time_gait_delay[i]++>vmc_all.delay_time[2]/dt)
					{
						time_gait_delay[i]=0;
					  state_trig[i]=0;
						en_gait[i]=0;
					}
				break;
			}	
		}
	}
/**----------------------------------蹬腿----------------------------------------**/		
	 //attitude_control(dt);//姿态控制
	
   for(i=0;i<4;i++){
		   //姿态控制
		  // cal_att2(&vmc[i],dt);//test 1
		 if(vmc[i].ground){
			 //跨腿下蹬
			 if(vmc[i].force_deng[2]==1){
			 vmc[i].force_deng[0]=-cosd(vmc[i].force_deng[1])*vmc_all.kp_deng[i]; 
			 vmc[i].force_deng[1]+=180/(vmc_all.stance_time+vmc_all.delay_time[2]+vmc_all.gait_delay_time)*dt;
			 if(vmc[i].force_deng[1]>180)
				 vmc[i].force_deng[2]=vmc[i].force_deng[1]=0;
			 }else 
			 vmc[i].force_deng[1]=vmc[i].force_deng[0]=0;
			 
			 cal_jacobi(&vmc[i]);
			 
			 //反馈控制
			 cal_force_y(&vmc[i],dt);
			 cal_force_z(&vmc[i],dt);	
				
			 vmc[i].force[Zr]+=vmc[i].force_deng[0];
			 	 
			 //力矩输出
			 cal_torque(&vmc[i]);
			 vmc[i].sita1+=vmc[i].param.spd_dj[0]*dt;
			 vmc[i].sita2+=vmc[i].param.spd_dj[1]*dt;
			 LIMIT(vmc[i].sita1,-45,90);LIMIT(vmc[i].sita2,90,215);
		 }
   }
	 
	 //计算PWM
	 if(vmc_all.sita_test[4]){//强制角度测试
			 for(i=1;i<5;i++){
				 leg[1].sita[0]=vmc_all.sita_test[0];
				 leg[1].sita[1]=vmc_all.sita_test[1];
				 leg[1].sita[2]=vmc_all.sita_test[2];
				 leg[1].sita[3]=vmc_all.sita_test[3];
				 cal_pwm_from_sita(&leg[i]);//计算PWM由角度	
				}
   }else{
		 convert_mine_to_vmc(&vmc[FL1]);convert_mine_to_vmc(&vmc[BL1]);
		 convert_mine_to_vmc(&vmc[FL2]);convert_mine_to_vmc(&vmc[BL2]);
   }		 
}

