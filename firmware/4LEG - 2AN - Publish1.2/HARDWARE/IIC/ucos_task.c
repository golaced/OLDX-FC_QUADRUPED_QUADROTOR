#include "include.h" 
#include "bat.h"
#include "imu.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "sbus.h"
#include "rc_mine.h"
#include "filter.h"
#include "dog.h"
#include "vmc.h"
#include "iic_vl53.h"
#include "usart_fc.h"
#include "pwm_out.h"
#include "beep.h"
#include "nav.h"
#include "ms5611.h"
float leg_dt[GET_TIME_NUM];
OS_STK FUSION_TASK_STK[FUSION_STK_SIZE];
char en_nav=1,en_hml=0;
float FLT_ACC=10;
void fusion_task(void *pdata)
{
 u8 i;
 static u8 init;	
 static u16 cnt_init;
 static float timer_baro;
 	while(1)
	{
	leg_dt[0] = Get_Cycle_T(GET_T_LEG1); 				
	IMU_Read(); 														
	IMU_Data_Prepare( leg_dt[0] );		

	Read_Ground_Key(leg_dt[0]);
	for(i=0;i<4;i++)
	  vmc[i].ground_s=ls53[i].mode;
		
	if(cnt_init++>1/0.005){cnt_init=65530;
	madgwick_update_new(leg_dt[0],
		mems.Gyro_deg.x/57.3, mems.Gyro_deg.y/57.3,mems.Gyro_deg.z/57.3, 
		mems.Acc.x, mems.Acc.y, mems.Acc.z,
		mems.Mag.x*module.hml_imu*mems.Mag_Have_Param*en_hml,
		mems.Mag.y*module.hml_imu*mems.Mag_Have_Param*en_hml,
		mems.Mag.z*module.hml_imu*mems.Mag_Have_Param*en_hml,
		&Pitch,&Roll,&Yaw);
	
  float a_br[3],acc_temp[3];
	static float acc_flt[3];
	a_br[0] =(float) mems.Acc.x/4096.;
	a_br[1] =(float) mems.Acc.y/4096.;
	a_br[2] =(float) mems.Acc.z/4096.;
	acc_temp[0] = a_br[1]*reference_vr[2]  - a_br[2]*reference_vr[1] ;
	acc_temp[1] = a_br[2]*reference_vr[0]  - a_br[0]*reference_vr[2] ;
	acc_temp[2] = reference_vr[2] *a_br[2] + reference_vr[0] *a_br[0]+ reference_vr[1] *a_br[1] - 1 ;
	
	vmc_all.att[PITr]=Pitch;
	vmc_all.att[ROLr]=-Roll;
	vmc_all.att[YAWr]=Yaw;		
	#if VIR_MODEL
	  vmc_all.att[YAWr]=nav.fake_yaw;
	#endif
	vmc_all.att_rate[PITr]=mems.Gyro_deg.x;
	vmc_all.att_rate[ROLr]=mems.Gyro_deg.y;
	vmc_all.att_rate[YAWr]=mems.Gyro_deg.z;
	DigitalLPF( acc_temp[0]*9.8, &vmc_all.acc[Xr], FLT_ACC, leg_dt[0]);
	DigitalLPF(-acc_temp[1]*9.8, &vmc_all.acc[Yr], FLT_ACC, leg_dt[0]);
	DigitalLPF( acc_temp[2]*9.8, &vmc_all.acc[Zr], FLT_ACC, leg_dt[0]);
		
	MS5611_Update(leg_dt[0]);
  }
	delay_ms(5);
	}
}		

OS_STK POSE_FUSION_TASK_STK[POSE_FUSION_STK_SIZE];
void pose_fusion_task(void *pdata)
{
 u8 i;
 static u8 init;	
 static u16 cnt_init;
 static float timer_pose_fushion;
 	while(1)
	{
	  leg_dt[GET_T_FUSHION] = Get_Cycle_T(GET_T_FUSHION); 			
		if(cnt_init++>1/0.005){cnt_init=65530;	
    if(en_nav){
    baro_fushion(leg_dt[GET_T_FUSHION]);
		vmc_all.pos_n.z=nav.pos_n[Zr];
			
    timer_pose_fushion+=leg_dt[GET_T_FUSHION];	
    if(timer_pose_fushion>0.03)
			{pose_fushion(timer_pose_fushion);	
			 vmc_all.pos_n.x=nav.pos_n[Xr];
			 vmc_all.pos_n.y=nav.pos_n[Yr];
			 timer_pose_fushion=0;}
	  }
	}
		delay_ms(10);
	}
}		


//========================外环  任务函数============================路径规划
float k_rc_spd=0.002;//速度遥控增益
float k_z_c= 0.08;//旋转遥控增益
OS_STK BRAIN_TASK_STK[BRAIN_STK_SIZE];
float force_time_set=0.4;
float force_time=0.45;
float spd_force=0;//强制给定前进速度
void brain_task(void *pdata)
{	static u8 cnt,cnt1,cnt2,init,rc_update;	
	float T;float spd,spdy,spdx,yaw=0,w_rad;
  static int flag_dP=1;
	int i;
	u16 temps;
	float ero[2];
	float ero_r[2];
 	while(1)
	{	
	leg_dt[4] = Get_Cycle_T(GET_T_BRAIN);								//获取外环准确的执行周期
  T=LIMIT(leg_dt[4],0.001,0.05);//
	if(!init){init=1;	vmc_init();}	
				
	if(vmc_all.param.cal_flag[0]&&module.flash){
	for(i=0;i<4;i++)
		vmc_all.param.ground_force[i][0]=press_leg_end[i+1]*1.068;
		mems.Gyro_CALIBRATE=1;
	vmc_all.param.cal_flag[0]=0;
	}else if(vmc_all.param.cal_flag[1]==1&&module.flash){
		mems.Gyro_CALIBRATE=1;
		vmc_all.param.cal_flag[1]=0;
	}else if(vmc_all.param.cal_flag[1]==2&&module.flash){
		mems.Acc_CALIBRATE=mems.Gyro_CALIBRATE=1;
		vmc_all.param.cal_flag[1]=0;
	}
	
	if(Rc_Get_SBUS.update)//--------------使用SBUS
	{
	  Rc_Get.THROTTLE=LIMIT(Rc_Get_SBUS.THROTTLE,1000,2000)	;
		Rc_Get.ROLL=my_deathzoom_rc(Rc_Get_SBUS.ROLL,2)	;
		Rc_Get.PITCH=my_deathzoom_rc(Rc_Get_SBUS.PITCH,2)	;
		Rc_Get.YAW=my_deathzoom_rc(Rc_Get_SBUS.YAW,2)	;
		Rc_Get.AUX1=Rc_Get_SBUS.AUX1;
		Rc_Get.AUX2=Rc_Get_SBUS.AUX2;
		Rc_Get.AUX3=Rc_Get_SBUS.AUX3;
		Rc_Get.AUX4=Rc_Get_SBUS.AUX4;
		spd=LIMIT(my_deathzoom(Rc_Get.THROTTLE-1500,25)*MAX_SPD/1000.,-MAX_SPD,MAX_SPD);
	  w_rad=LIMIT(my_deathzoom(my_deathzoom(Rc_Get.YAW-1500,25)*k_z_c,1.68),-MAX_SPD_RAD,MAX_SPD_RAD);//degree

	  vmc_all.tar_spd.z=w_rad;
	  vmc_all.tar_spd.y=LIMIT(my_deathzoom(my_deathzoom(Rc_Get.ROLL-1500,25)*k_z_c,1.68),-MAX_SPD_RAD,MAX_SPD_RAD);
    vmc_all.tar_spd.x=spd*2;
	 
	  vmc_all.tar_att[ROLr]=LIMIT(vmc_all.tar_spd.y*2,-3+vmc_all.tar_att_off[ROLr],3+vmc_all.tar_att_off[ROLr]);

	  if(ABS(vmc_all.tar_spd_rc.z)>0.1)
			 vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
		
			 if(vmc_all.tar_spd.x==0&&vmc_all.tar_spd.z==0)
		vmc_all.param.have_cmd_rc=0;
	 else 
		vmc_all.param.have_cmd_rc=1;	
	 static int reg_aux;
	 //power off
	 if(Rc_Get.AUX1<1500&&reg_aux>1500&&vmc_all.sita_test[4]==0){
		  vmc_all.power_state=vmc_all.leg_power=vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x=0;
	    for(int i=0;i<4;i++){
				vmc[i].sita1=0-25;
				vmc[i].sita2=180-(-25);
				#if USE_DEMO
				estimate_end_state_d(&vmc[i],T);
				#else
				estimate_end_state(&vmc[i],T);
				#endif
				vmc[i].ground=1; 
	     }
	 }
	 reg_aux=Rc_Get.AUX1;  
	}
	else if(module.nrf==2){//----------------使用手持遥控器	
	 spd=LIMIT(my_deathzoom(Rc_Get.THROTTLE-1500,25)*MAX_SPD/1000.,-MAX_SPD,MAX_SPD);
	 w_rad=LIMIT(my_deathzoom(my_deathzoom(Rc_Get.YAW-1500,25)*k_z_c,1.68),-MAX_SPD_RAD,MAX_SPD_RAD);//degree

	 {vmc_all.tar_spd.z=-w_rad;vmc_all.tar_spd.y=0;}
	 
	 vmc_all.tar_att[ROLr]=LIMIT(vmc_all.tar_spd.y*2,-3+vmc_all.tar_att_off[ROLr],3+vmc_all.tar_att_off[ROLr]);
	 
	 if(KEY[1]){
		 vmc_all.tar_spd.x+=20*my_deathzoom(spd,0.25*MAX_SPD)*T;
		 vmc_all.tar_spd.x=LIMIT(vmc_all.tar_spd.x,-MAX_SPD*0.95,MAX_SPD*0.95);
	 }else vmc_all.tar_spd.x=spd*2;

	  if(ABS(vmc_all.tar_spd_rc.z)>0.1)
			 vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
		
			 if(vmc_all.tar_spd.x==0&&vmc_all.tar_spd.z==0)
		vmc_all.param.have_cmd_rc=0;
	 else 
		vmc_all.param.have_cmd_rc=1;	
		//power off
	 if(KEY[1]&&KEY[0]&&vmc_all.sita_test[4]==0){
		  vmc_all.power_state=vmc_all.leg_power=vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x=0;
	    for(int i=0;i<4;i++){
				vmc[i].sita1=0-25;
				vmc[i].sita2=180-(-25);
				#if USE_DEMO
				estimate_end_state_d(&vmc[i],T);
				#else
				estimate_end_state(&vmc[i],T);
				#endif
				vmc[i].ground=1; 
	     }
		}
   }else //无遥控速度清零
	  vmc_all.param.have_cmd_rc=vmc_all.tar_spd.x=vmc_all.tar_spd.y=vmc_all.tar_spd.z=vmc_all.tar_att[ROLr]=0;
   	//---------On_board_control------------
	 vmc_all.param.en_sdk=KEY[4];
	 smart_control(T);
	//-------------------------------------
  if(spd_force!=0)//强制速度测试
	 vmc_all.tar_spd.x=spd_force;
	
	if(vmc_all.err==1){
		vmc_all.err=2;
	  IWDG_Init(4,500*1);
	}
	
	fly_ready=LIMIT(vmc_all.leg_power,0,1);
	#if USE_DEMO
	if(power_task_d(T)!=2&&vmc_all.err==0)
	#else
	if(power_task(T)!=2&&vmc_all.err==0)
	#endif
		 IWDG_Feed();
	//vmc_all.leg_power=0;
  leg_power_control(vmc_all.leg_power);
	
	#if DEBUG_MODE
	if(cnt2++>3||0)
	#endif
	{cnt2=0;
	#if USE_DEMO
  	VMC_DEMO(T);
	#else
		VMC_OLDX_VER1(T);
  #endif
	}
	
	Bat_protect(T);
	Set_DJ_PWM();
	delay_ms(5);
	}
}		

//=======================串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=1;//<------------------------------UART UPLOAD DATA SEL
float ground_check_show=0;
void uart_task(void *pdata)
{	static u8 cnt[4];	
 	while(1)
	{		leg_dt[1] = Get_Cycle_T(GET_T_LEG2); 		
			switch(UART_UP_LOAD_SEL)
			{
			case 0:
			data_per_uart_rc(
			ls53[0].mode*100,-ls53[1].mode*100,ls53[2].mode*200,
			-ls53[3].mode*200,ls53[0].distance,ls53[0].ambient/10,
			press_leg_end[2]*100,press_leg_end[3]*100,0,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;		
			case 1:
			data_per_uart_rc(
			vmc_all.body_spd[Zr]*1000,vmc[0].epos.z*1000,vmc[1].epos.z*1000,
			vmc[0].tar_epos.z*1000,vmc[0].spd.x*1000,vmc[0].tar_spd.x*1000,
			(vmc_all.deng_all*0.0001)*1000,vmc_all.tar_pos.z*1000,vmc_all.pos.z*1000,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;	
		  case 2:
			data_per_uart_rc(
			ls53[0].distance,ls53[1].distance,ls53[2].distance,
			ls53[3].distance,ls53[0].ambient/10,ls53[1].ambient/10,
			ls53[2].ambient/10,ls53[3].ambient/10,0,
			(int16_t)(Pitch*10),(int16_t)(Roll*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;			
			case 3:
			data_per_uart_rc(
			vmc_all.tar_pos.z*1000,vmc_all.pos.z*1000,0,
		  vmc_all.tar_att[YAWr],vmc_all.att_ctrl[YAWr],0,
			vmc_all.tar_spd.z,vmc_all.att_rate_ctrl[YAWr],0,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;	
		  case 4:
			data_per_uart_rc(
			vmc_all.att_ctrl[ROLr]*10,vmc_all.att[ROLr]*10,(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr])*10,
		  vmc_all.att_rate_ctrl[ROLr]*10,vmc_all.att_rate[ROLr]*10,vmc_all.tar_att_rate[ROLr]*10,
			vmc_all.tar_pos.z*1000,vmc_all.pos.z*1000,vmc_all.param.w_t[ROLr]*100,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 5:
			data_per_uart_rc(
			nav.acc_b_o[Xr]*100,nav.acc_b_o[Yr]*100,0,
		  nav.acc_n_o[Xr]*100,nav.acc_n_o[Yr]*100,0,
			vmc_all.tar_pos.z*1000,vmc_all.pos.z*1000,vmc_all.param.w_t[ROLr]*100,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
      case 6:
			data_per_uart_rc(
			vmc[0].ground_force[0]*100,vmc[1].ground_force[0]*100,vmc[2].ground_force[0]*100,
		  vmc[3].ground_force[0]*100,ground_check_show*100,vmc[0].ground_force[1]*100,
			vmc[1].ground_force[1]*100,vmc[2].ground_force[1]*100,vmc[3].ground_force[1]*100,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 7:
			data_per_uart_rc(
			press_leg_end[1]*100,press_leg_end[2]*100,press_leg_end[3]*100,
		  press_leg_end[4]*100,ground_check_show*100,vmc[0].ground_force[1]*100,
			vmc[1].ground_force[1]*100,vmc[2].ground_force[1]*100,vmc[3].ground_force[1]*100,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 8:
			data_per_uart_rc(
			vmc_all.param.line_z[0]*1000,vmc_all.param.line_z[1]*1000,press_leg_end[3]*100,
		  vmc[0].epos.x*100,vmc[1].epos.x*100,vmc[0].ground_force[1]*100,
			vmc[1].ground_force[1]*100,vmc[2].ground_force[1]*100,vmc[3].ground_force[1]*100,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 9:
			data_per_uart_rc(
			vmc_all.acc[Zr]*100,vmc_all.body_spd[Zr]*100,nav.spd_n[Zr]*100,
			ms5611.baroAlt_flt,ms5611.baroAlt-ms5611.baroAlt_off,nav.pos_n[Zr]*100,
			nav.acc_n[Zr]*10,ms5611.baroAlt_flt1*100,0,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			case 10:
			data_per_uart_rc(
			nav.spd_b[Xr]*100,nav.spd_b_o[Yr]*100,nav.spd_b[Yr]*100,
			nav.pos_n[Xr]*100,nav.pos_n[Yr]*100,nav.att[YAWr]*10,
			vmc_all.att_rate[Zr],0,nav.spd_b_o[Zr],
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;

			default:break;
			}
			
			Nrf_Check_Event();
			RC_Send_Task();		
     
			if(cnt[0]++>0){cnt[0]=0;
			#if defined (LEG_USE_VLX)	
				READ_VL53(0);
				READ_VL53(1);
				READ_VL53(2);
				READ_VL53(3);
			#endif
			#if defined (LEG_USE_VL6)
				RangePollingRead();
			#endif
						
				#if EN_DMA_UART1 					
				if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)
					{ 
						DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		
						clear_leg_uart();
						data_per_uart1();
						USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);      
						MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);    
					}	
				#endif			
			}
			delay_ms(10);  
	}
}	


//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
 		
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}

//软件定时器2的回调函数				  50ms	 
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
u8 i;	
static u16 cnt_1,cnt_2;	
static u8 cnt;
	LEDRGB_STATE(0.05);
	if(Rc_Get_SBUS.lose_cnt++>2/0.05)Rc_Get_SBUS.connect=0;
	
	if(vmc_all.param.rc_mode[1]++>125)vmc_all.param.rc_mode[0]=0;
	if(pi.lost_cnt++>3/0.05)pi.connect=0;
	if(flow.lost_cnt++>3/0.05)flow.connect=0;
}
