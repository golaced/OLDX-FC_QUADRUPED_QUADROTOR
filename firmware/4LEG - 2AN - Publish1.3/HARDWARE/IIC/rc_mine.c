#include "include.h"
#include "imu.h"
#include "nrf.h"
#include "rc.h"
#include "bat.h"
#include "usart_fc.h"
#include "imu.h"
#include "time.h"
#include "eso.h"
#include "rc_mine.h"
#include "vmc.h"
#include "nav.h"
MOUDLE module;
int PARAM_NRF[18][3];
u8 KEY[8];
RC_GETDATA Rc_Get;
u8 send_pid=0,read_pid=0;;
u16 data_rate;
u8 key_rc_reg[7][MID_RC_KEY];
float RC_GET[4][MID_RC_GET];
void NRF_DataAnl(void)
{ 

u8 temp;
	u8 i=0,j,sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_RXDATA[i];
	if(!(sum==NRF24L01_RXDATA[31]))	
	{i=0;
		return;
		}	 
	if(NRF24L01_RXDATA[0]==0x01)								
	{ 
		module.nrf=2;
		data_rate++;
		Rc_Get.THROTTLE = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		Rc_Get.YAW			= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		Rc_Get.ROLL		  = (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		Rc_Get.PITCH 		= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		Rc_Get.AUX1			= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		Rc_Get.AUX2			= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];
		Rc_Get.AUX3			= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		Rc_Get.AUX4			= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		Rc_Get.AUX5			= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];
		
	  KEY[0]=(NRF24L01_RXDATA[19])&0x01;
		KEY[1]=(NRF24L01_RXDATA[19]>>1)&0x01;
		KEY[2]=(NRF24L01_RXDATA[19]>>2)&0x01;
		KEY[3]=(NRF24L01_RXDATA[19]>>3)&0x01;
		KEY[4]=(NRF24L01_RXDATA[19]>>4)&0x01;
		
		temp=NRF24L01_RXDATA[21];
		if(temp==1)
			read_pid=1;
	 }
	else if(NRF24L01_RXDATA[0]==0x02)//pid1
	{
	  PARAM_NRF[0][0] = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		PARAM_NRF[0][1]	= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		PARAM_NRF[0][2]	= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		PARAM_NRF[1][0] = (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		PARAM_NRF[1][1]	= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		PARAM_NRF[1][2]	= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		PARAM_NRF[2][0] = (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		PARAM_NRF[2][1]	= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		PARAM_NRF[2][2]	= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		PARAM_NRF[3][0] = (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		PARAM_NRF[3][1]	= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		PARAM_NRF[3][2]	= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		
		PARAM_NRF[4][0] = (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		PARAM_NRF[4][1]	= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
		PARAM_NRF[4][2]	= (vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30];
	
	}
	else if(NRF24L01_RXDATA[0]==0x03)//pid2
	{
	  PARAM_NRF[5][0] = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		PARAM_NRF[5][1]	= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		PARAM_NRF[5][2]	= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		PARAM_NRF[6][0] = (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		PARAM_NRF[6][1]	= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		PARAM_NRF[6][2]	= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		PARAM_NRF[7][0] = (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		PARAM_NRF[7][1]	= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		PARAM_NRF[7][2]	= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		PARAM_NRF[8][0] = (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		PARAM_NRF[8][1]	= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		PARAM_NRF[8][2]	= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		
		PARAM_NRF[9][0] = (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		PARAM_NRF[9][1]	= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
		PARAM_NRF[9][2]	= (vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30];
	}
	else if(NRF24L01_RXDATA[0]==0x04)//pid3
	{
	  PARAM_NRF[10][0] = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		PARAM_NRF[10][1]	= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		PARAM_NRF[10][2]	= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		PARAM_NRF[11][0] = (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		PARAM_NRF[11][1]	= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		PARAM_NRF[11][2]	= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		PARAM_NRF[12][0] = (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		PARAM_NRF[12][1]	= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		PARAM_NRF[12][2]	= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		PARAM_NRF[13][0] = (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];
		PARAM_NRF[13][1]	= (vs16)(NRF24L01_RXDATA[21]<<8)|NRF24L01_RXDATA[22];
		PARAM_NRF[13][2]	= (vs16)(NRF24L01_RXDATA[23]<<8)|NRF24L01_RXDATA[24];
		
		PARAM_NRF[14][0] = (vs16)(NRF24L01_RXDATA[25]<<8)|NRF24L01_RXDATA[26];
		PARAM_NRF[14][1]	= (vs16)(NRF24L01_RXDATA[27]<<8)|NRF24L01_RXDATA[28];
		PARAM_NRF[14][2]	= (vs16)(NRF24L01_RXDATA[29]<<8)|NRF24L01_RXDATA[30];
	}
	else if(NRF24L01_RXDATA[0]==0x05)//pid3
	{
	  PARAM_NRF[15][0] = (vs16)(NRF24L01_RXDATA[1]<<8)|NRF24L01_RXDATA[2];
		PARAM_NRF[15][1]	= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		PARAM_NRF[15][2]	= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		
		PARAM_NRF[16][0] = (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		PARAM_NRF[16][1]	= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
		PARAM_NRF[16][2]	= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];

		PARAM_NRF[17][0] = (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		PARAM_NRF[17][1]	= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		PARAM_NRF[17][2]	= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];

		send_pid=1;
	}
}


void Nrf_Check_Event(void)
{ 
	u8 rx_len =0;
	static u16 cnt_loss_rc=0;
	u8 sta;

	sta= NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);		
	if(sta & (1<<RX_DR))	 
	{ 
		cnt_loss_rc=0;
			
				rx_len= NRF_Read_Reg(R_RX_PL_WID);			
				if(rx_len<33)
				{
					NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
					NRF_DataAnl();	
				}
				else 
				{ 
					NRF_Write_Reg(FLUSH_RX,0xff );
				}
		}
	else//---------losing_nrf
	 {	
		 if(cnt_loss_rc++>200 )//0.5ms
		 {	
			 NRF_Write_Reg(FLUSH_RX,0xff);

		 }
  }
	if(sta & (1<<TX_DS))	
	{
	
	}

	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
}

u8 view_plan=0;
void NRF_Send1(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 1;	
	
	_temp =  vmc_all.att_ctrl[ROLr]*10;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  vmc_all.att_ctrl[PITr]*10;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
  _temp =  vmc_all.att_ctrl[YAWr]*10;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);

	_temp =  nav.spd_b[Xr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  nav.spd_b[Yr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  nav.spd_b[Zr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	if(mission_state==2&&view_plan)
	{
	_temp =  traj[0].pt[Xr]*100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  traj[0].pt[Yr]*100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	}else{
	_temp =  nav.pos_n[Xr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  nav.pos_n[Yr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	}
	_temp =  nav.pos_n[Zr]*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp =  0;//flow_5a.quality;	
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(fly_ready);
  NRF24L01_TXDATA[_cnt++]=BYTE0(vmc_all.param.control_mode_all);
	NRF24L01_TXDATA[_cnt++]=BYTE0(mission_flag);
	NRF24L01_TXDATA[_cnt++]= 0;//BYTE0(drone.acc_3d_step);  
	_temp =  bat.percent*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp=0;
	if(module.flow)
	_temp=1;

	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}


void NRF_Send2(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 2;	

	for(i=0+1;i<9+1;i++){//9
		_temp =  BLE_DEBUG[i];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	}
	for(i=0;i<5;i++){//5
		_temp =  0;//CH[i];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	}
		
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}
	

void NRF_Send3(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0,j;

	NRF24L01_TXDATA[_cnt++] = 3;	
	
	NRF24L01_TXDATA[_cnt++]=BYTE0(fly_ready);
  NRF24L01_TXDATA[_cnt++]=BYTE0(vmc_all.param.control_mode_all);
	NRF24L01_TXDATA[_cnt++]=BYTE0(mission_flag);
	NRF24L01_TXDATA[_cnt++]= 0;//BYTE0(drone.acc_3d_step);  
	_temp =  bat.percent*100;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	for(i=0;i<4;i++){//24
		for(j=0;j<3;j++){
			_temp =  PARAM_NRF[i][j];	
			NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
			NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		}
	}
	

	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send4(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0,j;

	NRF24L01_TXDATA[_cnt++] = 4;	

	for(i=4;i<9;i++){//30
		for(j=0;j<3;j++){
		_temp =  PARAM_NRF[i][j];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		}
	}
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}


void NRF_Send5(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0,j;

	NRF24L01_TXDATA[_cnt++] = 5;	

	for(i=9;i<13;i++){//30
		for(j=0;j<3;j++){
		_temp =  PARAM_NRF[i][j];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		}
	}
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}


void NRF_Send6(void)
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0,j;

	NRF24L01_TXDATA[_cnt++] = 6;	

	for(i=13;i<18;i++){//30
		for(j=0;j<3;j++){
		_temp =  PARAM_NRF[i][j];	
		NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
		NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		}
	}
	
	for( i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[31] = sum;
		NRF_TxPacket(NRF24L01_TXDATA,32);
}

void RC_Send_Task(void)
{
static u8 cnt[5]={0};
	if(cnt[0]++>10)
	 cnt[0]=0;
if(send_pid||read_pid){
	if(read_pid)
		pid_copy_param();
	if(cnt[1]==0){cnt[1]=1;
	NRF_Send3();}
	else if(cnt[1]==1){cnt[1]=2;
	NRF_Send4();}
	else if(cnt[1]==2){cnt[1]=3;
	NRF_Send5();}
	else{cnt[1]=0;
	NRF_Send6();
	 cnt[2]++;
	}
  if(cnt[2]++>5)
	{
	 cnt[2]=0;
	 if(read_pid==0){
	 send_pid=0;	
	 param_copy_pid();	
	 }else
	 read_pid=0;
	}
}else
{

if(cnt[0]%2==0)
	NRF_Send1();
	else
	NRF_Send2();
}

}


void param_copy_pid(void){

//	ctrl_1.PID[PIDPITCH].kp= ctrl_1.PID[PIDROLL].kp  = 0.001*PARAM[0][0];
//	ctrl_1.PID[PIDPITCH].ki= ctrl_1.PID[PIDROLL].ki  = 0.001*PARAM[0][1];
//	ctrl_1.PID[PIDPITCH].kd= ctrl_1.PID[PIDROLL].kd  = 0.001*PARAM[0][2];
//	eso_att_inner_c[ROLr].b0=eso_att_inner_c[PITr].b0= 			 PARAM[1][0];
//	//ki = PARAM[1][1];
//	eso_att_inner_c[ROLr].eso_dead=eso_att_inner_c[PITr].eso_dead=   PARAM[1][2]*0.001;
//	ctrl_1.PID[PIDYAW].kp 	= 0.001*PARAM[2][0];
//	ctrl_1.PID[PIDYAW].ki 	= 0.001*PARAM[2][1];
//	ctrl_1.PID[PIDYAW].kd 	= 0.001*PARAM[2][2];
//  //------
//	ctrl_2.PID[PIDPITCH].kp =ctrl_2.PID[PIDROLL].kp  = 0.001*PARAM[3][0];
//	ctrl_2.PID[PIDPITCH].ki =ctrl_2.PID[PIDROLL].ki  = 0.001*PARAM[3][1];
//	ctrl_2.PID[PIDPITCH].kd =ctrl_2.PID[PIDROLL].kd  = 0.001*PARAM[3][2];
//	//0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//	// 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//	// 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//	ctrl_2.PID[PIDYAW].kp 	= 0.001*PARAM[5][0];
//	ctrl_2.PID[PIDYAW].ki 	= 0.001*PARAM[5][1];
//	ctrl_2.PID[PIDYAW].kd 	= 0.001*PARAM[5][2];
//  //
//	wz_speed_pid.kp  = 0.001*PARAM[6][0];
//	wz_speed_pid.ki  = 0.001*PARAM[6][1];
//	wz_speed_pid.kd  = 0.001*PARAM[6][2];

//	ultra_pid.kp = 0.001*PARAM[7][0];
//	ultra_pid.ki = 0.001*PARAM[7][1];
//	ultra_pid.kd = 0.001*PARAM[7][2];

//	nav_spd_pid.kp	= 	0.001*PARAM[8][0];
//	nav_spd_pid.ki  =   0.001*PARAM[8][1];
//	nav_spd_pid.kd  =   0.001*PARAM[8][2];
//  //
//	nav_pos_pid.kp  = 0.001*PARAM[9][0];
//	nav_pos_pid.ki  = 0.001*PARAM[9][1];
//	nav_pos_pid.kd  = 0.001*PARAM[9][2];

//	eso_pos[Y].b0=eso_pos[X].b0 = PARAM[10][0];
//	eso_pos[Zr].b0=               PARAM[10][1];
//	eso_pos[Y].eso_dead=eso_pos[X].eso_dead = PARAM[10][2];

//	eso_pos_spd[Zr].b0 	= 				PARAM[11][0];
//	nav_spd_pid.flt_nav 	= 0.001*PARAM[11][1];
//	eso_pos_spd[Zr].eso_dead	=   PARAM[11][2];
//  //
//	eso_pos_spd[Y].b0=eso_pos_spd[X].b0  = PARAM[12][0];
//	//nav_spd_pid.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//	//nav_spd_pid.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );

//	nav_spd_pid.f_kp = 			 0.001*PARAM[13][0];
//	nav_spd_pid.flt_nav_kd = 0.001*PARAM[13][1];
//	nav_spd_pid.dead =             PARAM[13][2];

//	nav_acc_pid.f_kp 	= 			0.001*	PARAM[14][0];
//	nav_acc_pid.kp 	=         0.001*  PARAM[14][1];
//	nav_acc_pid.dead	=               PARAM[14][2];
//  //
//	imu_board.k_flow_sel  = 0.001*PARAM[15][0];
//	int temp1,temp2;
//	temp1=PARAM[15][1];
//	if(temp1>1000)imu_board.flow_module_offset_x=-(temp1-1000)*0.001;
//	else
//	imu_board.flow_module_offset_x=(temp1)*0.001;	
//	temp2=PARAM[15][2];
//	if(temp2>1000)imu_board.flow_module_offset_y=-(temp2-1000)*0.001;
//	else imu_board.flow_module_offset_y=(temp2)*0.001;


//	k_sensitivity[0] = 0.001*PARAM[16][0];
//	k_sensitivity[1] = 0.001*PARAM[16][1];
//	k_sensitivity[2] = 0.001*PARAM[16][2];

//	robot_land.k_f					 = 0.001*PARAM[17][0];
//	LENGTH_OF_DRONE=    						 PARAM[17][1];
//	UART_UP_LOAD_SEL_FORCE=          PARAM[17][2];
}



void pid_copy_param(void){

//	PARAM[0][0]=ctrl_1.PID[PIDPITCH].kp*1000;
//	PARAM[0][1]=ctrl_1.PID[PIDPITCH].ki*1000;
//	PARAM[0][2]=ctrl_1.PID[PIDPITCH].kd*1000;

//	
//	 PARAM[1][0]=eso_att_inner_c[ROLr].b0=eso_att_inner_c[PITr].b0;
//	//ki = PARAM[1][1];
//	PARAM[1][2]=eso_att_inner_c[ROLr].eso_dead*1000;
//	PARAM[2][0]=ctrl_1.PID[PIDYAW].kp *1000;
//	PARAM[2][1]=ctrl_1.PID[PIDYAW].ki *1000;
//	PARAM[2][2]=ctrl_1.PID[PIDYAW].kd *1000;
//  //
//	PARAM[3][0]=ctrl_2.PID[PIDPITCH].kp *1000;
//	PARAM[3][1]=ctrl_2.PID[PIDPITCH].ki *1000;
//	PARAM[3][2]=ctrl_2.PID[PIDPITCH].kd *1000;
//	//0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//	// 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//	// 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//	PARAM[5][0]=ctrl_2.PID[PIDYAW].kp*1000;
//	PARAM[5][1]=ctrl_2.PID[PIDYAW].ki*1000;
//	PARAM[5][2]=ctrl_2.PID[PIDYAW].kd*1000;
//  //
//	PARAM[6][0]=wz_speed_pid.kp*1000 ;
//	PARAM[6][1]=wz_speed_pid.ki*1000 ;
//	PARAM[6][2]=wz_speed_pid.kd*1000 ;

//	PARAM[7][0]=ultra_pid.kp *1000;
//	PARAM[7][1]=ultra_pid.ki *1000;
//	PARAM[7][2]=ultra_pid.kd *1000;

//	PARAM[8][0]=nav_spd_pid.kp	*1000;
//	PARAM[8][1]=nav_spd_pid.ki  *1000;
//	PARAM[8][2]=nav_spd_pid.kd  *1000;
//  //
//	PARAM[9][0]=nav_pos_pid.kp *1000;
//	PARAM[9][1]=nav_pos_pid.ki *1000;
//	PARAM[9][2]=nav_pos_pid.kd *1000;

//	PARAM[10][0]=eso_pos[Y].b0;
//	PARAM[10][1]=eso_pos[Zr].b0;
//	PARAM[10][2]=eso_pos[Y].eso_dead;

//	PARAM[11][0]=eso_pos_spd[Zr].b0;
//	PARAM[11][1]=nav_spd_pid.flt_nav *1000;
//	PARAM[11][2]=eso_pos_spd[Zr].eso_dead	;
//  //
//	PARAM[12][0]=eso_pos_spd[Y].b0;
//	//nav_spd_pid.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//	//nav_spd_pid.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );

//	PARAM[13][0]=nav_spd_pid.f_kp *1000;
//	PARAM[13][1]=nav_spd_pid.flt_nav_kd *1000;
//	PARAM[13][2]=nav_spd_pid.dead ;

//	PARAM[14][0]=nav_acc_pid.f_kp *1000;
//	PARAM[14][1]=nav_acc_pid.kp 	*1000;
//	PARAM[14][2]=nav_acc_pid.dead	;
//  //
//	 PARAM[15][0]=imu_board.k_flow_sel*1000;
//	float temp1,temp2;
//	temp1=	LIMIT(imu_board.flow_module_offset_x,-0.99,0.99);
//	if(temp1<0)
//	temp1=-temp1+1;
//	temp2=	LIMIT(imu_board.flow_module_offset_y,-0.99,0.99);
//	if(temp2<0)
//	temp2=-temp2+1;			
//  PARAM[15][1]=temp1*1000;
//	PARAM[15][2]=temp2*1000;

//	PARAM[16][0]=k_sensitivity[0] *1000;
//	PARAM[16][1]=k_sensitivity[1] *1000;
//	PARAM[16][2]=k_sensitivity[2] *1000;

//	PARAM[17][0]=robot_land.k_f				*1000;
//	PARAM[17][1]=LENGTH_OF_DRONE;
//	PARAM[17][2]=UART_UP_LOAD_SEL_FORCE;
}
