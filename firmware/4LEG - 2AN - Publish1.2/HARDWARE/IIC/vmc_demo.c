#include "vmc.h"
#include "my_math.h"
#include "math.h"
#include "bat.h"
#include "eso.h"
//vmc vmc_all h_pid_all att_pid_all[2];
PID pid_force[3];
float pit_out[4];
float gait_test[5]={0,0,  40   ,0.168,    0};
u8 force_dj_off_reset=0;
void DJ_init(VMC *in)
{
float dj_sel=DJ_KPOWER;
switch(in->param.id){
case 0:	//<--------------------------------1
in->param.sita_flag[0]=1;
in->param.sita_flag[1]=1;	
break;
case 1:	
in->param.sita_flag[0]=-1;
in->param.sita_flag[1]=-1;	
break;
case 2:	
in->param.sita_flag[0]=-1;
in->param.sita_flag[1]=-1;	
break;
case 3:	
in->param.sita_flag[0]=1;
in->param.sita_flag[1]=1;	
break;
}

#if defined(BIG_LITTRO_DOG)
dj_sel=DJ_DSB;//DJ_KPOWER  //DJ_DSB
#else
dj_sel=DJ_DSB;
#endif
switch(in->param.id){
case 0:
in->param.PWM_PER_DEGREE[0]=dj_sel;
in->param.PWM_PER_DEGREE[1]=dj_sel;
in->param.PWM_PER_DEGREE[2]=dj_sel;
in->param.PWM_PER_DEGREE[3]=dj_sel;
break;
case 1:
in->param.PWM_PER_DEGREE[0]=dj_sel;	
in->param.PWM_PER_DEGREE[1]=dj_sel;
in->param.PWM_PER_DEGREE[2]=dj_sel;
in->param.PWM_PER_DEGREE[3]=dj_sel;
break;
case 2:
in->param.PWM_PER_DEGREE[0]=dj_sel;//da
in->param.PWM_PER_DEGREE[1]=dj_sel;//xiao
in->param.PWM_PER_DEGREE[2]=dj_sel;//zhuan
in->param.PWM_PER_DEGREE[3]=dj_sel;
break;
case 3:	
in->param.PWM_PER_DEGREE[0]=dj_sel;	
in->param.PWM_PER_DEGREE[1]=dj_sel;
in->param.PWM_PER_DEGREE[2]=dj_sel;
in->param.PWM_PER_DEGREE[3]=dj_sel;
break;
}

int DL=68;
in->param.PWM_MIN[0]=500+DL; in->param.PWM_MIN[1]=500+DL;   in->param.PWM_MIN[2]=500+DL;  in->param.PWM_MIN[3]=500+DL;
in->param.PWM_MAX[0]=2500-DL;in->param.PWM_MAX[1]=2500-DL;	in->param.PWM_MAX[2]=2500-DL;	in->param.PWM_MAX[3]=2500-DL;	
}	

void vmc_init(void)
{ char i;
	get_license();
	//设置License
//	vmc_all.param.your_key[0]=176;
//	vmc_all.param.your_key[1]=46;
//	vmc_all.param.your_key[2]=159;
	
#if defined (BIG_LITTRO_DOG)	
	vmc_all.l1=0.076;
	vmc_all.l2=0.154;
	vmc_all.l3=0.022;
	vmc_all.l4=0.026/2;
	vmc_all.H=0.265;
	vmc_all.W=0.19;
	vmc_all.mess=0.7;//机体重量kg
	vmc_all.cog_off[F]=-vmc_all.H*0.000/0.16;//0.008;//重心偏差  前
	vmc_all.cog_off[B]=-vmc_all.H*0.000/0.16;//-0.008;//重心偏差  后
	vmc_all.cog_off[2]=3;// 前后移动时角度倾斜
	vmc_all.cog_off[3]=-3.2;// 旋转移动时角度倾斜 前
	vmc_all.cog_off[4]=-1.68;// 旋转移动时角度倾斜 后
	vmc_all.gait_time[0]=vmc_all.gait_time[1]=0.425;//步态周期
		//跨腿PID 足尖局部速度
	vmc_all.kp_trig[0]=0.25;//0.125;//跨腿前馈增益
	vmc_all.kp_trig[1]=0;//    跨腿加速度前馈增益
	vmc_all.param.kp_pose_reset[0]=150;//自动归中增益
	vmc_all.param.kp_pose_reset[1]=0;//自动归中增益
#endif
#if defined (MINI_LITTRO_DOG)	
	vmc_all.l1=0.0376;
	vmc_all.l2=0.075;
	vmc_all.l3=0.0;
	vmc_all.l4=0.0;
	vmc_all.H=0.16;
	vmc_all.W=0.09;
	vmc_all.mess=0.56;//机体重量kg
	vmc_all.cog_off[F]=-vmc_all.H*0.008/0.16;//0.008;//重心偏差  前
	vmc_all.cog_off[B]=-vmc_all.H*0.006/0.16;//-0.008;//重心偏差  后
	vmc_all.cog_off[2]=3;// 前后移动时角度倾斜
	vmc_all.cog_off[3]=-3.2;// 旋转移动时角度倾斜 前
	vmc_all.cog_off[4]=-1.68;// 旋转移动时角度倾斜 后
	
	#if HIGE_LEG_TRIG
	vmc_all.gait_time[0]=vmc_all.gait_time[1]=0.4;//步态周期
	#else
	vmc_all.gait_time[0]=vmc_all.gait_time[1]=0.376;//步态周期
	#endif
		//跨腿PID 足尖局部速度
	vmc_all.kp_trig[0]=0.125;//0.125;//跨腿前馈增益
	vmc_all.kp_trig[1]=0;//    跨腿加速度前馈增益
	vmc_all.param.kp_pose_reset[0]=400;//自动归中增益
	vmc_all.param.kp_pose_reset[1]=0;//自动归中增益
#endif

	vmc_all.gait_alfa=0.5;//步态占空比
	vmc_all.stance_time=vmc_all.gait_time[1]*vmc_all.gait_alfa;
	vmc_all.delay_time[0]=vmc_all.stance_time*0;//四足着地时间
	vmc_all.gait_delay_time=vmc_all.stance_time*0;//同相跨腿延时
	
	vmc_all.kp_touch=0.3;//全着地增益
		
	vmc_all.off_leg_dis[0]=0;//腿外扩  前后距离
	vmc_all.off_leg_dis[1]=0;
	
	vmc_all.kp_g[Yr]=3.5;//重力分量前后  前馈
	vmc_all.kp_g[Xr]=0;//重力分量左右  前馈
	vmc_all.ground_off[0]=0.035;//移动着地速度
	vmc_all.ground_off[1]=0.0086;//空闲时的 着地速度
	
	vmc_all.kp_deng[FL1]=30/0.56*vmc_all.mess;//下蹬力幅值
	vmc_all.kp_deng[FL2]=vmc_all.kp_deng[FL1];
	vmc_all.kp_deng[BL1]=vmc_all.kp_deng[FL1];
	vmc_all.kp_deng[BL2]=vmc_all.kp_deng[FL1];
	vmc_all.kd_deng[P]=-0.368;
	vmc_all.kp_deng_gain[F]=0.6;
	vmc_all.kp_deng_gain[B]=1;
	//机械参数
  vmc_all.tar_pos.z=-(sind(35)*vmc_all.l1+sind(50)*vmc_all.l2);//机器人起始高度
	#if HIGE_LEG_TRIG
		vmc_all.delta_ht[0]=vmc_all.delta_ht[1]=sind(55)*vmc_all.l1;//抬腿高度
	#else
		vmc_all.delta_ht[0]=vmc_all.delta_ht[1]=sind(45)*vmc_all.l1;//抬腿高度
	#endif
  MIN_Z=-(cosd(40)*vmc_all.l2-cosd(60)*vmc_all.l1);
	MAX_Z=-(sind(55)*vmc_all.l1+sind(55)*vmc_all.l2);
	
	MIN_X=-vmc_all.l1*1.15;
	MAX_X=vmc_all.l1*1.15;
	
//---------------------控制参数---------------------	

	
	vmc_all.pid[Xr][P]=268;//200//跨腿速度增益
	vmc_all.pid[Xr][I]=25;//20;//跨腿速度积分
	//---------------VMC库-------------------		
	//位置PID
	pos_pid_all.kp=0.4;
	pos_pid_all.ki=0.025;
	//高度PID
	h_pid_all.kp=3;
	h_pid_all.ki=15;
	h_pid_all.kd=4;
	
	//俯仰横滚姿态PID
	att_pid_all[PITr].ki=13;
	att_pid_all[PITr].kd=0.001;
	
	att_pid_all[ROLr].ki=18;
	att_pid_all[ROLr].kd=0.002;
	
	//---------------DEMO-------------------		
  //位姿控制PID
	pid_force[PITr].kp=350;
	pid_force[ROLr].kp=600;
  pid_force[Zr].kp=1600;
	pid_force[Zr].ki=800;
	
	vmc_all.pid[Zr][P]=8888;//888;//600;//Z轴增益
	vmc_all.pid[Zr][D]=1000;//15;//Z内环P 轴微分
	#if !USE_LOW_TORQUE
	 vmc_all.pid[Zr][P]=10000;//Z轴增益
	 vmc_all.pid[Zr][D]=200;//Z内环P 轴微分	
	#endif
	
  //航向蹬腿  PID
	att_pid_all[YAWr].kp=1.35;
	att_pid_all[YAWr].kd=0;//微分
	att_pid_all[YAWr].kp_i=0.002;
	att_pid_all[YAWr].kd_i=0.010;//微分
  att_pid_all[YAWr].fp_i=0.00358;//前馈
	
	vmc_all.flt_toqrue=0.05;//扭矩滤波  WS
	#if defined (BIG_LITTRO_DOG)	
		vmc_all.gain_torque=100;//扭矩 转速  增益
	#endif
	#if defined (MINI_LITTRO_DOG)	
		vmc_all.gain_torque=400;//扭矩 转速  增益
	#endif
  vmc_all.rst_dead=vmc_all.l1*0.1*2;//自动复位死区
	
	#if DEBUG_MODE
		vmc_all.use_att=0;					//使用姿态控制 0使用腿长度
		vmc_all.use_ground_sensor=0;//使用着地传感器
		vmc_all.gait_time[1]=vmc_all.gait_time[0]*4;//步态周期
		vmc_all.delta_ht[0]=vmc_all.delta_ht[1]=vmc_all.delta_ht[1]*1.5;
	#else
		vmc_all.use_att=1;				  //使用姿态控制 0使用腿长度
		vmc_all.use_ground_sensor=0;//使用着地传感器
	#endif
	#if VIR_MODEL
		vmc_all.use_att=0;	
	#endif
	//-------------全局参数---------------
	vmc_all.param.end_sample_dt=0.01;//足端速度微分时间
	vmc_all.param.ground_dump=0.06;//触底回缩比例
	vmc_all.param.trig_ground_st_rate=0.631;//跨腿后使能触地停止的比例
	vmc_all.param.ground_rst=0.5;//自触地周期
	vmc_all.param.angle_set_trig=8;//跨腿触地停止角度限制
	vmc_all.param.control_out_of_maxl=0.15;//姿态控制最大控制量与腿长比例
	vmc_all.param.att_limit_for_w=4.56;//计算权重最大的角度限制
	vmc_all.param.en_hold_on=1;

  vmc_all.param.en_att_tirg=1;//姿态倾斜影响跨腿轨迹
	vmc_all.k_auto_time=1;//自动步态时间调整增益
	vmc_all.trig_mode=1;//跨腿轨迹模式 0->摆线
	vmc_all.out_range_force=5;
	vmc_all.param.soft_start=1;
//-------------初始化各脚参数----------------------
	//固定参数
	vmc[FL1].flag_rl=vmc[BL1].flag_rl=1;
	vmc[FL2].flag_rl=vmc[BL2].flag_rl=-1;
	vmc[FL1].flag_fb=vmc[FL2].flag_fb=1;
	vmc[BL1].flag_fb=vmc[BL2].flag_fb=-1;
	vmc_all.sita_test[0]=vmc_all.sita_test[1]=90;//强制关节角度测试
	MAX_SPD=LIMIT(vmc_all.l1*2/(vmc_all.gait_time[0]+1e-6),0.01,MAX_FSPD);	
	for(i=0;i<4;i++){
  vmc[i].l1=vmc_all.l1;
	vmc[i].l2=vmc_all.l2;
	vmc[i].l3=vmc_all.l3;
	vmc[i].l4=vmc_all.l4;
  vmc[i].sita1=0+45;
	vmc[i].sita2=180-45;
	vmc[i].param.sita_limit=15;
	vmc[i].ground=1;
	vmc[i].param.delta_h=vmc_all.delta_ht[0];
	vmc[i].param.flt_toqrue=vmc_all.flt_toqrue;
	vmc[i].param.gain_torque=vmc_all.gain_torque;
	vmc[i].param.id=i;	
	vmc[i].param.trig_state=99;
	DJ_init(&vmc[i]);//设置舵机型号	
	#if USE_DEMO
		cal_jacobi_d(&vmc[i]);
	#else
		cal_jacobi(&vmc[i]);
	#endif	
	}
}
#if USE_DEMO
static void leg_off_publish_d(void)
{
	if(vmc[0].param.PWM_OFF[0]>2500||vmc[0].param.PWM_OFF[0]<50||force_dj_off_reset)
	{
	vmc[0].param.PWM_OFF[0]=600;	
	vmc[0].param.PWM_OFF[1]=600;	
	vmc[1].param.PWM_OFF[0]=2400;	
	vmc[1].param.PWM_OFF[1]=2420;	
	vmc[2].param.PWM_OFF[0]=2360;	
	vmc[2].param.PWM_OFF[1]=2380;	
	vmc[3].param.PWM_OFF[0]=600;	
	vmc[3].param.PWM_OFF[1]=600;	
	force_dj_off_reset=0;
	}
}

//状态复位 和  数据滤波
static void state_rst_d(float dt)
{ static float st;
	static float time[5];
	float att_use[3],err[3];
	char i;
 	st=vmc_all.gait_time[1]*vmc_all.gait_alfa;
	vmc_all.ground_num=0;
	leg_off_publish_d();
	
	for(i=0;i<4;i++){
	 vmc[i].tar_spd.x=vmc_all.param.tar_spd_use_rc.x;
	 vmc[i].param.delta_h=vmc_all.delta_ht[0];
	 vmc[i].param.pid[Xr][0]=vmc_all.pid[Xr][0];
	 vmc[i].param.pid[Xr][1]=vmc_all.pid[Xr][1];
	 vmc[i].param.pid[Xr][2]=vmc_all.pid[Xr][2];	
	 vmc[i].param.pid[Xr][3]=vmc_all.pid[Xr][3];
	 vmc[i].param.pid[Xr][4]=vmc_all.pid[Xr][4];
		
	 vmc[i].param.pid[Zr][0]=vmc_all.pid[Zr][0];
	 vmc[i].param.pid[Zr][1]=vmc_all.pid[Zr][1];
	 vmc[i].param.pid[Zr][2]=vmc_all.pid[Zr][2];	
	 vmc[i].param.pid[Zr][3]=vmc_all.pid[Zr][3];	
	 vmc[i].param.pid[Zr][4]=vmc_all.pid[Zr][4];	
		
	 vmc_all.kp_deng[3]=vmc_all.kp_deng[2]=vmc_all.kp_deng[1]=vmc_all.kp_deng[0];
		
	 vmc[i].param.flt_toqrue=vmc_all.flt_toqrue;
	 vmc[i].param.gain_torque=vmc_all.gain_torque;
   vmc_all.ground[0][i]=vmc[i].ground;
	 vmc_all.ground[1][i]=vmc[i].ground_s;
	 if(vmc[i].ground)
		  vmc_all.ground_num++;
  }
	
	  vmc_all.param.dt_size=0.4/(vmc_all.gait_time[1]+0.0000001);
	  vmc_all.param.max_l=(fabs(MAX_Z)-fabs(MIN_Z));//最大腿长
	//跨腿用  低通滤波
		DigitalLPF(vmc_all.att[PITr], &vmc_all.att_trig[PITr], 5, dt); 
		DigitalLPF(vmc_all.att[ROLr], &vmc_all.att_trig[ROLr], 5, dt);
		vmc_all.att_trig[YAWr]=vmc_all.att[YAWr];
		DigitalLPF(vmc_all.att_rate[PITr], &vmc_all.att_rate_trig[PITr], 5, dt);
		DigitalLPF(vmc_all.att_rate[ROLr], &vmc_all.att_rate_trig[ROLr], 5, dt);
		DigitalLPF(vmc_all.att_rate[YAWr], &vmc_all.att_rate_trig[YAWr], 5, dt);
		
		vmc_all.att_trig[PITr]=LIMIT(vmc_all.att_trig[PITr],-16,16);
	  vmc_all.att_trig[ROLr]=LIMIT(vmc_all.att_trig[ROLr],-16,16);
	//姿态控制用  低通滤波 	 
		DigitalLPF(vmc_all.att[PITr], &vmc_all.att_ctrl[PITr], 5, dt); 
		DigitalLPF(vmc_all.att[ROLr], &vmc_all.att_ctrl[ROLr], 5, dt);
	  vmc_all.att_ctrl[YAWr]=vmc_all.att[YAWr];
		DigitalLPF(vmc_all.att_rate[PITr], &vmc_all.att_rate_ctrl[PITr], 0.35, dt);
		DigitalLPF(vmc_all.att_rate[ROLr], &vmc_all.att_rate_ctrl[ROLr], 0.35, dt);
		DigitalLPF(vmc_all.att_rate[YAWr], &vmc_all.att_rate_ctrl[YAWr], 0.35, dt);
		
		//------------------------------------机械角度解算--------------------------------
		float temp[2]={0};
		if((vmc[0].ground+vmc[2].ground!=0) &&  (vmc[1].ground+vmc[3].ground!=0)){
		temp[0]=(vmc[0].epos.z*vmc[0].ground+vmc[2].epos.z*vmc[2].ground)/(vmc[0].ground+vmc[2].ground);
		temp[1]=(vmc[1].epos.z*vmc[1].ground+vmc[3].epos.z*vmc[3].ground)/(vmc[1].ground+vmc[3].ground);	
		DigitalLPF(-fast_atan2(temp[0]-temp[1], vmc_all.H)*57.3,&vmc_all.att_vm[PITr],
		 3,dt);
		}
		
		if((vmc[0].ground+vmc[1].ground!=0) &&  (vmc[2].ground+vmc[3].ground!=0)){
		temp[0]=(vmc[0].epos.z*vmc[0].ground+vmc[1].epos.z*vmc[1].ground)/(vmc[0].ground+vmc[1].ground);
		temp[1]=(vmc[2].epos.z*vmc[2].ground+vmc[3].epos.z*vmc[3].ground)/(vmc[2].ground+vmc[3].ground);
	  DigitalLPF(fast_atan2(temp[0]-temp[1], vmc_all.W)*57.3,&vmc_all.att_vm[ROLr],
		 3,dt);		
		}
		 vmc_all.att_vm[YAWr]=vmc_all.att_ctrl[YAWr]; 
		 
		 //高度估计
		if((vmc[0].ground+vmc[1].ground+vmc[2].ground+vmc[3].ground>=3)||(vmc[0].ground&&vmc[3].ground)||(vmc[1].ground&&vmc[2].ground)){
		 DigitalLPF((vmc[0].epos.z*vmc[0].ground+vmc[1].epos.z*vmc[1].ground+
								 vmc[2].epos.z*vmc[2].ground+vmc[3].epos.z*vmc[3].ground)/vmc_all.ground_num,
								&vmc_all.pos.z,
								3,dt);
	  vmc_all.pos.z=LIMIT(vmc_all.pos.z,MAX_Z*0.95,MIN_Z*1.05);
		}
		
		if(vmc[0].ground&&vmc[3].ground==1)
			DigitalLPF(vmc[0].epos.z/2+vmc[3].epos.z/2,&vmc_all.param.line_z[0],5,dt);
		if(vmc[1].ground&&vmc[2].ground==1)
			DigitalLPF(vmc[1].epos.z/2+vmc[2].epos.z/2,&vmc_all.param.line_z[1],5,dt);
		

		//机体速度估计  里程计
		//机械速度 x 前后
    if(vmc_all.ground_num>1){
			temp[0]=(vmc[0].spd.x*vmc[0].ground+vmc[1].spd.x*vmc[1].ground+
							 vmc[2].spd.x*vmc[2].ground+vmc[3].spd.x*vmc[3].ground)/vmc_all.ground_num;
			
		 	vmc_all.body_spd[Xr]=-temp[0];	
			
			temp[0]=(vmc[0].spd.z*vmc[0].ground+vmc[1].spd.z*vmc[1].ground+
				 vmc[2].spd.z*vmc[2].ground+vmc[3].spd.z*vmc[3].ground)/vmc_all.ground_num;
			
			DigitalLPF(-temp[0], &vmc_all.body_spd[Zr], 3, dt);
		}
		//机械速度  左右
		vmc_all.param.encoder_spd[R]=vmc[0].spd.x*vmc[0].ground+vmc[1].spd.x*vmc[1].ground;
		vmc_all.param.encoder_spd[L]=vmc[2].spd.x*vmc[2].ground+vmc[3].spd.x*vmc[3].ground;
		//机械速度 rad
		vmc_all.body_spd[YAWrr]=vmc_all.att_ctrl[YAWr];
		
//-------------------------------------------------------------------------------------------
	vmc_all.stance_time_auto=st;
	vmc_all.delay_time[1]=vmc_all.delay_time[0];
			
  vmc[FL1].force[Xr]= vmc[FL2].force[Xr]= vmc[BL1].force[Xr]= vmc[BL2].force[Xr]=0;
  vmc[FL1].force[Zr]= vmc[FL2].force[Zr]= vmc[BL1].force[Zr]= vmc[BL2].force[Zr]=0; 
}

//估计末端状态和 速度估计
char estimate_end_state_d(VMC *in,float dt)//每个腿的
{
	float alfa=(in->sita2-in->sita1)/2;
	float beta=(in->sita2+in->sita1)/2;
	//逆解
	in->r=in->l1*cosd(alfa)+sqrt(pow(in->l2,2)-pow(in->l1,2)*pow(sind(alfa),2));
	in->sita=beta;
	in->epos.x=cosd(in->sita)*in->r;
	in->epos.z=-sind(in->sita)*in->r;
	//spd origin
	float spd[3];
	in->param.spd_est_cnt+=dt;
	if(in->param.spd_est_cnt>0.015){
		spd[Xr]=(in->epos.x-in->epos_reg.x)/0.015;
		spd[Zr]=(in->epos.z-in->epos_reg.z)/0.015;
		in->epos_reg.x=in->epos.x;
		in->epos_reg.z=in->epos.z;
		if(in->ground){
			in->spd_o.x=-LIMIT(spd[Xr]/2,-1,1);
		  in->spd_o.z=LIMIT(spd[Zr],-1,1);
		}
		in->param.spd_est_cnt=0;	
	}
	DigitalLPF(in->spd_o.x, &in->spd.x, 6, dt) ;
  DigitalLPF(in->spd_o.z, &in->spd.z, 6, dt) ;
}

//计算落足点
static void cal_tar_end_pos_d(VMC *in)
{ 
	float att_off=0;
	float cog_off=0;
	float spd_off=0,acc_off=0;
	float size_off=0;
	float tar_spd;
	float cog_off_hover=0;
	
	if(vmc_all.param.tar_spd_use_rc.x>0)//机体偏差
	 cog_off_hover=vmc_all.cog_off[F];
	else
	 cog_off_hover=vmc_all.cog_off[B];		
	
	if(in->flag_rl==1)
		tar_spd=vmc_all.param.tar_spd_use[L].x;
	else
		tar_spd=vmc_all.param.tar_spd_use[R].x;	
	
	cog_off=LIMIT(0.56*cog_off_hover,MIN_X*0.35,MAX_X*0.35);//重心补偿
	
	att_off=LIMIT(tan((vmc_all.att_trig[PITr])*DEG_TO_RAD)*vmc_all.pos.z
					,MIN_X*0.6,MAX_X*0.6);//姿态地形跟随
	
	spd_off=vmc_all.kp_trig[0]*(tar_spd-in->spd.x);
  in->tar_epos.x=LIMIT(in->spd.x*(vmc_all.stance_time+vmc_all.delay_time[2])/2+
	                     spd_off+//速度前馈
	                     att_off+//地形跟随偏差
	                     cog_off
	                     ,MIN_X,MAX_X);
	
  in->tar_epos.z=LIMIT(in->tar_epos.z,MAX_Z,MIN_Z);
	in->ground=0;
}


static void spd_control_d(float dt){
	
	float att_use[3];
	//---------------速度权重分配输出
	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
	vmc_all.param.w_cmd[ROLr]=LIMIT(ABS(LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr],
	-25,25))/25,0,1);

	vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x*LIMIT(1-vmc_all.param.w_cmd[ROLr],0.5,0.8);
	
	//---------------航向控制
	static float rst_yaw[2];
	static float timer;
	if(fabs(vmc_all.tar_spd.z)<MAX_SPD_RAD*0.005&&fabs(vmc_all.param.tar_spd_use_rc.x)<MAX_SPD*0.005)
		rst_yaw[0]+=dt;
	else
		rst_yaw[0]=rst_yaw[1]=0;
	
	if(rst_yaw[0]>10)
	{rst_yaw[1]=1;rst_yaw[0]=0;}
	if(rst_yaw[1])//无控制复位
	  vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
	if(fabs(vmc_all.tar_spd.z)>MAX_SPD_RAD*0.05)//遥控复位
	  vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
  if(vmc_all.unmove||vmc_all.hand_hold)
		vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
	//外环
	static float err_yaw,yaw_out;
	DigitalLPF(LIMIT(To_180_degrees(dead(vmc_all.tar_att[YAWr]-vmc_all.att_ctrl[YAWr],0.6)),-20,20), 
		&err_yaw, 0.5, dt);

	vmc_all.param.w_cmd[YAWr]=LIMIT(err_yaw/25,0,1);//计算权重
	
	static float exp_yaw_rate=0;
	if(timer>0.01){timer=0;
		exp_yaw_rate=LIMIT(-att_pid_all[YAWr].kp*err_yaw,-MAX_SPD_RAD,MAX_SPD_RAD);
	}		
	timer+=dt;

	//内环
	if(fabs(vmc_all.tar_spd.z)>MAX_SPD_RAD*0.05)
	  exp_yaw_rate=vmc_all.tar_spd.z;
		
	exp_yaw_rate*=LIMIT(1-vmc_all.param.w_cmd[ROLr],0.2,0.7);
	
	static float rate_reg;
	float damp = (vmc_all.att_rate_ctrl[YAWr]- rate_reg) *( 0.005f/dt );
	yaw_out=LIMIT(dead(exp_yaw_rate-vmc_all.att_rate_ctrl[YAWr],0.5),-MAX_SPD_RAD,MAX_SPD_RAD)*att_pid_all[YAWr].kp_i+
								 -damp*att_pid_all[YAWr].kd_i+
	               exp_yaw_rate*att_pid_all[YAWr].fp_i;//前馈
	rate_reg=vmc_all.att_rate_ctrl[YAWr];
	#if DEBUG_MODE
	  yaw_out=0;
	#endif
	if(vmc_all.hand_hold)
		yaw_out=0;
	
	//控制差速输出
	vmc_all.param.tar_spd_use[L].x=vmc_all.param.tar_spd_use_rc.x+yaw_out;
	vmc_all.param.tar_spd_use[R].x=vmc_all.param.tar_spd_use_rc.x-yaw_out;
}	

static void vmc_force_control_d(VMC *in,float dt)
{
	u8 i;
	char id=in->param.id;
	float err[3];
	float att_use[3];
  //---高度控制
	pid_force[Zr].err=LIMIT(vmc_all.tar_pos.z+vmc_all.deng_all*0.0001-vmc_all.pos.z,-0.35*vmc_all.param.max_l,0.35*vmc_all.param.max_l);
	pid_force[Zr].p=pid_force[Zr].err*pid_force[Zr].kp;
	pid_force[Zr].out=pid_force[Zr].p;
	
	//---姿态控制
	if(vmc_all.unmove==0)
		att_use[PITr]=vmc_all.att_vm[PITr];//使用机械计算角度
	else
		att_use[PITr]=vmc_all.att_ctrl[PITr];//静止时进入姿态平滑模式
	 
	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
 
	err[PITr]=LIMIT(vmc_all.tar_att[PITr]+vmc_all.tar_att_off[PITr]-att_use[PITr],-16,16);
	err[ROLr]=-LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr],-16,16);
  //Pit
	pid_force[PITr].err=err[PITr];
	pid_force[PITr].p=vmc_all.H/2*sind(pid_force[PITr].err)*pid_force[PITr].kp;
	pid_force[PITr].out=pid_force[PITr].p;
	//Rol
	pid_force[ROLr].err=err[ROLr];
	pid_force[ROLr].p=vmc_all.W/2*sind(pid_force[ROLr].err)*pid_force[ROLr].kp;
	pid_force[ROLr].out=pid_force[ROLr].p;

	//---高度失配补偿
	if(id==0||id==3)
		pid_force[Zr].out+=-(vmc_all.param.line_z[0]-vmc_all.param.line_z[1])*pid_force[Zr].i;
	if(id==2||id==1)
		pid_force[Zr].out+= (vmc_all.param.line_z[0]-vmc_all.param.line_z[1])*pid_force[Zr].i;
	
	vmc_all.param.weight[ROLr]=LIMIT(1-ABS(LIMIT(vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-vmc_all.att_ctrl[ROLr],
	-6,6))/6+0.6,0,1);//横滚轴权重分离
		pit_out[id]=(pid_force[Zr].out*vmc_all.param.weight[ROLr]-in->flag_fb*pid_force[PITr].out-in->flag_rl*pid_force[ROLr].out);
}

	
//计算Y轴虚拟力
float reset_k=200;
static float cal_force_y_d(VMC *in,float dt)//前后
{
 static float interge;
 float tar_spd;	
	//旋转差速
	if(in->flag_rl==1)
		tar_spd=vmc_all.param.tar_spd_use[L].x;
	else
		tar_spd=vmc_all.param.tar_spd_use[R].x;	
	
 interge+=(tar_spd-in->spd.x)*in->param.pid[Xr][I]*dt;
 interge=LIMIT(interge,-50,50);
	
 if(vmc_all.unmove)//静止下足尖归中
	 in->force[Xr]=-reset_k*(sind(vmc_all.att_trig[PITr])*in->epos.z-in->epos.x);
 else
	 in->force[Xr]=in->param.pid[Xr][P]*(tar_spd-in->spd.x)//速度前馈
			+interge;//速度误差积分
 
 if(fabs(in->epos.z)<fabs(MIN_Z)||fabs(in->epos.z)>fabs(MAX_Z)||fabs(in->epos.x)>MAX_X)
	 in->force[Xr]*=0.1;
}

//计算Z轴虚拟力
static float cal_force_z_d(VMC *in,float dt)//高度
{
 char i;
 float att_off[2],cog_off[2],ground_off=0;
 float z_out;
 in->force[Zr]=pit_out[in->param.id];
 if(fabs(in->epos.z)<fabs(MIN_Z)||fabs(in->epos.z)>fabs(MAX_Z)||fabs(in->epos.x)>MAX_X)
	 in->force[Zr]*=0.1;
}

static char trig_curve_d(VMC *in,float *x,float *z,float dt,u8 sel)
{ END_POS epos; 
  float alfa=0;
  int id=in->param.id,temp=0;;
	switch(in->param.trig_state)
	{
		case 0:
				 alfa=2*PI*LIMIT(in->param.time_trig,0,vmc_all.gait_time[1]-vmc_all.stance_time)/(vmc_all.gait_time[1]-vmc_all.stance_time);
				if(in->param.time_trig<vmc_all.gait_time[1]-vmc_all.stance_time)
				{
				 in->param.time_trig+=dt;	
				 *x=(in->tar_pos.x-in->st_pos.x)*(alfa-sin(alfa))/(2*PI)+in->st_pos.x;
				 *z=(in->param.delta_h)*(1-cos(alfa))/2+in->st_pos.z+(in->tar_pos.z-in->st_pos.z)*in->param.time_trig/(vmc_all.gait_time[1]-vmc_all.stance_time);	
				} 
		 
			if(in->param.time_trig>(vmc_all.gait_time[1]-vmc_all.stance_time)&&in->epos.z-in->tar_epos.z>0.005)
				in->param.trig_state=1;//到达步态周期还未满足设定高度  跳转到延伸状态
			else if(in->param.time_trig>(vmc_all.gait_time[1]-vmc_all.stance_time)*0.5
			  &&fabs(in->epos.z)>fabs(in->tar_epos.z)){//还未到达周期却已满足高度
					 in->ground=1;
					 in->tar_epos.z=in->epos.z;
					 in->param.trig_state=99;
					 return 1;
			}else if(in->param.time_trig>(vmc_all.gait_time[1]-vmc_all.stance_time)){//超出跨腿周期
					 in->ground=1;
					 in->tar_epos.z=in->epos.z;
					 in->param.trig_state=99;
					 return 1;
			 }
		break;
		case 1://倾斜延迟跨腿
		  in->param.time_trig+=dt;
	  	if(fabs(in->epos.z)>fabs(in->tar_epos.z)){
					in->ground=1;
					in->param.trig_state=99;
					in->tar_epos.z=in->epos.z;
					return 1;
				}
			*z-=vmc_all.kp_touch*dt;
		break;
	}
	*x=LIMIT(*x,-MAX_X,MAX_X);
	*z=LIMIT(*z,MAX_Z,MIN_Z);
	return 0;
}

//运动学逆解
char inv_end_state_d(float x,float z,float *sita1,float *sita2)
{
  float sita=-(fast_atan2(-z,x+0.00001)*RAD_TO_DEG-90);
	float r=sqrt(z*z+x*x)+0.00001;
	*sita1=90-sita-acos((r*r+vmc_all.l1*vmc_all.l1-vmc_all.l2*vmc_all.l2)/(2*vmc_all.l1*r))*RAD_TO_DEG;
  *sita2=90-sita+acos((r*r+vmc_all.l1*vmc_all.l1-vmc_all.l2*vmc_all.l2)/(2*vmc_all.l1*r))*RAD_TO_DEG;
}

//计算雅克比矩阵  
char cal_jacobi_d(VMC *in)
{
	float alfa=(in->sita2-in->sita1)/2;
	float beta=(in->sita2+in->sita1)/2;
	
	float r=in->l1*cosd(beta)+sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001;
	//x
  in->jacobi[0]=-sind(beta)*r*0.5+cosd(beta)*(in->l1*sind(alfa)*0.5+0.5*in->l1*in->l1*sind(alfa)*cosd(alfa)/
	sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001);
	in->jacobi[1]=-sind(beta)*r*0.5+cosd(beta)*(-in->l1*sind(alfa)*0.5-0.5*in->l1*in->l1*sind(alfa)*cosd(alfa)/
	sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001);
	//z
	in->jacobi[2]=cosd(beta)*r*0.5+sind(beta)*(in->l1*sind(alfa)*0.5+0.5*in->l1*in->l1*sind(alfa)*cosd(alfa)/
	sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001);
	in->jacobi[3]=cosd(beta)*r*0.5+sind(beta)*(-in->l1*sind(alfa)*0.5-0.5*in->l1*in->l1*sind(alfa)*cosd(alfa)/
	sqrt(in->l2*in->l2-in->l1*in->l1*pow(sind(alfa),2))+0.00001);
}

//由力求取力矩
void cal_torque_d(VMC *in,float dt)
{
  in->torque[Xr]=(-in->jacobi[0]*in->force[Xr]+(-in->jacobi[2]*in->force[Zr]))*in->param.flt_toqrue+(1-in->param.flt_toqrue)*in->torque[Xr];
  in->torque[Zr]=(-in->jacobi[1]*in->force[Xr]+(-in->jacobi[3]*in->force[Zr]))*in->param.flt_toqrue+(1-in->param.flt_toqrue)*in->torque[Zr];
	
	in->param.spd_dj[0]=in->torque[Xr]*in->param.gain_torque;
  in->param.spd_dj[1]=in->torque[Zr]*in->param.gain_torque;
}

void out_range_protect_d(void)
{ int i=0;
	float err[2];
	for(i=0;i<4;i++){
   if(fabs(vmc[i].epos.z)<fabs(MIN_Z)*1.05)
	  vmc[i].force[Zr]=-vmc_all.out_range_force;
   if(fabs(vmc[i].epos.z)>fabs(MAX_Z)*0.95)
    vmc[i].force[Zr]=vmc_all.out_range_force;
	 
	 if(vmc[i].epos.x>MAX_X*0.95)
    vmc[i].force[Xr]=-vmc_all.out_range_force;
	 if(vmc[i].epos.x<MIN_X*0.95)
    vmc[i].force[Xr]=vmc_all.out_range_force;
  }
	
	u8 flag_nan=0;
	for(i=0;i<4;i++)
	{
		if( isnan(vmc[i].sita1)||isnan(vmc[i].sita2))
	   flag_nan++;
	}
	#if !DEBUG_MODE		
	static float err_timer=0;
	if(flag_nan>0||
	   fabs(vmc_all.att_ctrl[PITr])>75||fabs(vmc_all.att_ctrl[ROLr])>75)
	  err_timer+=0.005;
	else
		err_timer=0;
	if(err_timer>0.05){
		err_timer=0;
		vmc_all.leg_power=0;
	  vmc_all.err=1;
	}
	#endif
}

//舵机角度坐标系转换
void vcal_pwm_from_sita_d(VMC *in)
{ u8 i=0;
	for(i=0;i<2;i++){
	in->param.PWM_OUT[i]=LIMIT(in->param.PWM_OFF[i]+in->param.sita_flag[i]*in->param.sita[i]*in->param.PWM_PER_DEGREE[i],
	in->param.PWM_MIN[i],in->param.PWM_MAX[i]);
	}
}	

void convert_mine_to_vmc_test_d(VMC *vmc)	
{ float limit=vmc->param.sita_limit;
	vmc->sita1=LIMIT(vmc->sita1,-limit,180-limit);
	vmc->sita2=LIMIT(vmc->sita2,limit,180+limit);
	vmc->param.sita[0]=vmc->sita1;
  vmc->param.sita[1]=(vmc->sita2);
	vcal_pwm_from_sita_d(vmc);
}

void convert_mine_to_vmc_d(VMC *vmc)	
{ float limit=vmc->param.sita_limit;
	vmc->sita1=LIMIT(vmc->sita1,-limit,180-limit);
	vmc->sita2=LIMIT(vmc->sita2,limit,180+limit);
	vmc->param.sita[0]=vmc->sita1+90;
  vmc->param.sita[1]=360-(vmc->sita2+90);
	vcal_pwm_from_sita_d(vmc);
}

static float att_trig_reset_dead=2.56;
char power_task_d(float dt)
{
	u8 i;
	static float t,t_rst[3],timer[5];	
	static u8 state,rst_state,soft_start_state;
	float cog_off,att_off;
	float end_dis[4];
	float att_use[3],err[2];

	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
	err[ROLr]=vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr];
	
	switch(vmc_all.power_state)
	{
		case 0:
			if(ABS(vmc_all.param.tar_spd_use_rc.x)>MIN_SPD_ST||vmc_all.sita_test[4]) 
				 t+=dt;
			else
				 t=0;
			
			if(t>1)
			{vmc_all.leg_power=vmc_all.power_state=1;t=0;}
    break;		
	  case 1://启动时缩腿保护
			 vmc_all.gait_on=0;
			 for(i=0;i<4;i++){
				vmc[i].sita1=0-30;
				vmc[i].sita2=180-(-30);
				estimate_end_state(&vmc[i],dt); 
				vmc[i].ground=1; 
	     }
			 t+=dt;
			 if(t>2)
			 {
				t=0;vmc_all.power_state=2;
				vmc[FL1].e_pos_base[0].z=vmc[FL1].e_pos_base[1].z=vmc_all.tar_pos.z;
				vmc[BL1].e_pos_base[0].z=vmc[BL1].e_pos_base[1].z=vmc_all.tar_pos.z;
				vmc[FL2].e_pos_base[0].z=vmc[FL2].e_pos_base[1].z=vmc_all.tar_pos.z;
				vmc[BL2].e_pos_base[0].z=vmc[BL2].e_pos_base[1].z=vmc_all.tar_pos.z;
			 }
		break;	
		case 2://站立
			vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.y=vmc_all.tar_spd.z=0;
		  vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
		  vmc_all.unmove=1;
		  for(i=0;i<4;i++){
		   vmc[i].tar_pos.z=vmc_all.tar_pos.z;
			 vmc[i].tar_epos.z=vmc_all.tar_pos.z;
			}
		   t+=dt;
			 if(t>2)
			 {t=0;vmc_all.power_state=3;}
		break;
    case 3://正常工作
				cog_off=vmc_all.param.cog_off_use[0];//重心补偿
				att_off=vmc_all.param.cog_off_use[1];//姿态地形跟随

		 //------------------------自动复位跨腿--------------------------
			for(i=0;i<4;i++)
			  vmc_all.end_dis[i]=end_dis[i]=ABS(vmc[i].epos.x-(cog_off+att_off));

			switch(rst_state)
			{	
				case 0:
					if(vmc_all.param.have_cmd)
						 rst_state=1;
					
					if(((end_dis[0]+end_dis[3])>vmc_all.rst_dead //腿超出自复位
						||(end_dis[1]+end_dis[2])>vmc_all.rst_dead)
					  &&vmc_all.hand_hold==0&&vmc_all.unmove==0&&fabs(err[ROLr])<att_trig_reset_dead)
					 {rst_state=2;vmc_all.delta_ht[0]=0.6*vmc_all.delta_ht[1];}
				break;
				case 1://一段时间无遥控
			   if(vmc_all.param.have_cmd==0)
					t_rst[0]+=dt;
				 else
				  t_rst[0]=0;
				 
				  vmc_all.delta_ht[0]+=(vmc_all.delta_ht[1]-vmc_all.delta_ht[0])*0.8*dt;

					if(t_rst[0]>T_RST&&fabs(err[ROLr])<att_trig_reset_dead)
					{rst_state=t_rst[0]=0;vmc_all.delta_ht[0]=0.45*vmc_all.delta_ht[1];}
				break;
				case 2://主动复位
						vmc_all.gait_on=1;
				   if(vmc_all.param.have_cmd==1)
					 {rst_state=t_rst[0]=0;}
					 else
						t_rst[0]+=dt; 
				    
					 if(t_rst[0]>vmc_all.stance_time*2)
						{rst_state=t_rst[0]=0;}
			  break;	
		  }
    break;
	}
	if(vmc_all.param.soft_start==0)
	 vmc_all.delta_ht[0]= vmc_all.delta_ht[1];
	if(vmc_all.power_state<=2)
	 vmc_all.tar_att[YAWr]=vmc_all.att_ctrl[YAWr];
	return vmc_all.leg_power;
}	

static void state_check_d(float dt)
{
 static u8 state_hold,state_unmove;
 static float timer[5];
 static float ground_reg[2][4];
 float att_use[3],err[2];
 u8 have_cmd=0,i;	
 u8 ground_num[2]={0,0};
 float acc_norm=sqrt(vmc_all.acc[Xr]*vmc_all.acc[Xr]+vmc_all.acc[Yr]*vmc_all.acc[Yr]+vmc_all.acc[Zr]*vmc_all.acc[Zr]);
  vmc_all.acc[3]=acc_norm;	
	att_use[ROLr]=vmc_all.att_ctrl[ROLr];
	err[ROLr]=vmc_all.tar_att[ROLr]+vmc_all.tar_att_off[ROLr]-att_use[ROLr];
 
 for(i=0;i<4;i++)
   ground_num[0]+=vmc[i].ground;

 if(ABS(vmc_all.param.tar_spd_use_rc.x)>MAX_SPD*0.015||ABS(vmc_all.tar_spd.z)>MAX_SPD_RAD*0.015)	
	 have_cmd=1;
//---------------------静止判断-------------------------
	switch(state_unmove)
	 {
	  case 0:	  
			vmc_all.unmove=0;
			if(have_cmd)
				timer[3]=0;
			else
				timer[3]+=dt;
			
			if(timer[3]>0.25){
				state_unmove=1;
				vmc_all.unmove=1;
			}
		break;
		case 1:
			if(have_cmd)
				state_unmove=0;
		break;
	 }	
}

void onboard_control_d(float dt)
{
	static u8 state;
	static float cnt;
	
	switch(state)
	{
	  case 0:
		if(vmc_all.param.have_cmd_rc==0){
		if(vmc_all.param.smart_control_mode[0]==MODE_SPD){//移动速度
		 vmc_all.tar_spd.x=LIMIT(vmc_all.param.tar_spd_on.x,-MAX_SPD,MAX_SPD);
		 vmc_all.tar_spd.z=LIMIT(vmc_all.param.tar_spd_on.z,-MAX_SPD_RAD,MAX_SPD_RAD);
		}else if(vmc_all.param.smart_control_mode[0]==MODE_ATT)
     vmc_all.tar_att[2]=vmc_all.param.tar_att_on[2];
			
		if(vmc_all.param.smart_control_mode[1]==MODE_POS){//高度
		 vmc_all.tar_pos.z=LIMIT(vmc_all.param.tar_pos_on.z,MAX_Z,MIN_Z);
		}

		if(vmc_all.param.smart_control_mode[2]==MODE_ATT){//姿态角
		 vmc_all.tar_att[0]=vmc_all.param.tar_att_on[1];
		 vmc_all.tar_att[1]=vmc_all.param.tar_att_on[0];
		}
		
	  }else
		{
		state=0;cnt=0;
		}
		break;
		case 1://RESET
			vmc_all.tar_att[0]=vmc_all.tar_att[1]=0;
		  vmc_all.tar_att[2]=vmc_all.att_ctrl[2];
		  vmc_all.tar_spd.x=vmc_all.tar_spd.z=0;
		  vmc_all.tar_pos.z=vmc_all.pos.z;
		  state=2;
		break;
	  case 2:
			
			if(vmc_all.param.have_cmd_rc==1)
		   cnt=0;
		  else
			 cnt+=dt;
			
			if(cnt>3)
			{state=cnt=0;}
		break;
	}		  
}

void  VMC_DEMO(float dt)
{
 static char init=0,state=0,switch_flag=0,cnt_time_change=0;
 static u16 time,time1;
 float out[2];	
 char i;
 static float timer[10];
 static char front_leg,back_leg;
	if(!init)
	{
	  init=1;
	}	
	onboard_control_d(dt);
	state_rst_d(dt);
	state_check_d(dt);
	//估计足末状态
	estimate_end_state_d(&vmc[FL1],dt);estimate_end_state_d(&vmc[BL2],dt);
	estimate_end_state_d(&vmc[FL2],dt);estimate_end_state_d(&vmc[BL1],dt);
	switch(state)
	{
		case 0:
		if(vmc_all.unmove==0&&vmc_all.hand_hold==0&&	
		  (ABS(vmc_all.param.tar_spd_use_rc.x)>MIN_SPD_ST||ABS(vmc_all.tar_spd.z)>MIN_SPD_ST||ABS(vmc_all.tar_spd.y)>MIN_SPD_ST||vmc_all.gait_on)){
				cnt_time_change++;
			  if(cnt_time_change>=3){
			  vmc_all.stance_time=vmc_all.stance_time_auto;		
			  vmc_all.delay_time[2]=vmc_all.delay_time[1];			
				cnt_time_change=0;}
			//规划落足点
			if(switch_flag)	
			{
				cal_tar_end_pos_d(&vmc[FL1]);cal_tar_end_pos_d(&vmc[BL2]);
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
			}
			else
			{
				cal_tar_end_pos_d(&vmc[FL2]);cal_tar_end_pos_d(&vmc[BL1]);
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
			}	
			state++;
	  }
		break;
		case 1:
			//跨腿
			if(switch_flag)	
			{
				trig_curve_d(&vmc[FL1],&vmc[FL1].param.tar_epos.x,&vmc[FL1].param.tar_epos.z,dt,vmc_all.trig_mode);
        vmc[FL1].param.tar_epos.z=LIMIT(vmc[FL1].param.tar_epos.z,MAX_Z,MIN_Z);vmc[FL1].param.tar_epos.x=LIMIT(vmc[FL1].param.tar_epos.x,MIN_X,MAX_X);
				trig_curve_d(&vmc[BL2],&vmc[BL2].param.tar_epos.x,&vmc[BL2].param.tar_epos.z,dt,vmc_all.trig_mode);
				vmc[BL2].param.tar_epos.z=LIMIT(vmc[BL2].param.tar_epos.z,MAX_Z,MIN_Z);vmc[BL2].param.tar_epos.x=LIMIT(vmc[BL2].param.tar_epos.x,MIN_X,MAX_X);
				inv_end_state_d(vmc[FL1].param.tar_epos.x,vmc[FL1].param.tar_epos.z,&vmc[FL1].sita1,&vmc[FL1].sita2);
				inv_end_state_d(vmc[BL2].param.tar_epos.x,vmc[BL2].param.tar_epos.z,&vmc[BL2].sita1,&vmc[BL2].sita2);
				if(vmc[FL1].ground&&vmc[FL2].ground&&vmc[BL1].ground&&vmc[BL2].ground)
				{state++;vmc[FL1].param.trig_state=vmc[BL2].param.trig_state=0;
				 vmc[FL1].param.ground_state=vmc[BL2].param.ground_state=1;
				}
			}
			else
			{
				trig_curve_d(&vmc[FL2],&vmc[FL2].param.tar_epos.x,&vmc[FL2].param.tar_epos.z,dt,vmc_all.trig_mode);
				vmc[FL2].param.tar_epos.z=LIMIT(vmc[FL2].param.tar_epos.z,MAX_Z,MIN_Z);vmc[FL2].param.tar_epos.x=LIMIT(vmc[FL2].param.tar_epos.x,MIN_X,MAX_X);
				trig_curve_d(&vmc[BL1],&vmc[BL1].param.tar_epos.x,&vmc[BL1].param.tar_epos.z,dt,vmc_all.trig_mode);
				vmc[BL1].param.tar_epos.z=LIMIT(vmc[BL1].param.tar_epos.z,MAX_Z,MIN_Z);vmc[BL1].param.tar_epos.x=LIMIT(vmc[BL1].param.tar_epos.x,MIN_X,MAX_X);
				inv_end_state_d(vmc[FL2].param.tar_epos.x,vmc[FL2].param.tar_epos.z,&vmc[FL2].sita1,&vmc[FL2].sita2);
				inv_end_state_d(vmc[BL1].param.tar_epos.x,vmc[BL1].param.tar_epos.z,&vmc[BL1].sita1,&vmc[BL1].sita2);
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

	 spd_control_d(dt);
	  
	 if(gait_test[0]){
		gait_test[1]+=dt*(gait_test[2]);if(gait_test[1]>360)gait_test[1]=0;
		gait_test[4]=sind(gait_test[1])*vmc_all.param.max_l*gait_test[3];
   }
	 if(gait_test[0]==1)//高度SIN
		vmc_all.tar_pos.z=LIMIT(MIN_Z-vmc_all.param.max_l*0.6+gait_test[4],MAX_Z,MIN_Z);
   else if(gait_test[0]==2)//俯仰SIN
	  vmc_all.tar_att_off[PITr]=LIMIT(sind(gait_test[1])*11,-22,22);
	 else if(gait_test[0]==3)//力矩SIN
		 for(i=0;i<4;i++)
				vmc[i].tar_epos.z=LIMIT(MIN_Z-vmc_all.param.max_l*0.6+gait_test[4],MAX_Z,MIN_Z);
	 
	 //---------------足尖虚拟力控制-------------------
   for(i=0;i<4;i++){	
		 if(vmc[i].ground){	
			 //力控制
			 if(vmc_all.power_state>=2)
				 vmc_force_control_d(&vmc[i],dt);
			 
			 cal_jacobi_d(&vmc[i]);
				
			 //反馈控制
			 if(vmc_all.power_state==3&&!DEBUG_MODE)
			   cal_force_y_d(&vmc[i],dt);
			 cal_force_z_d(&vmc[i],dt);	
			 		 	 
			 out_range_protect_d(); 		 
			 //力矩输出
			 cal_torque_d(&vmc[i],dt);
			 vmc[i].sita1+=vmc[i].param.spd_dj[0]*dt;
			 vmc[i].sita2+=vmc[i].param.spd_dj[1]*dt;
			 LIMIT(vmc[i].sita1,-40,90);LIMIT(vmc[i].sita2,90,210);
			 vmc[i].tar_epos.z=LIMIT(vmc[i].epos.z,MAX_Z,MIN_Z);
		 }
   }	 
	 //计算PWM
	 if(vmc_all.sita_test[4]){//强制角度测试
	 for(i=0;i<4;i++){
		 vmc[i].sita1=vmc_all.sita_test[0];
		 vmc[i].sita2=vmc_all.sita_test[1];
		 convert_mine_to_vmc_test_d(&vmc[i]);
	 }
   }else{
		 convert_mine_to_vmc_d(&vmc[FL1]);convert_mine_to_vmc_d(&vmc[BL1]);
		 convert_mine_to_vmc_d(&vmc[FL2]);convert_mine_to_vmc_d(&vmc[BL2]);
   }			
}
#endif
