
#include "stm32f4xx.h"
/*
Copyright(C) 2018 Golaced Innovations Corporation. All rights reserved.
OLDX_VMC四足机器人库 designed by Golaced from 云逸创新

qq群:567423074
淘宝店铺：https://shop124436104.taobao.com/?spm=2013.1.1000126.2.iNj4nZ
手机店铺地址：http://shop124436104.m.taobao.com
vmc.lib 封装了虚拟力模型下足式机器人步态算法基本的库
*/

#define USE_DEMO 1   //使用开源DEMO程序
#define VIR_MODEL 0  //使用虚拟机器人模型

#define CHECK_USE_SENSOR 0//跨腿使用机器人传感器数据估计着地点
#define DEBUG_MODE 0 			//测试模式
#define USE_LOW_TORQUE 1		//低舵机力矩
#define HIGE_LEG_TRIG 0 	//高抬腿轨迹
#define SLIP_MODE     1   //弹簧模式
//----ROBOT Select
#define MINI_LITTRO_DOG  
//#define BIG_LITTRO_DOG

#define MIN_SPD_ST 0.0005
#define T_RST 1.5
#define MAX_FSPD 0.6   //最大速度限制m/s
#define POS_DEAD 0.05  //航点死区m
#define YAW_POS_MAX 25

extern float MIN_Z,MAX_SPD,MAX_SPD_RAD;
extern float MAX_Z,MIN_X,MAX_X;
extern float gait_test[5];
extern u8 force_dj_off_reset;
typedef struct 
{
	float x;
	float y;
	float z;
	float zz;
}END_POS;

typedef struct 
{
	float fp,kp,ki,kd,max_ki,max_out;
	float p,i,d,out;
	float err,interge,err_reg;
	float kp_i,k_i,kd_i,max_ki_i,max_out_i;
	float fp_i,p_i,i_i,d_i,out_i;
	float err_i,interge_i,err_reg_i;
	float in_set_force;
}PID;

extern PID h_pid_all,att_pid_all[3],pos_pid[2],pos_pid_all;

typedef struct
{
	PID out;
	PID in;
}D_PID;

typedef struct 
{
  char id,trig_state,ground_state;
	float sita_limit;
	float spd_dj[2];
	float flt_toqrue;
	float gain_torque;
	float delta_h,delta_h_att_off;
	float kp_trig;
	float time_trig;
	float spd_est_cnt;
	float pid[3][5];
	float ground_force[3];
	END_POS tar_epos;
	u16 PWM_OFF[4];
	u16 PWM_INIT[3];
	u16 PWM_MIN[4],PWM_MAX[4];
	float PWM_OUT[4];
	float PWM_PER_DEGREE[4];
	int sita_flag[4];
	float sita[4];
}PARAM;


typedef struct 
{
	float l1,l2,l3,l4;
	float r;
	float sita1,sita2,sita;
	END_POS epos,e_pos_base[2],epos_reg,spd,spd_o,spd_o_angle,tar_epos,tar_epos_force,check_epos,tar_spd,st_pos,tar_pos;
	float sita_reg[2];
	float tar_h,h,dh;
	double jacobi[4];
	char ground,ground_s;
	float ground_force[3];
	float force[3],force_cmd[3];
	float torque[3];
	int flag_fb,flag_rl;
	float force_deng[4];
	PARAM param;
}VMC;


//----------------------------------VMC全局控制器结构体--------------------------
typedef struct 
{
  //----------系统参数----------------
  float end_sample_dt;//足端速度微分时间
	float ground_dump;//触底回缩比例
	float trig_ground_st_rate;//跨腿后使能触地停止的比例
	float ground_rst;//自触地周期
	float angle_set_trig;//跨腿触地停止角度限制
	float control_out_of_maxl;//姿态控制最大控制量与腿长比例	
	float att_limit_for_w;//计算权重最大的角度限制
	float kp_pose_reset[2];//自复位回中增益
	//------------on board----------
	END_POS tar_pos_on,tar_spd_on;
	float tar_att_on[3];
  //-----------系统变量--------------
	float ground_force[4][3];
	float encoder_spd[2];
	float cog_off_use[4];
	END_POS tar_spd_use[2],tar_spd_use_rc;
	float line_z[2];//交叉失配高度
	float cog_z_curve[3];//模型预测COG各阶状态
	float w[10],w_t[10],w_cmd[10],weight[10];
	float max_l;//最大腿长
	float dt_size;//控制周期变化导致的参数缩放比例
	char jump_state;
	float jump_tar_param[2];//高度 距离
	float jump_param_use[5];
	float jump_out[3];
	//-------------模式-------------
	char control_mode_all;
	char have_cmd,have_cmd_rc,en_sdk,soft_start;
	char control_mode,rc_mode[2];//控制模式
	char en_hold_on;
	char en_att_tirg;
	char cal_flag[5];//机器人校准标志位
	char smart_control_mode[3];//pos/spd   high  att/rad
}PARAM_ALL;

typedef struct 
{
	//-------------授权码相关--------------
	u8 key_right;//授权码是否正确
	char your_key[3];
	int board_id[3];//控制器ID
	float sita_test[5];
	//---------------------------------------
	u8 ground[2][4];
	END_POS tar_pos,tar_spd,tar_spd_rc;
	float tar_att[3],tar_att_rate[3],tar_att_off[3],ground_off[2];
	//------------------------------------------
	float kp_trig[2],kp_deng_gain[2],kp_deng[4],kd_deng[2],deng_all,kp_deng_ctrl[2][4],k_auto_time,kp_g[2],kp_touch,rst_dead,out_range_force;
  float pid[3][7],att_gain_length[2];
	float delta_ht[2],delta_h_att_off,gain_torque;
	float cog_off[5],off_leg_dis[2];
	//
  float l1,l2,l3,l4,W,H,mess,flt_toqrue;
	float gait_time[4];
	float gait_alfa;//0~1
	float delay_time[3],gait_delay_time,stance_time,stance_time_auto;
	//
	float att_trig[3],att_ctrl[4],att_rate_trig[3],att_rate_ctrl[3],att[3],att_rate[3],att_vm[3],att_vmo[3],acc[4];
	float body_spd[4];
	END_POS pos,pos_n;
	END_POS spd,spd_n;
	END_POS acc_n,acc_b;
	float acc_norm;
	char use_att,att_imu,trig_mode,gait_on,use_ground_sensor,ground_num,leg_power,power_state;
	float end_dis[4];
	u8 err,unmove,hand_hold,fall,fly;
	PARAM_ALL param;
}VMC_ALL;

extern VMC_ALL vmc_all;
extern VMC vmc[4];
void get_license(void);

void VMC_DEMO(float dt);
void VMC_OLDX_VER1(float dt);
void VMC_OLDX_VER2(float dt);
void vmc_init(void);
char power_task(float dt);
char power_task_d(float dt);
char estimate_end_state_d(VMC *in,float dt);
char cal_jacobi(VMC *in);
char cal_jacobi_d(VMC *in);
//----------------------------VMC.lib 机器人库头文件-------------------------------
char estimate_end_state(VMC *in,float dt);
float cosd(double in);
float tand(double in);
float sind(float in);
float dead(float x,float zoom);
void DigitalLPF(float in, float* out, float cutoff_freq, float dt);
float att_to_weight(float in, float dead, float max);


//-------------------------------SDK--------------------------
extern int mission_sel_lock;
extern int mission_state;
extern u8 mission_flag;
void smart_control(float dt);
	
	
	
#define L 0
#define R 1
#define F 0
#define B 1
#define FL 0
#define BL 1
#define FL1 0
#define BL1 1
#define FL2 2
#define BL2 3
#define Xr 0
#define Yr 1
#define Zr 2
#define P 0
#define I 2
#define D 1
#define FB 3
#define IP 4
#define II 5
#define ID 6
#define PITr 0
#define ROLr 1
#define YAWr 2
#define YAWrr 3

#define MODE_SPD 1
#define MODE_POS 2
#define MODE_RAD 3
#define MODE_ATT 4
#define MODE_BODY 5
#define MODE_GLOBAL 6
#define MODE_ATT_YAW_ONLY 7

#define RAD_TO_DEG 180/PI
#define DEG_TO_RAD PI/180
#define PI 3.14159267

//舵机时间参数
#define DJ_MG995 9.34
#define DJ_MG956 9.34
#define DJ_MG955 9.34
#define DJ_MG945 5.7
#define DJ_MG355 11.34
#define DJ_6221MG 12.64
#define DJ_DSERVO 11.34
#define DJ_9G 9.34
#define DJ_9G1 10.34
#define DJ_DS 11.3
#define DJ_DSB 11.3
#define DJ_DSB2 19.3
#define DJ_KPOWER 11.3