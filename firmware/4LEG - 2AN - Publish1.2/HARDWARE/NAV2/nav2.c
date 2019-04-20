#include "include.h"
#include "my_math.h"
#include "filter.h"
#include "imu.h"
#include "nav.h"
#include "vmc.h"
//#include "ekf_pos.h"
_NAV nav;
//-----------------KF  parameter------------------
double P_kf_pose[36]; 
double X_kf_pose[6];

double Q_pose[4]={0.001,0.001,0.001,0.001};
double R_pose[5]={0.001,0.001,0.001,0.001,0.001};
void pose_fushion(float T)
{
  double Zm[9],Z_flag[4]={0};
	
	if(nav.init_cnt[Xr]++>5/T)
		nav.init[Xr]=nav.init[Yr]=1;
			
	//pose
	Zm[0]=0;
	Zm[1]=0;
	//yaw
	Zm[2]=vmc_all.att[YAWr]*DEG_TO_RAD;
	//spd
	Zm[3]=0;
	Zm[4]=0;
	//w
	Zm[5]=vmc_all.att_rate[Zr]*DEG_TO_RAD;
	//encoder
	Zm[6]=vmc_all.param.encoder_spd[R];
	Zm[7]=vmc_all.param.encoder_spd[L];
	//L
	Zm[8]=vmc_all.W/2;
	
	Z_flag[0]=0;//pose
	Z_flag[1]=0;//yaw
	Z_flag[2]=0;//spd
	Z_flag[3]=1;//w
	if(nav.init[Xr])
		ekf_pos(X_kf_pose, P_kf_pose,  Zm, Z_flag, Q_pose, R_pose,  T);
	
	nav.pos_n[Xr]=X_kf_pose[0];
	nav.pos_n[Yr]=X_kf_pose[1];
	nav.att[YAWr]=X_kf_pose[2]*RAD_TO_DEG;
	nav.spd_b[Xr]=X_kf_pose[3];
	nav.spd_b[Yr]=X_kf_pose[4];
	nav.spd_b[Zr]=X_kf_pose[5]*RAD_TO_DEG;
}