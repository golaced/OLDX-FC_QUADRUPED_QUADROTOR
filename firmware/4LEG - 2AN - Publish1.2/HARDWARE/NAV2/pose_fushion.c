#include "include.h"
#include "my_math.h"
#include "filter.h"
#include "imu.h"
#include "nav.h"
#include "vmc.h"

_NAV nav;
//-----------------KF  parameter------------------
double P_kf_pose[2][16]; 
double X_kf_pose[2][4];

double Q_pose[4]={0.001,0.001,0.001,0.001};
double R_pose[3]={0.1,0.001,0.05};
float w_spd=0.3;//外部里程计置信因子
float k_fack_yaw=0.1;
void pose_fushion(float T)
{
  double Zm[2][3],M_flag[4]={0,0,1,0};//p v a bv
	float yaw_use;
	if(nav.init_cnt[Xr]++>5/T)
		nav.init[Xr]=nav.init[Yr]=1;
		
	if(M_flag[0]){//position :uwb/ laser/ gps / visual odometer
		
	}

	nav.spd_b_o[Yr]=(vmc_all.param.encoder_spd[R]+vmc_all.param.encoder_spd[L])/(2);
	nav.spd_b_o[Zr]=(vmc_all.param.encoder_spd[R]-vmc_all.param.encoder_spd[L])/(vmc_all.W/2)*RAD_TO_DEG;
	nav.fake_yaw=nav.fake_yaw-k_fack_yaw*nav.spd_b_o[Zr]*T;
	nav.fake_yaw=To_180_degrees(nav.fake_yaw);
	
	if(M_flag[1]){//visual speed:optical flow / visual odometer
		nav.spd_b_o[Yr]=nav.spd_b_o[Xr]*w_spd+(1-w_spd);
		nav.spd_b_o[Yr]=nav.spd_b_o[Yr]*w_spd+(1-w_spd);
	}
	
  yaw_use=vmc_all.att[YAWr];
	
	nav.acc_b_o[Xr]=vmc_all.acc[Yr];
	nav.acc_b_o[Yr]=vmc_all.acc[Xr];
	nav.acc_n_o[Xr]=nav.acc_b_o[Yr]*sind(yaw_use)+nav.acc_b_o[Xr]*cosd(yaw_use);
  nav.acc_n_o[Yr]=nav.acc_b_o[Yr]*cosd(yaw_use)-nav.acc_b_o[Xr]*sind(yaw_use);
	
	nav.spd_n_o[Yr]=cosd(yaw_use)*nav.spd_b_o[Yr];
	nav.spd_n_o[Xr]=sind(yaw_use)*nav.spd_b_o[Yr];
	
  Zm[Xr][0]=nav.pos_n_o[Xr];
	Zm[Xr][1]=nav.spd_n_o[Xr];
	Zm[Xr][2]=nav.acc_n_o[Xr];
  Zm[Yr][0]=nav.pos_n_o[Yr];
	Zm[Yr][1]=nav.spd_n_o[Yr];
  Zm[Yr][2]=nav.acc_n_o[Yr];	
	
	
	double Z_f[3]={M_flag[0],M_flag[2],M_flag[3]};
	if(nav.init[Xr]){
		 pose_kf(X_kf_pose[Xr], P_kf_pose[Xr],  Zm[Xr], Z_f, Q_pose, R_pose,  T);
		 pose_kf(X_kf_pose[Yr], P_kf_pose[Yr],  Zm[Yr], Z_f, Q_pose, R_pose,  T);
	}
	
	nav.pos_n[Xr]=X_kf_pose[Xr][0];
	nav.pos_n[Yr]=X_kf_pose[Yr][0];
	nav.att[YAWr]=yaw_use;
	nav.spd_b[Xr]=-X_kf_pose[Yr][1]*sind(yaw_use)+X_kf_pose[Xr][1]*cosd(yaw_use);
	nav.spd_b[Yr]= X_kf_pose[Yr][1]*cosd(yaw_use)+X_kf_pose[Xr][1]*sind(yaw_use);
	nav.att_rate[Zr]=vmc_all.att_rate[YAWr];
}