#include "imu.h"
#include "filter.h"
#include "arm_math.h"
#include "nav.h"
#include "ms5611.h"
#include "vmc.h"
#include "baro_kf.h"
double X_f32[4];
double P_f32[16] =
{
  10.0,     0,      0,     0,
  0,     10,      0,    0,
  0,     0,      10,      0,
  0,     0,     0,    10,
};

float32_t Is_4_4[16] =
{
  1,       0,      0,     0,
  0,       1,      0,     0,
  0,       0,      1,     0,
  0,       0,      0,     1
};

double Qs_4_4[4] =
{
  /* Const,   numTaps,   blockSize,   numTaps*blockSize */
   0.00000001,    
   0.00000001,  
   0.01,  
   0.00000001
};

double Rs_2_2[2] =
{
  /* Const,   numTaps,   blockSize,   numTaps*blockSize */
  10,   
      0.02
};


void baro_fushion(float dt)
{
int i;	
static char init;	
uint32_t srcRows, srcColumns;  /* Temporary variables */
arm_status status;

float32_t As_4_4[16] =
{
  1.0,   dt,        0.5*dt*dt,     0,
  0,     dt,        0,             0,
  0,     0,         1.0,           0,
  0,     0,         0,           1.0,
};

float32_t Hs_2_4[8] =
{
  1 ,0,0,0,
	0 ,0,1,1
};

if(!init){
	init=1;
}

if(ms5611.init_off){
 	
//	arm_matrix_instance_f32 As,Hs,Qs,Rs,X_pre,P_pre,Is;
//	float32_t P_pre_4_4[16],X_pre_4_1[4];
//  arm_mat_init_f32(&As, 4, 4, As_4_4);
//	arm_mat_init_f32(&P_pre, 4, 4, P_pre_4_4);	
//  arm_mat_init_f32(&X_pre, 4, 1, X_pre_4_1);		
//	arm_mat_init_f32(&Qs, 4, 4, Qs_4_4);	
//	arm_mat_init_f32(&Is, 4, 4, Is_4_4);		
//	arm_mat_init_f32(&Rs, 2, 2, Rs_2_2);		
//	arm_mat_init_f32(&Hs, 2, 4, Hs_2_4);		

//	arm_matrix_instance_f32 Temp44[10],Temp42[10],Temp41[10],Temp22[10],Temp24[10],Temp21[10];
//	float32_t Temp_4_4[10][16],Temp_2_2[10][4],Temp_4_2[10][8],Temp_4_1[10][4],Temp_2_4[10][8],Temp_2_1[10][2];
//	for(i=0;i<10;i++){
//	arm_mat_init_f32(&Temp44[i], 4, 4, Temp_4_4[i]);
//	arm_mat_init_f32(&Temp22[i], 2, 2, Temp_2_2[i]);
//	arm_mat_init_f32(&Temp42[i], 4, 2, Temp_4_2[i]);
//	arm_mat_init_f32(&Temp41[i], 4, 1, Temp_4_1[i]);
//	arm_mat_init_f32(&Temp24[i], 2, 4, Temp_2_4[i]);
//	arm_mat_init_f32(&Temp21[i], 2, 1, Temp_2_1[i]);
//}

//arm_matrix_instance_f32 Xss,Ps;      /* Matrix A Instance */
//arm_mat_init_f32(&Xss, 4, 1, X_f32);
//arm_mat_init_f32(&Ps,  4, 4, P_f32);
// // Ô¤²â X=X+A*X
////arm_mat_mult_f32 (&As,&Xss,&Temp41[0]);
////arm_mat_add_f32 (&Temp41[0],&Xss,&X_pre);
//Xss.pData[0]=X_f32[0]+X_f32[1]*dt+0.5*X_f32[2]*dt*dt;
//Xss.pData[1]=X_f32[1]+X_f32[2]*dt;
//Xss.pData[2]=X_f32[2];
//Xss.pData[3]=X_f32[3];
// // Ô¤²âÐ­·½²î P=A*P*A'+Q
// arm_mat_mult_f32 (&As,&Ps,&Temp44[0]);
// arm_mat_trans_f32(&As,&Temp44[1]);
// arm_mat_mult_f32 (&Temp44[0],&Temp44[1],&Temp44[2]);
// arm_mat_add_f32  (&Temp44[2],&Qs,&P_pre);
// 
// // ×´Ì¬¸üÐÂ
//arm_matrix_instance_f32 Ss; 
//float32_t Ss_2_2[4];
//arm_mat_init_f32(&Ss, 2, 2, Ss_2_2);	
////s_k=H*P_apr*H'+R;
//arm_mat_mult_f32 (&Hs,&P_pre,&Temp24[0]);
//arm_mat_trans_f32(&Hs,&Temp42[0]);
//arm_mat_mult_f32 (&Temp24[0],&Temp42[0],&Temp22[0]);
//arm_mat_add_f32  (&Temp22[0],&Rs,&Ss);
// 
////K_k=(P_apr*H'/(s_k));
//arm_matrix_instance_f32 Ks42; 
//float32_t Ks_4_2[8];
//arm_mat_init_f32(&Ks42, 4, 2, Ks_4_2);	

//arm_mat_trans_f32(&Hs,&Temp42[0]);
//arm_mat_mult_f32 (&P_pre,&Temp42[0],&Temp42[1]);
//arm_mat_inverse_f32(&Ss,&Temp22[0]);
//arm_mat_mult_f32(&Temp42[1],&Temp22[0],&Ks42);

////y_k=z-H*x_apr;
//arm_matrix_instance_f32 Z21; 
double Z_2_1[2] ={(float)ms5611.baroAlt,vmc_all.acc[Zr]*100 };
//arm_mat_init_f32(&Z21, 2, 1, Z_2_1);	

//arm_mat_mult_f32 (&Hs,&X_pre,&Temp21[0]);
//arm_mat_sub_f32 (&Z21,&Temp21[1],&Temp21[1]);

////x_apo=x_apr+K_k*y_k;
//arm_mat_mult_f32 (&Ks42,&Temp21[1],&Temp41[0]);
//arm_mat_add_f32 (&X_pre,&Temp41[0],&Xss);

////P_apo=(eye(4)-K_k*H)*P_apr;
//arm_mat_mult_f32 (&Ks42,&Hs,&Temp44[0]);
//arm_mat_sub_f32  (&Is,&Temp44[0],&Temp44[1]);
//arm_mat_mult_f32 (&Temp44[1],&P_pre,&Ps);

//for(i=0;i<4;i++)
//	X_f32[i]=Xss.pData[i];
//for(i=0;i<16;i++)
//	P_f32[i]=Ps.pData[i];

double T=dt;
baro_kf(X_f32, P_f32, Z_2_1, ms5611.update, Qs_4_4,Rs_2_2,  T);
ms5611.update=0;

nav.pos_n[Zr]=X_f32[0]/100;
nav.spd_n[Zr]=X_f32[1]/100;
nav.acc_n[Zr]=X_f32[2];
}
}