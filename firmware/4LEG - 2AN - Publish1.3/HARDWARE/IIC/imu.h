#ifndef _IMU_H_
#define	_IMU_H_

#include "stm32f4xx.h"

#include "parameter.h"
#include "my_math.h"
#include "math.h"

typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;
int madgwick_update_new(float T,float wx, float wy, float wz, float ax, float ay, float az,float mx,float my,float mz,float *rol,float *pit,float *yaw);
extern xyz_f_t reference_v;
extern float reference_vr[3],accConfidence;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw);
extern float Roll,Pitch,Yaw;
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
extern float ref_q[4] , q_nav[4];

extern float reference_vr[3];
extern float Roll,Pitch,Yaw,accConfidence;    				//вкл╛╫г
#endif

