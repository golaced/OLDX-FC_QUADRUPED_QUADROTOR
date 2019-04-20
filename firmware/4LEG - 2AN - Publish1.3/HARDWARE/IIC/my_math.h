#include "stm32f4xx.h"

#ifndef __MYMATH_H__
#define __MYMATH_H__

#define REAL   float
#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256
#define MY_PPPIII   3.14159f
#define MY_PPPIII_HALF   1.570796f

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
float my_atan(float x, float y);
float my_abs(float f);
REAL fast_atan2(REAL y, REAL x);
float my_pow(float a);
float my_sqrt(float number);
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deathzoom(float x,float zoom);
float my_deathzoom_2(float x,float zoom);
float To_180_degrees(float x);
float my_pow_2_curve(float in,float a,float max);
float limit_mine(float x,float zoom);
float limit_mine2(float x,float min,float max);
float my_deathzoom_21(float x,float zoom);
float my_deathzoom_rc(float x,float zoom);


typedef struct
{ //control parameter
	float h0;
	float v1,v2,r0;
}ESO_X;
void OLDX_SMOOTH_IN_ESOX(ESO_X *eso_in,float in);


typedef struct
{
 float pt[3];
 float vt[3];
 float at[3];
 float ps[3];
 float vs[3];
 float as[3];
 float pe[3];
 float ve[3];
 float ae[3];
 float param[10];
 float Time,time_now,Dis;
 float cost,cost_all;
 float traj_pre_d;
 char defined[3];
	
}_TRA;

extern _TRA traj[10];

void plan_tra(_TRA *tra);
void get_tra(_TRA *tra,float t);
#endif

