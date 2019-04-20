#ifndef __CIRCLE_H
#define __CIRCLE_H	 
#include "stm32f4xx.h" 
#include "include.h" 
#include "gps.h"
 typedef struct
{
 int x,y;
 int r;
 int x_flp,y_flp;
 u8 check;
 u8 connect,lose_cnt;
 int control[2];
 float control_k;
 float control_k_miss; 
	float control_yaw;
 float forward;
 float forward_end_dj_pwm;
	u8 dj_fly_line;
}CIRCLE;
extern CIRCLE circle,track,mouse;
extern float nav_circle[2],nav_land[2];
void circle_control(float T);
#define MID_Y 125
#define MID_X 140
extern float circle_use[2];
extern float  integrator[2];
void  GPS_hold(nmea_msg *gpsx_in,float T);

 typedef struct
{
 float Pit,Rol,Yaw;
 float Lat,Lon;
 float H,H_Spd;	
 u8 GPS_STATUS;	//>3  5->Best
/*
Flight status val	status name
1	standby
2	take_off
3	in_air
4	landing
5	finish_landing
*/
 u8 STATUS;
 float Bat;	
 int Rc_pit,Rc_rol,Rc_yaw,Rc_thr,Rc_mode,Rc_gear;

}M100;
extern M100 m100;

#endif











