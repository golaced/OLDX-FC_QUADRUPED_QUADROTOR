#include "include.h" 
#include "usart_fc.h"
#include "my_math.h"

LEG_STRUCT leg[5];
BRAIN_STRUCT brain;
//% leg ending positon calculate'
//%         _                side view            ----  ------>x  back view
//%         |  l1                               3  ||   |
//%      __ O                o---> y               OO   |
//%     1  //                |                     ||   \/ Z 
//%       //     l2          |                     ||
//%      O                 Z \/                    OO
//%    /  \\                                       ||
//%   / 2  \\    l3                                ||
//%          O                                     OO
//PWM gain of Pan


#define DJ_KPOWER1 11.3//11.56//12.35
float off_local[2]={0,0.4};
float k_z=1;


#if TIRG_CURVE_USE_BAI
float flt_leg=0;
#else
float flt_leg=1;//腿限制幅度滤波	
#endif
float flt1=0.5;//跨腿最终点滤波
u8 en_pos_recal=0;//使用POSnow计算值作为输出
u16 SET_PWM3_OFF=0;

void READ_LEG_ID(LEG_STRUCT *in)
{
in->sys.id=0;
}

void leg_init( LEG_STRUCT *in,u8 id)
{
static u8 init;
if(!init)
	init=1;
	
in->sys.id=id;
in->leg_ground=1;
in->sys.init_mode=0;	

in->sys.l1=3.7;
in->sys.l2=7.2;
in->sys.l3=7.2;//6.8;	

in->sys.off_local[0]=off_local[0];
in->sys.off_local[1]=off_local[1];
switch(in->sys.id){
case 1:	//<--------------------------------1
in->sys.leg_set_invert=0;
in->sys.PWM_OFF[0]=640;	
in->sys.PWM_OFF[1]=165;	
in->sys.sita_flag[0]=1;
in->sys.sita_flag[1]=1;	
in->sys.pwm_id[0]=0;
in->sys.pwm_id[1]=1;
break;
case 2:	
in->sys.leg_set_invert=0;
in->sys.PWM_OFF[0]=2300;	
in->sys.PWM_OFF[1]=2400;	
in->sys.sita_flag[0]=-1;
in->sys.sita_flag[1]=-1;	
in->sys.pwm_id[0]=3;
in->sys.pwm_id[1]=4;
break;
case 3:	
in->sys.leg_set_invert=0;
in->sys.PWM_OFF[0]=2320;	
in->sys.PWM_OFF[1]=2360;	
in->sys.sita_flag[0]=-1;
in->sys.sita_flag[1]=-1;	
in->sys.pwm_id[0]=5;
in->sys.pwm_id[1]=6;
break;
case 4:	
in->sys.leg_set_invert=0;
in->sys.PWM_OFF[0]=860;	
in->sys.PWM_OFF[1]=580;	
in->sys.sita_flag[0]=1;
in->sys.sita_flag[1]=1;	
in->sys.pwm_id[0]=7;
in->sys.pwm_id[1]=8;
break;
}
switch(in->sys.id){
case 1:
in->sys.PWM_PER_DEGREE[0]=DJ_KPOWER1;
in->sys.PWM_PER_DEGREE[1]=DJ_KPOWER1;
in->sys.PWM_PER_DEGREE[2]=DJ_KPOWER1;
in->sys.PWM_PER_DEGREE[3]=DJ_KPOWER1;
break;
case 2:
in->sys.PWM_PER_DEGREE[0]=DJ_KPOWER1;	
in->sys.PWM_PER_DEGREE[1]=DJ_KPOWER1;
in->sys.PWM_PER_DEGREE[2]=DJ_KPOWER1;
in->sys.PWM_PER_DEGREE[3]=DJ_KPOWER1;
break;
case 3:
in->sys.PWM_PER_DEGREE[0]=DJ_KPOWER1;//da
in->sys.PWM_PER_DEGREE[1]=DJ_KPOWER1;//xiao
in->sys.PWM_PER_DEGREE[2]=DJ_KPOWER1;//zhuan
in->sys.PWM_PER_DEGREE[3]=DJ_KPOWER1;
break;
case 4:	
in->sys.PWM_PER_DEGREE[0]=DJ_KPOWER1;	
in->sys.PWM_PER_DEGREE[1]=DJ_KPOWER1;
in->sys.PWM_PER_DEGREE[2]=DJ_KPOWER1;
in->sys.PWM_PER_DEGREE[3]=DJ_KPOWER1;
break;
}

int flag[2]={1,1};
if(in->sys.id==3||in->sys.id==4)
	flag[0]=-1;
if(in->sys.id==2||in->sys.id==4)
	flag[1]=-1;
in->sys.init_end_pos.x=in->pos_tar[2].x=in->sys.pos_tar_trig_test[2].x=in->sys.off_local[0]*flag[0];	
in->sys.init_end_pos.y=in->pos_tar[2].y=in->sys.pos_tar_trig_test[2].y=in->sys.off_local[1]*flag[1];		
in->sys.init_end_pos.z=in->pos_tar[2].z=in->sys.pos_tar_trig_test[2].z=(in->sys.l1*sin(45*0.0173)+in->sys.l2*sin(45*0.0173))*k_z;
	
in->pos_tar_trig[2].x=in->sys.init_end_pos.x;
in->pos_tar_trig[2].y=in->sys.init_end_pos.y;
in->pos_tar_trig[2].z=in->sys.init_end_pos.z;
in->pos_now_trig_f[2].z=in->sys.init_end_pos.z;
in->sys.limit.x=(in->sys.l1+in->sys.l2+in->sys.l3)*sin(35*ANGLE_TO_RADIAN);	
in->sys.limit.y=(in->sys.l1+in->sys.l2+in->sys.l3)*sin(35*ANGLE_TO_RADIAN);//0.25;	
in->sys.limit.z=(in->sys.l1+in->sys.l2*cos(30*ANGLE_TO_RADIAN)+in->sys.l3*cos(30*ANGLE_TO_RADIAN))*1.3;	
	
in->sys.limit_min.z=(in->sys.l3-(in->sys.l2-in->sys.l1))*1.25;
in->sita_ss_limit=45;
in->sys.desire_time=0.4;

int DL=88;
in->sys.PWM_MIN[0]=500+DL;	
in->sys.PWM_MIN[1]=500+DL;
in->sys.PWM_MIN[2]=500+DL;
in->sys.PWM_MIN[3]=500+DL;
in->sys.PWM_MAX[0]=2500-DL;
in->sys.PWM_MAX[1]=2500-DL;	
in->sys.PWM_MAX[2]=2500-DL;	
in->sys.PWM_MAX[3]=2500-DL;	
in->sys.en_pwm_out=1;

in->sys.leg_up_high=3;

in->leg_ground=1;

}	

////位置超出可移动范围检测
//u8 pos_range_check(LEG_STRUCT * in,float x,float y,float z)
//{
// float r=(in->sys.l1+in->sys.l2+in->sys.l3)*1;
// float r_pos=sqrt(x*x+y*y+z*z);
// if(r_pos>r||r_pos<(in->sys.l3-(in->sys.l2-in->sys.l1))*1)
//	 return 0;
// else
//	 return 1;
//}

////
//u8 cal_sita_form_pos_tri_leg(float r1,float r2,float x,float y,float z,float *sita1,float *sita2,float *sita3)
//{
//	u8 i;
//	float sita_out[3];

//	sita_out[2]=atan2(x,sqrt(y*y+z*z))*57.3;	
//	
//	
//	float y_t=y,z_t=-z/cos(sita_out[2]*ANGLE_TO_RADIAN);
//	
//	float r=sqrt(z_t*z_t+y_t*y_t);//%??? 
//	if (r1+r2<=r || ABS(r1-r2)>=r) 	
//		return 0;
//	float seta;
//	seta=acos((r1*r1+r*r-r2*r2)/2/r/r1); // %??????????????? 
//	float r_seta;
//	r_seta=atan2(z_t,y_t);		//	%???????(?x???) 
//	float alpha[2];
//  alpha[0]=r_seta-seta;
//  alpha[1]=r_seta+seta;  	//%?????x???? 
//	float crossy[2],crossz[2];
//	for (i=0;i<2;i++){
//	crossy[i]=r1*cos(alpha[i]); 
//	crossz[i]=r1*sin(alpha[i]);
//  }
//	
//	float s1,s2;
//	if (crossy[0]<0)
//	s1=-atan(crossz[0]/crossy[0])*57.3+90+180;   
//	else    
//	s1=-atan(crossz[0]/crossy[0])*57.3+90;
//	
//	if (crossy[1]<0)
//	s2=-atan(crossz[1]/crossy[1])*57.3+90+180;  
//	else   
//	s2=-atan(crossz[1]/crossy[1])*57.3+90;
//	
//	if(s1>s2)
//	{sita_out[1]=s1;sita_out[0]=s2;}
//	else
//  {sita_out[1]=s2;sita_out[0]=s1;}
//	*sita1=sita_out[0];
//	*sita2=360-sita_out[1];
//	*sita3=sita_out[2];
//	
//	return 1;
//}

//void cal_pos_from_sita(LEG_STRUCT * in)
//{
//float pos1[2],pos2[2];
//pos1[Xr]=sin(in->sita[1])*in->sys.l1;
//pos1[Yr]=cos(in->sita[1])*in->sys.l1;

//pos2[Xr]=sin(in->sita[0])*in->sys.l1;
//pos2[Yr]=cos(in->sita[0])*in->sys.l1;
//	
//float d=sqrt(pow(pos1[Xr]-pos2[Xr],2)+pow(pos1[Yr]-pos2[Yr],2));	
//	
//float center[2];
//center[Xr]=	(pos1[Xr]+pos2[Xr])/2;
//center[Yr]=	(pos1[Yr]+pos2[Yr])/2;	
//	
//float sita=atan2(center[Xr],center[Yr])*57.3;		
//	
//float d1=sqrt(pow(in->sys.l2,2)-pow(d/2,2));
//float d2=sqrt(pow(center[Xr],2)+pow(center[Yr],2));		
//float d3=d1+d2;	
//	
//in->pos_now[2].y=sin(sita*0.0173)*d3;
//in->pos_now[2].z=cos(sita*0.0173)*d3;	
//}


////从关节角度计算舵机PWM
void cal_pwm_from_sita(LEG_STRUCT * in)
{ u8 i=0;
	in->sys.PWM_OUT[i]=LIMIT(in->sys.PWM_OFF[i]+0*in->sys.sita_flag[i]*in->sys.PWM_PER_DEGREE[i]
	+in->sys.sita_flag[i]*(in->sita[i]-35)*in->sys.PWM_PER_DEGREE[i],in->sys.PWM_MIN[i],in->sys.PWM_MAX[i]);
	for(i=1;i<4;i++)
	in->sys.PWM_OUT[i]=LIMIT(in->sys.PWM_OFF[i]+in->sys.sita_flag[i]*in->sita[i]*in->sys.PWM_PER_DEGREE[i],in->sys.PWM_MIN[i],in->sys.PWM_MAX[i]);
}	

////计算采样点曲线三维坐标
//float curve_cal(float c0,float c3,float c4,float c5,float c6,float t)
//{
//float temp;
//temp=c0+c3*pow(t,3)+c4*pow(t,4)+c5*pow(t,5)+c6*pow(t,6);
//return temp;
//}

////计算二次曲线系数
//float c0[5][3]; //x y z
//float c3[5][3];
//float c4[5][3];
//float c5[5][3];
//float c6[5][3];
//u8 en_xie=1;
//void cal_curve_from_pos(LEG_STRUCT * in,float desire_time)
//{
//u8 id=in->sys.id;	
//float pos_now[3];
//float pos_tar[3];
//pos_now[Xs]=in->pos_now[2].x;
//pos_now[Ys]=in->pos_now[2].y;
//pos_now[Zs]=in->pos_now[2].z;
//pos_tar[Xs]=in->pos_tar_trig[2].x;
//pos_tar[Ys]=in->pos_tar_trig[2].y;
//pos_tar[Zs]=in->pos_tar_trig[2].z;
//	
//		float t1=0;
//	  t1=desire_time/2; //middle 0.5s
//    float t2=0;
//	  t2=desire_time; //end 1s
//    float p0[3];
//    float p1[3];
//    float p2[3];
////----------start
//    p0[0]=pos_now[Xs];
//   	p0[1]=pos_now[Ys];
//   	p0[2]=pos_now[Zs];
////--------------end
//	  p2[0]=pos_tar[Xs];//x
//   	p2[1]=pos_tar[Ys];//y
//   	p2[2]=pos_tar[Zs];//z
////-------------middle
//    float k,b;
//		p1[0]=(p0[0]+p2[0])/2;
//   	p1[1]=(p0[1]+p2[1])/2;
//		p1[2]=LIMIT((p0[2]+p2[2])/2-in->sys.leg_up_high,in->sys.limit_min.z,30);//wait
//		
//  float p1_p0[3];
//	float p0_p2[3];
//	int i;
//	for(i=0;i<3;i++)
//	{
//		c0[id][i]=p0[i];
//		p1_p0[i]=p1[i]-p0[i];
//		p0_p2[i]=p0[i]-p2[i];
//	}

//	float t1_3=pow(t1,3);
//	float t1_4=pow(t1,4);
//	float t1_5=pow(t1,5);
//	float t1_6=pow(t1,6);

//	double t2_2=pow(t2,2);
//	double t2_3=pow(t2,3);
//	double t2_5=pow(t2,5);
//	double t2_6=pow(t2,6);

//	float temp1=0;temp1=1/(t1_3*pow((t1-t2),3)*t2_3);
//	for(i=0;i<3;i++)
//	{	c3[id][i]=-1*temp1*(t2_6*(p1_p0[i])+5*t1_4*t2_2*3*(p0_p2[i])
//		+2*t1_6*5*(p0_p2[i])-3*t1_5*t2*8*(p0_p2[i]));

//    c4[id][i]=temp1/t2*(3*t2_6*(p1_p0[i])+15*t1_3*t2_3*(p0_p2[i])
//    		-27*t1_5*t2*(p0_p2[i])+t1_6*15*(p0_p2[i]));

//	c5[id][i]=-temp1/t2_2*3*(
//			t2_6*(p1_p0[i])
//			+2*t1_6*(p0_p2[i])
//			+t1_3*t2_3*8*(p0_p2[i])
//			-t1_4*t2_2*9*(p0_p2[i]));

//	c6[id][i]=temp1/t2_2*
//	(t2_5*(p1_p0[i])
//			+6*t1_5*(p0_p2[i])
//			+10*t1_3*t2_2*(p0_p2[i])
//			-t1_4*t2*15*(p0_p2[i]));}
//}

////读出规划曲线各采样三维位置
//void  cal_pos_tar_from_curve(LEG_STRUCT * in,float time_now,float dt)
//{
//u8 id=in->sys.id;	
//float cal_curve[3];	
//u8 i;
//for(i=0;i<3;i++)
//cal_curve[i]=curve_cal(c0[id][i],c3[id][i],c4[id][i],c5[id][i],c6[id][i],time_now);
//	
//in->pos_tar[2].x=cal_curve[Xs];
//in->pos_tar[2].y=cal_curve[Ys];
//in->pos_tar[2].z=cal_curve[Zs];	
//}	

//void leg_curve_bai(LEG_STRUCT * in,float sx,float sy,float sz,float tx,float ty,float tz,float h,float off_h,float time_now,float dt,float T)
//{
//float sita=(2*PI/(T/dt))*(time_now/dt);
//in->pos_tar[2].x=(tx-sx)*(sita-sin(sita))/(2*PI)+sx;
//in->pos_tar[2].y=(ty-sy)*(sita-sin(sita))/(2*PI)+sy;
//in->pos_tar[2].z=sz-(h+off_h)*(1-cos(sita))/2;
//}	


//float rate_delay_kuai=0.066;
//float delay_time_kuai=0.66;
//float off_time_k=0.0;
////跨腿轨迹跟踪
//void leg_follow_curve(LEG_STRUCT * in,float desire_time,u8 *en,float dt,float down_h,float down_t)
//{
//u8 id=in->sys.id;
//static u16 ground_mask[5];	
//static u8 state[5];
//static float time[5],delay[5];	
//static float temp_h;
//static float pos_str[5][3],pos_tar[5][3];
//static float off_h[5];
//float dis_tar;

//dis_tar=sqrt(pow(in->pos_tar_trig[2].x-in->pos_now[2].x,2)*0+
//						 pow(in->pos_tar_trig[2].y-in->pos_now[2].y,2)*0+
//						 pow(in->pos_tar_trig[2].z-in->pos_now[2].z,2));	
////判断是否重合等
//	
//switch(state[id])
//{
//case 0:
//if(*en){//由着地点规划当前轨迹
//state[id]=3;	
//ground_mask[id]=time[id]=delay[id]=0;	
//}
//break;
//case 1://有时间和轨迹计算每一时间的曲线坐标
//if(*en){//踩
//	in->pos_tar[2].z=in->pos_now[2].z+down_h;
//	temp_h=in->pos_now[2].z;
//	state[id]=2;
//}
//break;
//case 2:
//if(*en){//抬
//	time[id]+=dt;
//	if(time[id]>down_t)	
//	{
//	state[id]=3;
//	time[id]=0;		
//	in->pos_tar[2].z=temp_h-1.61*down_h;
//	}
//}
//break;///////////////
//case 3:
//if(*en){//抬
//in->leg_ground=0;	
//in->trig_flag=1;	
//#if !TIRG_CURVE_USE_BAI
//	cal_curve_from_pos(in,desire_time);	
//#else	
//	in->pos_tar[2].z=temp_h-1.61*down_h;		
//	pos_str[id][Xs]=in->pos_now[2].x;
//	pos_str[id][Ys]=in->pos_now[2].y;
//	pos_str[id][Zs]=in->pos_now[2].z;
//	pos_tar[id][Xs]=in->pos_tar_trig[2].x;
//	pos_tar[id][Ys]=in->pos_tar_trig[2].y;
//	pos_tar[id][Zs]=in->pos_tar_trig[2].z;
//	off_h[id]=(pos_tar[id][Zs]-pos_str[id][Zs])*1	;	
//#endif
//state[id]=4;
//}
//break;
//case 4:
//if(*en){//轨迹
//#if !TIRG_CURVE_USE_BAI
//	cal_pos_tar_from_curve(in,time[id],dt);
//#else
//	leg_curve_bai(in,pos_str[id][Xr],pos_str[id][Yr],pos_str[id][Zr],pos_tar[id][Xr],pos_tar[id][Yr],pos_tar[id][Zr]
//		,in->sys.leg_up_high,off_h[id],time[id],dt,desire_time);
//#endif
//	
//#if TWO_LEG_TEST
//	if(time[id]<desire_time/2)	
//	time[id]+=dt;
//#else	
//	time[id]+=dt;
//	if(time[id]<desire_time/3)
//		in->leg_ground=0;	
//	float ero;
//	ero=sqrt(pow(in->pos_tar_trig[2].y-in->pos_now[2].y,2)+pow(in->pos_tar_trig[2].z-in->pos_now[2].z,2));
//	if((time[id]>desire_time+off_time_k)||ero<0.025)	
//	{state[id]=5;}
//#endif

////传感器着地检测打断跨腿
////if(brain.en_ad_ground&&in->leg_ground&&time[id]>desire_time/2)
////	 state[id]=5;
//}
//break;
//case 5:
//	in->leg_ground=1;
//  in->pos_now_trig_f[2].z=in->pos_now[2].z*(1-flt1)+in->pos_tar[2].z*flt1;
//  state[id]=0;
//  if(in->rst_leg)
//		in->rst_leg=0;
//  *en=0;
//break;
//}	
//}

////蹬腿逻辑
//float k_trot_r=6;
//float k_z_att=1;
//void  cal_pos_tar_for_deng(LEG_STRUCT * in,float spdx,float spdy,float dt)
//{
//float x_temp,y_temp;	
//u8 id=in->sys.id;
//static u8 state;
//static float h[5];
//static float time;
//static float reg[5][3];	
//float spd_wx,spd_wy;
//static int deng_flag[5];
//static float deng_cnt[5];
//static float sita[5];
//	//旋转速度赋值
//  y_temp=brain.rotate_linear;
//	switch(id)
//	{
//	case 1:
//		//if(brain.rotate_linear<0)
//		spd_wy=-y_temp;
//	break;
//	case 2:
//		//if(brain.rotate_linear<0)
//		spd_wy=-y_temp;
//	break;
//	case 3:
//		//if(brain.rotate_linear>0)
//		spd_wy=y_temp;
//	break;
//	case 4:
//		//if(brain.rotate_linear>0)
//		spd_wy=y_temp;
//	break;
//	}
//  
//	float spd_use;
//	spd_use=spdy+spd_wy;
//	
//  //修正速度由姿态倾斜
//	float spd_temp[3];
//	spd_temp[Xr]=0;
//	if(brain.en_att_change)
//	{
//	 spd_temp[Yr]=spd_use*cos(-LIMIT(brain.sample_att_buf[0],-15,15)/57.3);
//	 spd_temp[Zr]=spd_use*sin(-LIMIT(brain.sample_att_buf[0],-15,15)/57.3)*k_z_att;
//		
//		if(ABS(in->pos_now[2].z-in->sys.init_end_pos.z)<3)
//			spd_temp[Zr]+=att_control_out[in->sys.id];
//	}	
//	else
//	{
//	 spd_temp[Yr]=spd_use;
//	 spd_temp[Zr]=0;
//	}	
//	
//	//跨腿时着地腿蹬腿速度赋值  和  姿态修正
//	if(in->deng_z!=0&&deng_flag[id]==0){
//		deng_flag[id]=1;
//		sita[id]=0;
//	}
//	if(in->deng_z==0)
//		 sita[id]=deng_cnt[id]=deng_flag[id]=0;

//		if(deng_flag[id]==1){
//			deng_cnt[id]+=dt;
//			sita[id]=deng_cnt[id]*(180/brain.sys.desire_time);
//			if(deng_cnt[id]>brain.sys.desire_time){
//			deng_cnt[id]=0;
//			deng_flag[id]=2;
//			}
//			
//			//fix att change
//			float temp=cos(sita[id]/57.3)*in->deng_z;
//			float temp_z=temp*cos(LIMIT(brain.sample_att_buf[0],-15,15)/57.3);
//			float temp_y=temp*sin(LIMIT(brain.sample_att_buf[0],-15,15)/57.3);
//			if(!brain.en_att_change)
//			{temp_z=temp;temp_y=0;}
//			
//			spd_temp[Zr]-=temp_z;
//			spd_temp[Yr]-=temp_y;
//		}
//	if(in->leg_ground==0&&deng_flag[id]==2)
//		 sita[id]=deng_cnt[id]=deng_flag[id]=0;
//		
//	
//	//速度限制

//	
//	//速度输出
//	if(!in->err&&in->leg_ground){
//		
//	if(in->sys.leg_set_invert){	
//		in->pos_tar[2].x+=-(spd_temp[Xr])*dt;
//		in->pos_tar[2].y+=-(spd_temp[Yr])*dt;
//	}else{
//		in->pos_tar[2].x+=-(spd_temp[Xr])*dt;
//		in->pos_tar[2].y+=-(spd_temp[Yr])*dt;
//	}
//	in->pos_tar[2].z+=-(spd_temp[Zr])*dt; 
//	//protect	
//	if(isnan(in->pos_tar[2].x))
//		in->pos_tar[2].x=reg[id][0];
//	if(isnan(in->pos_tar[2].y))
//		in->pos_tar[2].y=reg[id][1];
//	if(isnan(in->pos_tar[2].z))
//		in->pos_tar[2].z=reg[id][2];

//	if(!isnan(in->pos_tar[2].x))
//		reg[id][0]=in->pos_tar[2].x;
//	if(!isnan(in->pos_tar[2].y))
//		reg[id][1]=in->pos_tar[2].y;
//	if(!isnan(in->pos_tar[2].z))
//		reg[id][2]=in->pos_tar[2].z; 
//  if(in->pos_now[2].z==0)
//	{
//		in->pos_now[2].x=in->sys.init_end_pos.x;
//		in->pos_now[2].y=in->sys.init_end_pos.y;
//		in->pos_now[2].z=in->sys.init_end_pos.z;
//	} 
//	float limit_mask=1;
//	in->pos_tar[2].x=LIMIT(in->pos_tar[2].x,-in->sys.limit.x*limit_mask,in->sys.limit.x*limit_mask);
//	in->pos_tar[2].y=LIMIT(in->pos_tar[2].y,-in->sys.limit.y*limit_mask,in->sys.limit.y*limit_mask);	 
//	limit_range_leg(in->pos_tar[2].x,in->pos_tar[2].y,in->sys.limit.x*limit_mask,in->sys.limit.y*limit_mask,&in->pos_tar[2].x,&in->pos_tar[2].y);
//  in->pos_tar[2].z=LIMIT(in->pos_tar[2].z,in->sys.limit_min.z,in->sys.limit.z);	
//	}
//}	

////着地检测
//void  leg_ground_check(LEG_STRUCT * in)
//{
//u8 id=in->sys.id;
//static u8 state;
//static float time;	
//   if(in->sys.leg_ground_force==2)
//   in->leg_ground=1;
//	 else if(in->sys.leg_ground_force==1)
//	 in->leg_ground=0;	 
//	 
// 	 if(brain.en_ad_ground){
//		 if(press_leg_end[in->sys.id]>0.3&&in->leg_ground==0)
//			  in->leg_ground=1;	 
//	 }
//	 
//}	


////主动着地
//float k_ground=1.23;
//void leg_to_ground(LEG_STRUCT * in,float dt)
//{u8 i;
//  switch(in->trig_flag)
//	{
//	  case 1:
//			 if(press_leg_end[i]==0)
//		       in->trig_flag=2;
//		break;
//	  case 2:
//			  if(in->pos_now[2].z<in->sys.init_end_pos.z*1.1)
//					in->pos_tar[2].z+=k_ground*dt; 
//		     
//		   if(press_leg_end[i]==1)
//				 in->trig_flag=0;
//	  break;
//	}
//}	

//float sita_test[4]={90,0,0};
//u8 line_test[4];
//u8 force_test_mode;
//void leg_publish(LEG_STRUCT * in)
//{
//	
////跨腿逻辑	
//u8 id=in->sys.id;
//float x_temp,y_temp,z_temp;
//static u16 cnt[5];	
//	if(in->sys.id==1||in->sys.id==3){
//	in->sys.off_all.x=brain.sys.center_off.x+center_control_out[Xr]*0;
//	in->sys.off_all.y=brain.sys.center_off.y+center_control_out[Yr]*0;
//	in->sys.off_all.z=brain.sys.center_off.z;
//	}
//	else
//	{
//	in->sys.off_all.x=brain.sys.center_off1.x+center_control_out[Xr]*0;
//	in->sys.off_all.y=brain.sys.center_off1.y+center_control_out[Yr]*0;
//	in->sys.off_all.z=brain.sys.center_off1.z;
//	}
//	in->sys.leg_up_high=brain.sys.leg_h[id];
//	in->sys.desire_time=brain.sys.desire_time;
//	if(brain.power_all)
//		in->leg_power=1;
//	if(brain.control_mode)
//		in->control_mode=1;
//	
//	static u8 rst_all;
//	static u8 rst_flag=1;
//	switch(rst_all){
//		case 0:
//			if(brain.rst_all){
//			brain.spd=0;
//			leg[1].rst_leg=1;
//		  rst_all=1;
//			}
//		break;
//		case 1:
//			if(leg[rst_flag].rst_leg==0)
//			{rst_flag++;leg[rst_flag].rst_leg=1;}
//		if(rst_flag==5)
//		{rst_flag=1;brain.rst_all=0;rst_all=0;}
//		break;	
//	}

//  if(in->control_mode||force_test_mode)
//	{
//	x_temp=in->sys.init_end_pos.x;
//	y_temp=in->pos_tar[2].y;
//	z_temp=in->pos_tar[2].z;
//	//由坐标计算角度
//	cal_sita_form_pos_tri_leg(in->sys.l1,in->sys.l2,x_temp,y_temp,z_temp,&in->sita[0],&in->sita[1],&in->sita[2]);
//		
//	 if(line_test[3]){//强制角度测试
//	 in->sita[0]=sita_test[0];in->sita[1]=sita_test[1];in->sita[2]=sita_test[2];in->sita[3]=sita_test[3];}
//	 else if(brain.sys.control_angle)//强制角度控制
//	 { in->sita[0]=in->sita_force[0];
//		 in->sita[1]=in->sita_force[1];
//		 in->sita[2]=in->sita_force[2];
//	   in->sita[3]=in->sita_force[3];}
//  cal_pwm_from_sita(in);//计算PWM由角度	
//	//从角度反推位置	
//	//cal_pos_from_sita(in);
//	in->pos_now[2].x=in->sys.init_end_pos.x;
//	in->pos_now[2].y=y_temp;
//	in->pos_now[2].z=z_temp;
//		 
//	}
//}
//float k_add_test22=0.01;
//float test22[9]={8,15,0,0,13,0,0,0};
//void leg_drive(LEG_STRUCT * in,float dt)
//{  
// u8 id=in->sys.id; 	
// static u8 init[5];	
//	if(!init[id])
//	{
//	init[id]=1;	
//	in->sys.pos_tar_reg[0]=in->pos_tar_trig[2].x;
//	in->sys.pos_tar_reg[1]=in->pos_tar_trig[2].y;
//	in->sys.pos_tar_reg[2]=in->pos_tar_trig[2].z;	
//	} 
//	  if(line_test[0]){
//			static u8 flag;
//			if(flag)
//			test22[4]+=k_add_test22;
//			else
//			test22[4]-=k_add_test22;	
//			
//			if(test22[4]>(test22[1]+test22[0])*0.7)
//				flag=0;
//			else if(test22[4]<test22[0])
//				flag=1;
//		}
//		if(line_test[1]){
//			static u8 flag;
//			if(flag)
//			test22[3]+=k_add_test22;
//			else
//			test22[3]-=k_add_test22;	
//			
//			if(test22[3]>(test22[1])*0.7)
//				flag=0;
//			else if(test22[3]<-test22[0]*0.7)
//				flag=1;
//		}
//		
//  
//	  if(in->rst_leg)
//		{
//		in->pos_tar_trig[2].x=in->sys.init_end_pos.x;
//		in->pos_tar_trig[2].y=in->sys.init_end_pos.y;
//		in->pos_tar_trig[2].z=in->sys.init_end_pos.z+(float)RNG_Get_RandomRange(-1000,1000)/100000.;
//		}
//	  in->sita_ss=fast_atan2(in->pos_now[2].y,in->pos_now[2].z)*57.3;
//		if((in->pos_tar_trig[2].x!=in->sys.pos_tar_reg[0]||
//			 in->pos_tar_trig[2].y!=in->sys.pos_tar_reg[1]||
//			 in->pos_tar_trig[2].z!=in->sys.pos_tar_reg[2])&&!in->curve_trig)	
//			{in->curve_trig=1;}//trig 使能
//    //跨
//		if(in->curve_trig&&in->sys.desire_time>0)
//		leg_follow_curve(in,in->sys.desire_time,& in->curve_trig,dt,brain.sys.down_h,brain.sys.down_t);
//		//着地
//		leg_ground_check(in);
//		//蹬
//		if(!in->curve_trig&&1)
//		cal_pos_tar_for_deng(in,in->deng[0],in->deng[1],dt);	
//		
//		if(brain.en_ad_ground)
//		leg_to_ground(in,dt);
//		//输出		
//	  leg_publish(in);
//		
//		in->sys.pos_tar_reg[0]=in->pos_tar_trig[2].x;
//		in->sys.pos_tar_reg[1]=in->pos_tar_trig[2].y;
//		in->sys.pos_tar_reg[2]=in->pos_tar_trig[2].z;
//		//
//}
////--------------------------------------------------VMC control-----------------------------














