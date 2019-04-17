#include "include.h" 
#include "usart.h" 
#include "filter.h"
u8 trig_list_trot[5]={0,14,23,14,23};
//------------------------------------------------------------brain---------------------------------------------------------------------
//       
//<-----W-------->
//			  y
//	3----------1          /\
// 	     |                |
//			 O                L
//			 |                |
//	4----------2   x			\/
u8 stop_leg=4;
#define MAX_ATT 20
float set_max_tro=2.618;//2
float min_spd_tro=0.0321;
static u8 stop_leg_tro;
float set_trig_value_tro=0.9;
float trot_delay=0.6;//
u8 per_stop=10;

float k_size_trot=0.0068;
float size_k_trot;
float limit_deng_tro=8.8;//12;
float k_acc_control_trot[2]={-0.036,-0.036};
float k_spd_control_trot[2]={0.25,0.15};
float min_off_for_stable=0.0;


float kp_att=0.08;
float ki_att=0.000;
float kd_att=0;
float flt=0.223;

void trot_gait(float dt);
void barin_init(BRAIN_STRUCT *in)
{
float W=10.5  *2;//cm	
float L=25.0+7;
in->sys.yaw_trig=fast_atan2(W/2,L/2)*57.3;	
	
in->sys.leg_local[1].x=W/2;
in->sys.leg_local[1].y=L/2;
in->sys.leg_local[1].z=0;	
	
in->sys.leg_local[2].x=W/2;
in->sys.leg_local[2].y=-L/2;
in->sys.leg_local[2].z=0;	
	
in->sys.leg_local[3].x=-W/2;
in->sys.leg_local[3].y=L/2;
in->sys.leg_local[3].z=0;	
	
in->sys.leg_local[4].x=-W/2;
in->sys.leg_local[4].y=-L/2;
in->sys.leg_local[4].z=0;	

in->global.area_of_leg[1]=in->area_of_leg[1]=W*L;
	
in->sys.min_range=0.707*leg[1].sys.l3*0.3;//7.8 4.742556
in->sys.max_range=0.707*leg[1].sys.l2*0.6;//6.3 3.830526
in->sys.leg_move_range[Yr]=MAX(in->sys.max_range, in->sys.min_range);//cm		
in->sys.leg_move_range[Xr]=W/4*0.88;//cm	
//for leg 1
#if MINI_ROBOT
in->sys.leg_move_range1[1]=0.707*leg[1].sys.l2;
in->sys.leg_move_range1[2]=sin(22/57.3)*(leg[1].sys.l2+leg[1].sys.l3);
in->sys.leg_move_range1[3]=0.707*leg[1].sys.l3*0.3;
in->sys.leg_move_range1[4]=MIN(sin(35/57.3)*(leg[1].sys.l2+leg[1].sys.l3)*0.618,W/2*0.268);
#else
in->sys.leg_move_range1[1]=0.707*leg[1].sys.l2;
in->sys.leg_move_range1[2]=sin(22/57.3)*(leg[1].sys.l2+leg[1].sys.l3);
in->sys.leg_move_range1[3]=0.707*leg[1].sys.l3*0.3/1.618;
in->sys.leg_move_range1[4]=MIN(sin(35/57.3)*(leg[1].sys.l2+leg[1].sys.l3)*0.618,W/2*0.268)/1.618;
#endif
in->sys.kp_center[0]=0.8;
in->sys.kp_center[1]=0.6;
in->sys.k_center_fp=0.0;

in->sys.move_range_k=0.66;

in->min_st[0]=2;//2.68;//cm
in->min_st[1]=in->min_st[0];
in->min_st[2]=in->min_st[0];
in->sys.k_center_c[0]=100;
in->sys.k_center_c[1]=100;

in->sys.att_off[0]=4;
in->sys.att_off[1]=4;

in->sys.center_off.x=0;
in->sys.center_off.y=0;
in->sys.center_off1.x=0;
in->sys.center_off1.y=0;

in->sys.center_off_when_move[Xr]=0;//-0.2;
in->sys.center_off_when_move[Yr]=0;//-0.88;
in->sys.leg_t=0.5;

in->sys.leg_h[1]=in->sys.leg_h[2]=in->sys.leg_h[3]=in->sys.leg_h[4]=6.66;


#if USE_LEG_TRIG_DELAY // BEST TIM 0.4
in->sys.desire_time=0.333;//0.7;//0.76;
#else
in->sys.desire_time=0.333;//0.7;//0.76;
#endif
#if MINI_ROBOT
in->sys.desire_time=0.2;
#endif
in->sys.desire_time_init=in->sys.desire_time;
in->sys.leg_move_min_dt=in->sys.desire_time*0.68;//35 ;

in->sys.k_spd_to_range=2;//1.618;

in->sys.yaw_dead=10;

in->sys.down_h=0;//0.88;
in->sys.down_t=0;//0.2;
in->sys.in_rst_check=2;

leg[1].sys.id=1;
leg[2].sys.id=2;
leg[3].sys.id=3;
leg[4].sys.id=4;

brain.tar_h=leg[1].sys.init_end_pos.z;
#if TEST_MODE1
in->sys.k_center_c[0]=4;
in->sys.k_center_c[1]=4;
in->sys.desire_time=3.3;//0.76;
in->sys.leg_move_min_dt=0;
in->sys.down_t=in->sys.down_h=0;
#endif
}	


void leg_task1(float dt)//$%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
{
static u16 cnt_init;	
static u8 init;
static u8 cnt_stop;	
static u16 cnt[10];	
static u8 leg_trig_flag_reg;
u8 i;
if(!init&&cnt_init++>3/0.02){init=1;
barin_init(&brain);}

	if(init){
				if(cnt_stop++>per_stop&&brain.leg_move_state>0)
				cnt_stop=0;
			 
				for(i=1;i<5;i++) 	  
				conver_legpos_to_barin(&brain,&leg[i],i);      
				estimate_center(&brain,brain.att,brain.now_spd,brain.now_acc,brain.sys.tar_spd,brain.tar_w);//1‡???˙Ï???D?
			  state_clear();	
			  cal_pos_global(dt);
				att_control(dt);
			  cal_center_of_leg_ground(&brain);//1‡??◊?µ??‡±?D?
  
			#if defined(TRIG_TEST)
			#else
				center_control_global_tro_ss(dt);//?˙◊?µ???ÛÚ?˙Ú??Ø??D?
      
				trot_gait_ss(dt);				
			#endif
		brain.power_all=1;
	}//end_init
}
//------------------------------------------------START HERE-----------------------------------------------------------------

///////////////////////////////////////////////////OLD LOCAL///////////////////////////////////////////////


float k_acc=0;
float flt_att=0.05;
void estimate_center(BRAIN_STRUCT *in,float att[3],float spd_body[3],float acc_body[3],float spd_tar[3],float w_tar)//1‡???˙Ï???D?
{ u8 i,j;
	float high_robot,temp,k_off_z;
	for(i=1;i<5;i++)
		if(leg[i].leg_ground)
		{temp+=leg[i].pos_now[2].z;j++;}
		if(j>0){
		high_robot=temp/j;
		k_off_z=LIMIT(high_robot/15.5*2.68,0.2,2.2);
		}else
		k_off_z=1;

		in->center.x=0+brain.now_acc[0]*k_acc;
		in->center.y=0+brain.now_acc[1]*k_acc;
	  in->center.z=high_robot;
		
		brain.att_flt[0]=LIMIT(brain.att[0],-MAX_ATT*0.66,MAX_ATT)*flt_att+(1-flt_att)*brain.att_flt[0];
		brain.att_flt[1]=LIMIT(brain.att[1],-MAX_ATT*0.66,MAX_ATT)*flt_att+(1-flt_att)*brain.att_flt[1];
}


void cal_center_of_leg_ground(BRAIN_STRUCT *in)
{
u8 i=0,j=0;	
double x[5]={-5,0,4,0},y[5]={0,5,0,0};
float temp[2][3];
in->ground_leg_num=0;
  for(i=1;i<5;i++){
		if(leg[i].leg_ground){
		x[j]=leg[i].pos_now_brain[2].x;
		y[j]=leg[i].pos_now_brain[2].y;
		j++;
		}
  }
in->ground_leg_num=j;
  
switch(in->ground_leg_num) 
 {	 
 case 3:
   in->leg_ground_center[Xr]=(float)(x[0]+x[1]+x[2])/3.;
   in->leg_ground_center[Yr]=(float)(y[0]+y[1]+y[2])/3.;
   in->area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2]);
   in->steady_value=cal_steady_s(in->leg_ground_center[Xr],in->leg_ground_center[Yr],x[0],y[0], x[1],y[1], x[2],y[2]);
 break;
 case 4:
    temp[0][Xr]=(float)(x[0]+x[1]+x[2])/3.;
    temp[0][Yr]=(float)(y[0]+y[1]+y[2])/3.;
    temp[1][Xr]=(float)(x[1]+x[2]+x[3])/3.;
    temp[1][Yr]=(float)(y[1]+y[2]+y[3])/3.;
    in->leg_ground_center[Xr]=(float)(temp[0][Xr]+temp[1][Xr])/2.;
    in->leg_ground_center[Yr]=(float)(temp[0][Yr]+temp[1][Yr])/2.;
    in->area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2])/2+cal_area_trig( x[3],y[3], x[1],y[1], x[2],y[2])/2;
 break;
 default:
	  in->area_of_leg[0]=0;
 break;
 }	 
}

float att_control_out[5];
void att_control(float dt)
{ static float int_ero[2];
  static float ero[2],ero_r[2];
  float control[2],ero_d[2];	
  ero[0]=0+brain.att_flt[1];
	
	if(ki_att==0)
	int_ero[1]=int_ero[0]=0;	
	else{
	int_ero[0]+= ero[0]*ki_att;
	}
	int_ero[0]=LIMIT(int_ero[0],-3,3);
	
	ero_d[0] = 0.02f/dt *kd_att * (ero[0]-ero_r[0]) *dt;
	
	control[0]=LIMIT(ero[0]*kp_att+int_ero[0]+ero_d[0],-1,1);
	ero_r[0]=ero[0];
	
	att_control_out[1] = LIMIT(-control[0],-6,6) ;
	att_control_out[2] = LIMIT(-control[0],-6,6) ;
	att_control_out[3] = LIMIT(control[0] ,-6,6) ;
	att_control_out[4] = LIMIT(control[0] ,-6,6) ;
	att_control_out[0] =  fabs(control[1])+fabs(control[0]);
}	

//--------------------------------------------Trot -----------------Gait 

//◊„º‚œﬂÀŸ∂»º∆À„
void center_control_global_tro_ss(float dt){
	u8 i,leg_ground[5];
	float center_control_out[2];
	u8 brain_ground_leg_num=brain.ground_leg_num;
	for(i=0;i<5;i++)
	leg_ground[i]=leg[i].leg_ground;
	
	float spd_use=brain.spd;
  float spd_stable[2]={0};
	spd_stable[Xr]=sin(brain.spd_yaw/57.3)*spd_use;
  spd_stable[Yr]=cos(brain.spd_yaw/57.3)*spd_use; 
	
	float tar_w_deng[5];
	tar_w_deng[1]=spd_stable[Xr];
	tar_w_deng[2]=spd_stable[Xr];
	tar_w_deng[3]=-spd_stable[Xr];
	tar_w_deng[4]=-spd_stable[Xr];
	
	center_control_out[Xr]=spd_stable[Xr];
	center_control_out[Yr]=spd_stable[Yr];
	for(i=1;i<5;i++){
	if((leg[i].control_mode||brain.control_mode)&&leg[i].leg_ground&&!leg[i].sys.leg_ground_force){	
	leg[i].deng[Xr]=LIMIT(center_control_out[Xr]*gain_spd_ss[0],-limit_deng_tro_ss,limit_deng_tro_ss);
	leg[i].deng[Yr]=LIMIT((center_control_out[Yr]+tar_w_deng[i])*gain_spd_ss[0],-limit_deng_tro_ss,limit_deng_tro_ss);	}
	}	
}

//¬‰◊„µ„πÊªÆ
void leg_tar_est_global_tro_ss(u8 id,float sita,float dt)
{
//PREDICT
float tempy=leg[id].pos_now[2].z*tan(sita/57.3);
float tempy_max=leg[id].pos_now[2].z*tan(leg[id].sita_ss_limit/57.3);	
float tempz=brain.tar_h;
//LIMIT
tempy=LIMIT(tempy,-tempy_max,tempy_max);
tempz=LIMIT(tempz,leg[id].sys.limit_min.z,leg[id].sys.limit.z);	
//OUT
leg[id].pos_tar_trig[2].x=0;
leg[id].pos_tar_trig[2].y=tempy;
leg[id].pos_tar_trig[2].z=tempz+RANDOM;
//RST
if(brain.rst_all_soft>0)
{
leg[id].pos_tar_trig[2].x=leg[id].sys.init_end_pos.x+RANDOM;//+off_x*cos(tar_yaw/ 57.3f)*en_off_trig;
leg[id].pos_tar_trig[2].y=leg[id].sys.init_end_pos.y+RANDOM;//+off_y*sin(tar_yaw/ 57.3f)*en_off_trig;
leg[id].pos_tar_trig[2].z=brain.tar_h;
}

}

float gain_spd_ss[2]={10,5};//8;//ÀŸ∂»‘ˆ“Ê
float limit_deng_tro_ss=16;//ÀŸ∂»œﬁ÷∆
float deng_z[2]={6,0};//◊≈µÿÕ»ƒ¨»œµ≈Õ»ÀŸ∂»mm/s
float pid_deng[3]={0.25,0.001,0};

float trot_delay_ss=0.1;//0.168;
float sita_off[2]={6,-6};//«∞Ω¯»˝Ω«–Œ∆´÷¥
float sita_off1[2]={-16,16};//∫ÛÕÀ»˝Ω«–Œ∆´÷¥
//float sita_off[2]={6,-18};
//float sita_off1[2]={18,-6};
float sita_off2[2]={-4,-4};//‘≠µÿ»˝Ω«–Œ∆´÷¥

float sita_dead=0.168;
float rotate_linear;
float control_att_deng;
//∂‘Ω«≤ΩÃ¨£®÷‹∆⁄øÿ÷∆ √ø∏ˆ≤ΩÃ¨÷‹∆⁄≤…”√◊ÀÃ¨Ω«£©
void trot_gait_ss(float dt){
	u8 i,j,k,l;
	float sita_ss_max[5]={0};
	float sita_ss_off[5]={0};
	float ero_ss[5];
	float spd_stable[2];
  float deng_z_use;
	float brain_rotate_linear;
	if(brain.spd_yaw==0)
		deng_z_use=deng_z[0]*(LIMIT(ABS(brain.spd)*1.5,0,MAX_FSPD)/MAX_FSPD);
	else
		deng_z_use=deng_z[1];
	
	brain.rotate_linear=LIMIT((brain.tar_w)*brain.sys.leg_local[1].x/100.,-0.6,0.6)*gain_spd_ss[1];
	brain_rotate_linear=LIMIT((brain.tar_w)*brain.sys.leg_local[1].x/100.,-0.6,0.6)*gain_spd_ss[1];
	rotate_linear=ABS(brain.rotate_linear);
	float max_range_spd[2];
	max_range_spd[0]=LIMIT(brain.sys.desire_time*(ABS(brain.spd)-brain_rotate_linear*1),-0.8,0.8);//left
	max_range_spd[1]=LIMIT(brain.sys.desire_time*(ABS(brain.spd)+brain_rotate_linear*1),-0.8,0.8);//right
	spd_stable[Xr]=sin(brain.spd_yaw/57.3)*brain.spd;
  spd_stable[Yr]=cos(brain.spd_yaw/57.3)*brain.spd; 
	
	//”…ÀŸ∂»º∆À„»˝Ω«–ŒΩ«∂»
	sita_ss_max[1]=ABS(fast_atan2(max_range_spd[1],leg[1].pos_now[2].z)*57.3);
	sita_ss_max[2]=ABS(fast_atan2(max_range_spd[1],leg[2].pos_now[2].z)*57.3);
	
	sita_ss_max[3]=ABS(fast_atan2(max_range_spd[0],leg[3].pos_now[2].z)*57.3);
	sita_ss_max[4]=ABS(fast_atan2(max_range_spd[0],leg[4].pos_now[2].z)*57.3);

	
	//”…“∆∂ØÀŸ∂»ÃÌº”»˝Ω«–Œ∆´÷¥
	if(spd_stable[Yr]>0.168)
	{
	  sita_ss_off[1]=sita_ss_off[3]=sita_off[0];
	  sita_ss_off[2]=sita_ss_off[4]=-sita_off[1];
	}
  else if(spd_stable[Yr]<-0.168)
	{
	  sita_ss_off[1]=sita_ss_off[3]=sita_off1[0];
	  sita_ss_off[2]=sita_ss_off[4]=-sita_off1[1];
	} 
	else 
	{
	  sita_ss_off[1]=sita_ss_off[3]=sita_off2[0];
	  sita_ss_off[2]=sita_ss_off[4]=-sita_off2[1];
	}
	
	//º∆À„◊ÀÃ¨Ω«±‰ªØ‘Ï≥…µƒ»˝Ω«–Œ∆´÷¥
	static float att_buf[2];
	for(i=1;i<5;i++){
	if(brain.en_att_change)	
	sita_ss_off[i]+=LIMIT(my_deathzoom_21(brain.tar_att[0]-brain.sample_att_buf[0],5),-MAX_ATT,MAX_ATT);
	sita_ss_off[i]=LIMIT(sita_ss_off[i],-leg[i].sita_ss_limit*0.86,leg[i].sita_ss_limit*0.86);
	}
	
	//◊ÀÃ¨ŒÛ≤Ó‘Ï≥…µƒµ≈Õ»ÀŸ∂»£®◊≈µÿÕ»£©
	float err;
	static float erri;
	static float control_att_deng_buf;
	static int trig_interge=0;
	err=brain.tar_att[0]-(brain.sample_att_buf[0]+sita_ss_off[0]);
	if(brain.ground_leg_num<4&&trig_interge==0)
		{erri+=err*pid_deng[1];trig_interge=1;}
	if(pid_deng[1]==0)erri=0;
	erri=LIMIT(erri,-deng_z_use/2,deng_z_use/2);
	
	control_att_deng=LIMIT(err*pid_deng[0],-deng_z_use,deng_z_use)+erri;
	
	
	//º∆À„»˝Ω«–Œ±ﬂΩÁÃıº˛£®‘ˆº”À¿«¯£© 
	float sita_max14,sita_max23;
	if(sita_ss_max[1]>sita_ss_max[4])
		sita_max14=sita_ss_max[1]+sita_dead;
	else
		sita_max14=sita_ss_max[4]+sita_dead;
	
	if(sita_ss_max[2]>sita_ss_max[3])
		sita_max23=sita_ss_max[2]+sita_dead;
	else
		sita_max23=sita_ss_max[3]+sita_dead;
	
	
	//‘ΩΩÁ≈–∂œ
	u8 out_flag14=0,out_flag23=0;
	if((leg[2].sita_ss<-sita_max23+ABS(sita_ss_off[2])&&leg[2].deng[1]>0)
		||(leg[2].sita_ss>sita_max23-ABS(sita_ss_off[2])&&leg[2].deng[1]<0))
		out_flag23=1;
	if((leg[3].sita_ss<-sita_max23+ABS(sita_ss_off[3])&&leg[3].deng[1]>0)
		||(leg[3].sita_ss>sita_max23-ABS(sita_ss_off[3])&&leg[3].deng[1]<0))
		out_flag23=1;
	if((leg[1].sita_ss<-sita_max14+ABS(sita_ss_off[1])&&leg[1].deng[1]>0)
		||(leg[1].sita_ss>sita_max14-ABS(sita_ss_off[1])&&leg[1].deng[1]<0))
		out_flag14=1;
	if((leg[4].sita_ss<-sita_max14+ABS(sita_ss_off[4])&leg[4].deng[1]>0)
		||(leg[4].sita_ss>sita_max14-ABS(sita_ss_off[4])&&leg[4].deng[1]<0))
		out_flag14=1;
	
	
	//∂‘Ω«≤ΩÃ¨◊¥Ã¨ª˙
	static u8 state;
	static u16 cnt[10]={0};
	static u8 switch_flag=0;
	switch(state){
	case 0://…œµÁ≥ı ºªØ	
	   state=2;
	break;
	case 1://∞⁄∂ØÕ»±Í÷æŒª«–ªª
		if(out_flag14&&switch_flag==0)
		{
		 cnt[0]=0;
		 state=14;
		}
		else if(out_flag23&&switch_flag==1)
		{
		 cnt[0]=0;
		 state=23;
		}
	break;
	case 2://≥ı ºªØ
		if(cnt[0]++>(brain.sys.desire_time*2+trot_delay_ss)/dt){
			cnt[0]=0;
		  state=1;
			trig_interge=leg[1].deng_z=leg[2].deng_z=leg[3].deng_z=leg[4].deng_z=0;
			control_att_deng_buf=control_att_deng;
			brain.sample_att_buf[0]=brain.att_flt[0];brain.sample_att_buf[1]=brain.att_flt[1];
			if(switch_flag==1)switch_flag=0;else switch_flag=1;
		}
  break;
	case 14:
		leg_tar_est_global_tro_ss(1,sita_max14+sita_ss_off[1],dt);
	  leg_tar_est_global_tro_ss(4,sita_max14+sita_ss_off[4],dt);
	  state=2;
	  leg[2].deng_z=deng_z_use+control_att_deng_buf;
	  leg[3].deng_z=deng_z_use-control_att_deng_buf;
	  leg[1].deng_z=leg[4].deng_z=0;
	break;
	case 23:
		leg_tar_est_global_tro_ss(2,sita_max23+sita_ss_off[2],dt);
	  leg_tar_est_global_tro_ss(3,sita_max23+sita_ss_off[3],dt);
	  state=2;
	  leg[1].deng_z=deng_z_use+control_att_deng_buf;
	  leg[4].deng_z=deng_z_use-control_att_deng_buf;
	  leg[2].deng_z=leg[3].deng_z=0;
	break;
	}

	//∏¥ŒªºÏ≤È
	static u16 reset_leg,cnt_reset_trig;
	if(brain.spd!=0||brain.tar_w!=0){
	  if(brain.rst_all_soft>0)
			brain.rst_all_soft=reset_leg=cnt_reset_trig=0;
	 }
  float dis[2];
	dis[0]=ABS(leg[1].sita_ss); 
	dis[1]=ABS(leg[2].sita_ss); 
	if(brain.rst_all_soft>0&&dis[0]<5&&dis[1]<5) 
	 brain.rst_all_soft=reset_leg=cnt_reset_trig=0;
//	else
//	 brain.rst_all_soft=1;
	
		
	//∏¥Œª≤ΩÃ¨◊¥Ã¨ª˙
	switch(reset_leg){
		case 0:
			 if(brain.rst_all_soft)
				 cnt_reset_trig++;
			 if(cnt_reset_trig>brain.sys.desire_time/dt*2){
				 reset_leg=1;
				 cnt_reset_trig=0;
			 }
		break;
	case 1:
	  if(dis[0]>dis[1])
			 reset_leg=14;
		else
			 reset_leg=23;
	break;
   case 14:
		leg_tar_est_global_tro_ss(1,sita_max14+sita_ss_off[1],dt);
	  leg_tar_est_global_tro_ss(4,sita_max14+sita_ss_off[4],dt);
	  leg[2].deng_z=deng_z_use+control_att_deng_buf;
	  leg[3].deng_z=deng_z_use-control_att_deng_buf;
	  leg[1].deng_z=leg[4].deng_z=0;
	  reset_leg=33;
	 break;
	 case 23:
		leg_tar_est_global_tro_ss(2,sita_max23+sita_ss_off[2],dt);
	  leg_tar_est_global_tro_ss(3,sita_max23+sita_ss_off[3],dt);
	  reset_leg=33;
	  leg[1].deng_z=deng_z_use+control_att_deng_buf;
	  leg[4].deng_z=deng_z_use-control_att_deng_buf;
	  leg[2].deng_z=leg[3].deng_z=0;
	 break;
	 case 33://∏¥Œª«–ªª
			 if(cnt_reset_trig++>brain.sys.desire_time/dt){
				 reset_leg=cnt_reset_trig=0;
				 trig_interge=leg[1].deng_z=leg[2].deng_z=leg[3].deng_z=leg[4].deng_z=0;
				 control_att_deng_buf=control_att_deng;
				 brain.sample_att_buf[0]=brain.att_flt[0];brain.sample_att_buf[1]=brain.att_flt[1];
			 }
				 
	 break;
 }
}	