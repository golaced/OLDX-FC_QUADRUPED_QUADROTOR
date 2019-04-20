#include "include.h"
//u8 trig_list_f[5]=		 {0,1,4,3,2};
//u8 trig_list_r[5]=		 {0,4,2,3,1};
//u8 trig_list_b[5]=		 {0,4,1,3,2};
//u8 trig_list_l[5]=		 {0,2,4,1,3};
u8 trig_list_f[5]=		 {0,1,4,2,3};
u8 trig_list_l[5]=		 {0,3,2,4,1};
u8 trig_list_b[5]=		 {0,4,1,3,2};
u8 trig_list_r[5]=		 {0,1,4,2,3};

u8 trig_list_tr[5]=		 {0,2,4,3,1};
u8 trig_list_tl[5]=		 {0,4,2,1,3};

u8 side_leg[5]    =		 {0,2,4,3,1};
u8 side_leg_f[5]    =		 {0,2,1,4,3};
//--------------------------------------------parameter----------------------------------------------
u8 area_protect=0;//ÇøÓòÐ¡¿çÍÈ±£»¤
u8 repeat_protect=1;//ÖØ¸´¿çÍÈ±£»¤
#if TIRG_USE_LITTLE_DOG
u8 out_protect=0;//³¬³ö·¶Î§ÓÅÏÈ¿çÍÈ
#else
u8 out_protect=1;//³¬³ö·¶Î§ÓÅÏÈ¿çÍÈ
#endif
u8 leg_off_ground_test=1;
u8 en_leg_trig_list=1;
float k_size=0.0068;
float k_force=3.333;//³¬ÏÞÇøÓò´óÐ¡ÔöÒæ
float force_rate=2;//³¬ÏÞÈ¨Öµ¼õÐ¡²½³¤
float size_k;
float limit_deng=16;//17.8;//µÅÍÈËÙ¶È¿ØÖÆÏÞÖÆ
float k_acc_control[2]={0,0};//{-0.1,-0.07};
float k_spd_control[2]={0.25,0.15};
float force_stop_range[2]={4.8,6};
int flag_acc[2]={-1,-1};
float k_acc1=1;

float set_max=5.7*2;//³¬ÏÞÈ¨Öµ¼ÆËã×î´ó·¶Î§
float min_spd=0.023;//³¬ÏÞÈ¨Öµ×îÐ¡Öµ
static u8 stop_leg;
float set_trig_value=0.9;
float center_stable_dead=1;//ÖÐÐÄÎÈ¶¨ÅÐ¶ÏËÀÇø

u8 en_off_trig=0;//¿çÍÈÌí¼Ó µ±Ç°Æ«²î
u8 trig_use_now_pos=1;//¿çÍÈÆðµãÎªµ±Ç°µã

float h_k2=0.98;//¿ç½Å¸ß¶ÈÔöÒæ
float k_off=0.2;
float k_trig1=2;//2;
float k_rad=1.68;//¿ç½ÅÐý×ªÔöÒæ
float k_trig=2;
float k_spd=5;
float min_steady_value_stable=0.8;
float max_dis_cog=5.7;
//---------------------------------------------------------------------------------------
void state_clear(void)
{
   u8 i;
   brain.force_stop=0;
	
		for(i=1;i<5;i++)
	{
		if(leg[i].leg_ground){
      leg[i].need_move=0;	
      brain.leg_out_range[i]=0;			
		}
	}
	
//	 if(brain.global.area_of_leg[0]<brain.global.area_of_leg[1]*0.45)
//		 brain.rst_all_soft=1;
	 if(check_leg_near_init(brain.sys.in_rst_check))
		 brain.rst_all_soft=0;
}	

void cal_pos_global(float dt)
{ u8 i=0,j=0;	
	float z_zero=brain.tar_h;
	float x[5]={-5,0,4,0},y[5]={0,5,0,0};
	float temp[2][3];
	float off_cor[2];
	u8 ground_leg_num=0;
	brain.sys.off_cor[Xr]=-(brain.sys.leg_local[1].x-leg[4].pos_now[2].x);
	brain.sys.off_cor[Yr]=-(brain.sys.leg_local[1].y-leg[4].pos_now[2].y);
	#if USE_GLOBAL_COOR
	brain.sys.off_cor[Xr]=brain.sys.off_cor[Yr]=0;
	#endif
	off_cor[Xr]=brain.sys.off_cor[Xr];
	off_cor[Yr]=brain.sys.off_cor[Yr];
	for(i=1;i<4;i++){
	brain.global.end_pos_global[i].x=leg[i].pos_now_brain[2].x+brain.sys.leg_local[1].x-leg[4].pos_now[2].x+off_cor[Xr];
	brain.global.end_pos_global[i].y=leg[i].pos_now_brain[2].y+brain.sys.leg_local[1].y-leg[4].pos_now[2].y+off_cor[Yr];
	brain.global.end_pos_global[i].z=leg[i].pos_now_brain[2].z-z_zero;
	}
	
  brain.global.end_pos_global[4].x=off_cor[Xr];
  brain.global.end_pos_global[4].y=off_cor[Yr];
	brain.global.end_pos_global[4].z=leg[4].pos_now_brain[2].z-z_zero;

	float k_acc1=10.0/brain.sys.k_center_c[0];
	float temp1=leg[1].pos_now_brain[2].z*leg[1].leg_ground+leg[2].pos_now_brain[2].z*leg[2].leg_ground+
	leg[3].pos_now_brain[2].z*leg[3].leg_ground+leg[4].pos_now_brain[2].z*leg[4].leg_ground;
	brain.global.end_pos_global[0].z=temp1/brain.ground_leg_num+0.5*brain.now_acc[2]*dt*dt;
	
  brain.global.end_pos_global[0].x=brain.sys.leg_local[1].x-leg[4].pos_now[2].x
	+flag_acc[0]*my_deathzoom(brain.global.end_pos_global[0].z/9.87*brain.now_acc[Xr],0.06)*k_acc1*0+off_cor[Xr];
	brain.global.end_pos_global[0].y=brain.sys.leg_local[1].y-leg[4].pos_now[2].y
	+flag_acc[1]*my_deathzoom(brain.global.end_pos_global[0].z/9.87*brain.now_acc[Yr],0.06)*k_acc1*0+off_cor[Yr];
	
	if(isnan(brain.global.end_pos_global[0].x))
	brain.global.end_pos_global[0].x=leg[1].sys.init_end_pos.x;
	if(isnan(brain.global.end_pos_global[0].y))
	brain.global.end_pos_global[0].y=leg[1].sys.init_end_pos.y;
	if(isnan(brain.global.end_pos_global[0].z))
	brain.global.end_pos_global[0].z=leg[1].sys.init_end_pos.z;
  //ZMP
	brain.global.ZMP.x=brain.sys.leg_local[1].x-leg[4].pos_now[2].x
	+flag_acc[0]*my_deathzoom(brain.global.end_pos_global[0].z/9.87*brain.now_acc[Xr],0.06)*k_acc1+off_cor[Xr];
	brain.global.ZMP.y=brain.sys.leg_local[1].y-leg[4].pos_now[2].y
	+flag_acc[1]*my_deathzoom(brain.global.end_pos_global[0].z/9.87*brain.now_acc[Yr],0.06)*k_acc1+off_cor[Yr];
	brain.global.ZMP.z=0;
	
	for(i=1;i<5;i++){
	brain.global.body_coner[i].x=brain.global.end_pos_global[0].x+brain.sys.leg_local[i].x;
	brain.global.body_coner[i].y=brain.global.end_pos_global[0].y+brain.sys.leg_local[i].y;
	brain.global.body_coner[i].z=brain.global.end_pos_global[0].z;	
	}

for(i=1;i<5;i++){
		if(leg[i].leg_ground){
		x[j]=brain.global.end_pos_global[i].x;
		y[j]=brain.global.end_pos_global[i].y;
		j++;
		}
  }
ground_leg_num=j;
//generate fake end pos
float cro_x,cro_y;
float k[2],b[2];	
switch(ground_leg_num) 
 {	 
	case 2://trot gait

	line_function_from_two_point(brain.global.end_pos_global[1].x,brain.global.end_pos_global[1].y,
	brain.global.end_pos_global[4].x,brain.global.end_pos_global[4].y,&k[0],&b[0]);
  line_function_from_two_point(brain.global.end_pos_global[2].x,brain.global.end_pos_global[2].y,
	brain.global.end_pos_global[3].x,brain.global.end_pos_global[3].y,&k[1],&b[1]);
	cross_point_of_lines(k[0],b[0],k[1],b[1],&cro_x,&cro_y);
	if(leg[1].leg_ground==0||leg[4].leg_ground==0){
	two_point_between_arror(cro_x,cro_y,brain.spd_yaw
	,&x[0],&y[0],&x[1],&y[1],1);
	x[2]=brain.global.end_pos_global[1].x;
	y[2]=brain.global.end_pos_global[1].y;	
	x[3]=brain.global.end_pos_global[4].x;
	y[3]=brain.global.end_pos_global[4].y;
	}
	else if(leg[2].leg_ground==0||leg[3].leg_ground==0){
	two_point_between_arror(cro_x,cro_y,brain.spd_yaw
	,&x[0],&y[0],&x[1],&y[1],1);	
	x[2]=brain.global.end_pos_global[2].x;
	y[2]=brain.global.end_pos_global[2].y;	
	x[3]=brain.global.end_pos_global[3].x;
	y[3]=brain.global.end_pos_global[3].y;	
	}
	 brain.global.leg_ground_center[Xr]=(float)(x[0]+x[1]+x[2]+x[3])/4.;
   brain.global.leg_ground_center[Yr]=(float)(y[0]+y[1]+y[2]+y[4])/4.;
   brain.global.area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2])+cal_area_trig( x[3],y[3], x[1],y[1], x[2],y[2]);
   brain.global.steady_value=cal_steady_s4( brain.global.ZMP.x,brain.global.ZMP.y,x[0],y[0], x[1],y[1], x[2],y[2],x[3],y[3]);	
 break;	 
 case 3:
   brain.global.leg_ground_center[Xr]=(float)(x[0]+x[1]+x[2])/3.;
   brain.global.leg_ground_center[Yr]=(float)(y[0]+y[1]+y[2])/3.;
   brain.global.area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2]);
   brain.global.steady_value=cal_steady_s( brain.global.ZMP.x,brain.global.ZMP.y,x[0],y[0], x[1],y[1], x[2],y[2]);
 break;
 case 4:
    temp[0][Xr]=(float)(x[0]+x[1]+x[2])/3.;
    temp[0][Yr]=(float)(y[0]+y[1]+y[2])/3.;
    temp[1][Xr]=(float)(x[1]+x[2]+x[3])/3.;
    temp[1][Yr]=(float)(y[1]+y[2]+y[3])/3.;
    brain.global.leg_ground_center[Xr]=(float)(temp[0][Xr]+temp[1][Xr])/2.;
    brain.global.leg_ground_center[Yr]=(float)(temp[0][Yr]+temp[1][Yr])/2.;
    brain.global.area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2])+cal_area_trig( x[3],y[3], x[1],y[1], x[2],y[2]);
    brain.global.steady_value=cal_steady_s4( brain.global.ZMP.x,brain.global.ZMP.y,x[0],y[0], x[1],y[1], x[3],y[3],x[2],y[2]);	
 break;
 default:
	  brain.global.area_of_leg[0]=0;
 break;
 }	 
 
 float dis[2];
 switch(ground_leg_num) 
 {	 
	 case 3:
		 if(leg[1].leg_ground==0||leg[3].leg_ground==0)
		 dis[0]=cal_dis_of_points(brain.global.end_pos_global[2].x,brain.global.end_pos_global[2].y,brain.global.end_pos_global[4].x,brain.global.end_pos_global[4].y);
		 else 
		 dis[0]=cal_dis_of_points(brain.global.end_pos_global[1].x,brain.global.end_pos_global[1].y,brain.global.end_pos_global[3].x,brain.global.end_pos_global[3].y);
   brain.global.min_width_value=LIMIT(dis[0]/(brain.sys.leg_local[1].x*2*1),0,1);
	 break;
	 case 4:
   dis[0]=cal_dis_of_points(brain.global.end_pos_global[1].x,brain.global.end_pos_global[1].y,brain.global.end_pos_global[3].x,brain.global.end_pos_global[3].y);
	 dis[1]=cal_dis_of_points(brain.global.end_pos_global[2].x,brain.global.end_pos_global[2].y,brain.global.end_pos_global[4].x,brain.global.end_pos_global[4].y);
	 
	 brain.global.min_width_value=LIMIT(MIN(dis[0],dis[1])/(brain.sys.leg_local[1].x*2*1.618),0,1);
	 break;
 }
	#if defined(CENTER_CONTROL_TEST)
	brain.leg_move_state=S_BODY_MOVE;
  brain.move_id=leg_off_ground_test;
	#endif 
//--------------------------Fake pos when trig leg
if(brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_LEG_TRIG){ 
j=0;
for(i=1;i<5;i++){
		if(i!=brain.move_id){
		x[j]=brain.global.end_pos_global[i].x;
		y[j]=brain.global.end_pos_global[i].y;
		j++;
		}
  }
   brain.global.leg_ground_center_trig[Xr]=(float)(x[0]+x[1]+x[2])/3.;
   brain.global.leg_ground_center_trig[Yr]=(float)(y[0]+y[1]+y[2])/3.;
	// brain.global.steady_value=cal_steady_s( brain.global.ZMP.x,brain.global.ZMP.y,x[0],y[0], x[1],y[1], x[2],y[2]);
} 

  brain.global.area_value=LIMIT((brain.area_of_leg[0]/brain.area_of_leg[1]),0,1);
}	


//----------Body center movement control
float control_draw[50];
float ero_center[2];
float center_control_out[2];
void center_control_global(float dt)
{
    static char reg_flag;
    char i;
    char id_f[3]={0};
    float ero[2];
    float center_tar_x,center_tar_y;
    float spd_use,yaw_use;
    float k_c=0,b_c=0;
    float k_c1=0,b_c1=0;
    float gx=brain.global.leg_ground_center[Xr];
    float gy=brain.global.leg_ground_center[Yr];
    float tar_x,tar_y;
    char brain_ground_leg_num=brain.ground_leg_num;
    char leg_ground[5];
    float leg_end_use[5][2];
    control_draw[0]=0;
    //--------------value flt
    brain.global.value[0]=0.9*brain.global.value[0]+0.1*brain.global.area_value*brain.global.out_value*LIMIT(brain.global.center_stable_weight,0.3,1);
    brain.global.value[1]=0.9*brain.global.value[1]+0.1*LIMIT(brain.global.area_value,0.68,1);
    brain.global.value[2]=0.9*brain.global.value[2]+0.1*brain.global.area_value* brain.global.min_width_value;
    if(brain.force_stop==2)
    brain.global.value[3]=0.1;
    else if(brain.force_stop==1)
    brain.global.value[3]-=(force_rate*dt);
    else
    brain.global.value[3]+=(force_rate*dt);
    brain.global.value[3]=LIMIT(brain.global.value[3],0.1,1);

    for(i=0;i<5;i++)
    leg_ground[i]=leg[i].leg_ground;
    #if defined(CENTER_CONTROL_TEST)
    brain.leg_move_state=S_BODY_MOVE;
    for(i=0;i<5;i++)
     if(i!=leg_off_ground_test)
     {leg_ground[i]=1;leg[i].sys.limit.z=brain.tar_h;}
     else
     {leg_ground[i]=0;leg[i].sys.limit.z=brain.tar_h-3;}
    #endif
    if(brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_LEG_TRIG){
    gx=brain.global.leg_ground_center_trig[Xr];
    gy=brain.global.leg_ground_center_trig[Yr];
    brain_ground_leg_num=3;
    leg_ground[brain.move_id]=0;
    }
    #if TEST_MODE1
    k_spd=10;
    if(brain.force_stop)
    spd_use=k_spd*brain.global.center_stable_weight/4;
    else
    spd_use=brain.spd*k_spd*brain.global.center_stable_weight;
    #else
    spd_use=brain.spd*k_spd*brain.global.value[0]*brain.global.value[3];
		//spd_use=brain.spd*k_spd;//*brain.global.center_stable_weight;
    #endif
    //resize
    float gx_use,gy_use;
    float low_k=brain.sys.leg_local[1].x/brain.sys.leg_local[1].y*k_size;
    float temp1=(1-low_k)/2;
    float temp2=cos(yaw_use*2*ANGLE_TO_RADIAN);
    size_k= LIMIT(temp1*temp2+low_k+temp1,0,1);
    //size_k=1;
    yaw_use=brain.spd_yaw;
    #if HORIZON_USE_FORWARD_CENTER
      if((brain.spd_yaw!=0&&brain.spd_yaw!=180)&&brain_ground_leg_num==3)
      {
            size_k=1;
            yaw_use=0;
            spd_use=0;
        }
    #endif
    float re_x=0,re_y=0;
    for(i=1;i<5;i++)
    resize_point_with_arrow(brain.global.end_pos_global[i].x,brain.global.end_pos_global[i].y,re_x,re_y,yaw_use,size_k,&leg_end_use[i][Xr],&leg_end_use[i][Yr]);

    resize_point_with_arrow(gx,gy,re_x,re_y,yaw_use,size_k,&gx_use,&gy_use);


    //normal
    line_function_from_arrow(gx_use,gy_use,yaw_use,&k_c,&b_c);


    switch(brain_ground_leg_num)
    {
        case 3:{//three leg ground
            //Debug

          if(leg[3].leg_ground==0)
           i=0;
            //
          char front_leg_num=0;
          float f_leg_x,f_leg_y;
          for(i=1;i<5;i++)
          if(leg_ground[i])
              if(check_point_front_arrow(leg_end_use[i][Xr],leg_end_use[i][Yr],gx_use,gy_use,yaw_use)){
                         id_f[front_leg_num++]=i;
                    }
      brain.sys.front_leg_num=front_leg_num;
          f_leg_x=leg_end_use[id_f[0]][Xr];
          f_leg_y=leg_end_use[id_f[0]][Yr];
     if(front_leg_num==1){	//one leg before center
         control_draw[0]=1;
         //cout<<control_draw[0]<<endl;
            float cross_point_x,cross_point_y;
            float k_f,b_f;
            for(i=1;i<5;i++)
                if(leg_ground[i]&&i!=id_f[0]&&i!=brain.move_id)
                {
                    float k_temp,b_temp;
                    line_function_from_two_point(f_leg_x,f_leg_y,leg_end_use[i][Xr],leg_end_use[i][Yr],&k_temp,&b_temp);
                    if(check_cross_arrow_line(gx_use,gy_use,yaw_use,k_temp,b_temp,&cross_point_x,&cross_point_y)){
                        id_f[1]=i;
                        k_f=k_temp;
                        b_f=b_temp;
                        break;}
                }

            cross_point_x-=brain.min_st[0]*sin((yaw_use)*ANGLE_TO_RADIAN)*size_k;
            cross_point_y-=brain.min_st[0]*cos((yaw_use)*ANGLE_TO_RADIAN)*size_k;
            control_draw[1]=cross_point_x;
            control_draw[2]=cross_point_y;
            control_draw[3]=f_leg_x;
            control_draw[4]=f_leg_y;
            float k_f_st,b_f_st;
            line_function_from_two_point(cross_point_x,cross_point_y,f_leg_x,f_leg_y,&k_f_st,&b_f_st);


            int traj_flag;

            traj_flag=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point_x,cross_point_y,yaw_use);
            //cout<<traj_flag<<endl;
            if(traj_flag){
            check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            brain.global.end_pos_global[0].y+cos(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            yaw_use,k_f_st,b_f_st,&tar_x,&tar_y);
            }
            else
            check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            brain.global.end_pos_global[0].y+cos(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            yaw_use,k_c,b_c,&tar_x,&tar_y);


        if(cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
          f_leg_x,f_leg_y)<brain.min_st[0]*size_k)
        {tar_x=f_leg_x-brain.min_st[0]*size_k*sin((yaw_use)*ANGLE_TO_RADIAN);tar_y=f_leg_y-brain.min_st[0]*size_k*cos((yaw_use)*ANGLE_TO_RADIAN);brain.force_stop=1;}
        }
      else//  two leg before
        { control_draw[0]=2;
          float f_leg_x1,f_leg_y1;
          f_leg_x1=leg_end_use[id_f[1]][Xr];
          f_leg_y1=leg_end_use[id_f[1]][Yr];

          char id3;
          for(i=1;i<5;i++)
              if(leg_ground[i]&&i!=id_f[0]&&i!=id_f[1]&&i!=brain.move_id)
              {
                  id3=i;
                  break;
              }
            float jiao[2][2];
            float coner[3][2];
            if(leg[4].leg_ground==0)
                i=0;
           // cout<<(int)id_f[0]<<" "<<(int)id_f[1]<<" "<<(int)id3<<endl;
            coner[0][Xr]=f_leg_x;coner[0][Yr]=f_leg_y;
            coner[1][Xr]=f_leg_x1;coner[1][Yr]=f_leg_y1;
            coner[2][Xr]=leg_end_use[id3][Xr];coner[2][Yr]=leg_end_use[id3][Yr];
            check_point_to_trig(gx_use,gy_use,yaw_use,f_leg_x,f_leg_y,f_leg_x1,f_leg_y1,leg_end_use[id3][Xr],leg_end_use[id3][Yr],
                                &jiao[0][Xr],&jiao[0][Yr],&jiao[1][Xr],&jiao[1][Yr]);
            float dis[2][3];
            dis[0][0]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],f_leg_x,f_leg_y);
            dis[0][1]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],f_leg_x1,f_leg_y1);
            dis[0][2]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],leg_end_use[id3][Xr],leg_end_use[id3][Yr]);
            dis[1][0]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],f_leg_x,f_leg_y);
            dis[1][1]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],f_leg_x1,f_leg_y1);
            dis[1][2]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],leg_end_use[id3][Xr],leg_end_use[id3][Yr]);
            float dis_big[2]={dis[0][0],dis[1][0]};
            char id_closet[2]={1,1};
            for(i=1;i<3;i++)
              if(dis[0][i]<dis_big[0])
                     id_closet[0]=i;
            for(i=1;i<3;i++)
              if(dis[1][i]<dis_big[1])
                     id_closet[1]=i;

            float cross_point[2][2];
            float coner_point[2][2];
            coner_point[0][Xr]=coner[id_closet[0]][Xr];
            coner_point[0][Yr]=coner[id_closet[0]][Yr];
            coner_point[1][Xr]=coner[id_closet[1]][Xr]+brain.min_st[1]*size_k*sin((yaw_use)*ANGLE_TO_RADIAN);
            coner_point[1][Yr]=coner[id_closet[1]][Yr]+brain.min_st[1]*size_k*cos((yaw_use)*ANGLE_TO_RADIAN);

            cross_point[0][Xr]=jiao[0][Xr]-brain.min_st[0]*size_k*sin((yaw_use)*ANGLE_TO_RADIAN);
            cross_point[0][Yr]=jiao[0][Yr]-brain.min_st[0]*size_k*cos((yaw_use)*ANGLE_TO_RADIAN);
            cross_point[1][Xr]=jiao[1][Xr]+brain.min_st[1]*size_k*sin((yaw_use)*ANGLE_TO_RADIAN);
            cross_point[1][Yr]=jiao[1][Yr]+brain.min_st[1]*size_k*cos((yaw_use)*ANGLE_TO_RADIAN);

            control_draw[1]=cross_point[0][Xr];
            control_draw[2]=cross_point[0][Yr];
            control_draw[3]=cross_point[1][Xr];
            control_draw[4]=cross_point[1][Yr];
            control_draw[5]=coner_point[0][Xr];
            control_draw[6]=coner_point[0][Yr];
            control_draw[7]=coner_point[1][Xr];
            control_draw[8]=coner_point[1][Yr];
            control_draw[9]=coner_point[2][Xr];
            control_draw[10]=coner_point[2][Yr];

            //
            float k_tr[3],b_tr[3];
            line_function_from_two_point(coner_point[0][Xr],coner_point[0][Yr],cross_point[0][Xr],cross_point[0][Yr],&k_tr[0],&b_tr[0]);
            line_function_from_two_point(cross_point[0][Xr],cross_point[0][Yr],cross_point[1][Xr],cross_point[1][Yr],&k_tr[1],&b_tr[1]);
            line_function_from_two_point(cross_point[1][Xr],cross_point[1][Yr],coner_point[1][Xr],coner_point[1][Yr],&k_tr[2],&b_tr[2]);

            int traj_flag[3];
            //
            traj_flag[0]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point[0][Xr],cross_point[0][Yr],yaw_use);
            traj_flag[1]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point[1][Xr],cross_point[1][Yr],yaw_use);

            if(traj_flag[0]==1&&traj_flag[1]==1&&1)//qian
            check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            brain.global.end_pos_global[0].y+cos(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            yaw_use,k_tr[0],b_tr[0],&tar_x,&tar_y);
            else if(traj_flag[0]==0&&traj_flag[1]==0)//hou
            check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            brain.global.end_pos_global[0].y+cos(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            yaw_use,k_tr[2],b_tr[2],&tar_x,&tar_y);
            //{tar_x=cross_point[1][Xr];tar_y=cross_point[1][Yr];}
            else//zhongj
            check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            brain.global.end_pos_global[0].y+cos(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            yaw_use,k_tr[1],b_tr[1],&tar_x,&tar_y);

            if(cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
          coner_point[0][Xr],coner_point[0][Yr])<brain.min_st[0]*size_k||
            cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
          coner_point[1][Xr],coner_point[1][Yr])<brain.min_st[1]*size_k)
        brain.force_stop=1;
        }//end----

          center_tar_x=tar_x;
            center_tar_y=tar_y;
        break;}
        case 4:
    {//4 leg ground
          check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            brain.global.end_pos_global[0].y+cos(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt,
            yaw_use,k_c,b_c,&tar_x,&tar_y);
            center_tar_x=tar_x;
            center_tar_y=tar_y;
    }
        break;
    }
    if(brain.global.steady_value<brain.min_st[2]){
    center_tar_x=brain.global.leg_ground_center[Xr];center_tar_y=brain.global.leg_ground_center[Yr];
    }
    brain.global.tar_center[Xr]=brain.tar_center[Xr]=center_tar_x;
    brain.global.tar_center[Yr]=brain.tar_center[Yr]=center_tar_y;

    ero_center[Xr]=my_deathzoom((brain.tar_center[Xr]-brain.global.end_pos_global[0].x
    +k_spd_control[Xr]*my_deathzoom(LIMIT(brain.now_spd[Xr],-0.66,0.66),0.01)
    +k_acc_control[Xr]*brain.global.end_pos_global[0].z/9.87*my_deathzoom(LIMIT(brain.now_acc[Xr],-2,2),0.3)),0.001);
  ero_center[Yr]=my_deathzoom((brain.tar_center[Yr]-brain.global.end_pos_global[0].y
    +k_spd_control[Yr]*my_deathzoom(LIMIT(brain.now_spd[Yr],-0.66,0.66),0.01)
    +k_acc_control[Yr]*brain.global.end_pos_global[0].z/9.87*my_deathzoom(LIMIT(brain.now_acc[Yr],-2,2),0.3)),0.001);

    float ero1[2];
    ero1[Xr]=my_deathzoom((brain.tar_center[Xr]-(brain.global.end_pos_global[0].x+sin(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt)),0.001);
  ero1[Yr]=my_deathzoom((brain.tar_center[Yr]-(brain.global.end_pos_global[0].y+cos(yaw_use*ANGLE_TO_RADIAN)*spd_use*dt)),0.001);
    float dis_ero=sqrt(pow(ero1[Xr],2)+pow(ero1[Yr],2));
    brain.global.center_stable_weight=LIMIT((brain.sys.leg_local[1].x/2-LIMIT(dis_ero,0,brain.sys.leg_local[1].x/2))/(brain.sys.leg_local[1].x/2),0,1);


    center_control_out[Xr]=ero_center[Xr]*brain.sys.k_center_c[Xr];
    center_control_out[Yr]=ero_center[Yr]*brain.sys.k_center_c[Yr];
    //cout<<"cx:"<< center_control_out[Xr]<<" cy:"<< center_control_out[Yr]<<endl;
    float k_ero_in=1;
    if(dis_ero<center_stable_dead)
        k_ero_in=1.618;
    if(dis_ero<center_stable_dead)
    brain.center_stable=1;
    else
    brain.center_stable=0;

    float fall_con[4]={0};
  //output
    for(i=1;i<5;i++){
    if((leg[i].control_mode||brain.control_mode)&&leg[i].leg_ground&&!leg[i].sys.leg_ground_force&&brain.trot_gait==0){
    leg[i].deng[Xr]=LIMIT(center_control_out[Xr],-limit_deng*brain.global.value[1]*k_ero_in,limit_deng*brain.global.value[1]*k_ero_in)+fall_con[i-1];
    leg[i].deng[Yr]=LIMIT(center_control_out[Yr],-limit_deng*brain.global.value[1]*k_ero_in,limit_deng*brain.global.value[1]*k_ero_in);	}
    }

     reg_flag=brain.leg_move_state;
}


//-*******************************************************************************************************************/
//-*******************************************************************************************************************/
//-*******************************************************************************************************************/
//-*******************************************************************************************************************/
//-*******************************************************************************************************************/
//-*******************************************************************************************************************/
//-*******************************************************************************************************************/
u8 last_move_id,last_last_move_id;
u16 out_range_move[5];
//Gait Plan
void check_leg_need_move_global(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)
{ u8 i,j;
	u8 need_move_leg=0;

	float spd=brain.spd;	
	static u8 lose_center_flag;
 
	if(brain.ground_leg_num<4)//may cause hard error when use if else
	brain.sys.dt_leg_min_trig_cnt=0;
	if(brain.sys.dt_leg_min_trig_cnt++>brain.sys.leg_move_min_dt*brain.sys.desire_time/dt)
	brain.can_move_leg=1;
	
	float yaw_in=To_180_degrees(brain.spd_yaw);
	float yaw_temp=brain.sys.yaw_trig*2;
	u8 way;
	u8 fp_point_id[4];//L  R
	if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)//r
	{fp_point_id[0]=1;fp_point_id[1]=2;   fp_point_id[2]=3;fp_point_id[3]=4;brain.way=way=2;}
	else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)//l
	{fp_point_id[0]=4;fp_point_id[1]=3;   fp_point_id[2]=2;fp_point_id[3]=1;brain.way=way=4;}
	else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))//f
	{fp_point_id[0]=3;fp_point_id[1]=1;   fp_point_id[2]=4;fp_point_id[3]=2;brain.way=way=1;}
	else//b
	{fp_point_id[0]=2;fp_point_id[1]=4;   fp_point_id[2]=1;fp_point_id[3]=3;brain.way=way=3;}
		

	float range_in[5]={1},range_out[5]={1},range_stop[5]={1};
	float jiaodiao[2][2]={0};	
 for(i=1;i<5;i++){
	//check out range
	 #if TEST_MODE1
	 if(leg[i].leg_ground||1){
	 #else
	 if(leg[i].leg_ground){
	 #endif
	 float length,tar_x,tar_y;
   tar_x=brain.global.end_pos_global[i].x;
	 tar_y=brain.global.end_pos_global[i].y;
	 
	 float circle_center[2]={0};
	 conver_body_to_global(
	 brain.sys.leg_local[i].x+leg[i].sys.init_end_pos.x,
	 brain.sys.leg_local[i].y+leg[i].sys.init_end_pos.y,
	 &circle_center[0],&circle_center[1]);
	 if(i==stop_leg)//debug
		 j=0;
	 cal_jiao_of_range_and_line_tangle(i,circle_center[0],circle_center[1],in->sys.min_range,in->sys.max_range,in->spd_yaw,&jiaodiao[0][Xr],&jiaodiao[0][Yr],&jiaodiao[1][Xr],&jiaodiao[1][Yr]);

	 float dis[2];
	 dis[0]=cal_dis_of_points(tar_x,tar_y,jiaodiao[0][Xr],jiaodiao[0][Yr]);
	 dis[1]=cal_dis_of_points(tar_x,tar_y,jiaodiao[1][Xr],jiaodiao[1][Yr]);
   float min_dis[2]={0};
	 range_in[i]=check_in_move_range_tangle(i,tar_x,tar_y,circle_center[0],circle_center[1],in->sys.min_range,in->sys.max_range,&min_dis[0]);

	 if(dis[0]<dis[1])
		 min_dis[1]=dis[0];
	 else
		 min_dis[1]=dis[1];
	 if(dis[1]<dis[0]&&range_in[i]==0)//not in move range
	 brain.global.dis_leg_out[i]=min_dis[0];
	 else
	 brain.global.dis_leg_out[i]=0;
	 
	 if(brain.rst_all_soft)
	  brain.tabu=need_move_leg=1;  	 
 
	 if(dis[1]<dis[0]&&range_in[i]==0)
	 { 
	 leg[i].need_move=3;
	 out_range_move[i]++;		 
	 }
	 
	 //outrange for slow
	 range_out[i]=check_in_move_range(i,tar_x,tar_y,circle_center[0],circle_center[1],in->sys.min_range*k_force,in->sys.max_range*k_force);
	 if(dis[1]<dis[0]&&range_out[i]==0)
	 { 
	   brain.force_stop=1;
	 }
	 
	 //outrange for stop
	 range_stop[i]=check_in_move_range(i,tar_x,tar_y,circle_center[0],circle_center[1],force_stop_range[0],force_stop_range[1]);
	 if(dis[1]<dis[0]&&range_stop[i]==0)
	 { 
	   brain.force_stop=2;
	 }
	 brain.leg_move[i]=leg[i].need_move;
  }
 } 
//calculate the out range value
 float max_out_range=0;
 for(i=0;i<5;i++)
 if(brain.global.dis_leg_out[i]>max_out_range) 
 {max_out_range=brain.global.dis_leg_out[i];}
  brain.global.out_value=1;//LIMIT(1-LIMIT(max_out_range,0,set_max)/set_max,min_spd,1);
 
//............................................................................................................
 //----------.......................Move leg condition......................................
 static u16 s_cnt;
 u8 center_stable=0;
 if(brain.center_stable)
 s_cnt++;
 else
 s_cnt=0;
 
 if(s_cnt>0.05/dt)
	 center_stable=1;
 if((brain.ground_leg_num==4&&center_stable)&&(check_leg_near_init(brain.sys.in_rst_check)==0||brain.spd>0))
 {
		   need_move_leg=1;
 }

//---------------------------------------Gait state machine----------------------------------	
 	
  #if !USE_LEG_TRIG_DELAY
   brain.leg_move_state=S_IDLE;
	#endif
  static float center_now[2];
  static u8 leg_flag=1;
  static u16 cnt;
  u8 temp=0; 		
		
	switch(brain.leg_move_state)
	{
	 case S_IDLE:			
		 if(need_move_leg&&brain.can_move_leg&&brain.ground_leg_num>3)
		 {
		 u8 id_need_to_move=0;
		 float yaw_in=To_180_degrees(in->spd_yaw);
		 float yaw_temp=in->sys.yaw_trig;
 
			 if(brain.rst_all_soft){//Reset leg			 
         for(i=1;i<5;i++)
				   if(cal_dis_of_points(leg[i].pos_now[2].x,leg[i].pos_now[2].y,
		        leg[i].sys.init_end_pos.x,leg[i].sys.init_end_pos.y)>brain.sys.in_rst_check)
					 {temp=i;
					 leg_flag=get_next_leg_flag_in_list(temp,brain.way)+1;
					 if(leg_flag>4)
					 leg_flag=1;	
					 break;}
				 }
				else{	// normal  crawl gait
         u8 out1=0,out2=0;
				  //most out with number
				 float most_out=0;
				 for(i=0;i<5;i++){
					 if(out_range_move[i]>most_out&&i!=last_move_id&&i!=last_last_move_id)	 
					 {out1=id_need_to_move=i;most_out=out_range_move[i];} 
				 }
				 
				 //most out with distance
				 most_out=0;
				 for(i=0;i<5;i++)
				   if(brain.global.dis_leg_out[i]>most_out&&i!=last_move_id&&i!=last_last_move_id)
					 {out2=id_need_to_move=i;most_out=brain.global.dis_leg_out[i];}
					 
				 if(out2==0&&out1!=0)
				 id_need_to_move=out1;	
				 if(out2!=out1)
				 {
				   for(i=0;i<2;i++)
				     if(out1==fp_point_id[i])
						 { id_need_to_move=out1;break;} 	 
						 id_need_to_move=out2;
				 } 
				 //id_need_to_move=out2;
				 //
				 
				 if(id_need_to_move>0&&out_protect)//out range to move
				 { out_range_move[id_need_to_move]=0;
					 temp=id_need_to_move;
					 leg_flag=get_next_leg_flag_in_list(temp,brain.way)+1;
					 if(leg_flag>4)
					 leg_flag=1;		
			     
				 }else if(en_leg_trig_list)//                 normal
				 {
				 if(brain.spd<0.3&&fabs(brain.tar_w)>0.3&&1)
					if(brain.tar_w>0)
					id_need_to_move=trig_list_tr[leg_flag];	
					else
					id_need_to_move=trig_list_tl[leg_flag];	 
         else{				 
				 if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
				 id_need_to_move=trig_list_r[leg_flag];
				 else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
				 id_need_to_move=trig_list_l[leg_flag];
				 else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
				 id_need_to_move=trig_list_f[leg_flag];
				 else
				 id_need_to_move=trig_list_b[leg_flag];
			   }
				 //id_need_to_move=planner_leg(last_move_id,last_last_move_id);///////////////////////////mine new/////////////////////////
				 temp=id_need_to_move;
	       leg_flag++;
				 if(leg_flag>4)
					leg_flag=1;		
			   }					 
				}//end
				 #if TIRG_USE_LITTLE_DOG
         in->move_id=planner_leg_little_dog(last_move_id, last_last_move_id);
         #else				
				 in->move_id=temp;
				 #endif
				 //repeat protect
				 if(repeat_protect)
				 in->move_id=leg_repeat_protect1(temp, last_move_id, last_last_move_id,brain.spd_yaw,brain.sys.yaw_trig);
				 
				 
				 //---area protect
				 float x[5],y[5];
				 leg_tar_est_global(&brain,&leg[brain.move_id],0,0,0,1,dt,1);	//fake
				 x[in->move_id]=brain.global.fake_tar_pos.x+brain.sys.leg_local[in->move_id].x;
				 y[in->move_id]=brain.global.fake_tar_pos.y+brain.sys.leg_local[in->move_id].y;
				 for(i=1;i<5;i++)
				  if(i!=in->move_id)
					{ x[i]=leg[i].pos_now[2].x+brain.sys.leg_local[i].x;
					  y[i]=leg[i].pos_now[2].y+brain.sys.leg_local[i].y;
					}
				  brain.global.area_of_leg[2]=cal_area_trig( x[1],y[1], x[2],y[2], x[3],y[3])/2+cal_area_trig( x[4],y[4], x[2],y[2], x[3],y[3])/2;	 
				 //---minwidth protect
					float dis[2],max_dis;
					dis[0]=cal_dis_of_points(x[1],y[1],x[3],y[3]);
					dis[1]=cal_dis_of_points(x[2],y[2],x[4],y[4]);
					if(dis[0]>dis[1])
					max_dis=dis[0];
					else
				  max_dis=dis[1];
					
				 //---error out	
					float min_area=brain.global.area_of_leg[1]*0.689;
				  float min_dis=MIN(in->sys.leg_local[1].x*2*1.618,in->sys.leg_local[1].y*2*1.618);
				 	if((brain.global.area_of_leg[2]<min_area||max_dis<min_dis)&&area_protect)//protect out
				  in->move_id=side_leg[in->move_id];
				
				 //------output---
				 if(in->move_id>0)
				 {
				 last_last_move_id=last_move_id;
				 last_move_id=in->move_id;	
				 brain.can_move_leg=0;
				 out_range_move[in->move_id]=0;					 
				 leg[brain.move_id].sys.leg_move_pass_cnt=0;cnt=0;
				 brain.leg_move_state=S_BODY_MOVE;		
				 }					
			 }// end if 
		break;
	  case S_BODY_MOVE: 
		 if(in->move_id==side_leg_f[last_move_id])
			 brain.leg_move_state=S_LEG_TRIG;
		 else 
			 cnt++;
		 if(cnt>0.05/dt)//give some delay for body move 
		  {brain.leg_move_state=S_LEG_TRIG;cnt=0;}
		break;
	  case S_LEG_TRIG:
			if(center_stable&&brain.ground_leg_num==4)
			 cnt++;
      else
       cnt=0;				
			if(cnt>0.05/dt)
			{cnt=0;
			brain.leg_move_state=S_LEG_TRIG_LEAVE_GROUND_CHECK;		 
			leg_tar_est_global(&brain,&leg[brain.move_id],0,0,0,1,dt,0);	
			}			
		break;
	  case S_LEG_TRIG_ING:	
		if(cnt++>3*brain.sys.desire_time/dt||brain.ground_leg_num<4){cnt=0;	
		brain.leg_move_state=S_LEG_TRIG_LEAVE_GROUND_CHECK;}
		break;
		case S_LEG_TRIG_LEAVE_GROUND_CHECK:
		if(brain.ground_leg_num>3)
	  {brain.leg_move_state=S_IDLE;}
		break;
	}
	
  if(brain.tar_att_force[0]!=0)
	 brain.tar_att[0]=brain.tar_att_force[0];
	if(brain.tar_att_force[1]!=0)
	 brain.tar_att[1]=brain.tar_att_force[1];
}


u8 planner_leg_little_dog(u8 last_move_id,u8 last_last_move_id)
{ 
  u8 i,j,k,in_trig=0;
	float cx,cy,gx,gy,steady_value,dis_cog_c,dis_can_move[5]={0};
	static float spd_yaw_reg;
	u8 leg_can_move[5]={0};
	u8 flag=0;
	float x[5]={0},y[5]={0};
	
	cx=brain.global.end_pos_global[0].x;
	cy=brain.global.end_pos_global[0].y;
	for(i=1;i<5;i++)
	{
 
			 //·½ÏòÎ´ÒÆ¶¯ È¡³öÉÏ´ÎÒÆ¶¯µÄÍÈ
		   if(spd_yaw_reg==brain.spd_yaw&&i==last_move_id)
		     continue;
			 
			 //ÅÐ¶ÏÒÆ¶¯¸ÃÍÈÊÇ·ñÔì³É»úÌå²»ÎÈ¶¨
			 k=x[0]=x[1]=x[2]=x[3]=x[4]=y[0]=y[1]=y[2]=y[3]=y[4]=0;
			 for(j=1;j<5;j++)
			  {
				  if(j!=i)
					{
					  x[k]=brain.global.end_pos_global[j].x;
					  y[k++]=brain.global.end_pos_global[j].y;
					}
				}
				cal_cog_tri(x[0],y[0],x[1],y[1],x[2],y[2],&gx,&gy);
				dis_cog_c=cal_dis_of_points(gx,gy,cx,cy);
				in_trig=inTrig(cx,cy,x[0],y[0],x[1],y[1],x[2],y[2]);
				steady_value=cal_steady_s(cx,cy,x[0],y[0],x[1],y[1],x[2],y[2]);
				if(in_trig==0)
				continue;	
				if(steady_value<min_steady_value_stable)
				continue;	
				if(dis_cog_c>max_dis_cog)
				continue;	
				//can move
		     leg_can_move[flag++]=i;
	}
	float jiao[2];
	float k1[2],b1[2];
	float max_dis;
	u8 temp_id=0,id_use=0;
	//ÕÒµ½ÒÆ¶¯·½ÏòÉÏÄÜÒÆ¶¯×îÔ¶µÄÍÈ
	if(flag>0)
	{	
	  for(i=0;i<flag;i++)
	   { id_use=LIMIT(leg_can_move[i],1,4);
			 line_function_from_arrow(leg[id_use].pos_now[2].x,leg[id_use].pos_now[2].y,brain.spd_yaw,&k1[0],&b1[0]);
			 line_function90_from_arrow(leg[id_use].sys.init_end_pos.x,leg[id_use].sys.init_end_pos.y,brain.spd_yaw,&k1[1],&b1[1]);
			 if(cross_point_of_lines(k1[0],b1[0],k1[1],b1[1],&jiao[Xr],&jiao[Yr])){
				if(!check_point_front_arrow(leg[id_use].pos_now[2].x,leg[id_use].pos_now[2].y,jiao[Xr],jiao[Yr],brain.spd_yaw)) 
		     dis_can_move[i]=cal_dis_of_points(leg[id_use].pos_now[2].x,leg[id_use].pos_now[2].y,jiao[Xr],jiao[Yr]);
       }				 
		 }
		 max_dis=dis_can_move[0];
		 temp_id=leg_can_move[i];
		 for(i=0;i<flag;i++)
		   if(dis_can_move[i]>max_dis)
			 {temp_id=leg_can_move[i];max_dis=dis_can_move[i];}
			 
		return LIMIT(temp_id,1,4);	 
	} 
	else
		return last_move_id;
	spd_yaw_reg=brain.spd_yaw;
}	
//-----------------------area eater planner
u8 planner_leg(u8 last_move_id,u8 last_last_move_id)
{ 
	u8 i;

	float yaw_in=To_180_degrees(brain.spd_yaw);
	float yaw_temp=brain.sys.yaw_trig;
	u8 way_sel;
	u8 fp_point_id[4];//L  R
	 if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)//r
	 {fp_point_id[0]=1;fp_point_id[1]=2;   fp_point_id[2]=3;fp_point_id[3]=4;way_sel=2;}
	 else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)//l
	 {fp_point_id[0]=4;fp_point_id[1]=3;   fp_point_id[2]=2;fp_point_id[3]=1;way_sel=4;}
	 else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))//f
	 {fp_point_id[0]=3;fp_point_id[1]=1;   fp_point_id[2]=4;fp_point_id[3]=2;way_sel=1;}
	 else//b
	 {fp_point_id[0]=2;fp_point_id[1]=4;   fp_point_id[2]=1;fp_point_id[3]=3;way_sel=3;}

   float k_lr,b_lr;
	 float k_l3,b_l3;
	 float k_r3,b_r3;
   line_function_from_two_point(brain.global.end_pos_global[fp_point_id[0]].x,brain.global.end_pos_global[fp_point_id[0]].y,
	 brain.global.end_pos_global[fp_point_id[1]].x,brain.global.end_pos_global[fp_point_id[1]].x,
	 &k_lr,&b_lr);
	 //L
	 line_function_from_two_point(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.global.end_pos_global[fp_point_id[2]].x,brain.global.end_pos_global[fp_point_id[2]].x,
	 &k_l3,&b_l3);
	 //R
	 line_function_from_two_point(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.global.end_pos_global[fp_point_id[3]].x,brain.global.end_pos_global[fp_point_id[3]].x,
	 &k_r3,&b_r3);
	 //????á?·??ò×óóòá?2à???y
	 float coner_l[4][2];
	 check_cross_arrow_line(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.spd_yaw,k_lr,b_lr,&coner_l[0][Xr],&coner_l[0][Yr]);
	 check_cross_arrow90_line(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.spd_yaw,k_l3,b_l3,&coner_l[1][Xr],&coner_l[1][Yr]);
	 coner_l[2][Xr]=brain.global.end_pos_global[fp_point_id[0]].x;coner_l[2][Yr]=brain.global.end_pos_global[fp_point_id[0]].y;
	 coner_l[3][Xr]=brain.global.end_pos_global[0].x;coner_l[3][Yr]=brain.global.end_pos_global[0].y;
	 float coner_r[4][2];
	 check_cross_arrow_line(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.spd_yaw,k_lr,b_lr,&coner_r[0][Xr],&coner_r[0][Yr]);
	 check_cross_arrow90_line(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.spd_yaw,k_r3,b_r3,&coner_r[1][Xr],&coner_r[1][Yr]);
	 coner_r[2][Xr]=brain.global.end_pos_global[fp_point_id[1]].x;coner_r[2][Yr]=brain.global.end_pos_global[fp_point_id[1]].y;
	 coner_r[3][Xr]=brain.global.end_pos_global[0].x;coner_r[3][Yr]=brain.global.end_pos_global[0].y; 
	 
	 //cal_area_size
	 float size[2];
	 //L
	 size[0]=cal_area_trig(coner_l[0][Xr],coner_l[0][Yr],coner_l[2][Xr],coner_l[2][Yr],coner_l[3][Xr],coner_l[3][Yr])+
	 cal_area_trig(coner_l[1][Xr],coner_l[1][Yr],coner_l[2][Xr],coner_l[2][Yr],coner_l[3][Xr],coner_l[3][Yr]);
	 //R
	 size[1]=cal_area_trig(coner_r[0][Xr],coner_r[0][Yr],coner_r[2][Xr],coner_r[2][Yr],coner_r[3][Xr],coner_r[3][Yr])+
	 cal_area_trig(coner_r[1][Xr],coner_r[1][Yr],coner_r[2][Xr],coner_r[2][Yr],coner_r[3][Xr],coner_r[3][Yr]);
	 
	 //plan way
	 u8 way;
	 if(size[0]>size[1])
		 way=0;
	 else
		 way=1;
	 
	 //plan leg
	 u8 can_leg_id[2];
	 
	 switch(way_sel)
	 {
		//f
		case 1:
		if(way==0)
		{can_leg_id[0]=3;can_leg_id[1]=4;}
		else
		{can_leg_id[0]=1;can_leg_id[1]=2;}	
		break;
		//r
		case 2:
		if(way==0)
		{can_leg_id[0]=1;can_leg_id[1]=3;}
		else
		{can_leg_id[0]=2;can_leg_id[1]=4;}	
		break;
		//b
		case 3:
		if(way==0)
		{can_leg_id[0]=2;can_leg_id[1]=1;}
		else
		{can_leg_id[0]=4;can_leg_id[1]=3;}	
		break;
		//l
		case 4:
		if(way==0)
		{can_leg_id[0]=4;can_leg_id[1]=2;}
		else
		{can_leg_id[0]=3;can_leg_id[1]=1;}	
		break;
	 }	 
	 
	 //decide final leg				 
		static u8 flag1=0;//
	 if(flag1==0)
    {	flag1=1;return can_leg_id[0];}		
		else
		{	flag1=0;return can_leg_id[1];}			
}	


void leg_tar_est_global(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt,u8 fake)
{
u8 id=leg->sys.id;
float band_x,band_y,range_limit;
float tar_yaw=in->spd_yaw;
float spd=in->spd;
float tar_x,tar_y,tar_z;	
float off_x,off_y;
float yaw_in=To_180_degrees(brain.spd_yaw);
float yaw_temp=brain.sys.yaw_trig;
u8 way_sel;
u8 fp_point_id[4];//L  R
 if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)//r
 {fp_point_id[0]=1;fp_point_id[1]=2;   fp_point_id[2]=3;fp_point_id[3]=4;way_sel=2;}
 else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)//l
 {fp_point_id[0]=4;fp_point_id[1]=3;   fp_point_id[2]=2;fp_point_id[3]=1;way_sel=4;}
 else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))//f
 {fp_point_id[0]=3;fp_point_id[1]=1;   fp_point_id[2]=4;fp_point_id[3]=2;way_sel=1;}
 else//b
 {fp_point_id[0]=2;fp_point_id[1]=4;   fp_point_id[2]=1;fp_point_id[3]=3;way_sel=3;}
 float off_k=1;
 if(id==fp_point_id[2]||id==fp_point_id[3])
	 off_k=1-k_off;
 else
   off_k=1+k_off;
float off_yaw=0;
if(leg->sys.id==1||leg->sys.id==2)
off_x=LIMIT(leg->pos_now[2].x-leg->sys.init_end_pos.x,0,leg->sys.off_local[0])*fabs(cos(off_yaw*ANGLE_TO_RADIAN));
else
off_x=LIMIT(leg->pos_now[2].x-leg->sys.init_end_pos.x,-leg->sys.off_local[0],0)*fabs(cos(off_yaw*ANGLE_TO_RADIAN));	
if(leg->sys.id==1||leg->sys.id==3)
off_y=LIMIT(leg->pos_now[2].y-leg->sys.init_end_pos.y,0,leg->sys.off_local[1])*fabs(sin(off_yaw*ANGLE_TO_RADIAN));
else
off_y=LIMIT(leg->pos_now[2].y-leg->sys.init_end_pos.y,-leg->sys.off_local[1],0)*fabs(sin(off_yaw*ANGLE_TO_RADIAN));	

float spd_wx,spd_wy;
float tar_w;
float min=1.45;
if(brain.tar_w<min&&brain.tar_w>0)
	tar_w=min;
else if(brain.tar_w>-min&&brain.tar_w<0)
	tar_w=-min;
else
	tar_w=brain.tar_w;
float x_temp=fabs(sin((90-brain.sys.yaw_trig)*ANGLE_TO_RADIAN))*tar_w*k_rad;
float y_temp=fabs(cos((90-brain.sys.yaw_trig)*ANGLE_TO_RADIAN))*tar_w*k_rad;
	switch(id)
	{
	case 1:
	spd_wx=x_temp;
	spd_wy=-y_temp;
	break;
	case 2:
	spd_wx=-x_temp;
	spd_wy=-y_temp;
	break;
	case 3:
	spd_wx=x_temp;
	spd_wy=y_temp;
	break;
	case 4:
	spd_wx=-x_temp;
	spd_wy=y_temp;
	break;
	}
	

tar_x=spd_wx+sin(tar_yaw*ANGLE_TO_RADIAN)*(spd)*off_k*in->sys.k_spd_to_range;//,-2*brain.sys.leg_move_range1[1],2*brain.sys.leg_move_range1[1]);//,-in->sys.leg_move_range[Xr],in->sys.leg_move_range[Xr]);
tar_y=spd_wy+cos(tar_yaw*ANGLE_TO_RADIAN)*(spd)*off_k*in->sys.k_spd_to_range;//,-2*brain.sys.leg_move_range1[1],2*brain.sys.leg_move_range1[1]);//,-in->sys.leg_move_range[Yr],in->sys.leg_move_range[Yr]);
float tempx,tempy,tempz;
float tempx1,tempy1,tempz1;
float tempx2,tempy2,tempz2;
if(trig_use_now_pos){
tempx=leg->sys.init_end_pos.x*k_trig1+tar_x+RANDOM+off_x*cos(off_yaw*ANGLE_TO_RADIAN)*en_off_trig;
tempy=leg->sys.init_end_pos.y*k_trig1+tar_y+RANDOM+off_y*sin(off_yaw*ANGLE_TO_RADIAN)*en_off_trig;
}else{
tempx=leg->sys.init_end_pos.x*k_trig1+RANDOM;//leg->pos_now[2].x+tar_x+RANDOM;
tempy=leg->sys.init_end_pos.y*k_trig1+RANDOM;//leg->pos_now[2].y+tar_y+RANDOM;
}
tempz=brain.tar_h*h_k2;

//LIMIT
limit_move_range_tangle( id,leg->sys.init_end_pos.x,leg->sys.init_end_pos.y, tempx,tempy,in->sys.min_range,in->sys.max_range,&tempx1,&tempy1);
tempz1=tempz;

tempx2=tempx1;tempy2=tempy1;tempz2=tempz1;
//tempx2=tempx;tempy2=tempy;
//limit_trig_pos_of_leg(tempx1,tempy1,tempz1,leg[id].sys.limit.x,leg[id].sys.limit.y,leg[id].sys.limit.z,&tempx2,&tempy2,&tempz2);
//limit_range_leg(tempx1,tempy1,leg[1].sys.limit.x,leg[1].sys.limit.y,&tempx2,&tempy2);

if(fake){
	brain.global.fake_tar_pos.x=tempx2;
	brain.global.fake_tar_pos.y=tempy2;
	brain.global.fake_tar_pos.z=tempz2;
}
else{
leg->pos_tar_trig[2].x=tempx2;
leg->pos_tar_trig[2].y=tempy2;
leg->pos_tar_trig[2].z=tempz2+RANDOM;

if(brain.rst_all_soft>0)
{
brain.rst_all_soft++;
leg->pos_tar_trig[2].x=leg->sys.init_end_pos.x*k_trig1+RANDOM;//+off_x*cos(tar_yaw/ 57.3f)*en_off_trig;
leg->pos_tar_trig[2].y=leg->sys.init_end_pos.y*k_trig1+RANDOM;//+off_y*sin(tar_yaw/ 57.3f)*en_off_trig;
leg->pos_tar_trig[2].z=brain.tar_h*h_k2;
}
if(brain.rst_all_soft>4)
brain.rst_all_soft=0;
}
}

//------------------------fall control
float k_fall=1.6;
float k_add=0.5;
void fall_treat(float dt,float *out)
{ static u8 state;
	float fall_control[4];
	if(fabs(brain.att[1])>8.89&&fabs(mpu6050_fc.Gyro_deg.x)>10&&brain.fall==0){
	
		if(brain.att[1]<0)//r
		{
		out[0]=-fabs(LIMIT(mpu6050_fc.Gyro_deg.x,-20,0)*k_fall);	
		out[1]=-fabs(LIMIT(mpu6050_fc.Gyro_deg.x,-20,0)*k_fall);		
		out[2]=0;	
		out[3]=0;	
		//leg[2].pos_tar[2].x-=k_add*dt;
		//leg[3].pos_tar[2].x-=k_add*dt;	
		}	
		else//l
		{
		out[2]=fabs(LIMIT(mpu6050_fc.Gyro_deg.x,0,20)*k_fall);	
		out[3]=fabs(LIMIT(mpu6050_fc.Gyro_deg.x,0,20)*k_fall);		
		out[0]=0;	
		out[1]=0;		
		//leg[1].pos_tar[2].x+=k_add*dt;
		//leg[2].pos_tar[2].x+=k_add*dt;	
		}		
		
	}
	else 
	{
		out[0]=out[1]=out[2]=out[3]=0;
	}
	
}

float dead_1[2]={3,3};
float k_fall1[3]={0.6,0.4};
float flt_fall1;
void fall_treat1(float dt,float *out)
{ static u8 state;
	float fall_control[4];
	if(fabs(brain.att[1])>=dead_1[0]&&fabs(mpu6050_fc.Gyro_deg.x)>dead_1[1]&&brain.fall==0){
	
	  out[0]=my_deathzoom(LIMIT(brain.att[1],-20,20),1)*k_fall1[0]+(LIMIT(mpu6050_fc.Gyro_deg.x,-33,33)*k_fall1[1]);	
		out[1]=my_deathzoom(LIMIT(brain.att[1],-20,20),1)*k_fall1[0]+(LIMIT(mpu6050_fc.Gyro_deg.x,-33,33)*k_fall1[1]);	
		out[2]=(my_deathzoom(LIMIT(brain.att[1],-20,20),1)*k_fall1[0]+(LIMIT(mpu6050_fc.Gyro_deg.x,-33,33)*k_fall1[1]));		
		out[3]=(my_deathzoom(LIMIT(brain.att[1],-20,20),1)*k_fall1[0]+(LIMIT(mpu6050_fc.Gyro_deg.x,-33,33)*k_fall1[1]));	
   if(brain.att[1]<0)//r
		{
		out[0]=-fabs(out[0]);	
		out[1]=-fabs(out[1]);		
		out[2]=0;	
		out[3]=0;	
		}	
		else//l
		{
		out[2]=fabs(out[2]);	
		out[3]=fabs(out[3]);		
		out[0]=0;	
		out[1]=0;		
		}		
	}
	else 
	{
		out[0]=out[1]=out[2]=out[3]=0;
	}
	
}