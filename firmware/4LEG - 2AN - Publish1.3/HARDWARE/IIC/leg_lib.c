#include "include.h" 
//--------------------------LEG_LIB---------------------------
#define USE_LISENCE 0
void cpuidGetId(void);

//三角形重心
void cal_cog_tri(float x1,float y1,float x2,float y2,float x3,float y3,float *cx,float *cy)
{
  *cx=(x1+x2+x3)/3;
  *cy=(y1+y2+y3)/3;
}

//转换腿局部坐标系到全局机体坐标系
void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id)
{ u8 i;
	static u8 init;
	
	for(i=0;i<3;i++){
	inl->pos_now_brain[i].x=inl->pos_now[i].x+in->sys.leg_local[id].x;
  inl->pos_now_brain[i].y=inl->pos_now[i].y+in->sys.leg_local[id].y;
	inl->pos_now_brain[i].z=inl->pos_now[i].z+in->sys.leg_local[id].z;
	}
}
//判断脚到达初始化的位置
u8 check_leg_near_init(float ero)
{
 u8 i;
 float dis[5];	
	for(i=1;i<5;i++)
	 {
	   dis[i]=cal_dis_of_points(leg[i].pos_now[2].x,leg[i].pos_now[2].y,
		 leg[i].sys.init_end_pos.x,leg[i].sys.init_end_pos.y);
	 }
#if USE_LISENCE	 
	if(license.state==0) 
	  return 0xf;
#endif	
  if(dis[1]<ero&&dis[2]<ero&&dis[3]<ero&&dis[4]<ero)
	  return 1;
	else 
		return 0;
}	

//获得当前脚在步态表中的标号
u8 get_next_leg_flag_in_list(u8 leg_id_now,u8 way)
{
 u8 *list,i;
 switch(way)
 {
	case 1:
	list=&trig_list_f[0];
	break;
	case 2:
	list=&trig_list_r[0];
	break;
	case 3:
	list=&trig_list_b[0];
	break;
	case 4:
	list=&trig_list_l[0];
	break;
 }
  for(i=1;i<5;i++)
        if(*(list+i)==leg_id_now)
					 return i;
}


//跨腿重复保护器
u8 leg_repeat_protect(u8 id,u8 last_move_id,u8 last_last_move_id,float yaw,float yaw_trig){
u8 i;	
u8 trig_list_f[5]=		 {0,1,4,3,2};
u8 trig_list_r[5]=		 {0,2,3,1,4};
u8 trig_list_b[5]=		 {0,4,1,2,3};
u8 trig_list_l[5]=		 {0,3,2,4,1};	
float yaw_in=To_180_degrees(yaw);
float yaw_temp=yaw_trig;
u8 out;
u8 temp_id;
				if(id==last_move_id)
				{
				  for(i=1;i<5;i++){
					if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
					temp_id=trig_list_r[i];
					else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
					temp_id=trig_list_l[i];
					else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
					temp_id=trig_list_f[i];
					else
					temp_id=trig_list_b[i];
					if(temp_id==id)
						break;
				  }
					if(i>=4)
						if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
						out= trig_list_r[1];
						else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
						out= trig_list_l[1];
						else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
						out= trig_list_f[1];
						else
						out= trig_list_b[1];
					else
						if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
						out= trig_list_r[i+1];
						else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
						out= trig_list_l[i+1];
						else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
						out= trig_list_f[i+1];
						else
						out= trig_list_b[i+1];
				}else	
        out=id;
				
			if(out==last_move_id)
			for(i=1;i<5;i++)
			{
			if(i!=last_move_id)
			{out=i;break;}
			}
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
			return out;	
}

//跨腿重复保护器
u8 leg_repeat_protect1(u8 id,u8 last_move_id,u8 last_last_move_id,float yaw,float yaw_trig){
u8 i;	
u8 trig_list_f[5]=		 {0,1,4,3,2};
u8 trig_list_r[5]=		 {0,2,3,1,4};
u8 trig_list_b[5]=		 {0,4,1,2,3};
u8 trig_list_l[5]=		 {0,3,2,4,1};	
float yaw_in=To_180_degrees(yaw);
float yaw_temp=yaw_trig;
u8 temp_id;
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		

				if(id==last_move_id||id==last_last_move_id)
				{
				  for(i=1;i<5;i++)
					 {
					   if(i!=last_move_id&&i!=last_last_move_id)
						 return id=i;
					 }
				}else	
        return id;
}

//点沿矢量对称缩放
void resize_point_with_arrow(float x,float y,float cx,float cy,float yaw,float k,float *nx,float *ny)
{
float kc,bc;	
float kp_90,bp_90;	
float cro_x,cro_y;
if(k==1)
{
*nx=x;
*ny=y;	
}else{	
line_function_from_arrow( cx, cy, yaw, &kc,&bc);
line_function90_from_arrow(x, y, yaw, &kp_90,&bp_90);
cross_point_of_lines(kc,bc,kp_90,bp_90,&cro_x,&cro_y);
float dis=cal_dis_of_points(x,y,cro_x,cro_y)*(1-k);	
u8 flag=check_point_front_arrow( cx, cy, x, y, yaw+90);
float yaw1;
if(flag)
yaw1=yaw+90;
else
yaw1=yaw-90;

*nx=x+sin(yaw1*ANGLE_TO_RADIAN)*dis;
*ny=y+cos(yaw1*ANGLE_TO_RADIAN)*dis;	
}	
}
//跨脚范围修正
void limit_trig_pos_of_leg(float cx,float cy,float cz,float rx,float ry,float rz,float *x,float *y,float *z)
{
 u8 flag[3];
 float k[3],b[3];
 float dis[2];
 float jiaodiaoxy[3][3]={0};
 float jiaodiaoxz[3][3]={0};
 float jiaodiaoyz[3][3]={0};	
 
	flag[0]=in_circle(0,0,rx,ry,cx,cy);
	flag[1]=in_circle(0,0,rz,ry,cz,cy);
	flag[2]=in_circle(0,0,rx,rz,cx,cz);
 if(flag[0]&&flag[1]&&flag[2])
 {
  *x=cx;
	*y=cy;
  *z=cz;
 }else
 {
 //xy
   k[0]=cy/(cx+0.00001);
	 //计算速度直线与椭圆交点
	 float temp=sqrt(pow(rx,2)/(1+pow(rx*k[0]/ry,2)));
	 //判断速度方向交点符号
	 jiaodiaoxy[0][Xr]=temp; 
	 jiaodiaoxy[1][Xr]=-temp; 
	 jiaodiaoxy[0][Yr]=k[0]*jiaodiaoxy[0][Xr];
	 jiaodiaoxy[1][Yr]=k[0]*jiaodiaoxy[1][Xr];
	 dis[0]=cal_dis_of_points(cx,cy,jiaodiaoxy[0][Xr],jiaodiaoxy[0][Yr]);
   dis[1]=cal_dis_of_points(cx,cy,jiaodiaoxy[1][Xr],jiaodiaoxy[1][Yr]);
	 if(dis[0]<dis[1])
	 {jiaodiaoxy[2][Xr]=jiaodiaoxy[0][Xr];jiaodiaoxy[2][Yr]=jiaodiaoxy[0][Yr];}
	 else
	 {jiaodiaoxy[2][Xr]=jiaodiaoxy[1][Xr];jiaodiaoxy[2][Yr]=jiaodiaoxy[1][Yr];}
	//xz 
   cal_jiao_of_tuo_and_line(cx,cz,rx,rz,90,&jiaodiaoxz[0][Xr],&jiaodiaoxz[0][Zr],&jiaodiaoxz[1][Xr],&jiaodiaoxz[1][Zr]);
	 dis[0]=cal_dis_of_points(cx,cz,jiaodiaoxz[0][Xr],jiaodiaoxz[0][Zr]);
   dis[1]=cal_dis_of_points(cx,cz,jiaodiaoxz[1][Xr],jiaodiaoxz[1][Zr]);
	 if(dis[0]<dis[1])
	 {jiaodiaoxz[2][Xr]=jiaodiaoxz[0][Xr];jiaodiaoxz[2][Zr]=jiaodiaoxz[0][Zr];}
	 else
	 {jiaodiaoxz[2][Xr]=jiaodiaoxz[1][Xr];jiaodiaoxz[2][Zr]=jiaodiaoxz[1][Zr];}
	//yz 
	 cal_jiao_of_tuo_and_line(cy,cz,ry,rz,90,&jiaodiaoyz[0][Yr],&jiaodiaoyz[0][Zr],&jiaodiaoyz[1][Yr],&jiaodiaoyz[1][Zr]);
	 dis[0]=cal_dis_of_points(cy,cz,jiaodiaoyz[0][Yr],jiaodiaoyz[0][Zr]);
   dis[1]=cal_dis_of_points(cy,cz,jiaodiaoyz[1][Yr],jiaodiaoyz[1][Zr]);
	 if(dis[0]<dis[1])
	 {jiaodiaoyz[2][Yr]=jiaodiaoyz[0][Yr];jiaodiaoyz[2][Zr]=jiaodiaoyz[0][Zr];}
	 else
	 {jiaodiaoyz[2][Yr]=jiaodiaoyz[1][Yr];jiaodiaoyz[2][Zr]=jiaodiaoyz[1][Zr];}
	
  if(fabs(jiaodiaoxz[2][Xr])<fabs(jiaodiaoxy[2][Xr]))	 
	*x=jiaodiaoxz[2][Xr];
	else
	*x=jiaodiaoxy[2][Xr];	
	
	if(fabs(jiaodiaoyz[2][Yr])<fabs(jiaodiaoxy[2][Yr]))	 
	*y=jiaodiaoyz[2][Yr];
	else
	*y=jiaodiaoxy[2][Yr];	
	
	*z=(jiaodiaoxz[2][Zr]+jiaodiaoyz[2][Zr])/2;	
 }
}
//点与球体的交点
u8 arrow_check_to_bow(float cx,float cy,float cz,float rx,float ry,float rz,float *x,float *y,float *z)
{
  float yaw[3];
  u8 flag[3];
	float k[3],b[3];
	float jiao1[3][3],jiao2[3][3];
	float jiaon[3][3]={0};
	float dis[2];
	flag[0]=in_circle(0,0,rx,ry,cx,cy);
	flag[1]=in_circle(0,0,rz,ry,cz,cy);
	flag[2]=in_circle(0,0,rx,rz,cx,cz);
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
	if(flag[0]&&flag[1]&&flag[2]){
  *x=cx;
	*y=cy;
  *z=cz;
  return 1; 		
	}
  else
  {
	//xy
  limit_range_leg( cx, cy,rx,ry,&jiaon[0][Xr],&jiaon[0][Yr]);
	//xz
	limit_range_leg( cx, cz,rx,rz,&jiaon[1][Xr],&jiaon[1][Zr]);
  //yz		
  limit_range_leg( cy, cz,ry,rz,&jiaon[2][Yr],&jiaon[2][Zr]);
		
	*x=(jiaon[0][Xr]+jiaon[1][Xr])/2;
	*y=(jiaon[0][Yr]+jiaon[2][Yr])/2;	
	*z=(jiaon[2][Zr]+jiaon[1][Zr])/2;	
//	*x=jiaon[0][Xr];
//	*y=jiaon[0][Yr];
//	*z=jiaon[2][Zr];
	 return 0;	
	}		
}


//两点求直线方程
void line_function_from_two_point(float x1,float y1,float x2,float y2,float *k,float *b)
{ 
	float k_temp=0;
  *k=k_temp=(y1-y2)/(x1-x2+0.000001);
  *b=y1-k_temp*x1;
}	

//矢量求直线方程
void line_function_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw+0.000011;
	float k_temp=0;
  *k=k_temp=tan(tyaw*ANGLE_TO_RADIAN);
  *b=y-k_temp*x;
}	

//矢量垂线方程
void line_function90_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw+0.000011;
	float k_temp=0;
  *k=k_temp=-1/tan(tyaw*ANGLE_TO_RADIAN);
  *b=y-k_temp*x;
}	

//点在直线上
u8 check_point_on_line(float x,float y,float k,float b,float err)
{ 
	float temp=k*x+b-y;
	if(ABS(temp)<err)
		return 1;
	else
		return 0;
}	

//两直线交点
u8 cross_point_of_lines(float k1,float b1,float k2,float b2,float *x,float *y)
{ 
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
	if(ABS(k1-k2)<0.001){
		*x=*y=0;
		return 0;}
	float x_temp;
	*x=x_temp=(b1-b2)/(k2-k1+0.00001);
//	if(fabs(k1)>10000&&fabs(b1)>10000)
//  *y=0;
//  else	
	*y=k1*x_temp+b1;
	
	return 1;
}	

//点在点矢量方向前
u8 check_point_front_arrow(float x,float y,float cx,float cy,float yaw)
{ 
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
  float tyaw=90-yaw+0.000011;
	float kc_90=-1/tan(tyaw*ANGLE_TO_RADIAN);
	float bc_90=cy-kc_90*cx;
	float cx_t=cx+sin(yaw*ANGLE_TO_RADIAN)*1,cy_t=cy+cos(yaw*ANGLE_TO_RADIAN)*1;
	float flag[2];
	flag[0]=kc_90*cx_t+bc_90-cy_t;
	flag[1]=kc_90*x+bc_90-y;
	if((flag[0]>0&&flag[1]>0)||(flag[0]<0&&flag[1]<0))
	return 1;
	else 
  return 0;
}	

//判断两点在线同一侧
u8 check_points_same_side(float x1,float y1,float x2,float y2,float k,float b)
{ 
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
	float flag[2];
	flag[0]=k*x1+b-y1;
	flag[1]=k*x2+b-y2;
	if(flag[0]*flag[1]>0)
	return 1;
	else 
  return 0;
}	

//点矢量与直线交点
u8 check_cross_arrow_line(float cx,float cy,float yaw,float k,float b,float *x,float *y)
{ 
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
  float tyaw=90-yaw+0.000011;
	float kc=tan(tyaw*ANGLE_TO_RADIAN);
	float bc=cy-kc*cx;
	float cro_x,cro_y;
	u8 flag;	
	flag=cross_point_of_lines(k,b,kc,bc,&cro_x,&cro_y);
	*x=cro_x;
	*y=cro_y;
	
	if(flag==0||fabs(cro_y)>100||fabs(cro_x)>100)
		return 0;
	//有交点且在前方
	if(check_point_front_arrow(cro_x,cro_y, cx, cy, yaw))	
	return 1;
	else{
	*x=*y=0;	
	return 0;
	}
}	

//点矢量垂线与直线交点
u8 check_cross_arrow90_line(float cx,float cy,float yaw,float k,float b,float *x,float *y)
{ 
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
  float tyaw=90-yaw+0.000011;
	float kc=tan(tyaw*ANGLE_TO_RADIAN);
	float kc_90=-1/(kc+0.00001);
	float bc_90=cy-kc_90*cx;
	float cro_x,cro_y;
	
	u8 flag;	
	flag=cross_point_of_lines(k,b,kc_90,bc_90,&cro_x,&cro_y);
	*x=cro_x;
	*y=cro_y;
	
	if(flag==0){
	*x=*y=0;	
  return 0;}
  else
	return 1;
}	
//计算三角形重心坐标
void cal_center_of_trig(float x1,float y1,float x2,float y2,float x3,float x4,float *cx,float *cy)
{

}	

//计算两点距离
float cal_dis_of_points(float x1,float y1,float x2,float y2)
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}	

//判断一个点在椭圆内部
u8 in_circle(float cx,float cy,float d_short,float d_long,float x,float y)
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
   float temp=pow(x-cx,2)/pow(d_short,2)+pow(y-cy,2)/pow(d_long,2);
   if(temp>1)//外面
		 return 0;
	 else 
		 return 1;
}

//点到直线距离
float dis_point_to_line(float x,float y,float k,float b)
{ 
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
  float k_90=-1/(k+0.000011);
	float b_90=y-k_90*x;
	float cx,cy;
	cross_point_of_lines(k,b,k_90,b_90,&cx,&cy);
	
	return cal_dis_of_points(x,y,cx,cy);
}	

//将机体坐标转换到全局坐标
void conver_body_to_global(float bx,float by,float *gx,float *gy)
{
 *gx=bx+brain.sys.leg_local[1].x-leg[4].pos_now[2].x+brain.sys.off_cor[Xr];
 *gy=by+brain.sys.leg_local[1].y-leg[4].pos_now[2].y+brain.sys.off_cor[Yr];
}

//计算点矢量两侧对称点
void two_point_between_arror(float cx,float cy,float yaw,float *x1,float *y1,float *x2,float *y2,float dis)
{
  float k_90,b_90;
  float jiao[2][2];
	line_function90_from_arrow(cx,cy,yaw,&k_90,&b_90);
	cal_jiao_of_tuo_and_line(cx,cy,yaw+90,dis,dis,&jiao[0][Xr],&jiao[0][Yr],&jiao[1][Xr],&jiao[1][Yr]); 
	*x1=jiao[0][Xr];
  *y1=jiao[0][Yr];
  *x2=jiao[1][Xr];
  *y2=jiao[1][Yr];
}	


void limit_range_leg(float x,float y,float min,float max,float *xout,float *yout)
{
  u8 flag;
	float yaw;
	float jiaodiao[2][2]={0};
	float temp;
	float k;
	float dis[2];
   flag=in_circle(0,0,min,max,x,y);
   if(flag==1){
		 *xout=x;
	   *yout=y;
	 }else 
	 {
	   k=y/(x+0.00001);
	
	 //计算速度直线与椭圆交点
	 float temp=sqrt(pow(min,2)/(1+pow(min*k/max,2)));
	 //判断速度方向交点符号
	 jiaodiao[0][Xr]=temp; 
	 jiaodiao[1][Xr]=-temp; 
	 jiaodiao[0][Yr]=k*jiaodiao[0][Xr];
	 jiaodiao[1][Yr]=k*jiaodiao[1][Xr];
	 dis[0]=cal_dis_of_points(x,y,jiaodiao[0][Xr],jiaodiao[0][Yr]);
   dis[1]=cal_dis_of_points(x,y,jiaodiao[1][Xr],jiaodiao[1][Yr]);
	 if(dis[0]<dis[1])
	 {*xout=jiaodiao[0][Xr];*yout=jiaodiao[0][Yr];}
	 else
	 {*xout=jiaodiao[1][Xr];*yout=jiaodiao[1][Yr];}
	 }
}	


//移动区域限幅度 方框
void limit_move_range_tangle(u8 id,float cx,float cy,float x,float y,float min,float max,float *outx,float *outy)
{
  float tangle[4][2];
	float length[5];
	
	switch(id){
	case 1:
	length[1]=brain.sys.leg_move_range1[1];
	length[2]=brain.sys.leg_move_range1[2];
	length[3]=brain.sys.leg_move_range1[3];
	length[4]=brain.sys.leg_move_range1[4];
	break;
	case 2:
	length[1]=brain.sys.leg_move_range1[3];
	length[2]=brain.sys.leg_move_range1[2];
	length[3]=brain.sys.leg_move_range1[1];
	length[4]=brain.sys.leg_move_range1[4];
	break;
	case 3:
	length[1]=brain.sys.leg_move_range1[1];
	length[2]=brain.sys.leg_move_range1[4];
	length[3]=brain.sys.leg_move_range1[3];
	length[4]=brain.sys.leg_move_range1[2];
	break;
	case 4:
	length[1]=brain.sys.leg_move_range1[3];
	length[2]=brain.sys.leg_move_range1[4];
	length[3]=brain.sys.leg_move_range1[1];
	length[4]=brain.sys.leg_move_range1[2];
	break;
  }
	tangle[0][Xr]=-length[4];tangle[0][Yr]=length[1];
	tangle[1][Xr]=length[2];tangle[1][Yr]=length[1];
	tangle[2][Xr]=length[2];tangle[2][Yr]=-length[3];
	tangle[3][Xr]=-length[4];tangle[3][Yr]=-length[3];
	
	*outx=LIMIT(x,tangle[0][Xr]+cx,tangle[1][Xr]+cx);
	*outy=LIMIT(y,tangle[2][Yr]+cy,tangle[1][Yr]+cy);
}	

//判断点在移动区域内部tangle
u8 check_in_move_range_tangle(u8 id,float x,float y,float cx,float cy,float min,float max,float *min_dis)
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
		float tangle[4][2];
		u8 flag[3];
		float length[5];
		switch(id){
		case 1:
		length[1]=brain.sys.leg_move_range1[1];
		length[2]=brain.sys.leg_move_range1[2];
		length[3]=brain.sys.leg_move_range1[3];
		length[4]=brain.sys.leg_move_range1[4];
		break;
		case 2:
		length[1]=brain.sys.leg_move_range1[3];
		length[2]=brain.sys.leg_move_range1[2];
		length[3]=brain.sys.leg_move_range1[1];
		length[4]=brain.sys.leg_move_range1[4];
		break;
		case 3:
		length[1]=brain.sys.leg_move_range1[1];
		length[2]=brain.sys.leg_move_range1[4];
		length[3]=brain.sys.leg_move_range1[3];
		length[4]=brain.sys.leg_move_range1[2];
		break;
		case 4:
		length[1]=brain.sys.leg_move_range1[3];
		length[2]=brain.sys.leg_move_range1[4];
		length[3]=brain.sys.leg_move_range1[1];
		length[4]=brain.sys.leg_move_range1[2];
		break;
		}
		tangle[0][Xr]=-length[4]+cx;tangle[0][Yr]=length[1]+cy;
		tangle[1][Xr]=length[2]+cx;tangle[1][Yr]=length[1]+cy;
		tangle[2][Xr]=length[2]+cx;tangle[2][Yr]=-length[3]+cy;
		tangle[3][Xr]=-length[4]+cx;tangle[3][Yr]=-length[3]+cy;

	 flag[0]=inTrig2(x,y,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
													 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr]); 
	 
	 *min_dis=get_min_dis_arrow_to_tangle(x,y,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
											 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr]);
		
	 if(flag[0])
	  return 1;
	 else 
		return 0;
}

//判断点在移动区域内部
u8 check_in_move_range(u8 id,float x,float y,float cx,float cy,float min,float max)
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
		float tangle[4][2];
		float tar_x,tar_y;
		u8 flag[3];
	
	  float cx1=x-cx,cy1=y-cy;
	 switch (id)
	 {
		case 1:
		tar_x=cx1;
		tar_y=cy1;
		break;
		case 2:
		tar_x=cx1;
		tar_y=-cy1;
		break;
		case 4:
		tar_x=-cx1;
		tar_y=-cy1;
		break;
		case 3:
		tar_x=-cx1;
		tar_y=cy1;
		break;
	 }
	 
	 tangle[0][Xr]=-min;tangle[0][Yr]=max;
	 tangle[1][Xr]=max;tangle[1][Yr]=max;
	 tangle[2][Xr]=max;tangle[2][Yr]=-min;
	 tangle[3][Xr]=-min;tangle[3][Yr]=-min;
	 flag[0]=inTrig2(tar_x,tar_y,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
													 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr]); 
	 flag[1]=in_circle(0,0,max,max,tar_x,tar_y);
	 flag[2]=in_circle(0,0,min,min,tar_x,tar_y);
	 
	 if(flag[0]&&flag[1]&&flag[2]==1)
	  return 1;
	 else 
		return 0;
}


//计算移动区域与矢量的两个交点
void cal_jiao_of_range_and_line(u8 id,float cx,float cy,float min,float max,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{

	 float jiaodiao[2][2]={0};
   float yaw_use;	
	 float tangle[4][2];
	 float tar_x=0,tar_y=0;
	 u8 flag[3];
	 switch (id)
	 {
		case 1:
		yaw_use=yaw;
		break;
		case 2:
		yaw_use=180-yaw;
		break;
		case 3:
		yaw_use=-yaw;
		break;
		case 4:
		yaw_use=yaw+180;
		break;
	 }
	 
	 tangle[0][Xr]=-min;tangle[0][Yr]=max;
	 tangle[1][Xr]=max;tangle[1][Yr]=max;
	 tangle[2][Xr]=max;tangle[2][Yr]=-min;
	 tangle[3][Xr]=-min;tangle[3][Yr]=-min;

	  u8 sel=0;
		float yaw_t=To_180_degrees(yaw_use+0.01);
		 if(yaw_t>0&&yaw_t<90)
			 sel=1;
		 else if(yaw_t>90&&yaw_t<=180)
		   sel=2;
		 else if(yaw_t<0&&yaw_t>-90)
		   sel=4;
		 else 
			 sel=3;
		 float jiao1[2][2];
		 float jiao2[2][2];
		 switch(sel){
			 case 1:
			 cal_jiao_of_tuo_and_line(0,0,max,max,yaw_t,&jiao1[0][Xr],&jiao1[0][Yr],&jiao1[1][Xr],&jiao1[1][Yr]);
       cal_jiao_of_tuo_and_line(0,0,min,min,yaw_t,&jiao2[0][Xr],&jiao2[0][Yr],&jiao2[1][Xr],&jiao2[1][Yr]);
			 jiaodiao[0][Xr]=jiao1[0][Xr];
			 jiaodiao[0][Yr]=jiao1[0][Yr];
			 jiaodiao[1][Xr]=jiao2[1][Xr];
			 jiaodiao[1][Yr]=jiao2[1][Yr];
			 break; 
			 case 3:
			 cal_jiao_of_tuo_and_line(0,0,max,max,yaw_t,&jiao1[0][Xr],&jiao1[0][Yr],&jiao1[1][Xr],&jiao1[1][Yr]);
       cal_jiao_of_tuo_and_line(0,0,min,min,yaw_t,&jiao2[0][Xr],&jiao2[0][Yr],&jiao2[1][Xr],&jiao2[1][Yr]);
			 jiaodiao[0][Xr]=jiao2[0][Xr];
			 jiaodiao[0][Yr]=jiao2[0][Yr];
			 jiaodiao[1][Xr]=jiao1[1][Xr];
			 jiaodiao[1][Yr]=jiao1[1][Yr]; 
			 break;
			 default:
			 check_point_to_tangle(0,0,yaw_t,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
													 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr],
														&jiao1[0][Xr],&jiao1[0][Yr],&jiao1[1][Xr],&jiao1[1][Yr]);	 	
			 jiaodiao[0][Xr]=jiao1[0][Xr];
			 jiaodiao[0][Yr]=jiao1[0][Yr];
			 jiaodiao[1][Xr]=jiao2[1][Xr];
			 jiaodiao[1][Yr]=jiao2[1][Xr];
			 break;
			 
		 }
	  int flag3[2];
		switch (id)
		{
		case 1:
    flag3[0]=1;
		flag3[1]=1;
		break;
		case 2:
    flag3[0]=1;
		flag3[1]=-1;
		break;
		case 4:
    flag3[0]=-1;
		flag3[1]=-1;
		break;
		case 3:
    flag3[0]=-1;
		flag3[1]=1;
		break;
		}
	 jiaodiao[0][Xr]*=flag3[0];
	 jiaodiao[0][Yr]*=flag3[1];
	 jiaodiao[1][Xr]*=flag3[0];
	 jiaodiao[1][Yr]*=flag3[1];
		
	 jiaodiao[0][Xr]+=cx;jiaodiao[1][Xr]+=cx;
	 jiaodiao[0][Yr]+=cy;jiaodiao[1][Yr]+=cy;

   *jiao1_x=jiaodiao[0][Xr];
	 *jiao1_y=jiaodiao[0][Yr];
	 *jiao2_x=jiaodiao[1][Xr];
	 *jiao2_y=jiaodiao[1][Yr];
	 
}


//计算移动区域与矢量的两个交点 方框
void cal_jiao_of_range_and_line_tangle(u8 id,float cx,float cy,float min,float max,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{

	 float jiaodiao[2][2]={0};
   float yaw_use;	
	 float tangle[4][2];
	 float tar_x=0,tar_y=0;
	 u8 flag[3];
     float length[5];

	switch(id){
	case 1:
	length[1]=brain.sys.leg_move_range1[1];
	length[2]=brain.sys.leg_move_range1[2];
	length[3]=brain.sys.leg_move_range1[3];
	length[4]=brain.sys.leg_move_range1[4];
	break;
	case 2:
	length[1]=brain.sys.leg_move_range1[3];
	length[2]=brain.sys.leg_move_range1[2];
	length[3]=brain.sys.leg_move_range1[1];
	length[4]=brain.sys.leg_move_range1[4];
	break;
	case 3:
	length[1]=brain.sys.leg_move_range1[1];
	length[2]=brain.sys.leg_move_range1[4];
	length[3]=brain.sys.leg_move_range1[3];
	length[4]=brain.sys.leg_move_range1[2];
	break;
	case 4:
	length[1]=brain.sys.leg_move_range1[3];
	length[2]=brain.sys.leg_move_range1[4];
	length[3]=brain.sys.leg_move_range1[1];
	length[4]=brain.sys.leg_move_range1[2];
	break;
  }
	tangle[0][Xr]=-length[4];tangle[0][Yr]=length[1];
	tangle[1][Xr]=length[2];tangle[1][Yr]=length[1];
	tangle[2][Xr]=length[2];tangle[2][Yr]=-length[3];
	tangle[3][Xr]=-length[4];tangle[3][Yr]=-length[3];
	 
	 check_point_to_tangle(0,0,yaw,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
											 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr],
												&jiaodiao[0][Xr],&jiaodiao[0][Yr],&jiaodiao[1][Xr],&jiaodiao[1][Yr]);	 
	
			
	 jiaodiao[0][Xr]+=cx;jiaodiao[1][Xr]+=cx;
	 jiaodiao[0][Yr]+=cy;jiaodiao[1][Yr]+=cy;

   *jiao1_x=jiaodiao[0][Xr];
	 *jiao1_y=jiaodiao[0][Yr];
	 *jiao2_x=jiaodiao[1][Xr];
	 *jiao2_y=jiaodiao[1][Yr]; 
}
//点矢与方框最短距离
float get_min_dis_arrow_to_tangle(float x,float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4)
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
	u8 i;
	float k[4],b[4];
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x2,y2,x3,y3,&k[1],&b[1]);
	line_function_from_two_point(x3,y3,x4,y4,&k[2],&b[2]);
	line_function_from_two_point(x4,y4,x1,y1,&k[3],&b[3]);
  float dis[4];
  dis[0]=dis_point_to_line(x,y,k[0],b[0]);
  dis[1]=dis_point_to_line(x,y,k[1],b[1]);
	dis[2]=dis_point_to_line(x,y,k[2],b[2]);
	dis[3]=dis_point_to_line(x,y,k[3],b[3]);
	float min_dis=dis[0];
  for(i=0;i<4;i++)	
	  if(dis[i]<min_dis)
	     min_dis=dis[i];
		
	return min_dis;
}

//点矢量与四边形交点
u8 check_point_to_tangle(float x,float y,float yaw,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4
	,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
	u8 i,j=0;
  float k[4],b[4];
  float kc,bc;
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x2,y2,x3,y3,&k[1],&b[1]);
	line_function_from_two_point(x3,y3,x4,y4,&k[2],&b[2]);
	line_function_from_two_point(x4,y4,x1,y1,&k[3],&b[3]);
  line_function_from_arrow(x,y,yaw,&kc,&bc);
	u8 flag[4];
	float cro_x[4],cro_y[4];
  flag[0]=cross_point_of_lines(kc,bc,k[0],b[0],&cro_x[0],&cro_y[0]);
	flag[1]=cross_point_of_lines(kc,bc,k[1],b[1],&cro_x[1],&cro_y[1]);
	flag[2]=cross_point_of_lines(kc,bc,k[2],b[2],&cro_x[2],&cro_y[2]);
	flag[3]=cross_point_of_lines(kc,bc,k[3],b[3],&cro_x[3],&cro_y[3]);
  
	
	if(flag[0]||flag[1]||flag[2]||flag[3]==1)
	{	
	float jiaodiao1[2][2];
	for(i=0;i<4;i++)
		{
		  if(flag[i]&&(fabs(cro_x[i])<fabs(x2)+fabs(x1))&&(fabs(cro_y[i])<fabs(y2)+fabs(y3)))
			{ jiaodiao1[j][Xr]=cro_x[i];jiaodiao1[j][Yr]=cro_y[i];
        j++;
				if(j>2)
					break;
      }				
		}
	
	if(j<1)
		return 0;
	
  u8 flag2;	
	flag2=check_point_front_arrow(jiaodiao1[0][Xr],jiaodiao1[0][Yr],x,y,yaw);
		
	 if(flag2){
	 *jiao1_x=jiaodiao1[0][Xr];
	 *jiao1_y=jiaodiao1[0][Yr];
	 *jiao2_x=jiaodiao1[1][Xr];
	 *jiao2_y=jiaodiao1[1][Yr];}
	 else
		{
	 *jiao1_x=jiaodiao1[1][Xr];
	 *jiao1_y=jiaodiao1[1][Yr];
	 *jiao2_x=jiaodiao1[0][Xr];
	 *jiao2_y=jiaodiao1[0][Yr];} 
	return 1;
	}else 
	return 0;
}
	


//点矢量与三角形交点
u8 check_point_to_trig(float x,float y,float yaw,float x1,float y1,float x2,float y2,float x3,float y3
	,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
	u8 i,j=0;
  float k[3],b[3];
  float kc,bc;
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x2,y2,x3,y3,&k[1],&b[1]);
	line_function_from_two_point(x3,y3,x1,y1,&k[2],&b[2]);
  line_function_from_arrow(x,y,yaw,&kc,&bc);
	u8 flag[3];
	float cro_x[3],cro_y[3];
  flag[0]=cross_point_of_lines(kc,bc,k[0],b[0],&cro_x[0],&cro_y[0]);
	flag[1]=cross_point_of_lines(kc,bc,k[1],b[1],&cro_x[1],&cro_y[1]);
	flag[2]=cross_point_of_lines(kc,bc,k[2],b[2],&cro_x[2],&cro_y[2]);
  
	float dis[3];
	dis[0]=cal_dis_of_points(x1,y1,x,y);
	dis[1]=cal_dis_of_points(x2,y2,x,y);
	dis[2]=cal_dis_of_points(x3,y3,x,y);
	
	
	if(flag[0]||flag[1]||flag[2])
	{	
	float jiaodiao1[2][2],temp;
	for(i=0;i<3;i++)
		{
			temp=cal_dis_of_points(cro_x[i],cro_y[i],x,y);
		  if(flag[i]&&(temp<dis[0]||temp<dis[1]||temp<dis[2]))
			{ jiaodiao1[j][Xr]=cro_x[i];jiaodiao1[j][Yr]=cro_y[i];
        j++;
				if(j>2)
					break;
      }				
		}
	
	if(j<1)
		return 0;
	
  u8 flag2;	
	flag2=check_point_front_arrow(jiaodiao1[0][Xr],jiaodiao1[0][Yr],x,y,yaw);
		
	 if(flag2){
	 *jiao1_x=jiaodiao1[0][Xr];
	 *jiao1_y=jiaodiao1[0][Yr];
	 *jiao2_x=jiaodiao1[1][Xr];
	 *jiao2_y=jiaodiao1[1][Yr];}
	 else
		{
	 *jiao1_x=jiaodiao1[1][Xr];
	 *jiao1_y=jiaodiao1[1][Yr];
	 *jiao2_x=jiaodiao1[0][Xr];
	 *jiao2_y=jiaodiao1[0][Yr];} 
	return 1;
	}else 
	return 0;
}

//计算椭圆与矢量的两个交点
void cal_jiao_of_tuo_and_line(float cx,float cy,float d_short,float d_long,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{
   float tyaw=90-yaw+0.000011;
	 float tan_yaw=tan(tyaw*ANGLE_TO_RADIAN);
	 float jiaodiao[2][2]={0};
	 //计算速度直线与椭圆交点
	 float temp=sqrt(pow(d_short,2)/(1+pow(d_short*tan_yaw/d_long,2)));
	 //判断速度方向交点符号
	 if(yaw+0.000011>0&&yaw+0.000011<180)
	 {jiaodiao[0][Xr]=temp; jiaodiao[1][Xr]=-temp;}
	 else 
	 {jiaodiao[0][Xr]=-temp;jiaodiao[1][Xr]=temp;}
	 
	 jiaodiao[0][Yr]=tan_yaw*jiaodiao[0][Xr];
	 jiaodiao[1][Yr]=tan_yaw*jiaodiao[1][Xr];
	 
	 jiaodiao[0][Xr]+=cx;jiaodiao[1][Xr]+=cx;
	 jiaodiao[0][Yr]+=cy;jiaodiao[1][Yr]+=cy;
	 
   *jiao1_x=jiaodiao[0][Xr];
	 *jiao1_y=jiaodiao[0][Yr];
	 *jiao2_x=jiaodiao[1][Xr];
	 *jiao2_y=jiaodiao[1][Yr];
}

static void swap(float *a, float *b)  
{  
    int   c;  
     c = *a;  
    *a = *b;  
    *b =  c;  
}  

//计算三角形面积
float cal_area_trig(float ax,float ay,float bx,float by,float cx,float cy)
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		

float mx=cx-ax,my=cy-ay,nx=bx-ax,ny=by-ay;
float Lm= sqrt(mx*mx+my*my),Ln= sqrt(nx*nx+ny*ny),cosA=(mx*nx+my*ny)/Lm/Ln;
float sinA=sqrt(1-cosA*cosA);
float S_tri=0.5*Lm*Ln*sinA;

S_tri=(ax*by+bx*cy+cx*ay-ay*bx-by*cx-cy*ax)/2;

float dx=ax-bx;
float dy=ay-by;
float p1Len=sqrt(dx*dx+dy*dy);
dx=bx-cx;
dy=by-cy;
float p2Len=sqrt(dx*dx+dy*dy);
dx=cx-ax;
dy=cy-ay;
float p3Len=sqrt(dx*dx+dy*dy);

float p=(p1Len+p2Len+p3Len)/2;
S_tri=sqrt(p*(p-p1Len)*(p-p2Len)*(p-p3Len));
if(S_tri>0)
    return S_tri;
else
    return 0;
}	

//一个点在三角形内部
u8 inTrig(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3) {
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
  POS a,b,c,p;
  p.x=x;p.y=y;
	a.x=x1;a.y=y1;
	b.x=x2;b.y=y2;
	c.x=x3;c.y=y3;
	
  float signOfTrig = (b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x);
  float signOfAB = (b.x - a.x)*(p.y - a.y) - (b.y - a.y)*(p.x - a.x);
  float signOfCA = (a.x - c.x)*(p.y - c.y) - (a.y - c.y)*(p.x - c.x);
  float signOfBC = (c.x - b.x)*(p.y - b.y) - (c.y - b.y)*(p.x - b.x);

  u8 d1 = (signOfAB * signOfTrig > 0);
  u8 d2 = (signOfCA * signOfTrig > 0);
  u8 d3 = (signOfBC * signOfTrig > 0);

  return d1 && d2 && d3;
}
//计算当前稳态余量
float cal_steady_s(float cx,float cy,float x1,float y1,float x2,float y2,float x3, float y3 )
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
 float ST;
 float k[3],b[3];
 float D[3],temp;
 u8 i,intrig;
	intrig=inTrig(cx,cy,x1,y1,x2,y2,x3,y3);
	if(intrig){
	if(x2==x1)
		x2+=0.001;
	if(x3==x1)
		x3+=0.001;
	
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x3,y3,x2,y2,&k[1],&b[1]);
	line_function_from_two_point(x1,y1,x3,y3,&k[2],&b[2]);
	
  for(i=0;i<3;i++)
	D[i]=dis_point_to_line(cx,cy,k[i],b[i]);
	
	ST=D[0];
	for(i=0;i<3;i++)
	  if(D[i]<ST)
			 ST=D[i];
		
   return ST;
  }else
  return 0;	
}	

//计算当前稳态余量4leg
float cal_steady_s4(float cx,float cy,float x1,float y1,float x2,float y2,float x3, float y3 ,float x4,float y4 )
{
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
 float ST;
 float k[4],b[4];
 float D[4],temp;
 u8 i,intrig;
	
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x2,y2,x3,y3,&k[1],&b[1]);
	line_function_from_two_point(x3,y3,x4,y4,&k[2],&b[2]);
	line_function_from_two_point(x4,y4,x1,y1,&k[3],&b[3]);
  for(i=0;i<4;i++)
	D[i]=dis_point_to_line(cx,cy,k[i],b[i]);
	
	ST=D[0];
	for(i=0;i<4;i++)
	  if(D[i]<ST)
			 ST=D[i];
		
   return ST;	
}	
//判断一个点在四边形内部
//			  y
//	d----------b          /\
// 	     |                |
//			 O                L
//			 |                |
//	a----------c   x			\/
u8 segmentsIntr(POS b,POS c,POS d,POS a,float *x,float *y){  
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		

/** 1 解线性方程组, 求线段交点. **/  
// 如果分母为0 则平行或共线, 不相交  
    float denominator = (b.y - a.y)*(d.x - c.x) - (a.x - b.x)*(c.y - d.y);  
    if (denominator==0) {  
        return 0;  
    }  
   
// 线段所在直线的交点坐标 (x , y)      
     *x = ( (b.x - a.x) * (d.x - c.x) * (c.y - a.y)   
                + (b.y - a.y) * (d.x - c.x) * a.x   
                - (d.y - c.y) * (b.x - a.x) * c.x ) / denominator ;  
     *y = -( (b.y - a.y) * (d.y - c.y) * (c.x - a.x)   
                + (b.x - a.x) * (d.y - c.y) * a.y   
                - (d.x - c.x) * (b.y - a.y) * c.y ) / denominator;  
  
/** 2 判断交点是否在两条线段上 **/  
    if (  
        // 交点在线段1上  
        (*x - a.x) * (*x - b.x) <= 0 && (*y - a.y) * (*y - b.y) <= 0  
        // 且交点也在线段2上  
         && (*x - c.x) * (*x - d.x) <= 0 && (*y - c.y) * (*y - d.y) <= 0  
        ){  
  
        // 返回交点p  
        return 1;
    }  
    //否则不相交  
    return 0;  
  
}  
//一个点在四边形内部
u8 inTrig2(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4) {
#if USE_LISENCE	 
if(license.state==0) 
return 0xf;
#endif		
	
	u8 in_tri1=0,in_tri2=0,in_line_t12=0,in_tri3=0,in_tri4=0;
	in_tri1=inTrig(x,y, x1, y1, x2, y2, x3, y3);
  in_tri2=inTrig(x,y, x2, y2, x3, y3, x4, y4);
  in_tri3=inTrig(x,y, x1, y1, x3, y3, x4, y4);
  in_tri4=inTrig(x,y, x2, y2, x1, y1, x4, y4);
	POS b, c, d, a;
	b.x=x1;b.y=y1;
	c.x=x2;c.y=y2;
	d.x=x3;d.y=y3;
	a.x=x4;a.y=y4;
	float x_o,y_o;
	u8 cross;
	cross=segmentsIntr( b, c, d, a, &x_o, &y_o); 
	if(x_o==x&&y_o==y&&cross)
	in_line_t12=1;
  return in_tri1||in_tri2||in_tri3||in_tri4||in_line_t12;
}

//找到离xy最近的点
void find_closet_point(u8*min_id,float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4,u8 num) {
	u8 i,j;
	float dis[4],dis_id[4]={0,1,2,3};
	  dis[0]=sqrt(pow(x-x1,2)+pow(y-y1,2));
	  dis[1]=sqrt(pow(x-x2,2)+pow(y-y2,2));
	  dis[2]=sqrt(pow(x-x3,2)+pow(y-y3,2));
	  dis[3]=sqrt(pow(x-x4,2)+pow(y-y4,2));
    for (i = 0; i < num; i++)  
    {  
        //每一次由底至上地上升  
        for (j = num-1; j > i; j--)  
        {  
            if (dis[j] < dis[j-1])  
            {  
                swap(&dis[i], &dis[j]); 
								swap(&dis_id[i], &dis_id[j]);  
            }  
        }  
    }  
   if(num==3)
	 {
	  min_id[0]=dis_id[0];
		min_id[1]=dis_id[1]; 
	 }
	 else 
	 {
	  min_id[0]=dis_id[0];
		min_id[1]=dis_id[1]; 
	 } 
}


//----- DSE
/*-------------------------------------------------------
      Data Encryption Standard  56位密钥加密64位数据 
                  2011.10
--------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
/*布尔型变量*/
//--------------------------------------------------------------
LIS license;
char KEY_LOCK[9]={'3','2','3','8','3','2','6','6'}; 
char KEY_UNLOCK[9]={'3','2','3','8','3','2','6','6'}; 
void des_test(char *data,char *key_lock,char * key_unlock,unsigned int num);
u8 license_check(char *data,char *key_lock,char * lisence,unsigned int num);
void set_lisence(char *in)
{ u8 i=0;
  for(i=0;i<128;i++)
   license.lisence[i]=in[i];
	cpuidGetId();
}	

void cpuidGetId(void)
{  char RX_BUF[125]={0};
    license.id[0]= *(__IO u32*)(0x1FFF7A10);
    license.id[1]= *(__IO u32*)(0x1FFF7A14);
    license.id[2]= *(__IO u32*)(0x1FFF7A18);
		RX_BUF[0]=license.id[0];
		RX_BUF[1]=license.id[1];
		RX_BUF[2]=license.id[2];
	  
	  //des_test(RX_BUF,KEY_LOCK,KEY_UNLOCK,125);
	  license.state=license_check(RX_BUF,KEY_LOCK,license.lisence,125);
}


typedef enum
  {
    false = 0,
    true  = 1
  } bool;
// 对明文执行IP置换得到L0,R0 （L左32位,R右32位）               [明文操作]
static const char IP_Table[64]={             
	58,50,42,34,26,18,10, 2,60,52,44,36,28,20,12, 4,
	62,54,46,38,30,22,14, 6,64,56,48,40,32,24,16, 8,
	57,49,41,33,25,17, 9, 1,59,51,43,35,27,19,11, 3,
	61,53,45,37,29,21,13, 5,63,55,47,39,31,23,15, 7 
};

// 对迭代后的L16,R16执行IP逆置换,输出密文
static const char IPR_Table[64]={              
	40, 8,48,16,56,24,64,32,39, 7,47,15,55,23,63,31,
	38, 6,46,14,54,22,62,30,37, 5,45,13,53,21,61,29,
	36, 4,44,12,52,20,60,28,35, 3,43,11,51,19,59,27,
	34, 2,42,10,50,18,58,26,33, 1,41, 9,49,17,57,25	
};

/*--------------------------- 迭代法则 ----------------------------*/ 

// F函数,32位的R0进行E变换,扩为48位输出 (R1~R16)        [备用A]  [明文操作] 
static char E_Table[48]={
	32, 1, 2, 3, 4, 5, 4, 5, 6, 7, 8, 9,
	 8, 9,10,11,12,13,12,13,14,15,16,17,
    16,17,18,19,20,21,20,21,22,23,24,25,
    24,25,26,27,28,29,28,29,30,31,32, 1
};

// 子密钥K(i)的获取 密钥为K 抛弃第6,16,24,32,40,48,64位          [密钥操作] 
// 用PC1选位 分为 前28位C0,后28位D0 两部分  
static char PC1_Table[56]={
	57,49,41,33,25,17, 9, 1,58,50,42,34,26,18,
	10, 2,59,51,43,35,27,19,11, 3,60,52,44,36,
	63,55,47,39,31,23,15, 7,62,54,46,38,30,22,
	14, 6,61,53,45,37,29,21,13, 5,28,20,12, 4
};

// 对C0,D0分别进行左移,共16次,左移位数与下面对应                 [密钥操作]
static char Move_Table[16]={
	 1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1
};

// C1,D1为第一次左移后得到,进行PC2选位,得到48位输出K1   [备用B]   [密钥操作]     
static char PC2_Table[48]={
	14,17,11,24, 1, 5, 3,28,15, 6,21,10,
	23,19,12, 4,26, 8,16, 7,27,20,13, 2,
	41,52,31,37,47,55,30,40,51,34,33,48,
	44,49,39,56,34,53,46,42,50,36,29,32	
};

/*------------- F函数 备用A和备用B 异或 得到48位输出 ---------------*/ 

// 异或后的结果48位分为8组,每组6位,作为8个S盒的输入             [组合操作] 
// S盒以6位作为输入(8组),4位作为输出(4*(8组)=32位)
// S工作原理 假设输入为A=abcdef ,则bcde所代表的数是0-15之间的
// 一个数记为 X=bcde ,af代表的是0-3之间的一个数,记为 Y=af 
// 在S1的X列,Y行找到一个数Value,它在0-15之间,可以用二进制表示
// 所以为4bit (共32位)  
static char S_Box[8][4][16]={
	//S1
	14, 4,13, 1, 2,15,11, 8, 3,10, 6,12, 5, 9, 0, 7,
	 0,15, 7, 4,14, 2,13, 1,10, 6,12,11, 9, 5, 3, 8,
	 4, 1,14, 8,13, 6, 2,11,15,12, 9, 7, 3,10, 5, 0,
	15,12, 8, 2, 4, 9, 1, 7, 5,11, 3,14,10, 0, 6,13,
	//S2
	15, 1, 8,14, 6,11, 3, 4, 9, 7, 2,13,12, 0, 5,10,
	 3,13, 4, 7,15, 2, 8,14,12, 0, 1,10, 6, 9,11, 5,
	 0,14, 7,11,10, 4,13, 1, 5, 8,12, 6, 9, 3, 2,15,
	13, 8,10, 1, 3,15, 4, 2,11, 6, 7,12, 0, 5,14, 9,
	//S3
	10, 0, 9,14, 6, 3,15, 5, 1,13,12, 7,11, 4, 2, 8,
	13, 7, 0, 9, 3, 4, 6,10, 2, 8, 5,14,12,11,15, 1,
	13, 6, 4, 9, 8,15, 3, 0,11, 1, 2,12, 5,10,14, 7,
	 1,10,13, 0, 6, 9, 8, 7, 4,15,14, 3,11, 5, 2,12,
	//S4
	 7,13,14, 3, 0, 6, 9,10, 1, 2, 8, 5,11,12, 4,15,
	13, 8,11, 5, 6,15, 0, 3, 4, 7, 2,12, 1,10,14, 9,
	10, 6, 9, 0,12,11, 7,13,15, 1, 3,14, 5, 2, 8, 4,
	 3,15, 0, 6,10, 1,13, 8, 9, 4, 5,11,12, 7, 2,14,
	//S5
	 2,12, 4, 1, 7,10,11, 6, 8, 5, 3,15,13, 0,14, 9,
	14,11, 2,12, 4, 7,13, 1, 5, 0,15,10, 3, 9, 8, 6,
	 4, 2, 1,11,10,13, 7, 8,15, 9,12, 5, 6, 3, 0,14,
	11, 8,12, 7, 1,14, 2,13, 6,15, 0, 9,10, 4, 5, 3,
	//S6
	12, 1,10,15, 9, 2, 6, 8, 0,13, 3, 4,14, 7, 5,11,
	10,15, 4, 2, 7,12, 0, 5, 6, 1,13,14, 0,11, 3, 8,
	 9,14,15, 5, 2, 8,12, 3, 7, 0, 4,10, 1,13,11, 6,
   	 4, 3, 2,12, 9, 5,15,10,11,14, 1, 7, 6, 0, 8,13,
	//S7
	 4,11, 2,14,15, 0, 8,13, 3,12, 9, 7, 5,10, 6, 1,
	13, 0,11, 7, 4, 0, 1,10,14, 3, 5,12, 2,15, 8, 6,
	 1, 4,11,13,12, 3, 7,14,10,15, 6, 8, 0, 5, 9, 2,
	 6,11,13, 8, 1, 4,10, 7, 9, 5, 0,15,14, 2, 3,12,
	//S8
	13, 2, 8, 4, 6,15,11, 1,10, 9, 3,14, 5, 0,12, 7,
	 1,15,13, 8,10, 3, 7, 4,12, 5, 6,11, 0,14, 9, 2,
	 7,11, 4, 1, 9,12,14, 2, 0, 6,10,13,15, 3, 5, 8,
	 2, 1,14, 7, 4,10, 8,13,15,12, 9, 0, 3, 5, 6,11
};

// F函数 最后第二步,对S盒输出的32进行P置换                     [组合操作]
// 输出的值参与一次迭代:
// L(i)=R(i-1)
// R(i)=L(i-1)^f(R(i-1),K(i)) 异或 
static char P_Table[32]={
	16, 7,20,21,29,12,28,17, 1,15,23,26, 5,18,31,10,
	 2, 8,24,14,32,27, 3, 9,19,13,30, 6,22,11, 4,25
};

// 16个子密钥K(1~16) 
static bool SubKey[16][48]={0};                              
void SetKey(char KeyIn[8]);                         // 设置密钥
void PlayDes(char MesOut[8],char MesIn[8]);       // 执行DES加密
void KickDes(char MesOut[8],char MesIn[8]);             // 执行DES解密 
char MesHex_OUT[128]={0};         // 16个字符数组用于存放 64位16进制的密文
char MyMessage_Kick[125]={0};
void des_test(char *data,char *key_lock,char * key_unlock,unsigned int num)
{
    u8 i=0; 
    char MesHex[16]={0};         // 16个字符数组用于存放 64位16进制的密文
		char MesHexr[16]={0};
    char MyKey[8]={0};           // 初始密钥 8字节*8
    char YourKey[8]={0};         // 输入的解密密钥 8字节*8
    char MyMessage[8]={0};       // 初始明文 
    char MyMessage_OUT[8]={0};       // 初始明文 
/*-----------------------------------------------*/
    u16 group,group_ero;
		u16 j=0,k=0,k1=0,l=0,l1=0;		
		group_ero=num%8;
		group=num/8;
		if(group_ero!=0)
    group++;
		
		for (i=0;i<8;i++){MyKey[i]=key_lock[i];YourKey[i]=key_unlock[i];}//copy key
		
		SetKey(MyKey);               // set key master

		for(j=0;j<group;j++)//lock
		{
		for (i=0;i<8;i++)
			MesHex[i]=0;//clear hex_buf
			
	  if(group_ero!=0&&j==group-2)
		{for (i=0;i<group_ero;i++)
			MyMessage[i]=data[l++];//copy group	
		 for (i=group_ero;i<8;i++)
			MyMessage[i]=0x00;//copy group	
		}
		else
		for (i=0;i<8;i++)
		MyMessage[i]=data[l++];//copy group
			
		PlayDes(MesHex,MyMessage);   // DSE
			
		for (i=0;i<16;i++)
		MesHex_OUT[k++]=MesHex[i];//out hex	
			
    }
    SetKey(YourKey);             // set key slave
  	for(j=0;j<group;j++)//unlock
		{
			for (i=0;i<16;i++)
			MesHexr[i]=MesHex_OUT[k1++];//out hex	
       KickDes(MyMessage_OUT,MesHexr);                     // 解密输出到MyMessage   
      for (i=0;i<8;i++)
			MyMessage_Kick[l1++]=MyMessage_OUT[i];//copy group
		}
}


u8 license_check(char *data,char *key_lock,char * lisence,unsigned int num)
{
     u8 i=0; 
    char MesHex[16]={0};         // 16个字符数组用于存放 64位16进制的密文
		char MesHexr[16]={0};
    char MyKey[8]={0};           // 初始密钥 8字节*8
    char YourKey[8]={0};         // 输入的解密密钥 8字节*8
    char MyMessage[8]={0};       // 初始明文 
    char MyMessage_OUT[8]={0};       // 初始明文 
/*-----------------------------------------------*/
    u16 group,group_ero;
		u16 j=0,k=0,k1=0,l=0,l1=0;		
		group_ero=num%8;
		group=num/8;
		if(group_ero!=0)
    group++;
		
		for (i=0;i<8;i++){MyKey[i]=key_lock[i];}//copy key
		
		SetKey(MyKey);               // set key master

		for(j=0;j<group;j++)//lock
		{
		for (i=0;i<8;i++)
			MesHex[i]=0;//clear hex_buf
			
	  if(group_ero!=0&&j==group-2)
		{for (i=0;i<group_ero;i++)
			MyMessage[i]=data[l++];//copy group	
		 for (i=group_ero;i<8;i++)
			MyMessage[i]=0x00;//copy group	
		}
		else
		for (i=0;i<8;i++)
		MyMessage[i]=data[l++];//copy group
			
		PlayDes(MesHex,MyMessage);   // DSE
			
		for (i=0;i<16;i++)
		MesHex_OUT[k++]=MesHex[i];//out hex	
			
    }
	
		for(i=0;i<125;i++)
		  if(lisence[i]!=MesHex_OUT[i])
				 return 0;
			
		return 1;	
}
/*-------------------------------
 把DatIn开始的长度位Len位的二进制
 复制到DatOut后
--------------------------------*/
void BitsCopy(bool *DatOut,bool *DatIn,int Len)     // 数组复制 OK 
{
    int i=0;
    for(i=0;i<Len;i++)
    {
        DatOut[i]=DatIn[i];
    }
}

/*-------------------------------
 字节转换成位函数 
 每8次换一个字节 每次向右移一位
 和1与取最后一位 共64位 
--------------------------------*/
void ByteToBit(bool *DatOut,char *DatIn,int Num)       // OK
{
    int i=0;
    for(i=0;i<Num;i++)
    {
        DatOut[i]=(DatIn[i/8]>>(i%8))&0x01;   
    }                                       
}

/*-------------------------------
 位转换成字节函数
 字节数组每8次移一位
 位每次向左移 与上一次或   
---------------------------------*/
void BitToByte(char *DatOut,bool *DatIn,int Num)        // OK
{
    int i=0;
    for(i=0;i<(Num/8);i++)
    {
        DatOut[i]=0;
    } 
    for(i=0;i<Num;i++)
    {
        DatOut[i/8]|=DatIn[i]<<(i%8);    
    }        
}


/*----------------------------------
 二进制密文转换为十六进制
 需要16个字符表示
-----------------------------------*/
void BitToHex(char *DatOut,bool *DatIn,int Num)
{
    int i=0;
    for(i=0;i<Num/4;i++)
    {
        DatOut[i]=0;
    }
    for(i=0;i<Num/4;i++)
    {
        DatOut[i] = DatIn[i*4]+(DatIn[i*4+1]<<1)
                    +(DatIn[i*4+2]<<2)+(DatIn[i*4+3]<<3);
        if((DatOut[i]%16)>9)
        {
            DatOut[i]=DatOut[i]%16+'7';       //  余数大于9时处理 10-15 to A-F
        }                                     //  输出字符 
        else
        {
            DatOut[i]=DatOut[i]%16+'0';       //  输出字符       
        }
    }
    
}

/*---------------------------------------------
 十六进制字符转二进制
----------------------------------------------*/
void HexToBit(bool *DatOut,char *DatIn,int Num)
{
    int i=0;                        // 字符型输入 
    for(i=0;i<Num;i++)
    {
        if((DatIn[i/4])>'9')         //  大于9 
        {
            DatOut[i]=((DatIn[i/4]-'7')>>(i%4))&0x01;               
        }
        else
        {
            DatOut[i]=((DatIn[i/4]-'0')>>(i%4))&0x01;     
        } 
    }    
}

// 表置换函数  OK
void TablePermute(bool *DatOut,bool *DatIn,const char *Table,int Num)  
{
    int i=0;
    static bool Temp[256]={0};
    for(i=0;i<Num;i++)                // Num为置换的长度 
    {
        Temp[i]=DatIn[Table[i]-1];  // 原来的数据按对应的表上的位置排列 
    }
    BitsCopy(DatOut,Temp,Num);       // 把缓存Temp的值输出 
} 

// 子密钥的移位
void LoopMove(bool *DatIn,int Len,int Num) // 循环左移 Len数据长度 Num移动位数
{
    static bool Temp[256]={0};    // 缓存   OK
    BitsCopy(Temp,DatIn,Num);       // 将数据最左边的Num位(被移出去的)存入Temp 
    BitsCopy(DatIn,DatIn+Num,Len-Num); // 将数据左边开始的第Num移入原来的空间
    BitsCopy(DatIn+Len-Num,Temp,Num);  // 将缓存中移出去的数据加到最右边 
} 

// 按位异或
void Xor(bool *DatA,bool *DatB,int Num)           // 异或函数
{
    int i=0;
    for(i=0;i<Num;i++)
    {
        DatA[i]=DatA[i]^DatB[i];                  // 异或 
    }
} 

// 输入48位 输出32位 与Ri异或
void S_Change(bool DatOut[32],bool DatIn[48])     // S盒变换
{
    int i,X,Y;                                    // i为8个S盒 
    for(i=0,Y=0,X=0;i<8;i++,DatIn+=6,DatOut+=4)   // 每执行一次,输入数据偏移6位 
    {                                              // 每执行一次,输出数据偏移4位
        Y=(DatIn[0]<<1)+DatIn[5];                          // af代表第几行
        X=(DatIn[1]<<3)+(DatIn[2]<<2)+(DatIn[3]<<1)+DatIn[4]; // bcde代表第几列
        ByteToBit(DatOut,&S_Box[i][Y][X],4);      // 把找到的点数据换为二进制    
    }
}

// F函数
void F_Change(bool DatIn[32],bool DatKi[48])       // F函数
{
    static bool MiR[48]={0};             // 输入32位通过E选位变为48位
    TablePermute(MiR,DatIn,E_Table,48); 
    Xor(MiR,DatKi,48);                   // 和子密钥异或
    S_Change(DatIn,MiR);                 // S盒变换
    TablePermute(DatIn,DatIn,P_Table,32);   // P置换后输出
}



void SetKey(char KeyIn[8])               // 设置密钥 获取子密钥Ki 
{
    int i=0;
    static bool KeyBit[64]={0};                // 密钥二进制存储空间 
    static bool *KiL=&KeyBit[0],*KiR=&KeyBit[28];  // 前28,后28共56
    ByteToBit(KeyBit,KeyIn,64);                    // 把密钥转为二进制存入KeyBit 
    TablePermute(KeyBit,KeyBit,PC1_Table,56);      // PC1表置换 56次
    for(i=0;i<16;i++)
    {
        LoopMove(KiL,28,Move_Table[i]);       // 前28位左移 
        LoopMove(KiR,28,Move_Table[i]);          // 后28位左移 
         TablePermute(SubKey[i],KeyBit,PC2_Table,48); 
         // 二维数组 SubKey[i]为每一行起始地址 
         // 每移一次位进行PC2置换得 Ki 48位 
    }        
}

void PlayDes(char MesOut[8],char MesIn[8])  // 执行DES加密
{                                           // 字节输入 Bin运算 Hex输出 
    int i=0;
    static bool MesBit[64]={0};        // 明文二进制存储空间 64位
    static bool Temp[32]={0};
    static bool *MiL=&MesBit[0],*MiR=&MesBit[32]; // 前32位 后32位 
    ByteToBit(MesBit,MesIn,64);                 // 把明文换成二进制存入MesBit
    TablePermute(MesBit,MesBit,IP_Table,64);    // IP置换 
    for(i=0;i<16;i++)                       // 迭代16次 
    {
        BitsCopy(Temp,MiR,32);            // 临时存储
        F_Change(MiR,SubKey[i]);           // F函数变换
        Xor(MiR,MiL,32);                  // 得到Ri 
        BitsCopy(MiL,Temp,32);            // 得到Li 
    }                    
    TablePermute(MesBit,MesBit,IPR_Table,64);
    BitToHex(MesOut,MesBit,64);
}

void KickDes(char MesOut[8],char MesIn[8])       // 执行DES解密
{                                                // Hex输入 Bin运算 字节输出 
    int i=0;
    static bool MesBit[64]={0};        // 密文二进制存储空间 64位
    static bool Temp[32]={0};
    static bool *MiL=&MesBit[0],*MiR=&MesBit[32]; // 前32位 后32位
    HexToBit(MesBit,MesIn,64);                 // 把密文换成二进制存入MesBit
    TablePermute(MesBit,MesBit,IP_Table,64);    // IP置换 
    for(i=15;i>=0;i--)
    {
        BitsCopy(Temp,MiL,32);
        F_Change(MiL,SubKey[i]);
        Xor(MiL,MiR,32);
        BitsCopy(MiR,Temp,32);
    }    
    TablePermute(MesBit,MesBit,IPR_Table,64);
    BitToByte(MesOut,MesBit,64);        
} 
