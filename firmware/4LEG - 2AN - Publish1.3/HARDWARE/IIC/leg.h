
#include "stm32f4xx.h"
#define D_LEG 0
#define X_LEG 1
#define T_LEG 2

#define MINI_ROBOT 0
#define TEST_MODE1 0
#define BODY_CONTROL_USE_GLOBAL 1
#define DENG_LIMIT_TEST 0
#define USE_LEG_TRIG_DELAY 1
#define USE_SIMPLE_CENTER 0
#define HORIZON_USE_FORWARD_CENTER 1
#define TWO_LEG_TEST 0
#define TIRG_CURVE_USE_BAI 0
#define USE_GLOBAL_COOR 0
#define TIRG_USE_LITTLE_DOG 1
#define USE_FALL_TREADT 1
#define TORT_RAD_DEAD 0

//#define CENTER_CONTROL_TEST //重心控制测试模式
//#define TRIG_TEST

#define S_IDLE 0
#define S_BODY_MOVE 1
#define S_LEG_TRIG 2
#define S_LEG_TRIG_LEAVE_GROUND_CHECK 3
#define S_LEG_TRIG_ING 4
#define S_LEG_TRIG_DONE 5
#define S_BODY_RETURN 6
#define S_LEG_TRIG_ING_RE 7

#define Xs 0
#define Ys 1
#define Zs 2
#define Xr 0
#define Yr 1
#define Zr 2

#define USE_UART_LEG 1
#define RANDOM (float)RNG_Get_RandomRange(-1000,1000)/1000000.
#define FLAG_IN(in) ((in) >= (0) ? 1 : -1)
#define CHECK(in) (isinf(in)||isnan(in))
#define PI 3.1415926
extern float center_control_out[2],att_control_out[5];;
extern u8 trig_list_f[5];
extern u8 trig_list_r[5];
extern u8 trig_list_b[5];
extern u8 trig_list_l[5];
extern u8 trig_list_tr[5];
extern u8 trig_list_tl[5];

typedef struct 
{
	u8 state;//1-> good  2->wrong lisence
	u8 id[3];//board id
	char lisence[125];//your lisence
}LIS;
extern LIS license;

typedef struct 
{
	float x;
	float y;
	float z;
	
}POS;

typedef struct 
{
	float leg_up_high;
	u8 leg_switch_flag,leg_ground_force;
	float z;
	POS off_all;
	POS init_end_pos;
	POS limit,limit_min;
	float l1;
	float l2;
	float l3;
	u8 pwm_id[4];
	u16 PWM_OFF[4];
	u16 PWM_INIT[3];
	u16 PWM_MIN[4],PWM_MAX[4];
	float PWM_OUT[4];
	float PWM_PER_DEGREE[4];
	u8 use_ground_check;
	u8 leg_set_invert,leg_ground_cnt;
	float pos_tar_reg[3],off_local[2];
	u8 en_pwm_out;
	u16 leg_loss_cnt;
	float desire_time;
	int sita_flag[4];
	float dsita[3];
	float dsita_tar[3];
	float init_sita[3];
	float sita_tar[3];
	POS pos_tar_trig_test[3];
	u8 id,init_mode,leg_power_force,control_mode_focre;
	u16 leg_move_pass_cnt;
	float deng_to_ground;
}LEG_SYS;


typedef struct 
{ u8 leg_connect,leg_ground,leg_ground_fake,err;
	u8 leg_power,rst_leg;
	u8 need_move;	
	u8 curve_trig,control_mode,trig_flag;
	POS pos_now[3],pos_now_trig_f[3],pos_now_brain[3];
	POS pos_tar[3],pos_tar_trig[3];
	float deng_z;
	float sita_ss,sita_ss_limit;
	float sita[4],sita_force[4];
	 //0->openloop 1->closeloop	
	float leg_end_force[4],leg_meme_angle[4];
	float deng[3];
	LEG_SYS sys;
}LEG_STRUCT;
//------------------------------BARIN-----

typedef struct 
{ u8 leg_connect,control_angle;
	u16 leg_loss_cnt,dt_leg_min_trig_cnt,no_control_cnt;
	float att_off[2],k_spd_to_range,kp_center[2],k_center_fp,move_range_k,k_center_c[2];
	float desire_time,desire_time_init,min_range,max_range,off_cor[2],down_h,down_t,yaw_dead;
	POS leg_local[5];
	u8 leg_use_ground;
	u8 init_mode;
	POS pos_tar_trig[5];
	u8 front_leg_num;
	u8 err,rst;
	float center_off_when_move[2];
	float leg_t,tar_spd[3],in_rst_check;
	float leg_h[5];	
	float leg_move_range[2],leg_move_range1[5],leg_move_min_dt;//cm
	float yaw_trig;
	POS off_leg[5],center_off,center_off1,center_scale;
}BRAIN_SYS;

typedef struct 
{ 
	POS end_pos_global[5],body_coner[5],ZMP;
	POS fake_tar_pos;
	float value[5],steady_value,out_value;
	float center_stable_weight;
	float area_value,min_width_value;
	float area_of_leg[3],dis_leg_out[5];
	double leg_ground_center[3],leg_ground_center_trig[3],leg_ground_center_trig_init[3],tar_center[2];
}BRAIN_GLOBAL;

typedef struct 
{ u8 control_mode,power_all,rst_all,rst_all_soft,tabu,leg_lun_mode;//leg_lun_mode-->1 轮子 0-->腿
	int fall,trot_gait;
	float steady_value,min_st[3];
	u8 force_stop,loss_center,ground_leg_num,can_move_leg;	
	u8 leg_move[5],leg_out_range[5],way;	
	BRAIN_GLOBAL global;
	float tar_att_force[3],tar_att[3],att[3],att_rate[3],att_flt[3],sample_att_buf[3],tar_w,tar_w_set,tar_h,spd,spd_d,spd_yaw;
	float rotate_linear;
  u8 center_stable,move_id,leg_move_state;
	u8 en_att_change;
	u8 en_ad_ground;
	float area_of_leg[2];
	POS center;
	double leg_ground_center[3],leg_ground_center_trig[3],leg_ground_center_trig_init[3],tar_center[2];
	float now_pos[3],now_spd[3],now_acc[3];
	int rc[4];
	BRAIN_SYS sys;
}BRAIN_STRUCT;


extern u8 last_move_leg;
extern LEG_STRUCT leg[5];
extern BRAIN_STRUCT brain;

void cal_sita_from_pos( LEG_STRUCT *in,float x_i,float y_i,float z_i,u8 out);
void cal_pos_from_sita(LEG_STRUCT * in);
void leg_init( LEG_STRUCT *in,u8 id);
u8 pos_range_check(LEG_STRUCT * in,float x,float y,float z);
void leg_drive(LEG_STRUCT * in,float dt);

//brain
extern float gain_spd_ss[2];//速度增益
extern float limit_deng_tro_ss;//速度限制
void leg_publish(LEG_STRUCT * in);
void cal_pwm_from_sita(LEG_STRUCT * in);
void trot_gait_ss(float dt);
void center_control_global_tro_ss(float dt);
void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id);
void estimate_center(BRAIN_STRUCT *in,float att[3],float spd_body[3],float acc_body[3],float spd_tar[3],float w_tar);
void barin_init(BRAIN_STRUCT *in);
void center_control(float dt);//PID
void cal_center_of_leg_ground(BRAIN_STRUCT *in);//PID
void check_leg_need_move(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt);
u8 inTrig(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3); 
u8 inTrig2(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4) ;
void find_closet_point(u8*min_id,float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4,u8 num); 
void find_leg_need_move(float spd_tar[3],float str[4],float end[4]) ;
void leg_task1(float dt);
void cal_deng_from_spd(BRAIN_STRUCT *in);
void cal_target_of_center_move(BRAIN_STRUCT *in);
void leg_tar_est(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt);
void att_control(float dt);
void fall_treat(float dt,float *out);
void fall_treat1(float dt,float *out);
void fall_treat_tro(float dt,float *out,float *out1);
void center_control_global_tro_new(float dt);
u8 planner_leg(u8 last_move_id,u8 last_last_move_id);
void check_leg_need_move_global(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt);
void center_control_global(float dt);//中心控制PID  GLOBAL
void state_clear(void);
void get_leg_tar_trig(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt);
void fall_reset(float dt);
void cal_pos_global(float dt);
u8 planner_leg_little_dog(u8 last_move_id,u8 last_last_move_id);
u8 planner_leg_all(u8 leg_flag,float dt);
void leg_tar_est_global(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt,u8 fake);
void center_control_global_tro(float dt);
void check_leg_need_move_global_tro(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt);
void leg_tar_est_global_tro(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt,u8 fake);
//----------------------------------------------LIB----------------------------------------------
//获得当前脚在步态表中的标号
u8 get_next_leg_flag_in_list(u8 leg_id_now,u8 way);
//设置使用授权码
void set_lisence(char *in);
//点与球体的交点
u8 arrow_check_to_bow(float cx,float cy,float cz,float rx,float ry,float rz,float *x,float *y,float *z);
//计算点矢量两侧对称点
void two_point_between_arror(float cx,float cy,float yaw,float *x1,float *y1,float *x2,float *y2,float dis);
//跨脚范围修正
void limit_trig_pos_of_leg(float cx,float cy,float cz,float rx,float ry,float rz,float *x,float *y,float *z);
//计算三角形重心坐标
void cal_center_of_trig(float x1,float y1,float x2,float y2,float x3,float x4,float *cx,float *cy);
//点到直线距离
float dis_point_to_line(float x,float y,float k,float b);
//判断点在移动区域内部tangle
u8 check_in_move_range_tangle(u8 id,float x,float y,float cx,float cy,float min,float max,float *min_dis);
//两点求直线方程
void line_function_from_two_point(float x1,float y1,float x2,float y2,float *k,float *b);
//矢量求直线方程
void line_function_from_arrow(float x,float y,float yaw,float *k,float *b);
//点矢与方框最短距离
float get_min_dis_arrow_to_tangle(float x,float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4);
//点沿矢量对称缩放
void resize_point_with_arrow(float x,float y,float cx,float cy,float yaw,float k,float *nx,float *ny);
//三角形重心
void cal_cog_tri(float x1,float y1,float x2,float y2,float x3,float y3,float *cx,float *cy);
//判断脚到达初始化的位置
u8 check_leg_near_init(float ero);
//跨腿重复保护器
u8 leg_repeat_protect1(u8 id,u8 last_move_id,u8 last_last_move_id,float yaw,float yaw_trig);
//跨腿重复保护器
u8 leg_repeat_protect(u8 id,u8 last_move_id,u8 last_last_move_id,float yaw,float yaw_trig);
//矢量垂线方程
void line_function90_from_arrow(float x,float y,float yaw,float *k,float *b);
//计算移动区域与矢量的两个交点 方框
void cal_jiao_of_range_and_line_tangle(u8 id,float cx,float cy,float min,float max,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y);
//点在直线上
u8 check_point_on_line(float x,float y,float k,float b,float err);
//两直线交点
u8 cross_point_of_lines(float k1,float b1,float k2,float b2,float *x,float *y);
//点在点矢量方向前
u8 check_point_front_arrow(float x,float y,float cx,float cy,float yaw);
//点矢量与直线交点
u8 check_cross_arrow_line(float cx,float cy,float yaw,float k,float b,float *x,float *y);
//判断两点在线同一侧
u8 check_points_same_side(float x1,float y1,float x2,float y2,float k,float b);
//点矢量垂线与直线交点
u8 check_cross_arrow90_line(float cx,float cy,float yaw,float k,float b,float *x,float *y);
//计算两点距离
float cal_dis_of_points(float x1,float y1,float x2,float y2);
//判断在椭圆内部
u8 in_circle(float cx,float cy,float d_short,float d_long,float x,float y);
//转换坐标
void conver_body_to_global(float bx,float by,float *gx,float *gy);
//计算直线和椭圆交点
void cal_jiao_of_tuo_and_line(float cx,float cy,float d_short,float d_long,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y);
//计算三角形面积
float cal_area_trig(float ax,float ay,float bx,float by,float cx,float cy);
//点矢量与三角形交点
u8 check_point_to_trig(float x,float y,float yaw,float x1,float y1,float x2,float y2,float x3,float y3,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y);
//计算移动区域与矢量的两个交点
void cal_jiao_of_range_and_line(u8 id,float cx,float cy,float min,float max,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y);
//移动区域限幅度 方框
void limit_move_range_tangle(u8 id,float cx,float cy,float x,float y,float min,float max,float *outx,float *outy);
//判断点在移动区域内部
u8 check_in_move_range(u8 id,float x,float y,float cx,float cy,float min,float max);
//点矢量与四边形交点
u8 check_point_to_tangle(float x,float y,float yaw,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y);
//一个点在三角形内部
u8 inTrig(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3);
//计算稳态余量
float cal_steady_s(float cx,float cy,float x1,float y1,float x2,float y2,float x3, float y3 );
//脚位置限制
void limit_range_leg(float x,float y,float min,float max,float *xout,float *yout);
//解方程
u8 segmentsIntr(POS b,POS c,POS d,POS a,float *x,float *y);
//一个点在四边形内部
u8 inTrig2(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4);
//找到离xy最近的点
void find_closet_point(u8*min_id,float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4,u8 num);
//转换腿局部坐标系到全局机体坐标系
void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id);
//计算稳态余量四边形
float cal_steady_s4(float cx,float cy,float x1,float y1,float x2,float y2,float x3, float y3 ,float x4,float y4 );