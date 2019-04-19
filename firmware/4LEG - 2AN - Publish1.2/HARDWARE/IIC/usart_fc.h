#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"
extern u8 KEY[8];
void Usart1_Init(u32 br_num);
void Usart2_Init(u32 br_num);
void Usart4_Init(u32 br_num);
void Usart3_Init(u32 br_num);
void Uart5_Init(u32 br_num);

typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
	      int16_t HEIGHT_MODE;
	      int16_t POS_MODE;
	      u8 update,Heart,Heart_rx,Heart_error;
	      u16 lose_cnt,lose_cnt_rx;
	      u8 connect;
				int16_t RST;}RC_GETDATA;

extern RC_GETDATA Rc_Get,Rc_Get_PWM,Rc_Get_SBUS,Rc_Wifi;//接收到的RC数据,1000~2000
							
typedef struct{
	char check;
	 int x;//目标的x坐标
	 int y;//目标的y坐标
	 int w;//目标的宽度
	 int h;//目标的高度
	 int s;
	float pos[3],att[3];
}_PIX_TAR;

typedef struct{
	_PIX_TAR cube,face;
	float target_pos[3];
	float target_att[3];
	
	float cmd_spd[3];
	float cmd_pos[3];
	float cmd_att[4];
	u8 power;
	u8 cmd_mode;
	u8 visual_mode;
	u8 connect;
	u16 lost_cnt;
}_PI;//识别结果

extern _PI pi;

typedef struct{
	int origin[2];
	float spd_o[2];
	float spd_flt[2];
	float dis;
	u8 mode;
	u8 connect;
	u16 lost_cnt;
}_ODOMETER;//识别结果

extern _ODOMETER flow;


#define SEND_BUF_SIZE1 64*4	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区

void data_per_uart1(void);
void clear_leg_uart(void);
extern u16 leg_uart_cnt;

#define SEND_BUF_SIZE2 40	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff2[SEND_BUF_SIZE2];	//发送数据缓冲区
void data_per_uart2(void);

#define SEND_BUF_SIZE3 32	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff3[SEND_BUF_SIZE3];	//发送数据缓冲区
void data_per_uart3(void);

#define SEND_BUF_SIZE4 64	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff4[SEND_BUF_SIZE4];	//发送数据缓冲区
void data_per_uart4(u8 sel);

extern int16_t BLE_DEBUG[16];
void data_per_uart_rc(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);				

#define MODE_CUBE 1
#define MODE_FACE 2
#define MODE_ROS  11
#endif
