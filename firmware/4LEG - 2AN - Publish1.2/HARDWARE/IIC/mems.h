#ifndef _MPU6050_H
#define _MPU6050_H

#include "stm32f4xx.h"
#include "include.h"
#include "parameter.h"
#define GYRO_DEATH_ZOOM 20   //  20 / 65536

#define OFFSET_AV_NUM 50
#define FILTER_NUM 10
#define ITEMS 7

typedef struct 
{ 
	char Acc_CALIBRATE;
	char Gyro_CALIBRATE;
	char Cali_3d;
  xyz_s16_t Acc_I16;
	xyz_s16_t Gyro_I16;

	xyz_f_t Acc;
	xyz_f_t Gyro;
	xyz_f_t Gyro_deg;
	xyz_f_t Mag;
	xyz_f_t Acc_Offset;
	xyz_f_t Gyro_Offset;
	xyz_f_t Gyro_Auto_Offset;
	xyz_f_t Gain_3d;
	xyz_f_t Off_3d;
 	char Mag_CALIBRATE,Mag_Have_Param,Mag_ERR,Mag_update;
	xyz_s16_t Mag_Adc,Mag_Adc_o;			//采样值
	xyz_f_t   Mag_Offset,Mag_Offseto;		//偏移值
	xyz_f_t   Mag_Offset_c,Mag_Offset_co;		//偏移值
	xyz_f_t   Mag_Gain,Mag_Gaino;		//偏移值
	xyz_f_t 	Mag_Gain_c,Mag_Gain_co;			//比例缩放	
	xyz_f_t 	Mag_Val,Mag_Val_t,Mag_Valo,Mag_Val_to;			//纠正后的值
	float hmlOneMAG,hmlOneACC;
  float Yaw_Mag;
	float Ftempreature;
}_MEMS;

extern _MEMS mems;
extern float mems_tmp[ITEMS];
extern u8 acc_3d_calibrate_f,acc_3d_step;

void IMU_Read(void);
void IMU_Data_Prepare(float T);
void LIS_CalOffset_Mag(float dt);
#endif
