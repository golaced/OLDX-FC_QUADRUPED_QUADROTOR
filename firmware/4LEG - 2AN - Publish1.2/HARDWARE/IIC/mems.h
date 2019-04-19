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

//	XYZ_STRUCT Acc_deg;
	xyz_f_t Gyro_deg;
	
	xyz_f_t Acc_Offset;
	xyz_f_t Gyro_Offset;
	xyz_f_t Gyro_Auto_Offset;
	xyz_f_t vec_3d_cali;
	xyz_f_t Gain_3d;
	xyz_f_t Off_3d;
	float att_off[2];
	float Acc_Temprea_Offset;
	float Gyro_Temprea_Offset;
	
	float Gyro_Temprea_Adjust;
	float ACC_Temprea_Adjust;
 
	s16 Tempreature;
	float TEM_LPF;
	float Ftempreature;
}_MEMS;

extern _MEMS mems;
extern float mems_tmp[ITEMS];
extern u8 acc_3d_calibrate_f,acc_3d_step;

void IMU_Read(void);
void IMU_Data_Prepare(float T);
#endif
