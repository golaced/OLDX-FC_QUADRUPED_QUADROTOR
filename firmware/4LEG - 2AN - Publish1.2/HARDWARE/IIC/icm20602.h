#ifndef _DRV_ICM20602_H_
#define _DRV_ICM20602_H_

#include "include.h"

typedef struct
{
	float center_pos_cm[3];
	float gyro_rad[3];
	float gyro_rad_old[3];
	float gyro_rad_acc[3];
	float linear_acc[3];
}_center_pos_st;
extern _center_pos_st center_pos;

extern u8 mpu_buffer[14];

typedef struct
{
  u8 surface_CALIBRATE;
	float surface_vec[3];
	float surface_unitvec[3];
	
}_sensor_rotate_st;
extern _sensor_rotate_st sensor_rot ;


u8 Icm20602Reg_Init(void);
void Icm20602_Read(void);
void Sensor_Data_Prepare(u8 dT_ms);
void Center_Pos_Set(void);
void  Acc_Prepare(float Pitch,float Roll,float Yaw);

typedef struct 
{ 
u8 Cali_3d;	
char Acc_CALIBRATE;
char Gyro_CALIBRATE;
xyz_s16_t Acc_I16;
xyz_s16_t Gyro_I16;

xyz_f_t Acc,Acc_t;
xyz_f_t Gyro,Gyro_t;

//	XYZ_STRUCT Acc_deg;
xyz_f_t Gyro_deg,Gyro_deg_t;

xyz_f_t Acc_Offset;
xyz_f_t Gyro_Offset;
u8 acc_cal_3d_step;	
xyz_f_t Gain_3d;
xyz_f_t Off_3d;
float att_off[2];
	
xyz_f_t Gyro_Auto_Offset;
float Temprea_Offset;
float Gyro_Temprea_Adjust;
float ACC_Temprea_Adjust;

s16 Tempreature;
float TEM_LPF;
float Ftempreature;

xyz_s16_t Mag_Adc;			//采样值
xyz_f_t   Mag_Offset,Mag_Offseto;		//偏移值
xyz_f_t   Mag_Offset_c,Mag_Offset_co;		//偏移值
xyz_f_t   Mag_Gain,Mag_Gaino;		//偏移值
xyz_f_t 	Mag_Gain_c,Mag_Gain_co;			//比例缩放	
xyz_f_t 	Mag_Val,Mag_Val_t,Mag_Valo,Mag_Val_to;			//纠正后的值
u8 Mag_CALIBRATED;
float yaw;
float Pressure;
float Tem_bmp;
float Alt;
float hmlOneMAG,hmlOneACC;
}LIS3MDL_S;

extern LIS3MDL_S lis3mdl;


#define   WHO_AM_I     0x0F
#define   CTRL_REG1    0x20
#define   CTRL_REG2    0x21
#define   CTRL_REG3    0x22
#define   CTRL_REG4    0x23
#define   CTRL_REG5    0x24
#define    XLDA            0x1 
#define   STATUS_REG   0x27
#define   OUT_X_L      0x28
#define   OUT_X_H      0x29
#define   OUT_Y_L      0x2A
#define   OUT_Y_H      0x2B
#define  OUT_Z_L      0x2C
#define  OUT_Z_H      0x2D
#define   TEMP_OUT_L   0x2E
#define  TEMP_OUT_H   0x2F
#define   INT_CFG      0x30
#define  INT_SRC      0x31
#define  INT_THS_L    0x32
#define   INT_THS_H    0x33

#define     LSM6D33_ADDR_READ               0xd5
#define     LSM6D33_ADDR_WRITE              0xd4
#define  CTRL1_XL           0x10
#define CTRL2_G            0x11
#define CTRL3_C            0x12
#define CTRL4_C            0x13
#define CTRL5_C            0x14
#define CTRL6_C            0x15
#define  CTRL7_G            0x16
#define CTRL8_XL          0x17
#define CTRL9_XL           0x18
#define CTRL10_C           0x19

#define   OUTX_L_G           0x22
#define   OUTX_H_G           0x23
#define   OUTY_L_G           0x24
#define   OUTY_H_G           0x25
#define  OUTZ_L_G           0x26
#define   OUTZ_H_G           0x27
#define  OUTX_L_XL          0x28
#define  OUTX_H_XL          0x29
#define  OUTY_L_XL          0x2A
#define  OUTY_H_XL          0x2B
#define OUTZ_L_XL          0x2C
#define OUTZ_H_XL          0x2D
void LIS3MDL_Init(void);
void DS33_Init(void);
void LIS3MDL_read(void);
void LSM6_readAcc(u8 fast);
void LSM6_readGyro(u8 fast);
#endif
