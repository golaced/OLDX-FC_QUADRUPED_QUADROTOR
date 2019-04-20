#include "math.h"
#include "spi.h"
#include "icm20602.h"
#include "cycle_cal_oldx.h"
LIS3MDL_S lis3mdl;
#define DS33_WHO_AM_I_ID     0x69  
#define DS33_SA0_HIGH_ADDRESS 0x6b
#define DS33_SA0_LOW_ADDRESS  0x6a
// Reads the 3 mag channels and stores them in vector m
#define DS33_ADDRESS1  (DS33_SA0_HIGH_ADDRESS << 1)
#define DS33_ADDRESS2  (DS33_SA0_LOW_ADDRESS << 1)
#define DS33_IIC_ID DS33_ADDRESS1
#define ST_SENSORS_SPI_READ			0x80
void Lis3mdl_SPI_WR(u8 add,u8 wrdata,u8 sel)
{
  SPI_CS(sel,0);
	//Spi_RW(add&0x7F );	
  Spi_RW(add);
  Spi_RW(wrdata);
  SPI_CS(sel,1);
}


void SPI_BufferRead(u8*buf, u8 add, u8 len,u8 sel)
{
	u8 i=0;
  SPI_CS(sel,0);
	if(sel!=CS_LIS)
	Spi_RW(add|ST_SENSORS_SPI_READ);	
	else
	Spi_RW(add|0xC0);
	for(i=0;i<len;i++)
	{
	 *buf++ = Spi_RW(0xff); 
	} 
 SPI_CS(sel,1);
}


uint8_t id[3] ;
// Reads the 3 accelerometer channels and stores them in vector a
void LSM6_readAcc(u8 fast)
{u8 buffer[6];
	SPI_BufferRead(buffer, OUTX_L_XL, 6, MPU9250);
	xyz_s16_t data;
  data.x = (buffer[1] << 8) | buffer[0];
	data.y = (buffer[3] << 8) | buffer[2];
	data.z = (buffer[5] << 8) | buffer[4];
	
	if(abs(data.x)<6000&&abs(data.y)<6000&&abs(data.z)<6000)
	{
	lis3mdl.Acc_I16.x=data.x;
	lis3mdl.Acc_I16.y=data.y;
	lis3mdl.Acc_I16.z=data.z;
	}
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6_readGyro(u8 fast)
{
	u8 buffer[6];
	SPI_BufferRead(buffer, OUTX_L_G, 6, MPU9250);
  xyz_s16_t data;
	data.x = (buffer[1] << 8) | buffer[0];
	data.y = (buffer[3] << 8) | buffer[2];
	data.z = (buffer[5] << 8) | buffer[4];
	if(abs(data.x)<6000&&abs(data.y)<6000&&abs(data.z)<6000)
	{
	lis3mdl.Gyro_I16.x=data.x;
	lis3mdl.Gyro_I16.y=data.y;
	lis3mdl.Gyro_I16.z=data.z;
	}
}


void DS33_Init(void)
{
//---------------init acc & gro		
		Lis3mdl_SPI_WR(0x21,0x04,MPU9250);
		Delay_ms(10);
	  Lis3mdl_SPI_WR(CTRL1_XL, 0x4f,MPU9250);Delay_ms(10);
	  Lis3mdl_SPI_WR(CTRL2_G, 0x4c,MPU9250);Delay_ms(10);
	  Lis3mdl_SPI_WR(CTRL3_C, 0x04,MPU9250);Delay_ms(10);
		
		Delay_ms(10);
		SPI_CS(MPU9250,0);
		Spi_RW(0x80|0x0f);
		u8 l_u8_ID1= Spi_RW(0xFF);
	  SPI_CS(MPU9250,1);
		id[1] = l_u8_ID1;
		if(DS33_WHO_AM_I_ID==l_u8_ID1)
			 module.acc_imu =module.gyro_imu= 1; 

		LSM6_readGyro(0);
}
//--------------------------HML  LIS------------------------			
#define LIS3MDL_SA1_HIGH_ADDRESS  0x1E
#define LIS3MDL_SA1_LOW_ADDRESS   0x1C
#define LIS3MDL_ADDRESS1  (LIS3MDL_SA1_HIGH_ADDRESS << 1)
#define LIS3MDL_ADDRESS2  (LIS3MDL_SA1_LOW_ADDRESS << 1)
#define TEST_REG_ERROR -1
#define LIS3MDL_WHO_ID  0x3D
#define LIS3MDL_IIC_ID LIS3MDL_ADDRESS1

void LIS3MDL_Init(void)
{  
	  Lis3mdl_SPI_WR(CTRL_REG1, 0x74,CS_LIS);
	  Lis3mdl_SPI_WR(CTRL_REG2, 0x60,CS_LIS);
	  Lis3mdl_SPI_WR(CTRL_REG3, 0x00,CS_LIS);
	  Lis3mdl_SPI_WR(CTRL_REG4, 0x0C,CS_LIS);
	  SPI_CS(CS_LIS,0);
	  Spi_RW(0x80|0x0f);
	  u8 l_u8_ID= Spi_RW(0xFF);
	  
		if(l_u8_ID==LIS3MDL_WHO_ID)
			 module.hml_imu =1; 
		SPI_CS(CS_LIS,1);
}

void LIS3MDL_read(void)
{ u8 buffer[6];
	static u8 temp=0;
	static u8 cnt;
	xyz_s16_t data;
	SPI_BufferRead(buffer, OUT_X_L, 6, CS_LIS);
 
   data.x= (buffer[1] << 8) | buffer[0];
   data.y= (buffer[3] << 8) | buffer[2];
   data.z= (buffer[5] << 8) | buffer[4];
	
	 if(abs(data.x)<2500&&abs(data.y)<2500&&abs(data.z)<2500)
	 {
	   lis3mdl.Mag_Adc.x=data.x;
		 lis3mdl.Mag_Adc.y=data.y;
		 lis3mdl.Mag_Adc.z=data.z;
	 }
	 if(lis3mdl.Mag_Adc.z==0&&module.hml_imu)
		 cnt++;
	 else
		 cnt=0;
	 if(cnt>244)
		 module.hml_imu=0;
}

