
#include "spi.h"
#include "icm20602.h"
#include "cycle_cal_oldx.h"
u8 IMU1_Fast=0;
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
	//if(sel!=CS_LIS)
	Spi_RW(add|ST_SENSORS_SPI_READ);	
	//else
	//Spi_RW(add|0xC0);//Á¬Ðø¶ÁÔö¼ÓµØÖ·
	for(i=0;i<len;i++)
	{
	 *buf++ = Spi_RW(0xff); 
	} 
 SPI_CS(sel,1);
}


/* Write Data.
*/
void LSM6DS33_IO_Write(uint8_t* pBuffer,
                                               uint8_t  DeviceAddr,
                                               uint8_t  RegisterAddr,
                                               uint16_t NumByteToWrite
                                              )
{ uint16_t temp;

  SPI_CS(MPU9250,1);

  //Send Device Address.
  Spi_RW(RegisterAddr);

  for(temp = 0; temp < NumByteToWrite; temp ++)
  {
    Spi_RW( *(pBuffer + temp));
  }

  SPI_CS(MPU9250,0);
}

/* Read Data.
*/
void LSM6DS33_IO_Read(uint8_t* pBuffer,
                                              uint8_t  DeviceAddr,
                                              uint8_t  RegisterAddr,
                                              uint16_t NumByteToRead
                                             )
{ uint16_t temp;

  SPI_CS(MPU9250,1);

  //Send Device Address.
  Spi_RW((RegisterAddr | 0x80));

  for(temp = 0; temp < NumByteToRead; temp ++)
  {
    *(pBuffer + temp) = Spi_RW( 0x00);
  }

  SPI_CS(MPU9250,0);
}

uint8_t id[3] ;

// Reads the 3 accelerometer channels and stores them in vector a
void LSM6_readAcc(u8 fast)
{u8 buffer[6];
	IMU1_Fast=fast;
	SPI_BufferRead(buffer, OUTX_L_XL, 6, MPU9250);

  lis3mdl.Acc_I16.x = (buffer[1] << 8) | buffer[0];
	lis3mdl.Acc_I16.y = (buffer[3] << 8) | buffer[2];
	lis3mdl.Acc_I16.z = (buffer[5] << 8) | buffer[4];
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6_readGyro(u8 fast)
{
	u8 buffer[6];
	IMU1_Fast=fast;
	SPI_BufferRead(buffer, OUTX_L_G, 6, MPU9250);

	lis3mdl.Gyro_I16.x = (buffer[1] << 8) | buffer[0];
	lis3mdl.Gyro_I16.y = (buffer[3] << 8) | buffer[2];
	lis3mdl.Gyro_I16.z = (buffer[5] << 8) | buffer[4];
}


void LIS3MDL_enableDefault(void)
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

//-----------------------------------------------------
static void icm20602_readbuf(u8 reg, u8 length, u8 *data)
{
	SPI_CS(MPU9250,0);
	Spi_RW(reg|0x80);
	SPI_Receive(data,length);
	SPI_CS(MPU9250,1);
}

static u8 icm20602_writebyte(u8 reg, u8 data)
{
	u8 status;
	
	SPI_CS(MPU9250,0);
	status = Spi_RW(reg);
	Spi_RW(data);
	SPI_CS(MPU9250,1);
	return status;
}
/**************************实现函数********************************************
*功　　能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
*******************************************************************************/
static void icm20602_writeBit(u8 reg, u8 bitNum, u8 data) 
{
	u8 b;
	icm20602_readbuf(reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	icm20602_writebyte(reg, b);
}

static void icm20602_setIntEnabled ( void )
{
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_LEVEL_BIT, ICM_INTMODE_ACTIVEHIGH );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_OPEN_BIT, ICM_INTDRV_PUSHPULL );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_LATCH_INT_EN_BIT, ICM_INTLATCH_50USPULSE);//MPU6050_INTLATCH_WAITCLEAR );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCFG_INT_RD_CLEAR_BIT, ICM_INTCLEAR_ANYREAD );

	icm20602_writeBit ( MPUREG_INT_ENABLE, ICM_INTERRUPT_DATA_RDY_BIT, 1 );
}

/**************************实现函数********************************************
*功　　能:	    初始化icm进入可用状态。
*******************************************************************************/
u8 Icm20602Reg_Init(void)
{
	u8 tmp;
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x80);
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x01);
	Delay_ms(10);
	
	icm20602_readbuf(MPUREG_WHOAMI, 1, &tmp);
	if(tmp != MPU_WHOAMI_20602)
	module.acc_imu=module.gyro_imu=0;
	else
  module.acc_imu=module.gyro_imu=2;
	/*复位reg*/
	icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);
	Delay_ms(10);
  /*复位reg*/
	icm20602_writebyte(MPU_RA_USER_CTRL,0x01);	
	Delay_ms(10);

	icm20602_writebyte(0x70,0x40);//dmp 
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_PWR_MGMT_2,0x00);
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_SMPLRT_DIV,0);
	Delay_ms(10);

	icm20602_writebyte(MPU_RA_CONFIG,ICM20602_LPF_20HZ);
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_GYRO_CONFIG,(3 << 3));
	Delay_ms(10);
	icm20602_writebyte(MPU_RA_ACCEL_CONFIG,(2 << 3));
	Delay_ms(10);
	/*加速度计LPF 20HZ*/
	icm20602_writebyte(0X1D,0x04);
	Delay_ms(10);
	/*关闭低功耗*/
	icm20602_writebyte(0X1E,0x00);
	Delay_ms(10);
	/*关闭FIFO*/
	icm20602_writebyte(0X23,0x00);

	Delay_ms(100);
	Icm20602_Read();
	return 1;

}

u8 mpu_buffer[14];
void Icm20602_Read()
{
	icm20602_readbuf(MPUREG_ACCEL_XOUT_H,14,mpu_buffer);
	
	mems.Acc_I16.x =(s16)((((u16)mpu_buffer[0]) << 8) | mpu_buffer[1]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
	mems.Acc_I16.y =(s16)((((u16)mpu_buffer[2]) << 8) | mpu_buffer[3]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
	mems.Acc_I16.z =(s16)((((u16)mpu_buffer[4]) << 8) | mpu_buffer[5]);//>>1;// + 4 *sensor.Tempreature_C;// + 7 *sensor.Tempreature_C;

	mems.Gyro_I16.x=(s16)((((u16)mpu_buffer[ 8]) << 8) | mpu_buffer[ 9]) ;
	mems.Gyro_I16.y=(s16)((((u16)mpu_buffer[10]) << 8) | mpu_buffer[11]) ;
	mems.Gyro_I16.z=(s16)((((u16)mpu_buffer[12]) << 8) | mpu_buffer[13]) ;
}


