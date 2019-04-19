
#include "iic_vl53.h"
#include "time.h"
#include "bat.h"
volatile u8 I2C_FastMode_VL53;
LS53 ls53[12];
u8 delay_vl53=5,ls53_connect[12];
void I2c_Soft_delay_VL53()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(!I2C_FastMode_VL53)
	{
		u8 i = delay_vl53;
		while(i--);
	}
}
//u8 IIC_Write_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
//u8 IIC_Read_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
//u8 IIC_Write_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
//u8 IIC_Read_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

void Init_Ground_Key(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  //1	
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD1 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OD1, &GPIO_InitStructure);	
	//2
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OD2, &GPIO_InitStructure);	
	//3
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD3 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OD3, &GPIO_InitStructure);	
	//4
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD4 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O4 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OD4, &GPIO_InitStructure);	
}	

float ground_check_time=0.01;
void Read_Ground_Key(float dt)
{
	static u8 i,ground[4];
	static u16 cnt[2][4];
	#if defined(LEG_USE_SWITCH)
	ground[0]=!(GPIO_ReadInputDataBit(ANO_GPIO_I2C_VL53OD1,I2C_Pin_SDA_VL53O1)); 
  ground[1]=!(GPIO_ReadInputDataBit(ANO_GPIO_I2C_VL53OD2,I2C_Pin_SDA_VL53O2)); 
	ground[2]=!(GPIO_ReadInputDataBit(ANO_GPIO_I2C_VL53OD3,I2C_Pin_SDA_VL53O3)); 
	ground[3]=!(GPIO_ReadInputDataBit(ANO_GPIO_I2C_VL53OD4,I2C_Pin_SDA_VL53O4)); 
	#else
	ground_check_time=0;
	ground[0]=ground_vad[1];
	ground[1]=ground_vad[2];
	ground[2]=ground_vad[3];
	ground[3]=ground_vad[4];
	#endif

	for(i=0;i<4;i++)
	{
	  if(ls53[i].mode==0&&ground[i]==1)
	   cnt[0][i]++;
		else
		 cnt[0][i]=0;
		
		if(cnt[0][i]>ground_check_time/dt)
	  	{ls53[i].mode=1;cnt[0][i]=0;}
		
	 if(ls53[i].mode==1&&ground[i]==0)
	   cnt[1][i]++;
		else
		 cnt[1][i]=0;
		
		if(cnt[1][i]>ground_check_time/dt)
	  	{ls53[i].mode=0;cnt[1][i]=0;}
	}
}


void I2c_Soft_Init_VL53(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  //1
	
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC1 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OC1, &GPIO_InitStructure);	
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD1 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OD1, &GPIO_InitStructure);	
	//
	//2
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC2 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OC2, &GPIO_InitStructure);	
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD2 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OD2, &GPIO_InitStructure);	
	//
		//3
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC3 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OC3, &GPIO_InitStructure);	
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD3 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OD3, &GPIO_InitStructure);	
	//
		//4
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC4 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OC4, &GPIO_InitStructure);	
	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD4 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O4 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ANO_GPIO_I2C_VL53OD4, &GPIO_InitStructure);	

//		//5
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC5 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O5;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OC5, &GPIO_InitStructure);	
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD5 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O5 ;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OD5, &GPIO_InitStructure);	
//	
//		//6
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC6 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OC6, &GPIO_InitStructure);	
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD6 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O6 ;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OD6, &GPIO_InitStructure);	
//	
//			//7
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC7 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OC7, &GPIO_InitStructure);	
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD7 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O7 ;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OD7, &GPIO_InitStructure);	
//	
//			//8
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC8 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OC8, &GPIO_InitStructure);	
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD8 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O8 ;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OD8, &GPIO_InitStructure);	
//	
//			//9
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC9 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OC9, &GPIO_InitStructure);	
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD9 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O9 ;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OD9, &GPIO_InitStructure);	
//	
//		//10
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC10 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OC10, &GPIO_InitStructure);	
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD10 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O10 ;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OD10, &GPIO_InitStructure);	
//	
//		//9
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC11 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OC11, &GPIO_InitStructure);	
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD11 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O11 ;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OD11, &GPIO_InitStructure);	
//	
//		//9
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OC12 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_VL53O12;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OC12, &GPIO_InitStructure);	
//	RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_VL53OD12 , ENABLE );
//  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA_VL53O12 ;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(ANO_GPIO_I2C_VL53OD12, &GPIO_InitStructure);	
}

uint16_t makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

int set_dis=55;
u8 READ_VL53(u8 sel){
u8 val=0;
static u8 flag1[12]={0},init,state[12] ;
u8 gbuf[16];
uint16_t count[3];
u8 DeviceRangeStatusInternal;
	
	  switch(state[sel])	
		{
			case 0:IIC_Write_1Byte_VL53(VL53L0X_Add,VL53L0X_REG_SYSRANGE_START, 0x01,sel);
			state[sel]=1;
			flag1[sel]=0;
			break;
			case 1:
				IIC_Read_1Byte_VL53(VL53L0X_Add,VL53L0X_REG_RESULT_RANGE_STATUS,&val,sel);
				if( val == 94) 
				{ ls53_connect[sel]=1;
					ls53[sel].connect=1;
					IIC_Read_nByte_VL53(VL53L0X_Add, 0x14 , 12, gbuf,sel);	
					count[0] = makeuint16(gbuf[7], gbuf[6]);
					count[1] = makeuint16(gbuf[9], gbuf[8]);
					count[2] = makeuint16(gbuf[11], gbuf[10]);
					DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);

					ls53[sel].ambient= count[0];
					ls53[sel].signal= count[1];
					if(count[2]<1800){ 
					if(count[2]!=20)
					ls53[sel].distance= (count[2]);
					//ls53[sel].mode=1; 
					}else{
					//ls53[sel].mode=2;
					}	
					state[sel]=0;
					if(ls53[sel].distance<set_dis)
						ls53[sel].mode=1;
					else
						ls53[sel].mode=0;
				}	
				else
				flag1[sel]++;
				//if(flag1>10)
				if(flag1[sel]>2)
					state[sel]=0;
			break;
		}
}	 
	
	

int I2c_Soft_Start_VL53(u8 sel)
{
  switch(sel){
	case 0:
	SDA_H_VL53O1;
	SCL_H_VL53O1;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O1)return 0;	//SDA?????????,??
	SDA_L_VL53O1;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O1) return 0;	//SDA??????????,??
	SDA_L_VL53O1;
	I2c_Soft_delay_VL53();
	break;
	case 1:
	SDA_H_VL53O2;
	SCL_H_VL53O2;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O2)return 0;	//SDA?????????,??
	SDA_L_VL53O2;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O2) return 0;	//SDA??????????,??
	SDA_L_VL53O2;
	I2c_Soft_delay_VL53();
	break;
	case 2:
	SDA_H_VL53O3;
	SCL_H_VL53O3;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O3)return 0;	//SDA?????????,??
	SDA_L_VL53O3;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O3) return 0;	//SDA??????????,??
	SDA_L_VL53O3;
	I2c_Soft_delay_VL53();
	break;
		case 3:
	SDA_H_VL53O4;
	SCL_H_VL53O4;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O4)return 0;	//SDA?????????,??
	SDA_L_VL53O4;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O4) return 0;	//SDA??????????,??
	SDA_L_VL53O4;
	I2c_Soft_delay_VL53();
	break;
			case 4:
	SDA_H_VL53O5;
	SCL_H_VL53O5;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O5)return 0;	//SDA?????????,??
	SDA_L_VL53O5;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O5) return 0;	//SDA??????????,??
	SDA_L_VL53O5;
	I2c_Soft_delay_VL53();
	break;
						case 5:
	SDA_H_VL53O6;
	SCL_H_VL53O6;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O6)return 0;	//SDA?????????,??
	SDA_L_VL53O6;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O6) return 0;	//SDA??????????,??
	SDA_L_VL53O6;
	I2c_Soft_delay_VL53();
	break;
										case 6:
	SDA_H_VL53O7;
	SCL_H_VL53O7;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O7)return 0;	//SDA?????????,??
	SDA_L_VL53O7;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O7) return 0;	//SDA??????????,??
	SDA_L_VL53O7;
	I2c_Soft_delay_VL53();
	break;
												case 7:
	SDA_H_VL53O8;
	SCL_H_VL53O8;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O8)return 0;	//SDA?????????,??
	SDA_L_VL53O8;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O8) return 0;	//SDA??????????,??
	SDA_L_VL53O8;
	I2c_Soft_delay_VL53();
	break;
														case 8:
	SDA_H_VL53O9;
	SCL_H_VL53O9;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O9)return 0;	//SDA?????????,??
	SDA_L_VL53O9;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O9) return 0;	//SDA??????????,??
	SDA_L_VL53O9;
	I2c_Soft_delay_VL53();
	break;
																											case 9:
	SDA_H_VL53O10;
	SCL_H_VL53O10;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O10)return 0;	//SDA?????????,??
	SDA_L_VL53O10;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O10) return 0;	//SDA??????????,??
	SDA_L_VL53O10;
	I2c_Soft_delay_VL53();
	break;
																																								case 10:
	SDA_H_VL53O11;
	SCL_H_VL53O11;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O11)return 0;	//SDA?????????,??
	SDA_L_VL53O11;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O11) return 0;	//SDA??????????,??
	SDA_L_VL53O11;
	I2c_Soft_delay_VL53();
	break;
																																																					case 11:
	SDA_H_VL53O12;
	SCL_H_VL53O12;
	I2c_Soft_delay_VL53();
	if(!SDA_read_VL53O12)return 0;	//SDA?????????,??
	SDA_L_VL53O12;
	I2c_Soft_delay_VL53();
	if(SDA_read_VL53O12) return 0;	//SDA??????????,??
	SDA_L_VL53O12;
	I2c_Soft_delay_VL53();
	break;
	}
	
	return 1;	
}

void I2c_Soft_Stop_VL53(u8 sel)
{
  switch(sel){
	case 0:
	SCL_L_VL53O1;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O1;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O1;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O1;
	I2c_Soft_delay_VL53();
	break;
	case 1:
	SCL_L_VL53O2;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O2;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O2;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O2;
	I2c_Soft_delay_VL53();
	break;
	case 2:
	SCL_L_VL53O3;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O3;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O3;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O3;
	I2c_Soft_delay_VL53();
	break;
	case 3:
	SCL_L_VL53O4;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O4;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O4;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O4;
	I2c_Soft_delay_VL53();
	break;
	case 4:
	SCL_L_VL53O5;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O5;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O5;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O5;
	I2c_Soft_delay_VL53();
	break;
	case 5:
	SCL_L_VL53O6;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O6;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O6;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O6;
	I2c_Soft_delay_VL53();
	break;
		case 6:
	SCL_L_VL53O7;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O7;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O7;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O7;
	I2c_Soft_delay_VL53();
	break;
			case 7:
	SCL_L_VL53O8;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O8;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O8;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O8;
	I2c_Soft_delay_VL53();
	break;
				case 8:
	SCL_L_VL53O9;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O9;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O9;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O9;
	I2c_Soft_delay_VL53();
	break;
						case 9:
	SCL_L_VL53O10;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O10;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O10;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O10;
	I2c_Soft_delay_VL53();
	break;
								case 10:
	SCL_L_VL53O11;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O11;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O11;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O11;
	I2c_Soft_delay_VL53();
	break;
										case 11:
	SCL_L_VL53O12;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O12;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O12;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O12;
	I2c_Soft_delay_VL53();
	break;
	}
}

void I2c_Soft_Ask_VL53(u8 sel)
{
	switch(sel){
	case 0:
	SCL_L_VL53O1;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O1;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O1;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O1;
	I2c_Soft_delay_VL53();
  break;
	case 1:
	SCL_L_VL53O2;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O2;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O2;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O2;
	I2c_Soft_delay_VL53();
  break;
	case 2:
	SCL_L_VL53O3;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O3;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O3;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O3;
	I2c_Soft_delay_VL53();
  break;
	case 3:
	SCL_L_VL53O4;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O4;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O4;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O4;
	I2c_Soft_delay_VL53();
  break;
		case 4:
	SCL_L_VL53O5;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O5;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O5;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O5;
	I2c_Soft_delay_VL53();
  break;
			case 5:
	SCL_L_VL53O6;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O6;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O6;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O6;
	I2c_Soft_delay_VL53();
  break;
						case 6:
	SCL_L_VL53O7;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O7;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O7;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O7;
	I2c_Soft_delay_VL53();
  break;
								case 7:
	SCL_L_VL53O8;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O8;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O8;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O8;
	I2c_Soft_delay_VL53();
  break;
										case 8:
	SCL_L_VL53O9;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O9;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O9;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O9;
	I2c_Soft_delay_VL53();
  break;
											case 9:
	SCL_L_VL53O10;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O10;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O10;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O10;
	I2c_Soft_delay_VL53();
  break;
												case 10:
	SCL_L_VL53O11;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O11;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O11;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O11;
	I2c_Soft_delay_VL53();
  break;
													case 11:
	SCL_L_VL53O12;
	I2c_Soft_delay_VL53();
	SDA_L_VL53O12;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O12;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O12;
	I2c_Soft_delay_VL53();
  break;
}
}

void I2c_Soft_NoAsk_VL53(u8 sel)
{
	switch(sel){
	case 0:
	SCL_L_VL53O1;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O1;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O1;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O1;
	I2c_Soft_delay_VL53();
  break;
	case 1:
	SCL_L_VL53O2;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O2;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O2;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O2;
	I2c_Soft_delay_VL53();
  break;
	case 2:
	SCL_L_VL53O3;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O3;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O3;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O3;
	I2c_Soft_delay_VL53();
  break;
	case 3:
	SCL_L_VL53O4;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O4;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O4;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O4;
	I2c_Soft_delay_VL53();
  break;
		case 4:
	SCL_L_VL53O5;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O5;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O5;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O5;
	I2c_Soft_delay_VL53();
  break;
		case 5:
	SCL_L_VL53O6;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O6;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O6;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O6;
	I2c_Soft_delay_VL53();
  break;
			case 6:
	SCL_L_VL53O7;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O7;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O7;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O7;
	I2c_Soft_delay_VL53();
  break;
				case 7:
	SCL_L_VL53O8;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O8;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O8;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O8;
	I2c_Soft_delay_VL53();
  break;
					case 8:
	SCL_L_VL53O9;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O9;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O9;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O9;
	I2c_Soft_delay_VL53();
  break;
					case 9:
	SCL_L_VL53O10;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O10;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O10;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O10;
	I2c_Soft_delay_VL53();
  break;
					case 10:
	SCL_L_VL53O11;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O11;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O11;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O11;
	I2c_Soft_delay_VL53();
  break;
					case 12:
	SCL_L_VL53O12;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O12;
	I2c_Soft_delay_VL53();
	SCL_H_VL53O12;
	I2c_Soft_delay_VL53();
	SCL_L_VL53O12;
	I2c_Soft_delay_VL53();
  break;
	}
}

int I2c_Soft_WaitAsk_VL53(u8 sel) 	 //???:=1?ASK,=0?ASK
{
 
	u8 ErrTime = 0;
	switch(sel){
	case 0:
	SCL_L_VL53O1;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O1;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O1;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O1)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O1;
	I2c_Soft_delay_VL53();
	break;
		case 1:
	SCL_L_VL53O2;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O2;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O2;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O2)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O2;
	I2c_Soft_delay_VL53();
	break;
	
		case 2:
	SCL_L_VL53O3;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O3;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O3;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O3)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O3;
	I2c_Soft_delay_VL53();
	break;
	
			case 3:
	SCL_L_VL53O4;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O4;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O4;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O4)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O4;
	I2c_Soft_delay_VL53();
	break;
	case 4:
	SCL_L_VL53O5;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O5;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O5;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O5)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O5;
	I2c_Soft_delay_VL53();
	break;
	case 5:
	SCL_L_VL53O6;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O6;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O6;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O6)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O6;
	I2c_Soft_delay_VL53();
	break;
	case 6:
	SCL_L_VL53O7;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O7;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O7;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O7)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O7;
	I2c_Soft_delay_VL53();
	break;
	case 7:
	SCL_L_VL53O8;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O8;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O8;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O8)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O8;
	I2c_Soft_delay_VL53();
	break;
	case 8:
	SCL_L_VL53O9;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O9;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O9;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O9)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O9;
	I2c_Soft_delay_VL53();
	break;
	case 9:
	SCL_L_VL53O10;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O10;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O10;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O10)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O10;
	I2c_Soft_delay_VL53();
	break;
	case 10:
	SCL_L_VL53O11;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O11;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O11;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O11)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O11;
	I2c_Soft_delay_VL53();
	break;
	case 11:
	SCL_L_VL53O12;
	I2c_Soft_delay_VL53();
	SDA_H_VL53O12;			
	I2c_Soft_delay_VL53();
	SCL_H_VL53O12;
	I2c_Soft_delay_VL53();
	while(SDA_read_VL53O12)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_VL53(sel);
			return 1;
		}
	}
	SCL_L_VL53O12;
	I2c_Soft_delay_VL53();
	break;
	}
	
	return 0;
}

void I2c_Soft_SendByte_VL53(u8 SendByte,u8 sel) //????????//
{
	  u8 i=8;
	  switch(sel){
	  case 0:
    while(i--)
    {
        SCL_L_VL53O1;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O1;  
      else 
        SDA_L_VL53O1;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O1;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O1;
	  break;
		 case 1:
    while(i--)
    {
        SCL_L_VL53O2;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O2;  
      else 
        SDA_L_VL53O2;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O2;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O2;
	  break;
		 case 2:
    while(i--)
    {
        SCL_L_VL53O3;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O3;  
      else 
        SDA_L_VL53O3;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O3;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O3;
	  break;
		 case 3:
    while(i--)
    {
        SCL_L_VL53O4;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O4;  
      else 
        SDA_L_VL53O4;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O4;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O4;
	  break;
		 case 4:
    while(i--)
    {
        SCL_L_VL53O5;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O5;  
      else 
        SDA_L_VL53O5;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O5;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O5;
	  break;
		 case 5:
    while(i--)
    {
        SCL_L_VL53O6;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O6;  
      else 
        SDA_L_VL53O6;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O6;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O6;
	  break;
			 case 6:
    while(i--)
    {
        SCL_L_VL53O7;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O7;  
      else 
        SDA_L_VL53O7;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O7;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O7;
	  break;
		 case 7:
    while(i--)
    {
        SCL_L_VL53O8;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O8;  
      else 
        SDA_L_VL53O8;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O8;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O8;
	  break;
		 case 8:
    while(i--)
    {
        SCL_L_VL53O9;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O9;  
      else 
        SDA_L_VL53O9;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O9;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O9;
	  break;
			 case 9:
    while(i--)
    {
        SCL_L_VL53O10;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O10;  
      else 
        SDA_L_VL53O10;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O10;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O10;
	  break;
			 case 10:
    while(i--)
    {
        SCL_L_VL53O11;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O11;  
      else 
        SDA_L_VL53O11;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O11;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O11;
	  break;
			 case 11:
    while(i--)
    {
        SCL_L_VL53O12;
        I2c_Soft_delay_VL53();
      if(SendByte&0x80)
        SDA_H_VL53O12;  
      else 
        SDA_L_VL53O12;   
        SendByte<<=1;
        I2c_Soft_delay_VL53();
				SCL_H_VL53O12;
				I2c_Soft_delay_VL53();
    }
    SCL_L_VL53O12;
	  break;
	}
}  

//?1???,ack=1?,??ACK,ack=0,??NACK
u8 I2c_Soft_ReadByte_VL53(u8 ask,u8 sel)  //????????//
{ 
 u8 i=8;
    u8 ReceiveByte=0;
  switch(sel){
	case 0:
    SDA_H_VL53O1;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O1;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O1;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O1)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O1;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
		case 1:
    SDA_H_VL53O2;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O2;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O2;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O2)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O2;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 2:
    SDA_H_VL53O3;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O3;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O3;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O3)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O3;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 3:
    SDA_H_VL53O4;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O4;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O4;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O4)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O4;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 4:
    SDA_H_VL53O5;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O5;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O5;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O5)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O5;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 5:
    SDA_H_VL53O6;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O6;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O6;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O6)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O6;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 6:
    SDA_H_VL53O7;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O7;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O7;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O7)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O7;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 7:
    SDA_H_VL53O8;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O8;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O8;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O8)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O8;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 8:
    SDA_H_VL53O9;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O9;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O9;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O9)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O9;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 9:
    SDA_H_VL53O10;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O10;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O10;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O10)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O10;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 10:
    SDA_H_VL53O11;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O11;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O11;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O11)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O11;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
	case 11:
    SDA_H_VL53O12;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_VL53O12;
      I2c_Soft_delay_VL53();
			SCL_H_VL53O12;
      I2c_Soft_delay_VL53();	
      if(SDA_read_VL53O12)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_VL53O12;

	if (ask)
		I2c_Soft_Ask_VL53(sel);
	else
		I2c_Soft_NoAsk_VL53(sel);  
	break;
}
    return ReceiveByte;
} 


// IIC???????
u8 IIC_Write_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 REG_data,u8 sel)
{
	I2c_Soft_Start_VL53(sel);
	I2c_Soft_SendByte_VL53(SlaveAddress<<1,sel);   
	if(I2c_Soft_WaitAsk_VL53(sel))
	{
		I2c_Soft_Stop_VL53(sel);
		return 1;
	}
	I2c_Soft_SendByte_VL53(REG_Address,sel);       
	I2c_Soft_WaitAsk_VL53(sel);	
	I2c_Soft_SendByte_VL53(REG_data,sel);
	I2c_Soft_WaitAsk_VL53(sel);   
	I2c_Soft_Stop_VL53(sel); 
	return 0;
}

// IIC?1????
u8 IIC_Read_1Byte_VL53(u8 SlaveAddress,u8 REG_Address,u8 *REG_data,u8 sel)
{      		
	I2c_Soft_Start_VL53(sel);
	I2c_Soft_SendByte_VL53(SlaveAddress<<1,sel); 
	if(I2c_Soft_WaitAsk_VL53(sel))
	{
		I2c_Soft_Stop_VL53(sel);
		return 1;
	}
	I2c_Soft_SendByte_VL53(REG_Address,sel);     
	I2c_Soft_WaitAsk_VL53(sel);
	I2c_Soft_Start_VL53(sel);
	I2c_Soft_SendByte_VL53(SlaveAddress<<1 | 0x01,sel);
	I2c_Soft_WaitAsk_VL53(sel);
	*REG_data= I2c_Soft_ReadByte_VL53(0,sel);
	I2c_Soft_Stop_VL53(sel);
	return 0;
}	

// IIC?n????
u8 IIC_Write_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf,u8 sel)
{	
	I2c_Soft_Start_VL53(sel);
	I2c_Soft_SendByte_VL53(SlaveAddress<<1,sel); 
	if(I2c_Soft_WaitAsk_VL53(sel))
	{
		I2c_Soft_Stop_VL53(sel);
		return 1;
	}
	I2c_Soft_SendByte_VL53(REG_Address,sel); 
	I2c_Soft_WaitAsk_VL53(sel);
	while(len--) 
	{
		I2c_Soft_SendByte_VL53(*buf++,sel); 
		I2c_Soft_WaitAsk_VL53(sel);
	}
	I2c_Soft_Stop_VL53(sel);
	return 0;
}

// IIC?n????
u8 IIC_Read_nByte_VL53(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf,u8 sel)
{	
	I2c_Soft_Start_VL53(sel);
	I2c_Soft_SendByte_VL53(SlaveAddress<<1,sel); 
	if(I2c_Soft_WaitAsk_VL53(sel))
	{
		I2c_Soft_Stop_VL53(sel);
		return 1;
	}
	I2c_Soft_SendByte_VL53(REG_Address,sel); 
	I2c_Soft_WaitAsk_VL53(sel);
	
	I2c_Soft_Start_VL53(sel);
	I2c_Soft_SendByte_VL53(SlaveAddress<<1 | 0x01,sel); 
	I2c_Soft_WaitAsk_VL53(sel);
	while(len) 
	{
		if(len == 1)
		{
			*buf = I2c_Soft_ReadByte_VL53(0,sel);
		}
		else
		{
			*buf = I2c_Soft_ReadByte_VL53(1,sel);
		}
		buf++;
		len--;
	}
	I2c_Soft_Stop_VL53(sel);
	return 0;
}


