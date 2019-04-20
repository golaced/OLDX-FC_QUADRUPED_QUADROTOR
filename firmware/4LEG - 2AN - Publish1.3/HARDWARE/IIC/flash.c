
#include "include.h"
#include "flash.h"
#include "mems.h"	
#include "flash_w25.h"
#include "vmc.h"
#include "icm20602.h"	
#include "math.h"	
//#include "mavl.h"

u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}

void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 


void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}
//-----------------------------------------存储参数
#define SIZE_PARAM 50*2
float k_sensitivity[3]={0.78,1,1};//感度
float yaw_off_qr=0;
u32 FLASH_SIZE=16*1024*1024;	//FLASH 大小为16字节
u16 LENGTH_OF_DRONE=330;//飞行器轴距
int H_INT; //悬停油门
float SONAR_HEIGHT=0.3+0.015;//超声波安装高度
u8 need_init_mems=0;//mems flash error
u16 SBUS_MIN =954;
u16 SBUS_MAX =2108;
u16 SBUS_MID =1524;
u16 SBUS_MIN_A =954;
u16 SBUS_MAX_A =2108;
u16 SBUS_MID_A =1524;

void READ_PARM(void)
{
u8 FLASH_Buffer[SIZE_PARAM]={0};	
u8 need_init=0;	
#if FLASH_USE_STM32
STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)FLASH_Buffer,SIZE);	
#else	
W25QXX_Read(FLASH_Buffer,FLASH_SIZE-(SIZE_PARAM+10),SIZE_PARAM);					//从倒数第100个地址处开始,读出SIZE个字节
#endif
mems.Gyro_Offset.x=(vs16)(FLASH_Buffer[1]<<8|FLASH_Buffer[0]);
mems.Gyro_Offset.y=(vs16)(FLASH_Buffer[3]<<8|FLASH_Buffer[2]);
mems.Gyro_Offset.z=(vs16)(FLASH_Buffer[5]<<8|FLASH_Buffer[4]);
	
mems.Acc_Offset.x=(vs16)(FLASH_Buffer[7]<<8|FLASH_Buffer[6]);
mems.Acc_Offset.y=(vs16)(FLASH_Buffer[9]<<8|FLASH_Buffer[8]);
mems.Acc_Offset.z=(vs16)(FLASH_Buffer[11]<<8|FLASH_Buffer[10]);
	
mems.Mag_Offset.x=(vs16)(FLASH_Buffer[13]<<8|FLASH_Buffer[12]);
mems.Mag_Offset.y=(vs16)(FLASH_Buffer[15]<<8|FLASH_Buffer[14]);
mems.Mag_Offset.z=(vs16)(FLASH_Buffer[17]<<8|FLASH_Buffer[16]);
	
mems.Mag_Gain.x =(float)((vs16)((FLASH_Buffer[19]<<8|FLASH_Buffer[18])))/100.;
mems.Mag_Gain.y =(float)((vs16)((FLASH_Buffer[21]<<8|FLASH_Buffer[20])))/100.;
mems.Mag_Gain.z =(float)((vs16)((FLASH_Buffer[23]<<8|FLASH_Buffer[22])))/100.;
	
if(fabs(mems.Mag_Gain.x)<10&&mems.Mag_Gain.x!=-1&&fabs(mems.Mag_Offset.x)<666&&mems.Mag_Offset.x!=-1)
	mems.Mag_Have_Param=1;

vmc[0].param.PWM_OFF[0]=(vs16)(FLASH_Buffer[25]<<8|FLASH_Buffer[24]);
vmc[0].param.PWM_OFF[1]=(vs16)(FLASH_Buffer[27]<<8|FLASH_Buffer[26]);
vmc[0].param.PWM_OFF[2]=(vs16)(FLASH_Buffer[29]<<8|FLASH_Buffer[28]);

vmc[1].param.PWM_OFF[0]=(vs16)(FLASH_Buffer[31]<<8|FLASH_Buffer[30]);
vmc[1].param.PWM_OFF[1]=(vs16)(FLASH_Buffer[33]<<8|FLASH_Buffer[32]);
vmc[1].param.PWM_OFF[2]=(vs16)(FLASH_Buffer[35]<<8|FLASH_Buffer[34]);

vmc[2].param.PWM_OFF[0]=(vs16)(FLASH_Buffer[37]<<8|FLASH_Buffer[36]);
vmc[2].param.PWM_OFF[1]=(vs16)(FLASH_Buffer[39]<<8|FLASH_Buffer[38]);
vmc[2].param.PWM_OFF[2]=(vs16)(FLASH_Buffer[41]<<8|FLASH_Buffer[40]);

vmc[3].param.PWM_OFF[0]=(vs16)(FLASH_Buffer[43]<<8|FLASH_Buffer[42]);
vmc[3].param.PWM_OFF[1]=(vs16)(FLASH_Buffer[45]<<8|FLASH_Buffer[44]);
vmc[3].param.PWM_OFF[2]=(vs16)(FLASH_Buffer[47]<<8|FLASH_Buffer[46]);

vmc_all.param.ground_force[0][0]=(float)((vs16)(FLASH_Buffer[49]<<8|FLASH_Buffer[48]))/100.;
vmc_all.param.ground_force[1][0]=(float)((vs16)(FLASH_Buffer[51]<<8|FLASH_Buffer[50]))/100.;
vmc_all.param.ground_force[2][0]=(float)((vs16)(FLASH_Buffer[53]<<8|FLASH_Buffer[52]))/100.;
vmc_all.param.ground_force[3][0]=(float)((vs16)(FLASH_Buffer[55]<<8|FLASH_Buffer[54]))/100.;

vmc_all.param.ground_force[0][1]=(float)((vs16)(FLASH_Buffer[57]<<8|FLASH_Buffer[56]))/100.;
vmc_all.param.ground_force[1][1]=(float)((vs16)(FLASH_Buffer[59]<<8|FLASH_Buffer[58]))/100.;
vmc_all.param.ground_force[2][1]=(float)((vs16)(FLASH_Buffer[61]<<8|FLASH_Buffer[60]))/100.;
vmc_all.param.ground_force[3][1]=(float)((vs16)(FLASH_Buffer[63]<<8|FLASH_Buffer[62]))/100.;

vmc_all.tar_att_off[PITr]=(float)((vs16)(FLASH_Buffer[65]<<8|FLASH_Buffer[64]))/100.;
vmc_all.tar_att_off[ROLr]=(float)((vs16)(FLASH_Buffer[67]<<8|FLASH_Buffer[66]))/100.;

vmc_all.your_key[0]=FLASH_Buffer[68];
vmc_all.your_key[1]=FLASH_Buffer[69];
vmc_all.your_key[2]=FLASH_Buffer[70];

CHE=FLASH_Buffer[71];
if(CHE==255)
	CHE=11;
if(SBUS_MIN==65535)
{
SBUS_MIN=860;
SBUS_MID=1524;
SBUS_MAX=2180;

SBUS_MIN_A=644;
SBUS_MID_A=1524;
SBUS_MAX_A=2484;	
}	
}

void WRITE_PARM(void)
{ 

int16_t _temp;
u8 cnt=0,i;
u8 FLASH_Buffer[SIZE_PARAM]={0};
_temp=(int16_t)mems.Gyro_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mems.Gyro_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mems.Gyro_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);

_temp=(int16_t)mems.Acc_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mems.Acc_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mems.Acc_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mems.Mag_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mems.Mag_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mems.Mag_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);


_temp=(int16_t)(mems.Mag_Gain.x*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(mems.Mag_Gain.y*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(mems.Mag_Gain.z*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);


for(i=0;i<4;i++){
_temp=vmc[i].param.PWM_OFF[0];
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=vmc[i].param.PWM_OFF[1];
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=vmc[i].param.PWM_OFF[2];
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
}

for(i=0;i<4;i++){
_temp=vmc_all.param.ground_force[i][0]*100;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
}

for(i=0;i<4;i++){
_temp=vmc_all.param.ground_force[i][1]*100;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
}

_temp=vmc_all.tar_att_off[PITr]*100;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);

_temp=vmc_all.tar_att_off[ROLr]*100;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);

FLASH_Buffer[cnt++]=vmc_all.your_key[0];
FLASH_Buffer[cnt++]=vmc_all.your_key[1];
FLASH_Buffer[cnt++]=vmc_all.your_key[2];

FLASH_Buffer[cnt++]=BYTE0(CHE);

#if FLASH_USE_STM32
STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)FLASH_Buffer,SIZE);
#else
W25QXX_Write((u8*)FLASH_Buffer,FLASH_SIZE-(SIZE_PARAM+10),SIZE_PARAM);		//从倒数第100个地址处开始,写入SIZE长度的数据
#endif
}


//#define FRAM_SIZE 16
//#define SIZE_WAY FRAM_SIZE*(NAV_MAX_MISSION_LEGS+1)
//void WRITE_PARM_WAY_POINTS(void)
//{ 
//int16_t _temp;
//int32_t	_temp32;
//u16 cnt=0,i;
//u8 FLASH_Bufferw[SIZE_WAY]={0};		
//int max_num=LIMIT(SIZE_WAY/FRAM_SIZE-2,0,NAV_MAX_MISSION_LEGS);
//FLASH_Bufferw[cnt++]=navData.Leg_num;
//for(i=0;i<LIMIT(navData.Leg_num,0,max_num);i++){
//_temp32=(int32_t)(navData.missionLegs[i].targetLat*10000000);
//FLASH_Bufferw[cnt++]=BYTE0(_temp32);
//FLASH_Bufferw[cnt++]=BYTE1(_temp32);
//FLASH_Bufferw[cnt++]=BYTE2(_temp32);
//FLASH_Bufferw[cnt++]=BYTE3(_temp32);
//_temp32=(int32_t)(navData.missionLegs[i].targetLon*10000000);
//FLASH_Bufferw[cnt++]=BYTE0(_temp32);
//FLASH_Bufferw[cnt++]=BYTE1(_temp32);
//FLASH_Bufferw[cnt++]=BYTE2(_temp32);
//FLASH_Bufferw[cnt++]=BYTE3(_temp32);
//_temp=(int16_t)(navData.missionLegs[i].targetAlt*10);
//FLASH_Bufferw[cnt++]=BYTE0(_temp);
//FLASH_Bufferw[cnt++]=BYTE1(_temp);
//_temp=(int16_t)(navData.missionLegs[i].poiHeading*1000);
//FLASH_Bufferw[cnt++]=BYTE0(_temp);
//FLASH_Bufferw[cnt++]=BYTE1(_temp);
//_temp=(int16_t)(navData.missionLegs[i].maxHorizSpeed*100);
//FLASH_Bufferw[cnt++]=BYTE0(_temp);
//FLASH_Bufferw[cnt++]=BYTE1(_temp);
//_temp=(int16_t)(navData.missionLegs[i].loiterTime*100);
//FLASH_Bufferw[cnt++]=BYTE0(_temp);
//FLASH_Bufferw[cnt++]=BYTE1(_temp);
//}

//W25QXX_Write((u8*)FLASH_Bufferw,FLASH_SIZE-(SIZE_WAY+10+SIZE_PARAM+10),SIZE_WAY);		//从倒数第100个地址处开始,写入SIZE长度的数据
//}

//void READ_WAY_POINTS(void)
//{
//u16 i;
//u8 FLASH_Bufferw[SIZE_WAY]={0};	
//W25QXX_Read(FLASH_Bufferw,FLASH_SIZE-(SIZE_WAY+10+SIZE_PARAM+10),SIZE_WAY);					//从倒数第100个地址处开始,读出SIZE个字节
//navData.Leg_num=LIMIT(FLASH_Bufferw[0],0,NAV_MAX_MISSION_LEGS);
//for(i=0;i<navData.Leg_num;i++){
//navData.missionLegs[i].targetLat=(float)((vs32)(FLASH_Bufferw[4+i*FRAM_SIZE]<<24|FLASH_Bufferw[3+i*FRAM_SIZE]<<16|
//	FLASH_Bufferw[2+i*FRAM_SIZE]<<8|FLASH_Bufferw[1+i*FRAM_SIZE]))/10000000.;
//navData.missionLegs[i].targetLon=(float)((vs32)(FLASH_Bufferw[8+i*FRAM_SIZE]<<24|FLASH_Bufferw[7+i*FRAM_SIZE]<<16|
//	FLASH_Bufferw[6+i*FRAM_SIZE]<<8|FLASH_Bufferw[5+i*FRAM_SIZE]))/10000000.;
//navData.missionLegs[i].targetAlt=(float)((vs16)(FLASH_Bufferw[10+i*FRAM_SIZE]<<8|FLASH_Bufferw[9+i*FRAM_SIZE]))/10.;
//navData.missionLegs[i].poiHeading=(float)((vs16)(FLASH_Bufferw[12+i*FRAM_SIZE]<<8|FLASH_Bufferw[11+i*FRAM_SIZE]))/1000.;
//navData.missionLegs[i].maxHorizSpeed=(float)((vs16)(FLASH_Bufferw[14+i*FRAM_SIZE]<<8|FLASH_Bufferw[13+i*FRAM_SIZE]))/100.;
//navData.missionLegs[i].loiterTime=(float)((vs16)(FLASH_Bufferw[16+i*FRAM_SIZE]<<8|FLASH_Bufferw[15+i*FRAM_SIZE]))/100.;
//if(navData.missionLegs[i].targetLat>20&&navData.missionLegs[i].targetLon>20)
//	navData.missionLegs[i].type=2;
//}
//}