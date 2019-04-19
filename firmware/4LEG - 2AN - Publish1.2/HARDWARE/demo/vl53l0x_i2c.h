#ifndef __VL53L0_I2C_H
#define __VL53L0_I2C_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 探索者STM32F407开发板
//VL53L0X IIC驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//IO方向设置
#define VL_SDA_IN()  {GPIOB->MODER&=~(3<<(12*2));GPIOB->MODER|=0<<12*2;}	//PB10输入模式
#define VL_SDA_OUT() {GPIOB->MODER&=~(3<<(12*2));GPIOB->MODER|=1<<21*2;}    //PB10输出模式


//IO操作函数	 
#define VL_IIC_SCL    PBout(6) 		//SCL
#define VL_IIC_SDA    PBout(12) 		//SDA	 
#define VL_READ_SDA   PBin(12) 		    //输入SDA 

//状态
#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

//IIC操作函数
void VL53L0X_i2c_init(void);//初始化IIC的IO口

u8 VL53L0X_write_byte(u8 address,u8 index,u8 data);              //IIC写一个8位数据
u8 VL53L0X_write_word(u8 address,u8 index,u16 data);             //IIC写一个16位数据
u8 VL53L0X_write_dword(u8 address,u8 index,u32 data);            //IIC写一个32位数据
u8 VL53L0X_write_multi(u8 address, u8 index,u8 *pdata,u16 count);//IIC连续写

u8 VL53L0X_read_byte(u8 address,u8 index,u8 *pdata);             //IIC读一个8位数据
u8 VL53L0X_read_word(u8 address,u8 index,u16 *pdata);            //IIC读一个16位数据
u8 VL53L0X_read_dword(u8 address,u8 index,u32 *pdata);           //IIC读一个32位数据
u8 VL53L0X_read_multi(u8 address,u8 index,u8 *pdata,u16 count);  //IIC连续读


#endif 


