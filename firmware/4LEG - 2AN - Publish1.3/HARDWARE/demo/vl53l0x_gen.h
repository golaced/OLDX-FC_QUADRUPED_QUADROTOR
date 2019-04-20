#ifndef __VL53L0X_GEN_H
#define __VL53L0X_GEN_H

#include "vl53l0x.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 探索者STM32F407开发板
//VL53L0X-普通测量模式 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

extern VL53L0X_RangingMeasurementData_t vl53l0x_data;

VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode);
void vl53l0x_general_test(VL53L0X_Dev_t *dev);

#endif


