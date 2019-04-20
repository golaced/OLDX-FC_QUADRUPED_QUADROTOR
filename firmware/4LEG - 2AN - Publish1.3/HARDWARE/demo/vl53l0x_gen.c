#include "vl53l0x_gen.h"

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

VL53L0X_RangingMeasurementData_t vl53l0x_data;//测距测量结构体
vu16 Distance_data=0;//保存测距数据

//VL53L0X 测量模式配置
//dev:设备I2C参数结构体
//mode: 0:默认;1:高精度;2:长距离
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode)
{
	
	 VL53L0X_Error status = VL53L0X_ERROR_NONE;
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	
	 vl53l0x_reset(dev);//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	 status = VL53L0X_StaticInit(dev);

     if(AjustOK!=0)//已校准好了,写入校准值
     {
	    status= VL53L0X_SetReferenceSpads(dev,Vl53l0x_data.refSpadCount,Vl53l0x_data.isApertureSpads);//设定Spads校准值
        if(status!=VL53L0X_ERROR_NONE) goto error;	
        delay_ms(2);		 
	    status= VL53L0X_SetRefCalibration(dev,Vl53l0x_data.VhvSettings,Vl53l0x_data.PhaseCal);//设定Ref校准值
		if(status!=VL53L0X_ERROR_NONE) goto error;
		 delay_ms(2);
	    status=VL53L0X_SetOffsetCalibrationDataMicroMeter(dev,Vl53l0x_data.OffsetMicroMeter);//设定偏移校准值
		if(status!=VL53L0X_ERROR_NONE) goto error; 
		 delay_ms(2);
		status=VL53L0X_SetXTalkCompensationRateMegaCps(dev,Vl53l0x_data.XTalkCompensationRateMegaCps);//设定串扰校准值
		if(status!=VL53L0X_ERROR_NONE) goto error;
         delay_ms(2);		 
		 
     }
	 else
	 {
		status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref参考校准
		if(status!=VL53L0X_ERROR_NONE) goto error;
		delay_ms(2);
		status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//执行参考SPAD管理
		if(status!=VL53L0X_ERROR_NONE) goto error;
        delay_ms(2);		 	 
	 }
	 status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_SINGLE_RANGING);//使能单次测量模式
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,1);//使能SIGMA范围检查
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//使能信号速率范围检查
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,Mode_data[mode].sigmaLimit);//设定SIGMA范围
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,Mode_data[mode].signalLimit);//设定信号速率范围范围
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,Mode_data[mode].timingBudget);//设定完整测距最长时间
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//设定VCSEL脉冲周期
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//设定VCSEL脉冲周期范围
	 
	 error://错误信息
	 if(status!=VL53L0X_ERROR_NONE)
	 {
	//	print_pal_error(status);
	//	LCD_Fill(30,140+20,300,300,WHITE);
		return status;
	 }
	 return status;
	
}	

//VL53L0X 单次距离测量函数
//dev:设备I2C参数结构体
//pdata:保存测量数据结构体
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata,char *buf)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;
	
	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//执行单次测距并获取测距测量数据
	if(status !=VL53L0X_ERROR_NONE) return status;
   
	RangeStatus = pdata->RangeStatus;//获取当前测量状态
    memset(buf,0x00,VL53L0X_MAX_STRING_LENGTH);
	VL53L0X_GetRangeStatusString(RangeStatus,buf);//根据测量状态读取状态字符串
	
	Distance_data = pdata->RangeMilliMeter;//保存最近一次测距测量数据
	
    return status;
}


//启动普通测量
//dev：设备I2C参数结构体
//mode模式配置 0:默认;1:高精度;2:长距离
void vl53l0x_general_start(VL53L0X_Dev_t *dev,u8 mode)
{
	static char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区
	VL53L0X_Error Status=VL53L0X_ERROR_NONE;//工作状态
	u8 key=0;
	u8 i=0;
	
	//LED0=1;
	//mode_string(mode,buf);//显示当前配置的模式
	while(vl53l0x_set_mode(dev,mode))//配置测量模式
	{
		//LCD_ShowString(30,140+40,200,16,16,"Mode Set Error!!!");
		delay_ms(500);
		//LCD_ShowString(30,140+40,200,16,16,"                 ");
		delay_ms(500);
		i++;
		if(i==2) return;
	}
//	LCD_Fill(30,140+20,300,300,WHITE);
//	LCD_ShowString(30,140+30,200,16,16,"KEY_UP: Exit the test             ");		
//	LCD_ShowString(30,140+50,200,16,16,"Mode:        ");
//	LCD_ShowString(80,140+50,200,16,16,(u8*)buf);
//	LCD_ShowString(30,140+70,200,16,16,"State:");//显示测量状态	 
//	LCD_ShowString(30,140+90,200,16,16,"Distance:    0 mm");//显示测量距离 
//	while(1)
//	{
//		// key = KEY_Scan(0);
//		 if(key == WKUP_PRES)
//		 {
//			 //LED1=1;
//			 break;//返回上一菜单
//			 
//		 }
		 if(Status==VL53L0X_ERROR_NONE)
		 {
				Status = vl53l0x_start_single_test(dev,&vl53l0x_data,buf);//执行一次测量
			  //vl53l0x_data.RangeStatus
				//LCD_ShowString(85,140+70,200,16,16,(u8*)buf);
				//LCD_ShowxNum(110,140+90,Distance_data,4,16,0);
				//printf("State;%i , %s\r\n",vl53l0x_data.RangeStatus,buf);//打印测量状态	
				//printf("d: %4imm\r\n",Distance_data);//打印测量距离
		 }
		 i++; 
		 if(i==5)
		 {
			 i=0;
			 //LED1=!LED1;	
		 }
//         delay_ms(50);	
//		
//	}	
	
}

//vl53l0x普通测量模式UI
void general_ui(void)
{
//	LCD_Fill(30,140+20,300,300,WHITE);
//	POINT_COLOR=RED;        //设置字体为红色 
//	LCD_ShowString(30,140+30,300,16,16,"General Mode                  ");
//	LCD_ShowString(30,140+55,300,16,16,"KEY1: Switch working mode    ");
//	POINT_COLOR=BLUE;       //设置字体为蓝色 
//	LCD_ShowString(30,140+75,300,16,16, "KEY_UP: Return menu    ");
//	LCD_ShowString(30,140+95,300,16,16, "KEY0:   Default        ");
	
}

//vl53l0x普通测量模式测试
//dev:设备I2C参数结构体
void vl53l0x_general_test(VL53L0X_Dev_t *dev)
{
//	u8 key=0;
//	u8 i=0;
	u8 mode=0;
//	LED1=1;
//	general_ui();//显示普通测量模式UI
//	
//	while(1)
//	{	

//		key = KEY_Scan(0);
//		
//		if(key==WKUP_PRES)	break;//返回主菜单 	
//		
//		else if(key==KEY1_PRES)//选择工作模式
//		{
//             mode++;
	  mode=HIGH_SPEED;
//			 if(mode==4) mode=0;
////			 switch(mode)
////			 {
////				 case Default_Mode:  LCD_ShowString(95,140+95,300,16,16, "Default        "); break;//默认
////				 case HIGH_ACCURACY: LCD_ShowString(95,140+95,300,16,16, "High Accuracy  "); break;//高精度
////				 case LONG_RANGE:    LCD_ShowString(95,140+95,300,16,16, "Long Range     "); break;//长距离
////				 case HIGH_SPEED:    LCD_ShowString(95,140+95,300,16,16, "High Speed     "); break;//高速
////			 }
//		}	
//		else if(key==KEY0_PRES)//启动测量
//		{
     	vl53l0x_general_start(dev,mode);
//	        general_ui();
//			mode=0;
//		}			
//		i++;
//		if(i==5)
//		{
//			i=0;
//			LED0=!LED0;
//		}
//		delay_ms(50);

//	}
//	
}
