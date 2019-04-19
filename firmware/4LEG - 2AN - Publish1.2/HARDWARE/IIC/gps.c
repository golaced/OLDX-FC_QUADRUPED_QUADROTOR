#include "gps.h" 
#include "delay.h" 								   
#include "usart.h" 								   
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "math.h"
#include "time.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//ATK-NEO-6M GPS模块驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/10/26
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved							  
////////////////////////////////////////////////////////////////////////////////// 	   


//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n函数
//返回值:m^n次方.
u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	

float NMEA_Str2float(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	float res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires+(float)fres/NMEA_Pow(10,flen);
	if(mask&0X02)res=-res;		   
	return res;
}	  	
//分析GPGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx=0;
	u8 posx;   	 
	p=buf;
	p1=(u8*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//得到GPGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(u8*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个GPGSV信息
	}   
}
//分析GPGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
/*
定位数据指示	数据描述
0	修正无效或不可用
1	单点
2	伪距差分
4	RTK固定解  //---
5	RTK浮动解
6	航位推算模式
7	手动模式
8	模拟模式
9	WAAS (SBAS)模式
*/
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"GPGGA");
	posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}
//分析GPGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx; 
	u8 i;   
	p1=(u8*)strstr((const char *)buf,"$GNGSA");
	posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//得到定位卫星编号
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
//分析GPRMC信息
//gpsx:nmea信息结构体    longitude-->精度   LAT--》维度
//buf:接收到的GPS数据缓冲区首地址25.0772235
u8 GPS_SEL1=7;
float gps_dt;
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{u32 temp_gps;
	u8 *p1,dx;			 
	u8 posx;     
	u32 temp;	   
	float rs;  
	p1=(u8*)strstr((const char *)buf,"GPRMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	
	if(posx!=0XFF)
	{ // gps_dt = Get_Cycle_T(GET_T_GPS);		
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		temp_gps=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		temp_gps=temp_gps*NMEA_Pow(10,GPS_SEL1)+(rs*NMEA_Pow(10,GPS_SEL1-dx))/60;//转换为° 
		gpsx->latitude=(double)temp_gps/NMEA_Pow(10,GPS_SEL1);
	}
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		temp_gps=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		temp_gps=temp_gps*NMEA_Pow(10,GPS_SEL1)+(rs*NMEA_Pow(10,GPS_SEL1-dx))/60;//转换为° 
		gpsx->longitude=(double)temp_gps/NMEA_Pow(10,GPS_SEL1);
	}
	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
	posx=NMEA_Comma_Pos(p1,7);		//速度  m/s
	if(posx!=0XFF)
		{ 
			gpsx->spd=(float)(NMEA_Str2float(p1+posx,&dx)*1.852*1000)/3600.;       //???????????????(0-359?)(??100?)
		}
		posx=NMEA_Comma_Pos(p1,8);		//角度
	if(posx!=0XFF)
		{
			gpsx->angle=NMEA_Str2float(p1+posx,&dx);       //???????????????(0-359?)(??100?)
		}	
	/*
模式简写	模式指示
A	自助定位
D	差分
E	估算模式
M	手动输入
N	数据无效
*/		
	posx=NMEA_Comma_Pos(p1,2);								//东经还是西经
	if(posx!=0XFF)gpsx->rmc_mode=*(p1+posx);		 	
	
}


//分析GPVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
u8 buf[50];
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GNVTG");			
	posx=NMEA_Comma_Pos(p1,1);								
	/*if(posx!=0XFF)
	{
		gpsx->course_earth=NMEA_Str2num(p1+posx,&dx);       //???????????????(0-359?)(??100?)
	}	
	posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
	}*/
		p1=(uint8_t*)strstr((const char *)buf,"$GNVTG");				//?????$GNVTG?p?????????		 
		posx=NMEA_Comma_Pos(p1,1);			
	u8 i;
for(i=0;i<50;i++)
buf[i]=	posx++;
		if(posx!=0XFF)
		{
			gpsx->course_earth=NMEA_Str2num(p1+posx,&dx);       //???????????????(0-359?)(??100?)
		}
			posx=NMEA_Comma_Pos(p1,3);								
		if(posx!=0XFF)
		{
			gpsx->course_mag=NMEA_Str2num(p1+posx,&dx);         //???????????????(0-359?)(??100?)
		}
			posx=NMEA_Comma_Pos(p1,7);								//??????
		if(posx!=0XFF)
		{
			gpsx->speed=NMEA_Str2num(p1+posx,&dx);
			if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);             //????1000?(1??/?? -> 0.001??/??)
			else if(dx>3)gpsx->speed/=NMEA_Pow(10,dx-3);        //????1000?(1??/?? -> 0.001??/??)(??1000?)
		}
}  
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
	NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析
	NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA解析 	
	NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA解析
	NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC解析
	NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG解析
}

////GPS校验和计算
////buf:数据缓存区首地址
////len:数据长度
////cka,ckb:两个校验结果.
//void Ublox_CheckSum(u8 *buf,u16 len,u8* cka,u8*ckb)
//{
//	u16 i;
//	*cka=0;*ckb=0;
//	for(i=0;i<len;i++)
//	{
//		*cka=*cka+buf[i];
//		*ckb=*ckb+*cka;
//	}
//}
///////////////////////////////////////////UBLOX 配置代码/////////////////////////////////////
////检查CFG配置执行情况
////返回值:0,ACK成功
////       1,接收超时错误
////       2,没有找到同步字符
////       3,接收到NACK应答
//u8 Ublox_Cfg_Ack_Check(void)
//{			 
//	u16 len=0,i;
//	u8 rval=0;
//	while((USART3_RX_STA&0X8000)==0 && len<100)//等待接收到应答   
//	{
//		len++;
//		delay_ms(5);
//	}		 
//	if(len<250)   	//超时错误.
//	{
//		len=USART3_RX_STA&0X7FFF;	//此次接收到的数据长度 
//		for(i=0;i<len;i++)if(USART3_RX_BUF[i]==0XB5)break;//查找同步字符 0XB5
//		if(i==len)rval=2;						//没有找到同步字符
//		else if(USART3_RX_BUF[i+3]==0X00)rval=3;//接收到NACK应答
//		else rval=0;	   						//接收到ACK应答
//	}else rval=1;								//接收超时错误
//    USART3_RX_STA=0;							//清除接收
//	return rval;  
//}
////配置保存
////将当前配置保存在外部EEPROM里面
////返回值:0,执行成功;1,执行失败.
//u8 Ublox_Cfg_Cfg_Save(void)
//{
//	u8 i;
//	_ublox_cfg_cfg *cfg_cfg=(_ublox_cfg_cfg *)USART3_TX_BUF;
//	cfg_cfg->header=0X62B5;		//cfg header
//	cfg_cfg->id=0X0906;			//cfg cfg id
//	cfg_cfg->dlength=13;		//数据区长度为13个字节.		 
//	cfg_cfg->clearmask=0;		//清除掩码为0
//	cfg_cfg->savemask=0XFFFF; 	//保存掩码为0XFFFF
//	cfg_cfg->loadmask=0; 		//加载掩码为0 
//	cfg_cfg->devicemask=4; 		//保存在EEPROM里面		 
//	Ublox_CheckSum((u8*)(&cfg_cfg->id),sizeof(_ublox_cfg_cfg)-4,&cfg_cfg->cka,&cfg_cfg->ckb);
//	Ublox_Send_Date((u8*)cfg_cfg,sizeof(_ublox_cfg_cfg));//发送数据给NEO-6M     
//	for(i=0;i<6;i++)if(Ublox_Cfg_Ack_Check()==0)break;		//EEPROM写入需要比较久时间,所以连续判断多次
//	return i==6?1:0;
//}
////配置NMEA输出信息格式
////msgid:要操作的NMEA消息条目,具体见下面的参数表
////      00,GPGGA;01,GPGLL;02,GPGSA;
////		03,GPGSV;04,GPRMC;05,GPVTG;
////		06,GPGRS;07,GPGST;08,GPZDA;
////		09,GPGBS;0A,GPDTM;0D,GPGNS;
////uart1set:0,输出关闭;1,输出开启.	  
////返回值:0,执行成功;其他,执行失败.
//u8 Ublox_Cfg_Msg(u8 msgid,u8 uart1set)
//{
//	_ublox_cfg_msg *cfg_msg=(_ublox_cfg_msg *)USART3_TX_BUF;
//	cfg_msg->header=0X62B5;		//cfg header
//	cfg_msg->id=0X0106;			//cfg msg id
//	cfg_msg->dlength=8;			//数据区长度为8个字节.	
//	cfg_msg->msgclass=0XF0;  	//NMEA消息
//	cfg_msg->msgid=msgid; 		//要操作的NMEA消息条目
//	cfg_msg->iicset=1; 			//默认开启
//	cfg_msg->uart1set=uart1set; //开关设置
//	cfg_msg->uart2set=1; 	 	//默认开启
//	cfg_msg->usbset=1; 			//默认开启
//	cfg_msg->spiset=1; 			//默认开启
//	cfg_msg->ncset=1; 			//默认开启	  
//	Ublox_CheckSum((u8*)(&cfg_msg->id),sizeof(_ublox_cfg_msg)-4,&cfg_msg->cka,&cfg_msg->ckb);
//	Ublox_Send_Date((u8*)cfg_msg,sizeof(_ublox_cfg_msg));//发送数据给NEO-6M    
//	return Ublox_Cfg_Ack_Check();
//}
////配置NMEA输出信息格式
////baudrate:波特率,4800/9600/19200/38400/57600/115200/230400	  
////返回值:0,执行成功;其他,执行失败(这里不会返回0了)
//u8 Ublox_Cfg_Prt(u32 baudrate)
//{
//	_ublox_cfg_prt *cfg_prt=(_ublox_cfg_prt *)USART3_TX_BUF;
//	cfg_prt->header=0X62B5;		//cfg header
//	cfg_prt->id=0X0006;			//cfg prt id
//	cfg_prt->dlength=20;		//数据区长度为20个字节.	
//	cfg_prt->portid=1;			//操作串口1
//	cfg_prt->reserved=0;	 	//保留字节,设置为0
//	cfg_prt->txready=0;	 		//TX Ready设置为0
//	cfg_prt->mode=0X08D0; 		//8位,1个停止位,无校验位
//	cfg_prt->baudrate=baudrate; //波特率设置
//	cfg_prt->inprotomask=0X0007;//0+1+2
//	cfg_prt->outprotomask=0X0007;//0+1+2
// 	cfg_prt->reserved4=0; 		//保留字节,设置为0
// 	cfg_prt->reserved5=0; 		//保留字节,设置为0 
//	Ublox_CheckSum((u8*)(&cfg_prt->id),sizeof(_ublox_cfg_prt)-4,&cfg_prt->cka,&cfg_prt->ckb);
//	Ublox_Send_Date((u8*)cfg_prt,sizeof(_ublox_cfg_prt));//发送数据给NEO-6M   
//	delay_ms(200);				//等待发送完成 
//	//usart3_init(baudrate);	//重新初始化串口3  
//	return Ublox_Cfg_Ack_Check();//这里不会反回0,因为UBLOX发回来的应答在串口重新初始化的时候已经被丢弃了.
//} 
////配置UBLOX NEO-6的时钟脉冲输出
////interval:脉冲间隔(us)
////length:脉冲宽度(us)
////status:脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.
////返回值:0,发送成功;其他,发送失败.
//u8 Ublox_Cfg_Tp(u32 interval,u32 length,signed char status)
//{
//	_ublox_cfg_tp *cfg_tp=(_ublox_cfg_tp *)USART3_TX_BUF;
//	cfg_tp->header=0X62B5;		//cfg header
//	cfg_tp->id=0X0706;			//cfg tp id
//	cfg_tp->dlength=20;			//数据区长度为20个字节.
//	cfg_tp->interval=interval;	//脉冲间隔,us
//	cfg_tp->length=length;		//脉冲宽度,us
//	cfg_tp->status=status;	   	//时钟脉冲配置
//	cfg_tp->timeref=0;			//参考UTC 时间
//	cfg_tp->flags=0;			//flags为0
//	cfg_tp->reserved=0;		 	//保留位为0
//	cfg_tp->antdelay=820;    	//天线延时为820ns
//	cfg_tp->rfdelay=0;    		//RF延时为0ns
//	cfg_tp->userdelay=0;    	//用户延时为0ns
//	Ublox_CheckSum((u8*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
//	Ublox_Send_Date((u8*)cfg_tp,sizeof(_ublox_cfg_tp));//发送数据给NEO-6M  
//	return Ublox_Cfg_Ack_Check();
//}
////配置UBLOX NEO-6的更新速率	    
////measrate:测量时间间隔，单位为ms，最少不能小于200ms（5Hz）
////reftime:参考时间，0=UTC Time；1=GPS Time（一般设置为1）
////返回值:0,发送成功;其他,发送失败.
//u8 Ublox_Cfg_Rate(u16 measrate,u8 reftime)
//{
//	_ublox_cfg_rate *cfg_rate=(_ublox_cfg_rate *)USART3_TX_BUF;
// 	if(measrate<200)return 1;	//小于200ms，直接退出
// 	cfg_rate->header=0X62B5;	//cfg header
//	cfg_rate->id=0X0806;	 	//cfg rate id
//	cfg_rate->dlength=6;	 	//数据区长度为6个字节.
//	cfg_rate->measrate=measrate;//脉冲间隔,us
//	cfg_rate->navrate=1;		//导航速率（周期），固定为1
//	cfg_rate->timeref=reftime; 	//参考时间为GPS时间
//	Ublox_CheckSum((u8*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);
//	Ublox_Send_Date((u8*)cfg_rate,sizeof(_ublox_cfg_rate));//发送数据给NEO-6M 
//	return Ublox_Cfg_Ack_Check();
//}
////发送一批数据给Ublox NEO-6M，这里通过串口3发送
////dbuf：数据缓存首地址
////len：要发送的字节数
//void Ublox_Send_Date(u8* dbuf,u16 len)
//{
//	u16 j;
//	for(j=0;j<len;j++)//循环发送数据
//	{
//		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//		USART1->DR=dbuf[j];  
//	}	
//}






