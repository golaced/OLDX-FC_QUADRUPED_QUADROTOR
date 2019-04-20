/**********************************************************************************************************
                                本程序移植于：			天穹飞控
**********************************************************************************************************/
#include "ms5611.h"
#include "spi.h"
#include "math.h"

MS5611_t ms5611;
static void MS5611_Reset(void);
static void MS5611_Read_Prom(void);
static void MS5611_Start_T(void);
static void MS5611_Start_P(void);
static void MS5611_Read_Adc_T(void);
static void MS5611_Read_Adc_P(void);
static void MS5611_BaroAltCalculate(void);

/**********************************************************************************************************
*函 数 名: MS5611_Detect
*功能说明: 检测MS5611是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
u8 MS5611_Detect(void)
{
	return 1;
}

uint8_t Spi_SingleWirteAndRead(uint8_t deviceNum, uint8_t dat) 
{ 
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
        SPI_I2S_SendData(SPI2, dat); 
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
        return SPI_I2S_ReceiveData(SPI2);             
}

//50hz  2hz 
const static float b_baro_pressure[3]={0.01335920002786,  0.02671840005571,  0.01335920002786};
const static float a_baro_pressure[3]={1,   -1.647459981077,   0.7008967811884};
float LPButter_BaroAlt(float curr_input)
{
	static float input[3];
	static float output[3];

	/* 气压计高度Butterworth滤波 */
	/* 获取最新x(n) */
	input[2] = curr_input;

	/* Butterworth滤波 */
	output[2] = b_baro_pressure[0] * input[2] + b_baro_pressure[1] * input[1]
	+ b_baro_pressure[2] * input[0] - a_baro_pressure[1] * output[1] - a_baro_pressure[2] * output[0];

	/* x(n) 序列保存 */
	input[0] = input[1];
	input[1] = input[2];
	/* y(n) 序列保存 */
	output[0] = output[1];
	output[1] = output[2];

	return output[2];
}

/**********************************************************************************************************
*函 数 名: SPI_MultiWriteAndRead
*功能说明: SPI多字节读取
*形    参: 设备号 写入数据缓冲区指针 读出数据缓冲区指针 数据长度
            同时只能写入或者读出，写入时读取缓冲区设置为NULL，读出时反之
*返 回 值: 无
**********************************************************************************************************/
void SPI_MultiWriteAndRead(uint8_t deviceNum, uint8_t *out, uint8_t *in, int len)
{
    uint8_t b;
        while (len--) 
        {
            b = in ? *(in++) : 0xFF;		
            while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(SPI2, b); 
            while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
            b = SPI_I2S_ReceiveData(SPI2); 
            if (out)
                *(out++) = b;
        }
}

void Spi_BaroSingleWrite(uint8_t reg, uint8_t value)
{
	SPI_CS(MS5611,0);
	Spi_SingleWirteAndRead(BARO_SPI, reg);
	Spi_SingleWirteAndRead(BARO_SPI, value); 
	SPI_CS(MS5611,1);
}

/**********************************************************************************************************
*函 数 名: Spi_BaroMultiRead
*功能说明: 气压计多个寄存器读出
*形    参: 寄存器地址 读出缓冲区 读出长度
*返 回 值: 无
**********************************************************************************************************/
void Spi_BaroMultiRead(uint8_t reg,uint8_t *data, uint8_t length)
{
	SPI_CS(MS5611,0);
	Spi_SingleWirteAndRead(BARO_SPI, reg);
	SPI_MultiWriteAndRead(BARO_SPI, data, NULL, length);	
	SPI_CS(MS5611,1);	
}

void SoftDelayMs(uint32_t ms)
{
	uint32_t us_cnt; 
	for(; ms!= 0;ms--)
    {
        us_cnt = 42000;
		while(us_cnt)
        {
			us_cnt--;
		}	
    }		
}
/**********************************************************************************************************
*函 数 名: MS5611_Init
*功能说明: MS5611寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MS5611_Init(void)
{
	MS5611_Reset();
	SoftDelayMs(3);
	MS5611_Read_Prom();

	MS5611_Start_T();
}

/**********************************************************************************************************
*函 数 名: MS5611_Reset
*功能说明: MS5611复位
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Reset(void)
{
	Spi_BaroSingleWrite(CMD_RESET, 0x01);
}

/**********************************************************************************************************
*函 数 名: MS5611_Read_Prom
*功能说明: MS5611读取出厂校准参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Read_Prom(void)
{
	uint8_t rxbuf[2] = { 0, 0 };

	for (u8 i = 0; i < PROM_NB; i++)
	{
		Spi_BaroMultiRead((CMD_PROM_RD + i * 2), rxbuf, 2);
		ms5611.prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}
	if(ms5611.prom[0]!=0xFF&&ms5611.prom[0]!=0x00)
	   module.bmp=1;
}

/**********************************************************************************************************
*函 数 名: MS5611_Read_Adc_T
*功能说明: MS5611读取温度测量值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Read_Adc_T(void)
{
	Spi_BaroMultiRead(CMD_ADC_READ, ms5611.t_rxbuf, 3);
}

/**********************************************************************************************************
*函 数 名: MS5611_Read_Adc_P
*功能说明: MS5611读取气压测量值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Read_Adc_P(void)
{
	Spi_BaroMultiRead(CMD_ADC_READ, ms5611.p_rxbuf, 3);
}

/**********************************************************************************************************
*函 数 名: MS5611_Start_T
*功能说明: MS5611发送温度测量命令
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Start_T(void)
{
	Spi_BaroSingleWrite(CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 0x01);
}

/**********************************************************************************************************
*函 数 名: MS5611_Start_P
*功能说明: MS5611发送气压测量命令
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_Start_P(void)
{
    Spi_BaroSingleWrite(CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 0x01);
}

/**********************************************************************************************************
*函 数 名: MS5611_Update
*功能说明: MS5611数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MS5611_Update(float dt)
{
	static int state = 0;
	static int cnt_init;
	static float timer[2];
	
	switch(state){
		case  0:
			MS5611_Start_T();
			state = 1;
		  timer[0]=0;
		break;
		case 1:
			timer[0]+=dt;
		  if(timer[0]>0.010)
			{timer[0]=0;state=2;
			 MS5611_Read_Adc_T();
			 MS5611_Start_P();
			}
		break;
		case 2:
			timer[0]+=dt;
		  if(timer[0]>0.010)
			{timer[0]=0;
			MS5611_Read_Adc_P();
			MS5611_BaroAltCalculate();
		  ms5611.update=1;
			ms5611.baroAlt_flt=LPButter_BaroAlt(ms5611.baroAlt-ms5611.baroAlt_off);
			MS5611_Start_T();
			state=1;
			}
    break;
		}
	if(cnt_init++>2.5/0.02&&ms5611.init_off==0){
		ms5611.init_off=1;
		ms5611.baroAlt_off=ms5611.baroAlt;
	}
	timer[1]+=dt;
}

/**********************************************************************************************************
*函 数 名: MS5611_BaroAltCalculate
*功能说明: MS5611数据校准及温度补偿，并将气压值转换为高度值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void MS5611_BaroAltCalculate(void)
{
	int32_t  off2 = 0, sens2 = 0, delt;

	ms5611.ut = (ms5611.t_rxbuf[0] << 16) | (ms5611.t_rxbuf[1] << 8) | ms5611.t_rxbuf[2];
	ms5611.up = (ms5611.p_rxbuf[0] << 16) | (ms5611.p_rxbuf[1] << 8) | ms5611.p_rxbuf[2];

	int32_t dT = ms5611.ut - ((uint32_t)ms5611.prom[5] << 8);
	int64_t off = ((uint32_t)ms5611.prom[2] << 16) + (((int64_t)dT * ms5611.prom[4]) >> 7);
	int64_t sens = ((uint32_t)ms5611.prom[1] << 15) + (((int64_t)dT * ms5611.prom[3]) >> 8);
	ms5611.temperature = 2000 + (((int64_t)dT * ms5611.prom[6]) >> 23);

	if (ms5611.temperature < 2000) { // temperature lower than 20degC 
			delt = ms5611.temperature - 2000;
			delt = delt * delt;
			off2 = (5 * delt) >> 1;
			sens2 = (5 * delt) >> 2;
			if (ms5611.temperature < -1500) { // temperature lower than -15degC
					delt = ms5611.temperature + 1500;
					delt = delt * delt;
					off2  += 7 * delt;
					sens2 += (11 * delt) >> 1;
			}
	}
	off  -= off2; 
	sens -= sens2;
	ms5611.pressure = (((ms5611.up * sens ) >> 21) - off) >> 15;

	ms5611.baroAlt = (int32_t)((1.0f - pow(ms5611.pressure / 101325.0f, 0.190295f)) * 4433000.0f); // centimeter
}

/**********************************************************************************************************
*函 数 名: MS5611_Read
*功能说明: 读取MS5611的气压高度值
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MS5611_Read(int32_t* baroAlt)
{
    *baroAlt = ms5611.baroAlt;
}


