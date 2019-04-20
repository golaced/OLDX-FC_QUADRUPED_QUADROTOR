#ifndef __MS5611_H
#define	__MS5611_H

#include "stm32f4xx.h"
#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR				CMD_ADC_4096
#define BARO_SPI 0
typedef struct{
	int32_t baroAlt,baroAlt_flt,baroAlt_off;
	float baroAlt_flt1;
	int32_t pressure;
	int32_t temperature;	
	uint32_t ut;  // static result of temperature measurement
	uint32_t up;  // static result of pressure measurement
	uint16_t prom[PROM_NB];  // on-chip ROM
	uint8_t t_rxbuf[3];
	uint8_t p_rxbuf[3];
	uint8_t update,init_off;
}MS5611_t;
extern MS5611_t ms5611;
u8 MS5611_Detect(void);
void MS5611_Init(void);

void MS5611_Update(float dt);
void MS5611_Read(int32_t* baroAlt);

#endif
