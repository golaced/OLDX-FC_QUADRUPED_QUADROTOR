#ifndef __PRESSURE_H
#define __PRESSURE_H
#include "stm32f4xx.h"
#include "include.h"
 
#define DOUT1 PAin(1)	// DOUT
#define DOUT2 PAin(4)	// DOUT
#define DOUT3 PAin(5)	// DOUT
#define DOUT4 PAin(6)	// DOUT
#define SCK PDout(2)	// SCK
void pressure_init(void);
void Read_HX711(void);
void Read_HX7112(void);
void Read_HX7113(void);
void Read_HX7114(void);

extern int Pres1,Pres2,Pres3,Pres4;

#endif

