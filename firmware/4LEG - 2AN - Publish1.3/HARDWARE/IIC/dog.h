#ifndef _IWDG_H
#define _IWDG_H
#include "sys.h"

void IWDG_Init(u8 prer,u16 rlr);//IWDG³õÊ¼»¯
void IWDG_Feed(void);  //Î¹¹·º¯Êý
#endif
