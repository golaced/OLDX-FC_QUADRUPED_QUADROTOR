#ifndef __NAV_H
#define __NAV_H
#include "include.h"

typedef struct
{
	u8 init[3];
	float att[3];
	float pos_n[3];
	float spd_n[3];
	float spd_b[3];
	float acc_n[3];
	float acc_b[3];
	u16 init_cnt[3];
}_NAV;

extern _NAV nav;

void baro_fushion(float T);
void pose_fushion(float T);
#endif

