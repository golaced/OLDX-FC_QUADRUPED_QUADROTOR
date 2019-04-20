#include "imu.h"
#include "filter.h"
#include "arm_math.h"
#include "nav.h"
#include "ms5611.h"
#include "vmc.h"
#include "baro_kf.h"

double X_f32[4];
double P_f32[16] =
{
  100.0,     0,        0,       0,
  0,    	   100,      0,       0,
  0,         0,        100,     0,
  0,         0,        0,       100,
};

double Qs_4_4[4] ={0.002,    1e-8,   0.01,   0.00035};
double Rs_2_2[2] ={15, 0.02};
float FLT_BARO=2;
void baro_fushion(float dt)
{
int i;	
static char init;	
if(!init){
	init=1;
}

	if(ms5611.init_off){

	DigitalLPF((float)ms5611.baroAlt_flt/100., &ms5611.baroAlt_flt1, FLT_BARO, dt);
	double Z_2_1[2] ={(float)ms5611.baroAlt_flt1,vmc_all.acc[Zr] };
	double Z_2_2[2] ={(float)(ms5611.baroAlt-ms5611.baroAlt_off)/100.,vmc_all.acc[Zr] };
	baro_kf(X_f32, P_f32, Z_2_2, ms5611.update, Qs_4_4,Rs_2_2,  dt);
	ms5611.update=0;

	nav.pos_n[Zr]=X_f32[0];
	nav.spd_n[Zr]=X_f32[1];
	nav.acc_n[Zr]=X_f32[2];
	}
}