#ifndef __FILTER_H
#define __FILTER_H

#include "include.h"
void Low_Fass_Filter(float in, float* out, float cutoff_freq, float dt) ;
void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
float Moving_Median(u8 item,u8 width_num,float in);
extern double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
extern fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor);
#define _xyz_f_t xyz_f_t
void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out);
float my_deathzoom1(float x,float zoom);
#define NUMBER_OF_FIRST_ORDER_FILTERS 20
#define ACC_LOWPASS_X 0
#define ACC_LOWPASS_Y 1
#define ACC_LOWPASS_Z 2
#define BARO_LOWPASS   10
#define FLOW_LOWPASS_X   11
#define FLOW_LOWPASS_Y   12
typedef struct firstOrderFilterData {
  float   gx1;
  float   gx2;
  float   gx3;
  float   previousInput;
  float   previousOutput;
} firstOrderFilterData_t;

extern firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(float T);
float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters,float T);


//kf
/* 2 Dimension */
typedef struct {
    float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
    float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    float H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    float q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    float r;        /* measure noise convariance */
    float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    float gain[2];  /* 2x1 */
} kalman2_state;  
extern void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2]);
extern float kalman2_filter(kalman2_state *state, float z_measure);
#endif
