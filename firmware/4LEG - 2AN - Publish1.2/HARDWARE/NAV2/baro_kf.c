/*
 * File: baro_kf.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 04-Apr-2019 00:14:43
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "baro_kf.h"

/* Function Definitions */

/*
 * Arguments    : double xa_apo[4]
 *                double Pa_apo[16]
 *                const double z[2]
 *                double z_flag
 *                const double q[4]
 *                const double r[2]
 *                double dt
 * Return Type  : void
 */
int delay_sel[4]={1,0,0,0};
void baro_kf(double xa_apo[4], double Pa_apo[16], const double z[2], double
             z_flag,  double q[4],  double r[2], double dt)
{
	int i;
	static double pos_buf[30],vel_buf[30],acc_buf[30],bias_buf[30];
  double A[16];
  int r1;
  double R[4];
  static const signed char iv0[4] = { 0, 0, 1, 0 };

  static const signed char iv1[4] = { 0, 0, 0, 1 };

  double Q[16];
  int r2;
  double v[2];
  double s_k[4];
  double x_apr[4];
  double H[8];
  double b_A[16];
  int k;
  double K_k[8];
  static const signed char iv2[4] = { 0, 0, 1, 1 };

  double a21;
  double y[8];
  double P_apr[16];
  double a22;
  A[0] = 1.0;
  A[4] = dt;
  A[8] = 0.5 * (dt * dt);
  A[12] = 0.0;
  A[1] = 0.0;
  A[5] = 1.0;
  A[9] = dt;
  A[13] = 0.0;
  for (r1 = 0; r1 < 4; r1++) {
    A[2 + (r1 << 2)] = iv0[r1];
    A[3 + (r1 << 2)] = iv1[r1];
  }

  R[0] = q[0];
  R[1] = q[1];
  R[2] = q[2];
  R[3] = q[3];
  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      s_k[r2 + (r1 << 1)] = R[r2 + (r1 << 1)];
    }
  }

  memset(&Q[0], 0, sizeof(double) << 4);
  v[0] = r[0];
  v[1] = r[1];
  for (r1 = 0; r1 < 4; r1++) {
    Q[r1 + (r1 << 2)] = s_k[r1];
    R[r1] = 0.0;
  }

  for (r1 = 0; r1 < 2; r1++) {
    R[r1 + (r1 << 1)] = v[r1];
  }

  /*  copy the states */
  /*  position */
  /*  prediction section */
  x_apr[0] = xa_apo[0] + xa_apo[1] * dt + 0.5* xa_apo[2] * dt *dt;
  x_apr[1] = xa_apo[1] + xa_apo[2] * dt;
  x_apr[2] = xa_apo[2];
  x_apr[3] = xa_apo[3];
  
		for(i=29;i>0;i--)
	{
	pos_buf[i]=pos_buf[i-1];
	vel_buf[i]=vel_buf[i-1];
	acc_buf[i]=acc_buf[i-1];
	bias_buf[i]=bias_buf[i-1];		
	}
	pos_buf[0]=x_apr[0];	
  vel_buf[0]=x_apr[1];
	acc_buf[0]=x_apr[2];
	bias_buf[0]=x_apr[3];
  /* states */
  /*  update */
  H[0] = z_flag;
  H[2] = 0.0;
  H[4] = 0.0;
  H[6] = 0.0;
  for (r1 = 0; r1 < 4; r1++) {
    for (r2 = 0; r2 < 4; r2++) {
      b_A[r1 + (r2 << 2)] = 0.0;
      for (k = 0; k < 4; k++) {
        b_A[r1 + (r2 << 2)] += A[r1 + (k << 2)] * Pa_apo[k + (r2 << 2)];
      }
    }

    for (r2 = 0; r2 < 4; r2++) {
      a21 = 0.0;
      for (k = 0; k < 4; k++) {
        a21 += b_A[r1 + (k << 2)] * A[r2 + (k << 2)];
      }

      P_apr[r1 + (r2 << 2)] = a21 + Q[r1 + (r2 << 2)];
    }

    H[1 + (r1 << 1)] = iv2[r1];
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 4; r2++) {
      K_k[r1 + (r2 << 1)] = 0.0;
      for (k = 0; k < 4; k++) {
        K_k[r1 + (r2 << 1)] += H[r1 + (k << 1)] * P_apr[k + (r2 << 2)];
      }
    }

    for (r2 = 0; r2 < 2; r2++) {
      a21 = 0.0;
      for (k = 0; k < 4; k++) {
        a21 += K_k[r1 + (k << 1)] * H[r2 + (k << 1)];
      }

      s_k[r1 + (r2 << 1)] = a21 + R[r1 + (r2 << 1)];
    }
  }

  for (r1 = 0; r1 < 4; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      y[r1 + (r2 << 2)] = 0.0;
      for (k = 0; k < 4; k++) {
        y[r1 + (r2 << 2)] += P_apr[r1 + (k << 2)] * H[r2 + (k << 1)];
      }
    }
  }

  if (fabs(s_k[1]) > fabs(s_k[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = s_k[r2] / s_k[r1];
  a22 = s_k[2 + r2] - a21 * s_k[2 + r1];
  for (k = 0; k < 4; k++) {
    K_k[k + (r1 << 2)] = y[k] / s_k[r1];
    K_k[k + (r2 << 2)] = (y[4 + k] - K_k[k + (r1 << 2)] * s_k[2 + r1]) / a22;
    K_k[k + (r1 << 2)] -= K_k[k + (r2 << 2)] * a21;
  }

	float x_apr_delay[4];
	x_apr_delay[0]=pos_buf[delay_sel[0]];	
  x_apr_delay[1]=vel_buf[delay_sel[1]];
	x_apr_delay[2]=acc_buf[delay_sel[2]];
	x_apr_delay[3]=bias_buf[delay_sel[3]];
  for (r1 = 0; r1 < 2; r1++) {
    a21 = 0.0;
    for (r2 = 0; r2 < 4; r2++) {
			a21 += H[r1 + (r2 << 1)] * x_apr_delay[r2];
    }

    v[r1] = z[r1] - a21;
  }

  for (r1 = 0; r1 < 4; r1++) {
    a21 = 0.0;
    for (r2 = 0; r2 < 2; r2++) {
      a21 += K_k[r1 + (r2 << 2)] * v[r2];
    }

    xa_apo[r1] = x_apr[r1] + a21;
  }

  memset(&A[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    A[k + (k << 2)] = 1.0;
  }

  for (r1 = 0; r1 < 4; r1++) {
    for (r2 = 0; r2 < 4; r2++) {
      a21 = 0.0;
      for (k = 0; k < 2; k++) {
        a21 += K_k[r1 + (k << 2)] * H[k + (r2 << 1)];
      }

      b_A[r1 + (r2 << 2)] = A[r1 + (r2 << 2)] - a21;
    }

    for (r2 = 0; r2 < 4; r2++) {
      Pa_apo[r1 + (r2 << 2)] = 0.0;
      for (k = 0; k < 4; k++) {
        Pa_apo[r1 + (r2 << 2)] += b_A[r1 + (k << 2)] * P_apr[k + (r2 << 2)];
      }
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void baro_kf_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void baro_kf_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for baro_kf.c
 *
 * [EOF]
 */
