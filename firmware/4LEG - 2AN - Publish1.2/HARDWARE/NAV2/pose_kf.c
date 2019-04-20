/*
 * File: pose_kf.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 05-Apr-2019 13:20:36
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "pose_kf.h"

/* Function Definitions */

/*
 * Arguments    : double Xr[4]
 *                double Pr[16]
 *                const double Zr[3]
 *                const double Z_flag[3]
 *                const double qr[4]
 *                const double rr[3]
 *                double T
 * Return Type  : void
 */
void pose_kf(double Xr[4], double Pr[16], const double Zr[3], const double
             Z_flag[3], const double qr[4], const double rr[3], double T)
{
  double v[4];
  double Q[16];
  double Jx[16];
  int p1;
  int p2;
  static const signed char iv0[4] = { 0, 1, 0, 0 };

  int p3;
  static const signed char iv1[4] = { 0, 0, 1, 0 };

  static const signed char iv2[4] = { 0, 0, 0, 1 };

  double Jz1[12];
  double b_Jx[16];
  double absx11;
  double r1[3];
  double R1[9];
  double x[9];
  double a[9];
  double b_Jz1[12];
  double absx21;
  double absx31;
  int itmp;
  double b_Xr[3];
  double K1[12];
  double b_Zr[3];
  v[0] = qr[0];
  v[1] = qr[0];
  v[2] = qr[1];
  v[3] = qr[2];
  memset(&Q[0], 0, sizeof(double) << 4);

  /* ?? */
  Xr[0] = (Xr[0] + Xr[1] * T) + 0.5 * Xr[2] * T * T;

  /* P */
  Xr[1] += Xr[2] * T;

  /* V */
  /* A */
  /* BV */
  Jx[0] = 1.0;
  Jx[4] = T;
  Jx[8] = 0.5 * T * T;
  Jx[12] = 0.0;
  for (p1 = 0; p1 < 4; p1++) {
    Q[p1 + (p1 << 2)] = v[p1];
    Jx[1 + (p1 << 2)] = iv0[p1];
    Jx[2 + (p1 << 2)] = iv1[p1];
    Jx[3 + (p1 << 2)] = iv2[p1];
  }

  for (p2 = 0; p2 < 4; p2++) {
    for (p3 = 0; p3 < 4; p3++) {
      b_Jx[p2 + (p3 << 2)] = 0.0;
      for (p1 = 0; p1 < 4; p1++) {
        b_Jx[p2 + (p3 << 2)] += Jx[p2 + (p1 << 2)] * Pr[p1 + (p3 << 2)];
      }
    }
  }

  for (p2 = 0; p2 < 4; p2++) {
    for (p3 = 0; p3 < 4; p3++) {
      absx11 = 0.0;
      for (p1 = 0; p1 < 4; p1++) {
        absx11 += b_Jx[p2 + (p1 << 2)] * Jx[p3 + (p1 << 2)];
      }

      Pr[p2 + (p3 << 2)] = absx11 + Q[p2 + (p3 << 2)];
    }
  }

  Jz1[0] = Z_flag[0];
  Jz1[3] = 0.0;
  Jz1[6] = 0.0;
  Jz1[9] = 0.0;
  Jz1[1] = 0.0;
  Jz1[4] = 1.0;
  Jz1[7] = 0.0;
  Jz1[10] = Z_flag[2];
  Jz1[2] = 0.0;
  Jz1[5] = 0.0;
  Jz1[8] = Z_flag[1];
  Jz1[11] = 0.0;
  r1[0] = rr[0];
  r1[1] = rr[1];
  r1[2] = rr[2];
  memset(&R1[0], 0, 9U * sizeof(double));
  for (p1 = 0; p1 < 3; p1++) {
    R1[p1 + 3 * p1] = r1[p1];
    for (p2 = 0; p2 < 4; p2++) {
      b_Jz1[p1 + 3 * p2] = 0.0;
      for (p3 = 0; p3 < 4; p3++) {
        b_Jz1[p1 + 3 * p2] += Jz1[p1 + 3 * p3] * Pr[p3 + (p2 << 2)];
      }
    }
  }

  for (p2 = 0; p2 < 3; p2++) {
    for (p3 = 0; p3 < 3; p3++) {
      absx11 = 0.0;
      for (p1 = 0; p1 < 4; p1++) {
        absx11 += b_Jz1[p2 + 3 * p1] * Jz1[p3 + 3 * p1];
      }

      a[p2 + 3 * p3] = absx11 + R1[p2 + 3 * p3];
    }
  }

  memcpy(&x[0], &a[0], 9U * sizeof(double));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(a[0]);
  absx21 = fabs(a[1]);
  absx31 = fabs(a[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    x[0] = a[1];
    x[1] = a[0];
    x[3] = a[4];
    x[4] = a[3];
    x[6] = a[7];
    x[7] = a[6];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      x[0] = a[2];
      x[2] = a[0];
      x[3] = a[5];
      x[5] = a[3];
      x[6] = a[8];
      x[8] = a[6];
    }
  }

  absx11 = x[1] / x[0];
  x[1] /= x[0];
  absx21 = x[2] / x[0];
  x[2] /= x[0];
  x[4] -= absx11 * x[3];
  x[5] -= absx21 * x[3];
  x[7] -= absx11 * x[6];
  x[8] -= absx21 * x[6];
  if (fabs(x[5]) > fabs(x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    x[1] = absx21;
    x[2] = absx11;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }

  absx11 = x[5] / x[4];
  x[5] /= x[4];
  x[8] -= absx11 * x[7];
  absx11 = (x[5] * x[1] - x[2]) / x[8];
  absx21 = -(x[1] + x[7] * absx11) / x[4];
  R1[p1] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
  R1[p1 + 1] = absx21;
  R1[p1 + 2] = absx11;
  absx11 = -x[5] / x[8];
  absx21 = (1.0 - x[7] * absx11) / x[4];
  R1[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  R1[p2 + 1] = absx21;
  R1[p2 + 2] = absx11;
  absx11 = 1.0 / x[8];
  absx21 = -x[7] * absx11 / x[4];
  R1[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  R1[p3 + 1] = absx21;
  R1[p3 + 2] = absx11;
  for (p2 = 0; p2 < 4; p2++) {
    for (p3 = 0; p3 < 3; p3++) {
      b_Jz1[p2 + (p3 << 2)] = 0.0;
      for (p1 = 0; p1 < 4; p1++) {
        b_Jz1[p2 + (p3 << 2)] += Pr[p2 + (p1 << 2)] * Jz1[p3 + 3 * p1];
      }
    }

    for (p3 = 0; p3 < 3; p3++) {
      K1[p2 + (p3 << 2)] = 0.0;
      for (p1 = 0; p1 < 3; p1++) {
        K1[p2 + (p3 << 2)] += b_Jz1[p2 + (p1 << 2)] * R1[p1 + 3 * p3];
      }
    }
  }

  r1[0] = Zr[0];
  r1[1] = Zr[1];
  r1[2] = Zr[2];
  b_Xr[0] = Xr[0];
  b_Xr[1] = Xr[1];
  b_Xr[2] = Xr[2];
  for (p2 = 0; p2 < 3; p2++) {
    b_Zr[p2] = r1[p2] - b_Xr[p2];
  }

  for (p2 = 0; p2 < 4; p2++) {
    absx11 = 0.0;
    for (p3 = 0; p3 < 3; p3++) {
      absx11 += K1[p2 + (p3 << 2)] * b_Zr[p3];
    }

    Xr[p2] += absx11;
  }

  memset(&Q[0], 0, sizeof(double) << 4);
  for (p1 = 0; p1 < 4; p1++) {
    Q[p1 + (p1 << 2)] = 1.0;
  }

  for (p2 = 0; p2 < 4; p2++) {
    for (p3 = 0; p3 < 4; p3++) {
      absx11 = 0.0;
      for (p1 = 0; p1 < 3; p1++) {
        absx11 += K1[p2 + (p1 << 2)] * Jz1[p1 + 3 * p3];
      }

      Jx[p2 + (p3 << 2)] = Q[p2 + (p3 << 2)] - absx11;
    }

    for (p3 = 0; p3 < 4; p3++) {
      b_Jx[p2 + (p3 << 2)] = 0.0;
      for (p1 = 0; p1 < 4; p1++) {
        b_Jx[p2 + (p3 << 2)] += Jx[p2 + (p1 << 2)] * Pr[p1 + (p3 << 2)];
      }
    }
  }

  for (p2 = 0; p2 < 4; p2++) {
    for (p3 = 0; p3 < 4; p3++) {
      Pr[p3 + (p2 << 2)] = b_Jx[p3 + (p2 << 2)];
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void pose_kf_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void pose_kf_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for pose_kf.c
 *
 * [EOF]
 */
