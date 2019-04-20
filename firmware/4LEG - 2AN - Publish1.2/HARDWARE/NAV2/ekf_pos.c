/*
 * File: ekf_pos.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 05-Apr-2019 10:20:17
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "ekf_pos.h"
#include "mpower.h"

/* Function Definitions */

/*
 * Arguments    : double Xr[6]
 *                double Pr[36]
 *                const double Zr[9]
 *                const double Z_flag[4]
 *                const double qr[4]
 *                const double rr[5]
 *                double T
 * Return Type  : void
 */
void ekf_pos(double Xr[6], double Pr[36], const double Zr[9], const double
             Z_flag[4], const double qr[4], const double rr[5], double T)
{
  double v[6];
  double Q[36];
  double Jx[36];
  int j;
  int i1;
  static const signed char iv0[6] = { 0, 0, 0, 1, 0, 0 };

  int i2;
  static const signed char iv1[6] = { 0, 0, 0, 0, 1, 0 };

  static const signed char iv2[6] = { 0, 0, 0, 0, 0, 1 };

  double b_Jx[36];
  double r;
  double Jz3[24];
  static const signed char iv3[6] = { 1, 0, 0, 0, 0, 0 };

  double Jz5[12];
  double Jz4[18];
  static const signed char iv4[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv5[6] = { 0, 1, 0, 0, 0, 0 };

  double a[4];
  double r5[2];
  double r4[3];
  double R4[9];
  double R5[4];
  double R3[16];
  double b_Jz4[9];
  double b_Jz5[12];
  double c_Jz4[18];
  double t;
  double b_Jz3[16];
  double c_Jz3[24];
  double K4[18];
  double K3[24];
  double b_Zr[3];
  double K5[12];
  double b_Xr[3];
  double dv4[2];
  double c_Zr[4];
  double d_Zr[2];
  v[0] = qr[0];
  v[1] = qr[0];
  v[2] = qr[1];
  v[3] = qr[2];
  v[4] = qr[2];
  v[5] = qr[3];
  memset(&Q[0], 0, 36U * sizeof(double));

  /* ?? */
  /* X=X Y Z Vxb Vyb Wb */
  Xr[0] += (Xr[4] * cos(Xr[2]) - Xr[3] * sin(Xr[2])) * T;

  /* X */
  Xr[1] += (Xr[3] * cos(Xr[2]) + Xr[4] * sin(Xr[2])) * T;

  /* Y */
  Xr[2] += Xr[5] * T;

  /* angle */
  /*  */
  /*  */
  /*  */
  Jx[0] = 1.0;
  Jx[6] = 0.0;
  Jx[12] = (-Xr[4] * sin(Xr[2]) - Xr[3] * cos(Xr[2])) * T;
  Jx[18] = -sin(Xr[2]) * T;
  Jx[24] = cos(Xr[2]) * T;
  Jx[30] = 0.0;
  Jx[1] = 0.0;
  Jx[7] = 1.0;
  Jx[13] = (-Xr[3] * sin(Xr[2]) + Xr[4] * cos(Xr[2])) * T;
  Jx[19] = cos(Xr[2]) * T;
  Jx[25] = sin(Xr[2]) * T;
  Jx[31] = 0.0;
  Jx[2] = 0.0;
  Jx[8] = 0.0;
  Jx[14] = 1.0;
  Jx[20] = 0.0;
  Jx[26] = 0.0;
  Jx[32] = T;
  for (j = 0; j < 6; j++) {
    Q[j + 6 * j] = v[j];
    Jx[3 + 6 * j] = iv0[j];
    Jx[4 + 6 * j] = iv1[j];
    Jx[5 + 6 * j] = iv2[j];
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      b_Jx[i1 + 6 * i2] = 0.0;
      for (j = 0; j < 6; j++) {
        b_Jx[i1 + 6 * i2] += Jx[i1 + 6 * j] * Pr[j + 6 * i2];
      }
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      r = 0.0;
      for (j = 0; j < 6; j++) {
        r += b_Jx[i1 + 6 * j] * Jx[i2 + 6 * j];
      }

      Pr[i1 + 6 * i2] = r + Q[i1 + 6 * i2];
    }
  }

  if (Z_flag[0] != 0.0) {
    /* POSE */
    for (i1 = 0; i1 < 6; i1++) {
      Jz3[i1 << 2] = iv3[i1];
      Jz3[1 + (i1 << 2)] = iv5[i1];
    }

    Jz3[2] = 0.0;
    Jz3[6] = 0.0;
    Jz3[10] = 0.0;
    Jz3[14] = 0.0;
    Jz3[18] = 1.0;
    Jz3[22] = 0.5 * Zr[8];
    Jz3[3] = 0.0;
    Jz3[7] = 0.0;
    Jz3[11] = 0.0;
    Jz3[15] = 0.0;
    Jz3[19] = 1.0;
    Jz3[23] = -0.5 * Zr[8];
    a[0] = rr[0];
    a[1] = rr[0];
    a[2] = rr[4];
    a[3] = rr[4];
    for (i1 = 0; i1 < 2; i1++) {
      for (i2 = 0; i2 < 2; i2++) {
        R5[i2 + (i1 << 1)] = a[i2 + (i1 << 1)];
      }
    }

    memset(&R3[0], 0, sizeof(double) << 4);
    for (j = 0; j < 4; j++) {
      R3[j + (j << 2)] = R5[j];
      for (i1 = 0; i1 < 6; i1++) {
        c_Jz3[j + (i1 << 2)] = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          c_Jz3[j + (i1 << 2)] += Jz3[j + (i2 << 2)] * Pr[i2 + 6 * i1];
        }
      }
    }

    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < 4; i2++) {
        r = 0.0;
        for (j = 0; j < 6; j++) {
          r += c_Jz3[i1 + (j << 2)] * Jz3[i2 + (j << 2)];
        }

        b_Jz3[i1 + (i2 << 2)] = r + R3[i1 + (i2 << 2)];
      }
    }

    mpower(b_Jz3, R3);
    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 4; i2++) {
        c_Jz3[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          c_Jz3[i1 + 6 * i2] += Pr[i1 + 6 * j] * Jz3[i2 + (j << 2)];
        }
      }

      for (i2 = 0; i2 < 4; i2++) {
        K3[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 4; j++) {
          K3[i1 + 6 * i2] += c_Jz3[i1 + 6 * j] * R3[j + (i2 << 2)];
        }
      }
    }

    v[0] = 0.0;
    v[2] = 1.0;
    v[4] = 0.5 * Zr[8];
    v[1] = 0.0;
    v[3] = 1.0;
    v[5] = -0.5 * Zr[8];
    r4[0] = Xr[3];
    r4[1] = Xr[4];
    r4[2] = Xr[5];
    a[0] = Zr[0];
    a[1] = Zr[1];
    a[2] = Zr[6];
    a[3] = Zr[7];
    R5[0] = Xr[0];
    R5[1] = Xr[1];
    for (i1 = 0; i1 < 2; i1++) {
      dv4[i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv4[i1] += v[i1 + (i2 << 1)] * r4[i2];
      }

      R5[i1 + 2] = dv4[i1];
    }

    for (i1 = 0; i1 < 4; i1++) {
      c_Zr[i1] = a[i1] - R5[i1];
    }

    for (i1 = 0; i1 < 6; i1++) {
      v[i1] = 0.0;
      for (i2 = 0; i2 < 4; i2++) {
        v[i1] += K3[i1 + 6 * i2] * c_Zr[i2];
      }

      Xr[i1] += v[i1];
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        r = 0.0;
        for (j = 0; j < 4; j++) {
          r += K3[i1 + 6 * j] * Jz3[j + (i2 << 2)];
        }

        Jx[i1 + 6 * i2] = Q[i1 + 6 * i2] - r;
      }

      for (i2 = 0; i2 < 6; i2++) {
        b_Jx[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i1 + 6 * i2] += Jx[i1 + 6 * j] * Pr[j + 6 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        Pr[i2 + 6 * i1] = b_Jx[i2 + 6 * i1];
      }
    }
  } else if (Z_flag[1] != 0.0) {
    /* yaw */
    for (i1 = 0; i1 < 6; i1++) {
      Jz4[3 * i1] = iv4[i1];
    }

    Jz4[1] = 0.0;
    Jz4[4] = 0.0;
    Jz4[7] = 0.0;
    Jz4[10] = 0.0;
    Jz4[13] = 1.0;
    Jz4[16] = 0.5 * Zr[8];
    Jz4[2] = 0.0;
    Jz4[5] = 0.0;
    Jz4[8] = 0.0;
    Jz4[11] = 0.0;
    Jz4[14] = 1.0;
    Jz4[17] = -0.5 * Zr[8];
    r4[0] = rr[1];
    r4[1] = rr[4];
    r4[2] = rr[4];
    memset(&R4[0], 0, 9U * sizeof(double));
    for (j = 0; j < 3; j++) {
      R4[j + 3 * j] = r4[j];
      for (i1 = 0; i1 < 6; i1++) {
        c_Jz4[j + 3 * i1] = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          c_Jz4[j + 3 * i1] += Jz4[j + 3 * i2] * Pr[i2 + 6 * i1];
        }
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        r = 0.0;
        for (j = 0; j < 6; j++) {
          r += c_Jz4[i1 + 3 * j] * Jz4[i2 + 3 * j];
        }

        b_Jz4[i1 + 3 * i2] = r + R4[i1 + 3 * i2];
      }
    }

    b_mpower(b_Jz4, R4);
    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        c_Jz4[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          c_Jz4[i1 + 6 * i2] += Pr[i1 + 6 * j] * Jz4[i2 + 3 * j];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        K4[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 3; j++) {
          K4[i1 + 6 * i2] += c_Jz4[i1 + 6 * j] * R4[j + 3 * i2];
        }
      }
    }

    v[0] = 0.0;
    v[2] = 1.0;
    v[4] = 0.5 * Zr[8];
    v[1] = 0.0;
    v[3] = 1.0;
    v[5] = -0.5 * Zr[8];
    r4[0] = Xr[3];
    r4[1] = Xr[4];
    r4[2] = Xr[5];
    b_Zr[0] = Zr[2];
    b_Zr[1] = Zr[6];
    b_Zr[2] = Zr[7];
    b_Xr[0] = Xr[2];
    for (i1 = 0; i1 < 2; i1++) {
      dv4[i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv4[i1] += v[i1 + (i2 << 1)] * r4[i2];
      }

      b_Xr[i1 + 1] = dv4[i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      r4[i1] = b_Zr[i1] - b_Xr[i1];
    }

    for (i1 = 0; i1 < 6; i1++) {
      v[i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        v[i1] += K4[i1 + 6 * i2] * r4[i2];
      }

      Xr[i1] += v[i1];
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        r = 0.0;
        for (j = 0; j < 3; j++) {
          r += K4[i1 + 6 * j] * Jz4[j + 3 * i2];
        }

        Jx[i1 + 6 * i2] = Q[i1 + 6 * i2] - r;
      }

      for (i2 = 0; i2 < 6; i2++) {
        b_Jx[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i1 + 6 * i2] += Jx[i1 + 6 * j] * Pr[j + 6 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        Pr[i2 + 6 * i1] = b_Jx[i2 + 6 * i1];
      }
    }
  } else if (Z_flag[2] != 0.0) {
    /* spd body */
    for (i1 = 0; i1 < 6; i1++) {
      Jz3[i1 << 2] = iv0[i1];
      Jz3[1 + (i1 << 2)] = iv1[i1];
    }

    Jz3[2] = 0.0;
    Jz3[6] = 0.0;
    Jz3[10] = 0.0;
    Jz3[14] = 0.0;
    Jz3[18] = 1.0;
    Jz3[22] = 0.5 * Zr[8];
    Jz3[3] = 0.0;
    Jz3[7] = 0.0;
    Jz3[11] = 0.0;
    Jz3[15] = 0.0;
    Jz3[19] = 1.0;
    Jz3[23] = -0.5 * Zr[8];
    a[0] = rr[2];
    a[1] = rr[2];
    a[2] = rr[4];
    a[3] = rr[4];
    for (i1 = 0; i1 < 2; i1++) {
      for (i2 = 0; i2 < 2; i2++) {
        R5[i2 + (i1 << 1)] = a[i2 + (i1 << 1)];
      }
    }

    memset(&R3[0], 0, sizeof(double) << 4);
    for (j = 0; j < 4; j++) {
      R3[j + (j << 2)] = R5[j];
      for (i1 = 0; i1 < 6; i1++) {
        c_Jz3[j + (i1 << 2)] = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          c_Jz3[j + (i1 << 2)] += Jz3[j + (i2 << 2)] * Pr[i2 + 6 * i1];
        }
      }
    }

    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < 4; i2++) {
        r = 0.0;
        for (j = 0; j < 6; j++) {
          r += c_Jz3[i1 + (j << 2)] * Jz3[i2 + (j << 2)];
        }

        b_Jz3[i1 + (i2 << 2)] = r + R3[i1 + (i2 << 2)];
      }
    }

    mpower(b_Jz3, R3);
    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 4; i2++) {
        c_Jz3[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          c_Jz3[i1 + 6 * i2] += Pr[i1 + 6 * j] * Jz3[i2 + (j << 2)];
        }
      }

      for (i2 = 0; i2 < 4; i2++) {
        K3[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 4; j++) {
          K3[i1 + 6 * i2] += c_Jz3[i1 + 6 * j] * R3[j + (i2 << 2)];
        }
      }
    }

    v[0] = 0.0;
    v[2] = 1.0;
    v[4] = 0.5 * Zr[8];
    v[1] = 0.0;
    v[3] = 1.0;
    v[5] = -0.5 * Zr[8];
    r4[0] = Xr[3];
    r4[1] = Xr[4];
    r4[2] = Xr[5];
    a[0] = Zr[3];
    a[1] = Zr[5];
    a[2] = Zr[6];
    a[3] = Zr[7];
    R5[0] = Xr[3];
    R5[1] = Xr[4];
    for (i1 = 0; i1 < 2; i1++) {
      dv4[i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv4[i1] += v[i1 + (i2 << 1)] * r4[i2];
      }

      R5[i1 + 2] = dv4[i1];
    }

    for (i1 = 0; i1 < 4; i1++) {
      c_Zr[i1] = a[i1] - R5[i1];
    }

    for (i1 = 0; i1 < 6; i1++) {
      v[i1] = 0.0;
      for (i2 = 0; i2 < 4; i2++) {
        v[i1] += K3[i1 + 6 * i2] * c_Zr[i2];
      }

      Xr[i1] += v[i1];
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        r = 0.0;
        for (j = 0; j < 4; j++) {
          r += K3[i1 + 6 * j] * Jz3[j + (i2 << 2)];
        }

        Jx[i1 + 6 * i2] = Q[i1 + 6 * i2] - r;
      }

      for (i2 = 0; i2 < 6; i2++) {
        b_Jx[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i1 + 6 * i2] += Jx[i1 + 6 * j] * Pr[j + 6 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        Pr[i2 + 6 * i1] = b_Jx[i2 + 6 * i1];
      }
    }
  } else if (Z_flag[3] != 0.0) {
    /* w body */
    for (i1 = 0; i1 < 6; i1++) {
      Jz4[3 * i1] = iv2[i1];
    }

    Jz4[1] = 0.0;
    Jz4[4] = 0.0;
    Jz4[7] = 0.0;
    Jz4[10] = 0.0;
    Jz4[13] = 1.0;
    Jz4[16] = 0.5 * Zr[8];
    Jz4[2] = 0.0;
    Jz4[5] = 0.0;
    Jz4[8] = 0.0;
    Jz4[11] = 0.0;
    Jz4[14] = 1.0;
    Jz4[17] = -0.5 * Zr[8];
    r4[0] = rr[3];
    r4[1] = rr[4];
    r4[2] = rr[4];
    memset(&R4[0], 0, 9U * sizeof(double));
    for (j = 0; j < 3; j++) {
      R4[j + 3 * j] = r4[j];
      for (i1 = 0; i1 < 6; i1++) {
        c_Jz4[j + 3 * i1] = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          c_Jz4[j + 3 * i1] += Jz4[j + 3 * i2] * Pr[i2 + 6 * i1];
        }
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        r = 0.0;
        for (j = 0; j < 6; j++) {
          r += c_Jz4[i1 + 3 * j] * Jz4[i2 + 3 * j];
        }

        b_Jz4[i1 + 3 * i2] = r + R4[i1 + 3 * i2];
      }
    }

    b_mpower(b_Jz4, R4);
    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        c_Jz4[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          c_Jz4[i1 + 6 * i2] += Pr[i1 + 6 * j] * Jz4[i2 + 3 * j];
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        K4[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 3; j++) {
          K4[i1 + 6 * i2] += c_Jz4[i1 + 6 * j] * R4[j + 3 * i2];
        }
      }
    }

    v[0] = 0.0;
    v[2] = 1.0;
    v[4] = 0.5 * Zr[8];
    v[1] = 0.0;
    v[3] = 1.0;
    v[5] = -0.5 * Zr[8];
    r4[0] = Xr[3];
    r4[1] = Xr[4];
    r4[2] = Xr[5];
    b_Zr[0] = Zr[5];
    b_Zr[1] = Zr[6];
    b_Zr[2] = Zr[7];
    b_Xr[0] = Xr[5];
    for (i1 = 0; i1 < 2; i1++) {
      dv4[i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv4[i1] += v[i1 + (i2 << 1)] * r4[i2];
      }

      b_Xr[i1 + 1] = dv4[i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      r4[i1] = b_Zr[i1] - b_Xr[i1];
    }

    for (i1 = 0; i1 < 6; i1++) {
      v[i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        v[i1] += K4[i1 + 6 * i2] * r4[i2];
      }

      Xr[i1] += v[i1];
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        r = 0.0;
        for (j = 0; j < 3; j++) {
          r += K4[i1 + 6 * j] * Jz4[j + 3 * i2];
        }

        Jx[i1 + 6 * i2] = Q[i1 + 6 * i2] - r;
      }

      for (i2 = 0; i2 < 6; i2++) {
        b_Jx[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i1 + 6 * i2] += Jx[i1 + 6 * j] * Pr[j + 6 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        Pr[i2 + 6 * i1] = b_Jx[i2 + 6 * i1];
      }
    }
  } else {
    /* encoder */
    Jz5[0] = 0.0;
    Jz5[2] = 0.0;
    Jz5[4] = 0.0;
    Jz5[6] = 0.0;
    Jz5[8] = 1.0;
    Jz5[10] = 0.5 * Zr[8];
    Jz5[1] = 0.0;
    Jz5[3] = 0.0;
    Jz5[5] = 0.0;
    Jz5[7] = 0.0;
    Jz5[9] = 1.0;
    Jz5[11] = -0.5 * Zr[8];
    r5[0] = rr[4];
    r5[1] = rr[4];
    for (i1 = 0; i1 < 4; i1++) {
      R5[i1] = 0.0;
    }

    for (j = 0; j < 2; j++) {
      R5[j + (j << 1)] = r5[j];
      for (i1 = 0; i1 < 6; i1++) {
        b_Jz5[j + (i1 << 1)] = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          b_Jz5[j + (i1 << 1)] += Jz5[j + (i2 << 1)] * Pr[i2 + 6 * i1];
        }
      }
    }

    for (i1 = 0; i1 < 2; i1++) {
      for (i2 = 0; i2 < 2; i2++) {
        r = 0.0;
        for (j = 0; j < 6; j++) {
          r += b_Jz5[i1 + (j << 1)] * Jz5[i2 + (j << 1)];
        }

        a[i1 + (i2 << 1)] = r + R5[i1 + (i2 << 1)];
      }
    }

    if (fabs(a[1]) > fabs(a[0])) {
      r = a[0] / a[1];
      t = 1.0 / (r * a[3] - a[2]);
      R5[0] = a[3] / a[1] * t;
      R5[1] = -t;
      R5[2] = -a[2] / a[1] * t;
      R5[3] = r * t;
    } else {
      r = a[1] / a[0];
      t = 1.0 / (a[3] - r * a[2]);
      R5[0] = a[3] / a[0] * t;
      R5[1] = -r * t;
      R5[2] = -a[2] / a[0] * t;
      R5[3] = t;
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 2; i2++) {
        b_Jz5[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jz5[i1 + 6 * i2] += Pr[i1 + 6 * j] * Jz5[i2 + (j << 1)];
        }
      }

      for (i2 = 0; i2 < 2; i2++) {
        K5[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 2; j++) {
          K5[i1 + 6 * i2] += b_Jz5[i1 + 6 * j] * R5[j + (i2 << 1)];
        }
      }
    }

    v[0] = 0.0;
    v[2] = 1.0;
    v[4] = 0.5 * Zr[8];
    v[1] = 0.0;
    v[3] = 1.0;
    v[5] = -0.5 * Zr[8];
    r4[0] = Xr[3];
    r4[1] = Xr[4];
    r4[2] = Xr[5];
    r5[0] = Zr[6];
    r5[1] = Zr[7];
    for (i1 = 0; i1 < 2; i1++) {
      dv4[i1] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv4[i1] += v[i1 + (i2 << 1)] * r4[i2];
      }

      d_Zr[i1] = r5[i1] - dv4[i1];
    }

    for (i1 = 0; i1 < 6; i1++) {
      v[i1] = 0.0;
      for (i2 = 0; i2 < 2; i2++) {
        v[i1] += K5[i1 + 6 * i2] * d_Zr[i2];
      }

      Xr[i1] += v[i1];
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        r = 0.0;
        for (j = 0; j < 2; j++) {
          r += K5[i1 + 6 * j] * Jz5[j + (i2 << 1)];
        }

        Jx[i1 + 6 * i2] = Q[i1 + 6 * i2] - r;
      }

      for (i2 = 0; i2 < 6; i2++) {
        b_Jx[i1 + 6 * i2] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i1 + 6 * i2] += Jx[i1 + 6 * j] * Pr[j + 6 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        Pr[i2 + 6 * i1] = b_Jx[i2 + 6 * i1];
      }
    }
  }
}

/*
 * File trailer for ekf_pos.c
 *
 * [EOF]
 */
