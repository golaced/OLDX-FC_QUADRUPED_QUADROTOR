/*
 * File: ekf_pos.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 01-Apr-2019 20:13:10
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "ekf_pos.h"
#include "mpower.h"

/* Function Definitions */

/*
 * Arguments    : double X[6]
 *                double P[36]
 *                const double Z[9]
 *                const double Z_flag[5]
 *                const double q[4]
 *                const double r[5]
 *                double T
 * Return Type  : void
 */
void ekf_pos(double X[6], double P[36], const double Z[9], const double Z_flag[5],
             const double q[4], const double r[5], double T)
{
  double b_q[6];
  int i0;
  double Q[36];
  double K2[6];
  double Jx[36];
  int j;
  static const signed char iv0[6] = { 0, 0, 0, 1, 0, 0 };

  int i1;
  static const signed char iv1[6] = { 0, 0, 0, 0, 1, 0 };

  static const signed char a[6] = { 0, 0, 0, 0, 0, 1 };

  double b_Jx[36];
  double r1[2];
  double c;
  double R1[4];
  double b;
  double b_a[12];
  static const signed char b_b[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char c_a[6] = { 0, 0, 1, 0, 0, 0 };

  double d_a[4];
  double b_Z[2];
  static const signed char c_b[6] = { 0, 0, 0, 0, 0, 1 };

  double b_K2;
  static const signed char e_a[12] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

  double b_X[2];
  static const signed char d_b[12] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  double K1[12];
  static const signed char f_a[12] = { 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0 };

  static const signed char e_b[12] = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0 };

  double c_Z[2];
  double K5[12];
  double c_X[3];
  b_q[0] = q[0];
  b_q[1] = q[0];
  b_q[2] = q[1];
  b_q[3] = q[2];
  b_q[4] = q[2];
  b_q[5] = q[3];
  for (i0 = 0; i0 < 6; i0++) {
    K2[i0] = b_q[i0];
  }

  memset(&Q[0], 0, 36U * sizeof(double));

  /* Ô¤²â */
  /* X=X Y Z Vxb Vyb Wb */
  X[0] += (X[4] * cos(X[2]) - X[3] * sin(X[2])) * T;

  /* X */
  X[1] += (X[3] * cos(X[2]) + X[4] * sin(X[2])) * T;

  /* Y */
  X[2] += X[5] * T;

  /* angle */
  /*  */
  /*  */
  /*  */
  Jx[0] = 1.0;
  Jx[6] = 0.0;
  Jx[12] = (-X[4] * sin(X[2]) - X[3] * cos(X[2])) * T;
  Jx[18] = -sin(X[2]) * T;
  Jx[24] = cos(X[2]) * T;
  Jx[30] = 0.0;
  Jx[1] = 0.0;
  Jx[7] = 1.0;
  Jx[13] = (-X[3] * sin(X[2]) + X[4] * cos(X[2])) * T;
  Jx[19] = cos(X[2]) * T;
  Jx[25] = sin(X[2]) * T;
  Jx[31] = 0.0;
  Jx[2] = 0.0;
  Jx[8] = 0.0;
  Jx[14] = 1.0;
  Jx[20] = 0.0;
  Jx[26] = 0.0;
  Jx[32] = T;
  for (j = 0; j < 6; j++) {
    Q[j + 6 * j] = K2[j];
    Jx[3 + 6 * j] = iv0[j];
    Jx[4 + 6 * j] = iv1[j];
    Jx[5 + 6 * j] = a[j];
  }

  for (i0 = 0; i0 < 6; i0++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_Jx[i0 + 6 * i1] = 0.0;
      for (j = 0; j < 6; j++) {
        b_Jx[i0 + 6 * i1] += Jx[i0 + 6 * j] * P[j + 6 * i1];
      }
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    for (i1 = 0; i1 < 6; i1++) {
      c = 0.0;
      for (j = 0; j < 6; j++) {
        c += b_Jx[i0 + 6 * j] * Jx[i1 + 6 * j];
      }

      P[i0 + 6 * i1] = c + Q[i0 + 6 * i1];
    }
  }

  if (Z_flag[0] != 0.0) {
    r1[0] = r[0];
    r1[1] = r[0];
    for (i0 = 0; i0 < 4; i0++) {
      R1[i0] = 0.0;
    }

    for (j = 0; j < 2; j++) {
      R1[j + (j << 1)] = r1[j];
      for (i0 = 0; i0 < 6; i0++) {
        b_a[j + (i0 << 1)] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          b_a[j + (i0 << 1)] += (double)e_a[j + (i1 << 1)] * P[i1 + 6 * i0];
        }
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        c = 0.0;
        for (j = 0; j < 6; j++) {
          c += b_a[i0 + (j << 1)] * (double)d_b[j + 6 * i1];
        }

        d_a[i0 + (i1 << 1)] = c + R1[i0 + (i1 << 1)];
      }
    }

    mpower(d_a, R1);
    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        b_a[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 6; j++) {
          b_a[i0 + 6 * i1] += P[i0 + 6 * j] * (double)d_b[j + 6 * i1];
        }
      }

      for (i1 = 0; i1 < 2; i1++) {
        K1[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 2; j++) {
          K1[i0 + 6 * i1] += b_a[i0 + 6 * j] * R1[j + (i1 << 1)];
        }
      }
    }

    b_Z[0] = Z[0];
    b_Z[1] = Z[1];
    b_X[0] = X[0];
    b_X[1] = X[1];
    for (i0 = 0; i0 < 2; i0++) {
      c_Z[i0] = b_Z[i0] - b_X[i0];
    }

    for (i0 = 0; i0 < 6; i0++) {
      b_q[i0] = 0.0;
      for (i1 = 0; i1 < 2; i1++) {
        b_q[i0] += K1[i0 + 6 * i1] * c_Z[i1];
      }

      X[i0] += b_q[i0];
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        c = 0.0;
        for (j = 0; j < 2; j++) {
          c += K1[i0 + 6 * j] * (double)e_a[j + (i1 << 1)];
        }

        Jx[i0 + 6 * i1] = Q[i0 + 6 * i1] - c;
      }

      for (i1 = 0; i1 < 6; i1++) {
        b_Jx[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i0 + 6 * i1] += Jx[i0 + 6 * j] * P[j + 6 * i1];
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        P[i1 + 6 * i0] = b_Jx[i1 + 6 * i0];
      }
    }
  }

  if (Z_flag[1] != 0.0) {
    c = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      b_q[i0] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_q[i0] += (double)c_a[i1] * P[i1 + 6 * i0];
      }

      c += b_q[i0] * (double)b_b[i0];
    }

    c = 1.0 / (c + r[1]);
    b = Z[2] - X[2];
    for (i0 = 0; i0 < 6; i0++) {
      b_q[i0] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_q[i0] += P[i0 + 6 * i1] * (double)b_b[i1];
      }

      b_K2 = b_q[i0] * c;
      X[i0] += b_K2 * b;
      K2[i0] = b_K2;
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        Jx[i0 + 6 * i1] = Q[i0 + 6 * i1] - K2[i0] * (double)c_a[i1];
      }

      for (i1 = 0; i1 < 6; i1++) {
        b_Jx[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i0 + 6 * i1] += Jx[i0 + 6 * j] * P[j + 6 * i1];
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        P[i1 + 6 * i0] = b_Jx[i1 + 6 * i0];
      }
    }
  }

  if (Z_flag[2] != 0.0) {
    r1[0] = r[2];
    r1[1] = r[2];
    for (i0 = 0; i0 < 4; i0++) {
      R1[i0] = 0.0;
    }

    for (j = 0; j < 2; j++) {
      R1[j + (j << 1)] = r1[j];
      for (i0 = 0; i0 < 6; i0++) {
        b_a[j + (i0 << 1)] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          b_a[j + (i0 << 1)] += (double)f_a[j + (i1 << 1)] * P[i1 + 6 * i0];
        }
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        c = 0.0;
        for (j = 0; j < 6; j++) {
          c += b_a[i0 + (j << 1)] * (double)e_b[j + 6 * i1];
        }

        d_a[i0 + (i1 << 1)] = c + R1[i0 + (i1 << 1)];
      }
    }

    mpower(d_a, R1);
    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        b_a[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 6; j++) {
          b_a[i0 + 6 * i1] += P[i0 + 6 * j] * (double)e_b[j + 6 * i1];
        }
      }

      for (i1 = 0; i1 < 2; i1++) {
        K1[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 2; j++) {
          K1[i0 + 6 * i1] += b_a[i0 + 6 * j] * R1[j + (i1 << 1)];
        }
      }
    }

    b_Z[0] = Z[3];
    b_Z[1] = Z[5];
    b_X[0] = X[3];
    b_X[1] = X[4];
    for (i0 = 0; i0 < 2; i0++) {
      c_Z[i0] = b_Z[i0] - b_X[i0];
    }

    for (i0 = 0; i0 < 6; i0++) {
      b_q[i0] = 0.0;
      for (i1 = 0; i1 < 2; i1++) {
        b_q[i0] += K1[i0 + 6 * i1] * c_Z[i1];
      }

      X[i0] += b_q[i0];
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        c = 0.0;
        for (j = 0; j < 2; j++) {
          c += K1[i0 + 6 * j] * (double)f_a[j + (i1 << 1)];
        }

        Jx[i0 + 6 * i1] = Q[i0 + 6 * i1] - c;
      }

      for (i1 = 0; i1 < 6; i1++) {
        b_Jx[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i0 + 6 * i1] += Jx[i0 + 6 * j] * P[j + 6 * i1];
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        P[i1 + 6 * i0] = b_Jx[i1 + 6 * i0];
      }
    }
  }

  if (Z_flag[3] != 0.0) {
    c = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      b_q[i0] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_q[i0] += (double)a[i1] * P[i1 + 6 * i0];
      }

      c += b_q[i0] * (double)c_b[i0];
    }

    c = 1.0 / (c + r[3]);
    b = Z[5] - X[5];
    for (i0 = 0; i0 < 6; i0++) {
      b_q[i0] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_q[i0] += P[i0 + 6 * i1] * (double)c_b[i1];
      }

      b_K2 = b_q[i0] * c;
      X[i0] += b_K2 * b;
      K2[i0] = b_K2;
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        Jx[i0 + 6 * i1] = Q[i0 + 6 * i1] - K2[i0] * (double)a[i1];
      }

      for (i1 = 0; i1 < 6; i1++) {
        b_Jx[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i0 + 6 * i1] += Jx[i0 + 6 * j] * P[j + 6 * i1];
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        P[i1 + 6 * i0] = b_Jx[i1 + 6 * i0];
      }
    }
  }

  if (Z_flag[4] != 0.0) {
    b_a[0] = 0.0;
    b_a[2] = 0.0;
    b_a[4] = 0.0;
    b_a[6] = 0.0;
    b_a[8] = 1.0;
    b_a[10] = 0.5 * Z[8];
    b_a[1] = 0.0;
    b_a[3] = 0.0;
    b_a[5] = 0.0;
    b_a[7] = 0.0;
    b_a[9] = 1.0;
    b_a[11] = -0.5 * Z[8];
    for (i0 = 0; i0 < 2; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        K1[i1 + 6 * i0] = b_a[i1 + 6 * i0];
      }
    }

    r1[0] = r[4];
    r1[1] = r[4];
    for (i0 = 0; i0 < 4; i0++) {
      R1[i0] = 0.0;
    }

    for (j = 0; j < 2; j++) {
      R1[j + (j << 1)] = r1[j];
      for (i0 = 0; i0 < 6; i0++) {
        b_a[j + (i0 << 1)] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          b_a[j + (i0 << 1)] += K1[j + (i1 << 1)] * P[i1 + 6 * i0];
        }
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        c = 0.0;
        for (j = 0; j < 6; j++) {
          c += b_a[i0 + (j << 1)] * K1[i1 + (j << 1)];
        }

        d_a[i0 + (i1 << 1)] = c + R1[i0 + (i1 << 1)];
      }
    }

    mpower(d_a, R1);
    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        b_a[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 6; j++) {
          b_a[i0 + 6 * i1] += P[i0 + 6 * j] * K1[i1 + (j << 1)];
        }
      }

      for (i1 = 0; i1 < 2; i1++) {
        K5[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 2; j++) {
          K5[i0 + 6 * i1] += b_a[i0 + 6 * j] * R1[j + (i1 << 1)];
        }
      }
    }

    b_q[0] = 0.0;
    b_q[2] = 1.0;
    b_q[4] = 0.5 * Z[8];
    b_q[1] = 0.0;
    b_q[3] = 1.0;
    b_q[5] = -0.5 * Z[8];
    c_X[0] = X[3];
    c_X[1] = X[4];
    c_X[2] = X[5];
    b_Z[0] = Z[6];
    b_Z[1] = Z[7];
    for (i0 = 0; i0 < 2; i0++) {
      r1[i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        r1[i0] += b_q[i0 + (i1 << 1)] * c_X[i1];
      }

      c_Z[i0] = b_Z[i0] - r1[i0];
    }

    for (i0 = 0; i0 < 6; i0++) {
      b_q[i0] = 0.0;
      for (i1 = 0; i1 < 2; i1++) {
        b_q[i0] += K5[i0 + 6 * i1] * c_Z[i1];
      }

      X[i0] += b_q[i0];
    }

    memset(&Q[0], 0, 36U * sizeof(double));
    for (j = 0; j < 6; j++) {
      Q[j + 6 * j] = 1.0;
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        c = 0.0;
        for (j = 0; j < 2; j++) {
          c += K5[i0 + 6 * j] * K1[j + (i1 << 1)];
        }

        Jx[i0 + 6 * i1] = Q[i0 + 6 * i1] - c;
      }

      for (i1 = 0; i1 < 6; i1++) {
        b_Jx[i0 + 6 * i1] = 0.0;
        for (j = 0; j < 6; j++) {
          b_Jx[i0 + 6 * i1] += Jx[i0 + 6 * j] * P[j + 6 * i1];
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        P[i1 + 6 * i0] = b_Jx[i1 + 6 * i0];
      }
    }
  }
}

/*
 * File trailer for ekf_pos.c
 *
 * [EOF]
 */
