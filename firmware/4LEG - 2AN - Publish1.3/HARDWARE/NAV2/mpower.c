/*
 * File: mpower.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 01-Apr-2019 21:04:10
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "ekf_pos.h"
#include "mpower.h"
#include "inv.h"

/* Function Definitions */

/*
 * Arguments    : const double a[9]
 *                double c[9]
 * Return Type  : void
 */
void b_mpower(const double a[9], double c[9])
{
  double x[9];
  int p1;
  int p2;
  int p3;
  double absx11;
  double absx21;
  double absx31;
  int itmp;
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
  c[p1] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
  c[p1 + 1] = absx21;
  c[p1 + 2] = absx11;
  absx11 = -x[5] / x[8];
  absx21 = (1.0 - x[7] * absx11) / x[4];
  c[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  c[p2 + 1] = absx21;
  c[p2 + 2] = absx11;
  absx11 = 1.0 / x[8];
  absx21 = -x[7] * absx11 / x[4];
  c[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  c[p3 + 1] = absx21;
  c[p3 + 2] = absx11;
}

/*
 * Arguments    : const double a[16]
 *                double c[16]
 * Return Type  : void
 */
void mpower(const double a[16], double c[16])
{
  invNxN_POSE(a, c);
}

/*
 * File trailer for mpower.c
 *
 * [EOF]
 */
