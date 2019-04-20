/*
 * File: OLD_X_AVOID_N.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 23-Mar-2017 20:45:31
 */

#ifndef __OLD_X_AVOID_N_H__
#define __OLD_X_AVOID_N_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "OLD_X_AVOID_N_types.h"

/* Function Declarations */
extern void OLD_X_AVOID_N(const double X[4], const double Y[4], double A, double
  A_DEAD, unsigned int N, unsigned int Max_try, double *x_mid, double *y_mid,
  double *r_mid);
extern void OLD_X_AVOID_N_initialize(void);
extern void OLD_X_AVOID_N_terminate(void);

#endif

/*
 * File trailer for OLD_X_AVOID_N.h
 *
 * [EOF]
 */
