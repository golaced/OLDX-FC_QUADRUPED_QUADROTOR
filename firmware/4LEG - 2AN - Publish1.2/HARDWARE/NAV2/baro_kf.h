/*
 * File: baro_kf.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 03-Apr-2019 23:33:41
 */

#ifndef BARO_KF_H
#define BARO_KF_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"


/* Function Declarations */
 void baro_kf(double xa_apo[4], double Pa_apo[16], const double z[2],
                    double z_flag,  double q[4],  double r[2], double
                    dt);

#endif

/*
 * File trailer for baro_kf.h
 *
 * [EOF]
 */
