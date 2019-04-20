/*
 * File: ekf_pos.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 05-Apr-2019 10:17:07
 */

#ifndef EKF_POS_H
#define EKF_POS_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"


/* Function Declarations */
 void ekf_pos(double Xr[6], double Pr[36], const double Zr[9], const
                    double Z_flag[4], const double qr[4], const double rr[5],
                    double T);

#endif

/*
 * File trailer for ekf_pos.h
 *
 * [EOF]
 */
