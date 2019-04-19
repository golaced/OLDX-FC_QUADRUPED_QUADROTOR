/*
 * File: ekf_pos.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 01-Apr-2019 20:13:10
 */

#ifndef EKF_POS_H
#define EKF_POS_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>


/* Function Declarations */
extern void ekf_pos(double X[6], double P[36], const double Z[9], const double
                    Z_flag[5], const double q[4], const double r[5], double T);

#endif

/*
 * File trailer for ekf_pos.h
 *
 * [EOF]
 */
