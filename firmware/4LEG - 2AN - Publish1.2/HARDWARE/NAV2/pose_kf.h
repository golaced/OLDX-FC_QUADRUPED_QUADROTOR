/*
 * File: pose_kf.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 05-Apr-2019 12:55:43
 */

#ifndef POSE_KF_H
#define POSE_KF_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "pose_kf_types.h"

/* Function Declarations */
extern void pose_kf(double Xr[4], double Pr[16], const double Zr[3], const
                    double Z_flag[3], const double qr[4], const double rr[3],
                    double T);
extern void pose_kf_initialize(void);
extern void pose_kf_terminate(void);

#endif

/*
 * File trailer for pose_kf.h
 *
 * [EOF]
 */
