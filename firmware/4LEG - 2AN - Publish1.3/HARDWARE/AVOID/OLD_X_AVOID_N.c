/*
 * File: OLD_X_AVOID_N.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 23-Mar-2017 20:45:31
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "OLD_X_AVOID_N.h"

/* Function Declarations */
static void bubblesort(const double x[16], double *b_min, double *min1);

/* Function Definitions */

/*
 * Arguments    : const double x[16]
 *                double *b_min
 *                double *min1
 * Return Type  : void
 */
static void bubblesort(const double x[16], double *b_min, double *min1)
{
  double x_temp[16];
  int x_num[16];
  int i;
  int j;
  int b_j;
  double temp;

  /*  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  for (i = 0; i < 16; i++) {
    x_temp[i] = x[i];
    x_num[i] = 0;
  }

  for (i = 0; i < 4; i++) {
    x_num[i] = 1 + i;
  }

  for (i = 0; i < 3; i++) {
    for (j = 0; j <= 2 - i; j++) {
      b_j = (i + j) + 1;
      if (x_temp[i] > x_temp[b_j]) {
        temp = x_temp[i];
        x_temp[i] = x_temp[b_j];
        x_temp[b_j] = temp;
        temp = x_num[i];
        x_num[i] = x_num[b_j];
        x_num[b_j] = (int)temp;
      }
    }
  }

  *b_min = x_num[0];
  *min1 = x_num[1];
}

/*
 * Arguments    : const double X[4]
 *                const double Y[4]
 *                double A
 *                double A_DEAD
 *                unsigned int N
 *                unsigned int Max_try
 *                double *x_mid
 *                double *y_mid
 *                double *r_mid
 * Return Type  : void
 */
void OLD_X_AVOID_N(const double X[4], const double Y[4], double A, double A_DEAD,
                   unsigned int N, unsigned int Max_try, double *x_mid, double
                   *y_mid, double *r_mid)
{
  double chui_x[16];
  double chui_y[16];
  double dis[16];
  int i0;
  double a;
  unsigned int n;
  boolean_T exitg1;
  unsigned int i;
  double endx;
  double endy;
  double r;
  double Bx;
  double Ax;
  double Ay;
  double By;
  double min_dis;
  double Cx;
  double Cy;
  int32_T exitg2;

  /*  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  *x_mid = 0.0;
  *y_mid = 0.0;
  for (i0 = 0; i0 < 16; i0++) {
    chui_x[i0] = 0.0;
    chui_y[i0] = 0.0;
    dis[i0] = 0.0;
  }

  a = A;

  /* 步长 */
  *r_mid = 0.0;
  n = 1U;
  exitg1 = false;
  while ((!exitg1) && (n <= Max_try)) {
    i = 1U;
    while ((int)i <= (int)N) {
      if ((int)i == (int)N) {
        endx = X[0];
        endy = Y[0];
      } else {
        endx = X[(int)((i + 1U) & 255U) - 1];
        endy = Y[(int)((i + 1U) & 255U) - 1];
      }

      /* //线段两点距离平方   */
      /* //向量点乘=|a|*|b|*cosA   */
      r = ((*x_mid - X[(int)i - 1]) * (endx - X[(int)i - 1]) + (*y_mid - Y[(int)
            i - 1]) * (endy - Y[(int)i - 1])) / ((X[(int)i - 1] - endx) * (X
        [(int)i - 1] - endx) + (Y[(int)i - 1] - endy) * (Y[(int)i - 1] - endy));

      /* //r即点到线段的投影长度与线段长度比   */
      Bx = X[(int)i - 1] + r * (endx - X[(int)i - 1]);
      endx = Y[(int)i - 1] + r * (endy - Y[(int)i - 1]);
      chui_x[(int)i - 1] = Bx;
      chui_y[(int)i - 1] = endx;
      dis[(int)i - 1] = sqrt((*x_mid - Bx) * (*x_mid - Bx) + (*y_mid - endx) * (*
        y_mid - endx));
      i = (i + 1U) & 255U;
    }

    bubblesort(dis, &endx, &endy);
    Ax = chui_x[(int)endx - 1];
    Ay = chui_y[(int)endx - 1];
    Bx = chui_x[(int)endy - 1];
    By = chui_y[(int)endy - 1];
    min_dis = dis[(int)endx - 1];
    *r_mid = dis[(int)endx - 1];
    Cx = chui_x[(int)endx - 1] + dis[(int)endx - 1] / (dis[(int)endx - 1] + dis
      [(int)endy - 1]) * (chui_x[(int)endy - 1] - chui_x[(int)endx - 1]);
    Cy = chui_y[(int)endx - 1] + dis[(int)endx - 1] / (dis[(int)endx - 1] + dis
      [(int)endy - 1]) * (chui_y[(int)endy - 1] - chui_y[(int)endx - 1]);
    endx = *x_mid - Cx;
    endy = *y_mid - Cy;
    *x_mid += a * (*x_mid - Cx) / sqrt((endx * endx + endy * endy) + A_DEAD);
    endx = *x_mid - Cx;
    endy = *y_mid - Cy;
    *y_mid += a * (*y_mid - Cy) / sqrt((endx * endx + endy * endy) + A_DEAD);

    /* 更新圆心 */
    do {
      exitg2 = 0L;

      /* //线段两点距离平方   */
      /* //向量点乘=|a|*|b|*cosA   */
      r = ((*x_mid - Ax) * (Bx - Ax) + (*y_mid - Ay) * (By - Ay)) / ((Ax - Bx) *
        (Ax - Bx) + (Ay - By) * (Ay - By));

      /* //r即点到线段的投影长度与线段长度比   */
      endy = Ax + r * (Bx - Ax);
      endx = Ay + r * (By - Ay);
      if (sqrt((*x_mid - endy) * (*x_mid - endy) + (*y_mid - endx) * (*y_mid -
            endx)) == 0.0) {
        /* 共线 */
        Cx = Ax;
        Cy = Ay;
      }

      endx = *x_mid - Cx;
      endy = *y_mid - Cy;
      *x_mid += a * (*x_mid - Cx) / sqrt((endx * endx + endy * endy) + A_DEAD);
      endx = *x_mid - Cx;
      endy = *y_mid - Cy;
      *y_mid += a * (*y_mid - Cy) / sqrt((endx * endx + endy * endy) + A_DEAD);

      /* 更新圆心  */
      i = 1U;
      while ((int)i <= (int)N) {
        if ((int)i == (int)N) {
          endx = X[0];
          endy = Y[0];
        } else {
          endx = X[(int)((i + 1U) & 255U) - 1];
          endy = Y[(int)((i + 1U) & 255U) - 1];
        }

        /* //线段两点距离平方   */
        /* //向量点乘=|a|*|b|*cosA   */
        r = ((*x_mid - X[(int)i - 1]) * (endx - X[(int)i - 1]) + (*y_mid - Y
              [(int)i - 1]) * (endy - Y[(int)i - 1])) / ((X[(int)i - 1] - endx) *
          (X[(int)i - 1] - endx) + (Y[(int)i - 1] - endy) * (Y[(int)i - 1] -
          endy));

        /* //r即点到线段的投影长度与线段长度比   */
        Bx = X[(int)i - 1] + r * (endx - X[(int)i - 1]);
        endx = Y[(int)i - 1] + r * (endy - Y[(int)i - 1]);
        chui_x[(int)i - 1] = Bx;
        chui_y[(int)i - 1] = endx;
        dis[(int)i - 1] = sqrt((*x_mid - Bx) * (*x_mid - Bx) + (*y_mid - endx) *
                               (*y_mid - endx));
        i = (i + 1U) & 255U;
      }

      bubblesort(dis, &endx, &endy);
      Ax = chui_x[(int)endx - 1];
      Ay = chui_y[(int)endx - 1];
      Bx = chui_x[(int)endy - 1];
      By = chui_y[(int)endy - 1];
      if (dis[(int)endx - 1] > min_dis) {
        exitg2 = 1L;
      } else {
        a *= 0.6;
        if (a < A_DEAD) {
          exitg2 = 1L;
        }
      }
    } while (exitg2 == 0L);

    if (a < A_DEAD) {
      exitg1 = true;
    } else {
      n++;
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void OLD_X_AVOID_N_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void OLD_X_AVOID_N_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for OLD_X_AVOID_N.c
 *
 * [EOF]
 */
