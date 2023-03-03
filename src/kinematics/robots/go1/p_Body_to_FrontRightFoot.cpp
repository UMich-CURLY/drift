/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:50 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/p_Body_to_FrontRightFoot.h"

#ifdef _MSC_VER
#define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
#define INLINE static inline /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0 / sin(x); }
INLINE double Sec(double x) { return 1.0 / cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }


/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y, x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(Eigen::Matrix<double, 3, 1>& p_output1,
                    const Eigen::Matrix<double, 12, 1>& var1) {
  double t1070;
  double t5101;
  double t5111;
  double t5116;
  double t5121;
  double t5069;
  double t5145;
  double t5150;
  double t5117;
  double t5125;
  double t5126;
  double t5129;
  double t5131;
  double t5132;
  t1070 = Cos(var1[1]);
  t5101 = Cos(var1[2]);
  t5111 = -1. * t5101;
  t5116 = 1. + t5111;
  t5121 = Sin(var1[2]);
  t5069 = Sin(var1[1]);
  t5145 = Cos(var1[0]);
  t5150 = Sin(var1[0]);
  t5117 = -0.213 * t5116;
  t5125 = 0.1881 * t5121;
  t5126 = t5117 + t5125;
  t5129 = 0.1881 * t5116;
  t5131 = 0.213 * t5121;
  t5132 = t5129 + t5131;

  p_output1(0) = 0. + 0.1881 * (1. - 1. * t1070)
                 - 0.4205 * (t5069 * t5101 + t1070 * t5121)
                 + 0.1881 * (t1070 * t5101 - 1. * t5069 * t5121) + t5069 * t5126
                 + t1070 * t5132;
  p_output1(1)
      = 0. - 0.04675 * (1. - 1. * t5145) - 0.12675 * t5145
        - 0.1881 * t5069 * t5150 - 1. * t1070 * t5126 * t5150
        + t5069 * t5132 * t5150
        + 0.1881 * (t5069 * t5101 * t5150 + t1070 * t5121 * t5150)
        - 0.4205 * (-1. * t1070 * t5101 * t5150 + t5069 * t5121 * t5150);
  p_output1(2)
      = 0. + 0.1881 * t5069 * t5145 + t1070 * t5126 * t5145
        - 1. * t5069 * t5132 * t5145
        + 0.1881 * (-1. * t5069 * t5101 * t5145 - 1. * t1070 * t5121 * t5145)
        - 0.4205 * (t1070 * t5101 * t5145 - 1. * t5069 * t5121 * t5145)
        - 0.08 * t5150;
}


Eigen::Matrix<double, 3, 1> p_Body_to_FrontRightFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void p_Body_to_FrontRightFoot(Eigen::Matrix<double,3,1> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 3, 1> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
