/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:17 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/Jvb_Body_to_FrontRightFoot.h"

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
static void output1(Eigen::Matrix<double, 3, 12>& p_output1,
                    const Eigen::Matrix<double, 12, 1>& var1) {
  double t145;
  double t2792;
  double t3976;
  double t6067;
  double t6078;
  double t6079;
  double t6083;
  double t6087;
  double t6088;
  double t3479;
  double t6068;
  double t6069;
  double t6090;
  double t6094;
  double t6097;
  double t6101;
  double t6104;
  double t6105;
  double t6106;
  double t6116;
  double t6117;
  double t6118;
  double t6110;
  double t6111;
  double t6112;
  double t6113;
  double t6114;
  double t6115;
  double t6098;
  double t6099;
  double t6107;
  double t6108;
  t145 = Cos(var1[2]);
  t2792 = Sin(var1[1]);
  t3976 = Cos(var1[1]);
  t6067 = Sin(var1[2]);
  t6078 = t145 * t2792;
  t6079 = t3976 * t6067;
  t6083 = t6078 + t6079;
  t6087 = -1. * t145;
  t6088 = 1. + t6087;
  t3479 = -1. * t145 * t2792;
  t6068 = -1. * t3976 * t6067;
  t6069 = t3479 + t6068;
  t6090 = -0.213 * t6088;
  t6094 = -0.4205 * t145;
  t6097 = t6090 + t6094;
  t6101 = 0.1881 * t6088;
  t6104 = 0.1881 * t145;
  t6105 = -0.2075 * t6067;
  t6106 = t6101 + t6104 + t6105;
  t6116 = t3976 * t145;
  t6117 = -1. * t2792 * t6067;
  t6118 = t6116 + t6117;
  t6110 = -1. * t3976;
  t6111 = 1. + t6110;
  t6112 = 0.1881 * t6111;
  t6113 = t6097 * t2792;
  t6114 = t3976 * t6106;
  t6115 = t6112 + t6113 + t6114;
  t6098 = t3976 * t6097;
  t6099 = 0.1881 * t2792;
  t6107 = -1. * t2792 * t6106;
  t6108 = t6098 + t6099 + t6107;

  p_output1(0) = 0.04675 * t6069 + 0.12675 * t6083;
  p_output1(1) = 0. + t6118 * (-1. * t6083 * t6115 - 1. * t6108 * t6118)
                 + t6083 * (t6069 * t6108 + t6115 * t6118);
  p_output1(2) = -0.08 * t6118;
  p_output1(3) = -0.1881 * t6067 + t145 * t6097 + t6067 * t6106;
  p_output1(4) = 0.;
  p_output1(5) = t6067 * t6097 + t6104 - 1. * t145 * t6106;
  p_output1(6) = -0.2075;
  p_output1(7) = 0.;
  p_output1(8) = 0;
  p_output1(9) = 0;
  p_output1(10) = 0.;
  p_output1(11) = 0;
  p_output1(12) = 0;
  p_output1(13) = 0.;
  p_output1(14) = 0;
  p_output1(15) = 0;
  p_output1(16) = 0.;
  p_output1(17) = 0;
  p_output1(18) = 0;
  p_output1(19) = 0.;
  p_output1(20) = 0;
  p_output1(21) = 0;
  p_output1(22) = 0.;
  p_output1(23) = 0;
  p_output1(24) = 0;
  p_output1(25) = 0.;
  p_output1(26) = 0;
  p_output1(27) = 0;
  p_output1(28) = 0.;
  p_output1(29) = 0;
  p_output1(30) = 0;
  p_output1(31) = 0.;
  p_output1(32) = 0;
  p_output1(33) = 0;
  p_output1(34) = 0.;
  p_output1(35) = 0;
}


Eigen::Matrix<double, 3, 12> Jvb_Body_to_FrontRightFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void Jvb_Body_to_FrontRightFoot(Eigen::Matrix<double,3,12> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 3, 12> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
