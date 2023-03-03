/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:47 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/H_Body_to_FrontLeftFoot.h"

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
static void output1(Eigen::Matrix<double, 4, 4>& p_output1,
                    const Eigen::Matrix<double, 12, 1>& var1) {
  double t4935;
  double t4950;
  double t2051;
  double t4961;
  double t4951;
  double t4967;
  double t5011;
  double t5013;
  double t4981;
  double t4982;
  double t4987;
  double t4947;
  double t4952;
  double t4960;
  double t5015;
  double t5017;
  double t5018;
  double t5020;
  double t5023;
  double t5026;
  double t4962;
  double t4963;
  double t4965;
  double t4988;
  double t4992;
  double t4996;
  double t4969;
  double t4974;
  double t4977;
  double t4997;
  double t5004;
  double t5005;
  t4935 = Cos(var1[5]);
  t4950 = Sin(var1[4]);
  t2051 = Cos(var1[4]);
  t4961 = Sin(var1[3]);
  t4951 = Sin(var1[5]);
  t4967 = Cos(var1[3]);
  t5011 = -1. * t4935;
  t5013 = 1. + t5011;
  t4981 = t4935 * t4950;
  t4982 = t2051 * t4951;
  t4987 = t4981 + t4982;
  t4947 = t2051 * t4935;
  t4952 = -1. * t4950 * t4951;
  t4960 = t4947 + t4952;
  t5015 = -0.213 * t5013;
  t5017 = 0.1881 * t4951;
  t5018 = t5015 + t5017;
  t5020 = 0.1881 * t5013;
  t5023 = 0.213 * t4951;
  t5026 = t5020 + t5023;
  t4962 = t4935 * t4961 * t4950;
  t4963 = t2051 * t4961 * t4951;
  t4965 = t4962 + t4963;
  t4988 = -1. * t2051 * t4935 * t4961;
  t4992 = t4961 * t4950 * t4951;
  t4996 = t4988 + t4992;
  t4969 = -1. * t4967 * t4935 * t4950;
  t4974 = -1. * t4967 * t2051 * t4951;
  t4977 = t4969 + t4974;
  t4997 = t4967 * t2051 * t4935;
  t5004 = -1. * t4967 * t4950 * t4951;
  t5005 = t4997 + t5004;

  p_output1(0) = t4960;
  p_output1(1) = t4965;
  p_output1(2) = t4977;
  p_output1(3) = 0;
  p_output1(4) = 0;
  p_output1(5) = t4967;
  p_output1(6) = t4961;
  p_output1(7) = 0;
  p_output1(8) = t4987;
  p_output1(9) = t4996;
  p_output1(10) = t5005;
  p_output1(11) = 0;
  p_output1(12) = 0. + 0.1881 * (1. - 1. * t2051) + 0.1881 * t4960
                  - 0.4205 * t4987 + t4950 * t5018 + t2051 * t5026;
  p_output1(13) = 0. - 0.1881 * t4950 * t4961 + 0.1881 * t4965
                  + 0.04675 * (1. - 1. * t4967) + 0.12675 * t4967
                  - 0.4205 * t4996 - 1. * t2051 * t4961 * t5018
                  + t4950 * t4961 * t5026;
  p_output1(14) = 0. + 0.08 * t4961 + 0.1881 * t4950 * t4967 + 0.1881 * t4977
                  - 0.4205 * t5005 + t2051 * t4967 * t5018
                  - 1. * t4950 * t4967 * t5026;
  p_output1(15) = 1.;
}


Eigen::Matrix<double, 4, 4> H_Body_to_FrontLeftFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void H_Body_to_FrontLeftFoot(Eigen::Matrix<double,4,4> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 4, 4> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
