/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:48 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/p_Body_to_FrontLeftFoot.h"

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
  double t1877;
  double t5019;
  double t5023;
  double t5028;
  double t5031;
  double t5005;
  double t5052;
  double t5057;
  double t5030;
  double t5034;
  double t5035;
  double t5037;
  double t5038;
  double t5039;
  t1877 = Cos(var1[4]);
  t5019 = Cos(var1[5]);
  t5023 = -1. * t5019;
  t5028 = 1. + t5023;
  t5031 = Sin(var1[5]);
  t5005 = Sin(var1[4]);
  t5052 = Cos(var1[3]);
  t5057 = Sin(var1[3]);
  t5030 = -0.213 * t5028;
  t5034 = 0.1881 * t5031;
  t5035 = t5030 + t5034;
  t5037 = 0.1881 * t5028;
  t5038 = 0.213 * t5031;
  t5039 = t5037 + t5038;

  p_output1(0) = 0. + 0.1881 * (1. - 1. * t1877)
                 - 0.4205 * (t5005 * t5019 + t1877 * t5031)
                 + 0.1881 * (t1877 * t5019 - 1. * t5005 * t5031) + t5005 * t5035
                 + t1877 * t5039;
  p_output1(1)
      = 0. + 0.04675 * (1. - 1. * t5052) + 0.12675 * t5052
        - 0.1881 * t5005 * t5057 - 1. * t1877 * t5035 * t5057
        + t5005 * t5039 * t5057
        + 0.1881 * (t5005 * t5019 * t5057 + t1877 * t5031 * t5057)
        - 0.4205 * (-1. * t1877 * t5019 * t5057 + t5005 * t5031 * t5057);
  p_output1(2)
      = 0. + 0.1881 * t5005 * t5052 + t1877 * t5035 * t5052
        - 1. * t5005 * t5039 * t5052
        + 0.1881 * (-1. * t5005 * t5019 * t5052 - 1. * t1877 * t5031 * t5052)
        - 0.4205 * (t1877 * t5019 * t5052 - 1. * t5005 * t5031 * t5052)
        + 0.08 * t5057;
}


Eigen::Matrix<double, 3, 1> p_Body_to_FrontLeftFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void p_Body_to_FrontLeftFoot(Eigen::Matrix<double,3,1> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 3, 1> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
