/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:54 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/R_Body_to_HindRightFoot.h"

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
static void output1(Eigen::Matrix<double, 3, 3>& p_output1,
                    const Eigen::Matrix<double, 12, 1>& var1) {
  double t2599;
  double t5261;
  double t2224;
  double t5277;
  double t5264;
  double t5293;
  t2599 = Cos(var1[8]);
  t5261 = Sin(var1[7]);
  t2224 = Cos(var1[7]);
  t5277 = Sin(var1[6]);
  t5264 = Sin(var1[8]);
  t5293 = Cos(var1[6]);

  p_output1(0) = t2224 * t2599 - 1. * t5261 * t5264;
  p_output1(1) = t2599 * t5261 * t5277 + t2224 * t5264 * t5277;
  p_output1(2) = -1. * t2599 * t5261 * t5293 - 1. * t2224 * t5264 * t5293;
  p_output1(3) = 0;
  p_output1(4) = t5293;
  p_output1(5) = t5277;
  p_output1(6) = t2599 * t5261 + t2224 * t5264;
  p_output1(7) = -1. * t2224 * t2599 * t5277 + t5261 * t5264 * t5277;
  p_output1(8) = t2224 * t2599 * t5293 - 1. * t5261 * t5264 * t5293;
}


Eigen::Matrix<double, 3, 3> R_Body_to_HindRightFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void R_Body_to_HindRightFoot(Eigen::Matrix<double,3,3> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 3, 3> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
