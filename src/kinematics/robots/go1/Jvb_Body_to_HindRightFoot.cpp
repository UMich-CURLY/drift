/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:24 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/Jvb_Body_to_HindRightFoot.h"

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
  double t309;
  double t694;
  double t1057;
  double t1170;
  double t6248;
  double t6252;
  double t6255;
  double t6262;
  double t6263;
  double t1012;
  double t1888;
  double t2264;
  double t6265;
  double t6266;
  double t6267;
  double t6276;
  double t6277;
  double t6278;
  double t6280;
  double t6292;
  double t6293;
  double t6294;
  double t6286;
  double t6287;
  double t6288;
  double t6289;
  double t6290;
  double t6291;
  double t6269;
  double t6273;
  double t6283;
  double t6284;
  t309 = Cos(var1[8]);
  t694 = Sin(var1[7]);
  t1057 = Cos(var1[7]);
  t1170 = Sin(var1[8]);
  t6248 = t309 * t694;
  t6252 = t1057 * t1170;
  t6255 = t6248 + t6252;
  t6262 = -1. * t309;
  t6263 = 1. + t6262;
  t1012 = -1. * t309 * t694;
  t1888 = -1. * t1057 * t1170;
  t2264 = t1012 + t1888;
  t6265 = -0.213 * t6263;
  t6266 = -0.4205 * t309;
  t6267 = t6265 + t6266;
  t6276 = -0.1881 * t6263;
  t6277 = -0.1881 * t309;
  t6278 = -0.2075 * t1170;
  t6280 = t6276 + t6277 + t6278;
  t6292 = t1057 * t309;
  t6293 = -1. * t694 * t1170;
  t6294 = t6292 + t6293;
  t6286 = -1. * t1057;
  t6287 = 1. + t6286;
  t6288 = -0.1881 * t6287;
  t6289 = t6267 * t694;
  t6290 = t1057 * t6280;
  t6291 = t6288 + t6289 + t6290;
  t6269 = t1057 * t6267;
  t6273 = -0.1881 * t694;
  t6283 = -1. * t694 * t6280;
  t6284 = t6269 + t6273 + t6283;

  p_output1(0) = 0;
  p_output1(1) = 0.;
  p_output1(2) = 0;
  p_output1(3) = 0;
  p_output1(4) = 0.;
  p_output1(5) = 0;
  p_output1(6) = 0;
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
  p_output1(18) = 0.04675 * t2264 + 0.12675 * t6255;
  p_output1(19) = 0. + t6294 * (-1. * t6255 * t6291 - 1. * t6284 * t6294)
                  + t6255 * (t2264 * t6284 + t6291 * t6294);
  p_output1(20) = -0.08 * t6294;
  p_output1(21) = 0.1881 * t1170 + t309 * t6267 + t1170 * t6280;
  p_output1(22) = 0.;
  p_output1(23) = t1170 * t6267 + t6277 - 1. * t309 * t6280;
  p_output1(24) = -0.2075;
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


Eigen::Matrix<double, 3, 12> Jvb_Body_to_HindRightFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void Jvb_Body_to_HindRightFoot(Eigen::Matrix<double,3,12> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 3, 12> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
