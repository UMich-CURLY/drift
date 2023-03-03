/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:21 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/Jvb_Body_to_HindLeftFoot.h"

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
  double t785;
  double t942;
  double t2219;
  double t2997;
  double t6165;
  double t6168;
  double t6169;
  double t6175;
  double t6176;
  double t1288;
  double t6160;
  double t6163;
  double t6177;
  double t6179;
  double t6183;
  double t6188;
  double t6190;
  double t6193;
  double t6194;
  double t6204;
  double t6205;
  double t6206;
  double t6198;
  double t6199;
  double t6200;
  double t6201;
  double t6202;
  double t6203;
  double t6186;
  double t6187;
  double t6195;
  double t6196;
  t785 = Cos(var1[11]);
  t942 = Sin(var1[10]);
  t2219 = Cos(var1[10]);
  t2997 = Sin(var1[11]);
  t6165 = t785 * t942;
  t6168 = t2219 * t2997;
  t6169 = t6165 + t6168;
  t6175 = -1. * t785;
  t6176 = 1. + t6175;
  t1288 = -1. * t785 * t942;
  t6160 = -1. * t2219 * t2997;
  t6163 = t1288 + t6160;
  t6177 = -0.213 * t6176;
  t6179 = -0.4205 * t785;
  t6183 = t6177 + t6179;
  t6188 = -0.1881 * t6176;
  t6190 = -0.1881 * t785;
  t6193 = -0.2075 * t2997;
  t6194 = t6188 + t6190 + t6193;
  t6204 = t2219 * t785;
  t6205 = -1. * t942 * t2997;
  t6206 = t6204 + t6205;
  t6198 = -1. * t2219;
  t6199 = 1. + t6198;
  t6200 = -0.1881 * t6199;
  t6201 = t6183 * t942;
  t6202 = t2219 * t6194;
  t6203 = t6200 + t6201 + t6202;
  t6186 = t2219 * t6183;
  t6187 = -0.1881 * t942;
  t6195 = -1. * t942 * t6194;
  t6196 = t6186 + t6187 + t6195;

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
  p_output1(18) = 0;
  p_output1(19) = 0.;
  p_output1(20) = 0;
  p_output1(21) = 0;
  p_output1(22) = 0.;
  p_output1(23) = 0;
  p_output1(24) = 0;
  p_output1(25) = 0.;
  p_output1(26) = 0;
  p_output1(27) = -0.04675 * t6163 - 0.12675 * t6169;
  p_output1(28) = 0. + t6206 * (-1. * t6169 * t6203 - 1. * t6196 * t6206)
                  + t6169 * (t6163 * t6196 + t6203 * t6206);
  p_output1(29) = 0.08 * t6206;
  p_output1(30) = 0.1881 * t2997 + t2997 * t6194 + t6183 * t785;
  p_output1(31) = 0.;
  p_output1(32) = t2997 * t6183 + t6190 - 1. * t6194 * t785;
  p_output1(33) = -0.2075;
  p_output1(34) = 0.;
  p_output1(35) = 0;
}


Eigen::Matrix<double, 3, 12> Jvb_Body_to_HindLeftFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void Jvb_Body_to_HindLeftFoot(Eigen::Matrix<double,3,12> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 3, 12> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
