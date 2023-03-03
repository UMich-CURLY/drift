/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:54 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/H_Body_to_HindRightFoot.h"

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
  double t5245;
  double t5251;
  double t2996;
  double t5260;
  double t5253;
  double t5265;
  double t5300;
  double t5301;
  double t5272;
  double t5276;
  double t5277;
  double t5250;
  double t5257;
  double t5259;
  double t5304;
  double t5306;
  double t5308;
  double t5310;
  double t5311;
  double t5314;
  double t5261;
  double t5263;
  double t5264;
  double t5279;
  double t5280;
  double t5281;
  double t5267;
  double t5268;
  double t5269;
  double t5288;
  double t5289;
  double t5293;
  t5245 = Cos(var1[8]);
  t5251 = Sin(var1[7]);
  t2996 = Cos(var1[7]);
  t5260 = Sin(var1[6]);
  t5253 = Sin(var1[8]);
  t5265 = Cos(var1[6]);
  t5300 = -1. * t5245;
  t5301 = 1. + t5300;
  t5272 = t5245 * t5251;
  t5276 = t2996 * t5253;
  t5277 = t5272 + t5276;
  t5250 = t2996 * t5245;
  t5257 = -1. * t5251 * t5253;
  t5259 = t5250 + t5257;
  t5304 = -0.213 * t5301;
  t5306 = -0.1881 * t5253;
  t5308 = t5304 + t5306;
  t5310 = -0.1881 * t5301;
  t5311 = 0.213 * t5253;
  t5314 = t5310 + t5311;
  t5261 = t5245 * t5260 * t5251;
  t5263 = t2996 * t5260 * t5253;
  t5264 = t5261 + t5263;
  t5279 = -1. * t2996 * t5245 * t5260;
  t5280 = t5260 * t5251 * t5253;
  t5281 = t5279 + t5280;
  t5267 = -1. * t5265 * t5245 * t5251;
  t5268 = -1. * t5265 * t2996 * t5253;
  t5269 = t5267 + t5268;
  t5288 = t5265 * t2996 * t5245;
  t5289 = -1. * t5265 * t5251 * t5253;
  t5293 = t5288 + t5289;

  p_output1(0) = t5259;
  p_output1(1) = t5264;
  p_output1(2) = t5269;
  p_output1(3) = 0;
  p_output1(4) = 0;
  p_output1(5) = t5265;
  p_output1(6) = t5260;
  p_output1(7) = 0;
  p_output1(8) = t5277;
  p_output1(9) = t5281;
  p_output1(10) = t5293;
  p_output1(11) = 0;
  p_output1(12) = 0. - 0.1881 * (1. - 1. * t2996) - 0.1881 * t5259
                  - 0.4205 * t5277 + t5251 * t5308 + t2996 * t5314;
  p_output1(13) = 0. + 0.1881 * t5251 * t5260 - 0.1881 * t5264
                  - 0.04675 * (1. - 1. * t5265) - 0.12675 * t5265
                  - 0.4205 * t5281 - 1. * t2996 * t5260 * t5308
                  + t5251 * t5260 * t5314;
  p_output1(14) = 0. - 0.08 * t5260 - 0.1881 * t5251 * t5265 - 0.1881 * t5269
                  - 0.4205 * t5293 + t2996 * t5265 * t5308
                  - 1. * t5251 * t5265 * t5314;
  p_output1(15) = 1.;
}


Eigen::Matrix<double, 4, 4> H_Body_to_HindRightFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void H_Body_to_HindRightFoot(Eigen::Matrix<double,4,4> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 4, 4> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
