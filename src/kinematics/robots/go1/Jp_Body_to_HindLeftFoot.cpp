/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:53 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/Jp_Body_to_HindLeftFoot.h"

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
  double t5180;
  double t5224;
  double t5237;
  double t5238;
  double t5240;
  double t5243;
  double t5235;
  double t2133;
  double t5242;
  double t5245;
  double t5246;
  double t5250;
  double t5251;
  double t5252;
  double t5267;
  double t5268;
  double t5269;
  double t5258;
  double t5259;
  double t5260;
  double t5283;
  double t5285;
  double t5287;
  double t5288;
  double t5289;
  double t5290;
  double t5293;
  double t5296;
  double t5318;
  double t5319;
  double t5320;
  double t5322;
  double t5323;
  double t5303;
  double t5304;
  double t5305;
  double t5306;
  double t5307;
  double t5312;
  double t5313;
  double t5314;
  double t5315;
  double t5316;
  t5180 = Cos(var1[9]);
  t5224 = Sin(var1[10]);
  t5237 = Cos(var1[11]);
  t5238 = -1. * t5237;
  t5240 = 1. + t5238;
  t5243 = Sin(var1[11]);
  t5235 = Cos(var1[10]);
  t2133 = Sin(var1[9]);
  t5242 = -0.213 * t5240;
  t5245 = -0.1881 * t5243;
  t5246 = t5242 + t5245;
  t5250 = -0.1881 * t5240;
  t5251 = 0.213 * t5243;
  t5252 = t5250 + t5251;
  t5267 = t5237 * t2133 * t5224;
  t5268 = t5235 * t2133 * t5243;
  t5269 = t5267 + t5268;
  t5258 = -1. * t5180 * t5235 * t5237;
  t5259 = t5180 * t5224 * t5243;
  t5260 = t5258 + t5259;
  t5283 = -1. * t5237 * t5224;
  t5285 = -1. * t5235 * t5243;
  t5287 = t5283 + t5285;
  t5288 = -0.1881 * t5287;
  t5289 = t5235 * t5237;
  t5290 = -1. * t5224 * t5243;
  t5293 = t5289 + t5290;
  t5296 = -0.4205 * t5293;
  t5318 = -0.1881 * t5237;
  t5319 = -0.213 * t5243;
  t5320 = t5318 + t5319;
  t5322 = 0.213 * t5237;
  t5323 = t5322 + t5245;
  t5303 = -0.4205 * t5269;
  t5304 = t5235 * t5237 * t2133;
  t5305 = -1. * t2133 * t5224 * t5243;
  t5306 = t5304 + t5305;
  t5307 = -0.1881 * t5306;
  t5312 = -1. * t5180 * t5237 * t5224;
  t5313 = -1. * t5180 * t5235 * t5243;
  t5314 = t5312 + t5313;
  t5315 = -0.4205 * t5314;
  t5316 = -0.1881 * t5260;

  p_output1(0) = 0;
  p_output1(1) = 0;
  p_output1(2) = 0;
  p_output1(3) = 0;
  p_output1(4) = 0;
  p_output1(5) = 0;
  p_output1(6) = 0;
  p_output1(7) = 0;
  p_output1(8) = 0;
  p_output1(9) = 0;
  p_output1(10) = 0;
  p_output1(11) = 0;
  p_output1(12) = 0;
  p_output1(13) = 0;
  p_output1(14) = 0;
  p_output1(15) = 0;
  p_output1(16) = 0;
  p_output1(17) = 0;
  p_output1(18) = 0;
  p_output1(19) = 0;
  p_output1(20) = 0;
  p_output1(21) = 0;
  p_output1(22) = 0;
  p_output1(23) = 0;
  p_output1(24) = 0;
  p_output1(25) = 0;
  p_output1(26) = 0;
  p_output1(27) = 0;
  p_output1(28) = -0.08 * t2133 + 0.1881 * t5180 * t5224
                  - 0.1881 * (t5180 * t5224 * t5237 + t5180 * t5235 * t5243)
                  - 1. * t5180 * t5235 * t5246 + t5180 * t5224 * t5252
                  - 0.4205 * t5260;
  p_output1(29)
      = 0.08 * t5180 + 0.1881 * t2133 * t5224
        - 0.4205 * (-1. * t2133 * t5235 * t5237 + t2133 * t5224 * t5243)
        - 1. * t2133 * t5235 * t5246 + t2133 * t5224 * t5252 - 0.1881 * t5269;
  p_output1(30)
      = -0.1881 * t5224 + t5235 * t5246 - 1. * t5224 * t5252 + t5288 + t5296;
  p_output1(31) = 0.1881 * t2133 * t5235 + t2133 * t5224 * t5246
                  + t2133 * t5235 * t5252 + t5303 + t5307;
  p_output1(32) = -0.1881 * t5180 * t5235 - 1. * t5180 * t5224 * t5246
                  - 1. * t5180 * t5235 * t5252 + t5315 + t5316;
  p_output1(33) = t5288 + t5296 + t5224 * t5320 + t5235 * t5323;
  p_output1(34)
      = t5303 + t5307 - 1. * t2133 * t5235 * t5320 + t2133 * t5224 * t5323;
  p_output1(35)
      = t5315 + t5316 + t5180 * t5235 * t5320 - 1. * t5180 * t5224 * t5323;
}


Eigen::Matrix<double, 3, 12> Jp_Body_to_HindLeftFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void Jp_Body_to_HindLeftFoot(Eigen::Matrix<double,3,12> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 3, 12> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
