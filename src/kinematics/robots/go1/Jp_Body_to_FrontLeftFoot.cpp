/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:48 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/Jp_Body_to_FrontLeftFoot.h"

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
  double t510;
  double t3220;
  double t5042;
  double t5045;
  double t5046;
  double t5050;
  double t5040;
  double t189;
  double t5047;
  double t5051;
  double t5053;
  double t5055;
  double t5058;
  double t5059;
  double t5074;
  double t5075;
  double t5076;
  double t5065;
  double t5066;
  double t5067;
  double t5089;
  double t5091;
  double t5093;
  double t5095;
  double t5096;
  double t5097;
  double t5098;
  double t5101;
  double t5125;
  double t5126;
  double t5127;
  double t5129;
  double t5130;
  double t5109;
  double t5111;
  double t5112;
  double t5113;
  double t5114;
  double t5119;
  double t5120;
  double t5121;
  double t5122;
  double t5123;
  t510 = Cos(var1[3]);
  t3220 = Sin(var1[4]);
  t5042 = Cos(var1[5]);
  t5045 = -1. * t5042;
  t5046 = 1. + t5045;
  t5050 = Sin(var1[5]);
  t5040 = Cos(var1[4]);
  t189 = Sin(var1[3]);
  t5047 = -0.213 * t5046;
  t5051 = 0.1881 * t5050;
  t5053 = t5047 + t5051;
  t5055 = 0.1881 * t5046;
  t5058 = 0.213 * t5050;
  t5059 = t5055 + t5058;
  t5074 = t5042 * t189 * t3220;
  t5075 = t5040 * t189 * t5050;
  t5076 = t5074 + t5075;
  t5065 = -1. * t510 * t5040 * t5042;
  t5066 = t510 * t3220 * t5050;
  t5067 = t5065 + t5066;
  t5089 = -1. * t5042 * t3220;
  t5091 = -1. * t5040 * t5050;
  t5093 = t5089 + t5091;
  t5095 = 0.1881 * t5093;
  t5096 = t5040 * t5042;
  t5097 = -1. * t3220 * t5050;
  t5098 = t5096 + t5097;
  t5101 = -0.4205 * t5098;
  t5125 = 0.1881 * t5042;
  t5126 = -0.213 * t5050;
  t5127 = t5125 + t5126;
  t5129 = 0.213 * t5042;
  t5130 = t5129 + t5051;
  t5109 = -0.4205 * t5076;
  t5111 = t5040 * t5042 * t189;
  t5112 = -1. * t189 * t3220 * t5050;
  t5113 = t5111 + t5112;
  t5114 = 0.1881 * t5113;
  t5119 = -1. * t510 * t5042 * t3220;
  t5120 = -1. * t510 * t5040 * t5050;
  t5121 = t5119 + t5120;
  t5122 = -0.4205 * t5121;
  t5123 = 0.1881 * t5067;

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
  p_output1(10) = -0.08 * t189 - 0.4205 * t5067 - 0.1881 * t3220 * t510
                  - 1. * t5040 * t5053 * t510 + t3220 * t5059 * t510
                  + 0.1881 * (t3220 * t5042 * t510 + t5040 * t5050 * t510);
  p_output1(11) = -0.1881 * t189 * t3220
                  - 0.4205 * (-1. * t189 * t5040 * t5042 + t189 * t3220 * t5050)
                  - 1. * t189 * t5040 * t5053 + t189 * t3220 * t5059
                  + 0.1881 * t5076 + 0.08 * t510;
  p_output1(12)
      = 0.1881 * t3220 + t5040 * t5053 - 1. * t3220 * t5059 + t5095 + t5101;
  p_output1(13) = -0.1881 * t189 * t5040 + t189 * t3220 * t5053
                  + t189 * t5040 * t5059 + t5109 + t5114;
  p_output1(14) = 0.1881 * t5040 * t510 - 1. * t3220 * t5053 * t510
                  - 1. * t5040 * t5059 * t510 + t5122 + t5123;
  p_output1(15) = t5095 + t5101 + t3220 * t5127 + t5040 * t5130;
  p_output1(16)
      = t5109 + t5114 - 1. * t189 * t5040 * t5127 + t189 * t3220 * t5130;
  p_output1(17)
      = t5122 + t5123 + t5040 * t510 * t5127 - 1. * t3220 * t510 * t5130;
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
  p_output1(28) = 0;
  p_output1(29) = 0;
  p_output1(30) = 0;
  p_output1(31) = 0;
  p_output1(32) = 0;
  p_output1(33) = 0;
  p_output1(34) = 0;
  p_output1(35) = 0;
}


Eigen::Matrix<double, 3, 12> Jp_Body_to_FrontLeftFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void Jp_Body_to_FrontLeftFoot(Eigen::Matrix<double,3,12> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 3, 12> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
