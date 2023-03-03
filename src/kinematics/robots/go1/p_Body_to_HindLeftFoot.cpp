/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:52 GMT-05:00
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "kinematics/robots/go1/p_Body_to_HindLeftFoot.h"

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
  double t5169;
  double t5218;
  double t5219;
  double t5221;
  double t5225;
  double t5214;
  double t5244;
  double t5249;
  double t5224;
  double t5226;
  double t5227;
  double t5229;
  double t5230;
  double t5231;
  t5169 = Cos(var1[10]);
  t5218 = Cos(var1[11]);
  t5219 = -1. * t5218;
  t5221 = 1. + t5219;
  t5225 = Sin(var1[11]);
  t5214 = Sin(var1[10]);
  t5244 = Cos(var1[9]);
  t5249 = Sin(var1[9]);
  t5224 = -0.213 * t5221;
  t5226 = -0.1881 * t5225;
  t5227 = t5224 + t5226;
  t5229 = -0.1881 * t5221;
  t5230 = 0.213 * t5225;
  t5231 = t5229 + t5230;

  p_output1(0) = 0. - 0.1881 * (1. - 1. * t5169)
                 - 0.4205 * (t5214 * t5218 + t5169 * t5225)
                 - 0.1881 * (t5169 * t5218 - 1. * t5214 * t5225) + t5214 * t5227
                 + t5169 * t5231;
  p_output1(1)
      = 0. + 0.04675 * (1. - 1. * t5244) + 0.12675 * t5244
        + 0.1881 * t5214 * t5249 - 1. * t5169 * t5227 * t5249
        + t5214 * t5231 * t5249
        - 0.1881 * (t5214 * t5218 * t5249 + t5169 * t5225 * t5249)
        - 0.4205 * (-1. * t5169 * t5218 * t5249 + t5214 * t5225 * t5249);
  p_output1(2)
      = 0. - 0.1881 * t5214 * t5244 + t5169 * t5227 * t5244
        - 1. * t5214 * t5231 * t5244
        - 0.1881 * (-1. * t5214 * t5218 * t5244 - 1. * t5169 * t5225 * t5244)
        - 0.4205 * (t5169 * t5218 * t5244 - 1. * t5214 * t5225 * t5244)
        + 0.08 * t5249;
}


Eigen::Matrix<double, 3, 1> p_Body_to_HindLeftFoot(
    const Eigen::Matrix<double, 12, 1>& var1)
// void p_Body_to_HindLeftFoot(Eigen::Matrix<double,3,1> &p_output1, const
// Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double, 3, 1> p_output1;

  output1(p_output1, var1);

  return p_output1;
}
