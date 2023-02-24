/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:55 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_Body_to_HindRightFoot.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static inline        /* use standard inline */
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

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }


/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  double t1306;
  double t5306;
  double t5309;
  double t5310;
  double t5318;
  double t5298;
  double t5339;
  double t5344;
  double t5317;
  double t5319;
  double t5321;
  double t5324;
  double t5325;
  double t5326;
  t1306 = Cos(var1[7]);
  t5306 = Cos(var1[8]);
  t5309 = -1.*t5306;
  t5310 = 1. + t5309;
  t5318 = Sin(var1[8]);
  t5298 = Sin(var1[7]);
  t5339 = Cos(var1[6]);
  t5344 = Sin(var1[6]);
  t5317 = -0.213*t5310;
  t5319 = -0.1881*t5318;
  t5321 = t5317 + t5319;
  t5324 = -0.1881*t5310;
  t5325 = 0.213*t5318;
  t5326 = t5324 + t5325;

  p_output1(0)=0. - 0.1881*(1. - 1.*t1306) - 0.4205*(t5298*t5306 + t1306*t5318) - 0.1881*(t1306*t5306 - 1.*t5298*t5318) + t5298*t5321 + t1306*t5326;
  p_output1(1)=0. - 0.04675*(1. - 1.*t5339) - 0.12675*t5339 + 0.1881*t5298*t5344 - 1.*t1306*t5321*t5344 + t5298*t5326*t5344 - 0.1881*(t5298*t5306*t5344 + t1306*t5318*t5344) - 0.4205*(-1.*t1306*t5306*t5344 + t5298*t5318*t5344);
  p_output1(2)=0. - 0.1881*t5298*t5339 + t1306*t5321*t5339 - 1.*t5298*t5326*t5339 - 0.1881*(-1.*t5298*t5306*t5339 - 1.*t1306*t5318*t5339) - 0.4205*(t1306*t5306*t5339 - 1.*t5298*t5318*t5339) - 0.08*t5344;
}


       
Eigen::Matrix<double,3,1> p_Body_to_HindRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void p_Body_to_HindRightFoot(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



