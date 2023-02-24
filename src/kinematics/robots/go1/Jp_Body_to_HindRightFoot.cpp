/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:55 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_Body_to_HindRightFoot.h"

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
static void output1(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  double t1778;
  double t5259;
  double t5330;
  double t5332;
  double t5333;
  double t5337;
  double t5327;
  double t1009;
  double t5335;
  double t5338;
  double t5340;
  double t5342;
  double t5345;
  double t5346;
  double t5361;
  double t5362;
  double t5363;
  double t5352;
  double t5353;
  double t5354;
  double t5376;
  double t5378;
  double t5380;
  double t5382;
  double t5383;
  double t5384;
  double t5385;
  double t5388;
  double t5412;
  double t5413;
  double t5414;
  double t5416;
  double t5417;
  double t5396;
  double t5398;
  double t5399;
  double t5400;
  double t5401;
  double t5406;
  double t5407;
  double t5408;
  double t5409;
  double t5410;
  t1778 = Cos(var1[6]);
  t5259 = Sin(var1[7]);
  t5330 = Cos(var1[8]);
  t5332 = -1.*t5330;
  t5333 = 1. + t5332;
  t5337 = Sin(var1[8]);
  t5327 = Cos(var1[7]);
  t1009 = Sin(var1[6]);
  t5335 = -0.213*t5333;
  t5338 = -0.1881*t5337;
  t5340 = t5335 + t5338;
  t5342 = -0.1881*t5333;
  t5345 = 0.213*t5337;
  t5346 = t5342 + t5345;
  t5361 = t5330*t1009*t5259;
  t5362 = t5327*t1009*t5337;
  t5363 = t5361 + t5362;
  t5352 = -1.*t1778*t5327*t5330;
  t5353 = t1778*t5259*t5337;
  t5354 = t5352 + t5353;
  t5376 = -1.*t5330*t5259;
  t5378 = -1.*t5327*t5337;
  t5380 = t5376 + t5378;
  t5382 = -0.1881*t5380;
  t5383 = t5327*t5330;
  t5384 = -1.*t5259*t5337;
  t5385 = t5383 + t5384;
  t5388 = -0.4205*t5385;
  t5412 = -0.1881*t5330;
  t5413 = -0.213*t5337;
  t5414 = t5412 + t5413;
  t5416 = 0.213*t5330;
  t5417 = t5416 + t5338;
  t5396 = -0.4205*t5363;
  t5398 = t5327*t5330*t1009;
  t5399 = -1.*t1009*t5259*t5337;
  t5400 = t5398 + t5399;
  t5401 = -0.1881*t5400;
  t5406 = -1.*t1778*t5330*t5259;
  t5407 = -1.*t1778*t5327*t5337;
  t5408 = t5406 + t5407;
  t5409 = -0.4205*t5408;
  t5410 = -0.1881*t5354;

  p_output1(0)=0;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=0;
  p_output1(9)=0;
  p_output1(10)=0;
  p_output1(11)=0;
  p_output1(12)=0;
  p_output1(13)=0;
  p_output1(14)=0;
  p_output1(15)=0;
  p_output1(16)=0;
  p_output1(17)=0;
  p_output1(18)=0;
  p_output1(19)=0.08*t1009 + 0.1881*t1778*t5259 - 0.1881*(t1778*t5259*t5330 + t1778*t5327*t5337) - 1.*t1778*t5327*t5340 + t1778*t5259*t5346 - 0.4205*t5354;
  p_output1(20)=-0.08*t1778 + 0.1881*t1009*t5259 - 0.4205*(-1.*t1009*t5327*t5330 + t1009*t5259*t5337) - 1.*t1009*t5327*t5340 + t1009*t5259*t5346 - 0.1881*t5363;
  p_output1(21)=-0.1881*t5259 + t5327*t5340 - 1.*t5259*t5346 + t5382 + t5388;
  p_output1(22)=0.1881*t1009*t5327 + t1009*t5259*t5340 + t1009*t5327*t5346 + t5396 + t5401;
  p_output1(23)=-0.1881*t1778*t5327 - 1.*t1778*t5259*t5340 - 1.*t1778*t5327*t5346 + t5409 + t5410;
  p_output1(24)=t5382 + t5388 + t5259*t5414 + t5327*t5417;
  p_output1(25)=t5396 + t5401 - 1.*t1009*t5327*t5414 + t1009*t5259*t5417;
  p_output1(26)=t5409 + t5410 + t1778*t5327*t5414 - 1.*t1778*t5259*t5417;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
}


       
Eigen::Matrix<double,3,12> Jp_Body_to_HindRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jp_Body_to_HindRightFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



