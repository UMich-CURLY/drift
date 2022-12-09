/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:05:11 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jvs_Body_to_HindRightFoot.h"

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
  double t2342;
  double t2344;
  double t2345;
  double t2354;
  double t2355;
  double t2356;
  double t2370;
  double t2371;
  double t2372;
  double t2368;
  double t2383;
  double t2384;
  t2342 = Cos(var1[6]);
  t2344 = Sin(var1[6]);
  t2345 = -0.049*t2344;
  t2354 = -1.*t2342;
  t2355 = 1. + t2354;
  t2356 = -0.049*t2355;
  t2370 = Sin(var1[7]);
  t2371 = 0.19*t2370;
  t2372 = 0. + t2371;
  t2368 = Cos(var1[7]);
  t2383 = -1.*t2368;
  t2384 = 1. + t2383;

  p_output1(0)=0;
  p_output1(1)=0.;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0.;
  p_output1(8)=0;
  p_output1(9)=0;
  p_output1(10)=0.;
  p_output1(11)=0;
  p_output1(12)=0;
  p_output1(13)=0.;
  p_output1(14)=0;
  p_output1(15)=0;
  p_output1(16)=0.;
  p_output1(17)=0;
  p_output1(18)=0.;
  p_output1(19)=0.;
  p_output1(20)=0.049;
  p_output1(21)=0. - 1.*t2342*(0. + t2345) - 1.*t2344*(0. + t2356);
  p_output1(22)=0. - 0.19*t2344;
  p_output1(23)=0. + 0.19*t2342;
  p_output1(24)=-0.209*t2368 - 0.19*t2370 - 1.*t2342*(0. + t2345 - 1.*t2342*t2372) - 1.*t2344*(0. + t2356 - 1.*t2344*t2372);
  p_output1(25)=0. - 0.19*t2344*t2368 + 0.209*t2344*t2370 - 1.*t2344*(0. + 0.19*t2384);
  p_output1(26)=0.19*t2342*t2368 - 0.209*t2342*t2370 - 1.*t2342*(0. - 0.19*t2384);
  p_output1(27)=0;
  p_output1(28)=0.;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0.;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0.;
  p_output1(35)=0;
}


       
Eigen::Matrix<double,3,12> Jvs_Body_to_HindRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jvs_Body_to_HindRightFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



