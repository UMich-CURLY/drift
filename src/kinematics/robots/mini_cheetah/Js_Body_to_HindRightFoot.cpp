/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:05:09 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Js_Body_to_HindRightFoot.h"

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
static void output1(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  double t2306;
  double t2316;
  double t2309;
  double t2313;
  double t2320;
  double t2324;
  double t2325;
  double t2328;
  double t2330;
  double t2348;
  double t2349;
  double t2350;
  double t2346;
  double t2361;
  double t2362;
  t2306 = Cos(var1[6]);
  t2316 = Sin(var1[6]);
  t2309 = -1.*t2306;
  t2313 = 0. + t2309;
  t2320 = -1.*t2316;
  t2324 = 0. + t2320;
  t2325 = -0.049*t2316;
  t2328 = 1. + t2309;
  t2330 = -0.049*t2328;
  t2348 = Sin(var1[7]);
  t2349 = 0.19*t2348;
  t2350 = 0. + t2349;
  t2346 = Cos(var1[7]);
  t2361 = -1.*t2346;
  t2362 = 1. + t2361;

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
  p_output1(18)=0;
  p_output1(19)=0.;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0.;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0.;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0.;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0.;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0.;
  p_output1(35)=0;
  p_output1(36)=1.;
  p_output1(37)=0.;
  p_output1(38)=0.;
  p_output1(39)=0.;
  p_output1(40)=0.;
  p_output1(41)=0.049;
  p_output1(42)=0.;
  p_output1(43)=t2313;
  p_output1(44)=t2324;
  p_output1(45)=0. - 1.*t2306*(0. + t2325) - 1.*t2316*(0. + t2330);
  p_output1(46)=0. - 0.19*t2316;
  p_output1(47)=0. + 0.19*t2306;
  p_output1(48)=0.;
  p_output1(49)=t2313;
  p_output1(50)=t2324;
  p_output1(51)=-0.209*t2346 - 0.19*t2348 - 1.*t2306*(0. + t2325 - 1.*t2306*t2350) - 1.*t2316*(0. + t2330 - 1.*t2316*t2350);
  p_output1(52)=0. - 0.19*t2316*t2346 + 0.209*t2316*t2348 - 1.*t2316*(0. + 0.19*t2362);
  p_output1(53)=0.19*t2306*t2346 - 0.209*t2306*t2348 - 1.*t2306*(0. - 0.19*t2362);
  p_output1(54)=0;
  p_output1(55)=0.;
  p_output1(56)=0;
  p_output1(57)=0;
  p_output1(58)=0.;
  p_output1(59)=0;
  p_output1(60)=0;
  p_output1(61)=0.;
  p_output1(62)=0;
  p_output1(63)=0;
  p_output1(64)=0.;
  p_output1(65)=0;
  p_output1(66)=0;
  p_output1(67)=0.;
  p_output1(68)=0;
  p_output1(69)=0;
  p_output1(70)=0.;
  p_output1(71)=0;
}


       
Eigen::Matrix<double,6,12> Js_Body_to_HindRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Js_Body_to_HindRightFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



