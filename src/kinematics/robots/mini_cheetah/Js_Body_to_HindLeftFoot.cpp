/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:05:00 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Js_Body_to_HindLeftFoot.h"

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
  double t2188;
  double t2198;
  double t2191;
  double t2195;
  double t2202;
  double t2206;
  double t2207;
  double t2210;
  double t2212;
  double t2230;
  double t2231;
  double t2232;
  double t2228;
  double t2243;
  double t2244;
  t2188 = Cos(var1[9]);
  t2198 = Sin(var1[9]);
  t2191 = -1.*t2188;
  t2195 = 0. + t2191;
  t2202 = -1.*t2198;
  t2206 = 0. + t2202;
  t2207 = 0.049*t2198;
  t2210 = 1. + t2191;
  t2212 = 0.049*t2210;
  t2230 = Sin(var1[10]);
  t2231 = 0.19*t2230;
  t2232 = 0. + t2231;
  t2228 = Cos(var1[10]);
  t2243 = -1.*t2228;
  t2244 = 1. + t2243;

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
  p_output1(36)=0;
  p_output1(37)=0.;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0.;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0.;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0.;
  p_output1(47)=0;
  p_output1(48)=0;
  p_output1(49)=0.;
  p_output1(50)=0;
  p_output1(51)=0;
  p_output1(52)=0.;
  p_output1(53)=0;
  p_output1(54)=1.;
  p_output1(55)=0.;
  p_output1(56)=0.;
  p_output1(57)=0.;
  p_output1(58)=0.;
  p_output1(59)=-0.049;
  p_output1(60)=0.;
  p_output1(61)=t2195;
  p_output1(62)=t2206;
  p_output1(63)=0. - 1.*t2188*(0. + t2207) - 1.*t2198*(0. + t2212);
  p_output1(64)=0. - 0.19*t2198;
  p_output1(65)=0. + 0.19*t2188;
  p_output1(66)=0.;
  p_output1(67)=t2195;
  p_output1(68)=t2206;
  p_output1(69)=-0.209*t2228 - 0.19*t2230 - 1.*t2188*(0. + t2207 - 1.*t2188*t2232) - 1.*t2198*(0. + t2212 - 1.*t2198*t2232);
  p_output1(70)=0. - 0.19*t2198*t2228 + 0.209*t2198*t2230 - 1.*t2198*(0. + 0.19*t2244);
  p_output1(71)=0.19*t2188*t2228 - 0.209*t2188*t2230 - 1.*t2188*(0. - 0.19*t2244);
}


       
Eigen::Matrix<double,6,12> Js_Body_to_HindLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Js_Body_to_HindLeftFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



