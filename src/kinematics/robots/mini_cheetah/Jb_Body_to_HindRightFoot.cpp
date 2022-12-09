/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:05:05 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jb_Body_to_HindRightFoot.h"

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
  double t2249;
  double t2252;
  double t2233;
  double t2253;
  double t2261;
  double t2262;
  double t2264;
  double t2274;
  double t2277;
  double t2268;
  double t2269;
  double t2270;
  double t2279;
  double t2281;
  double t2282;
  double t2285;
  double t2286;
  double t2287;
  double t2288;
  double t2251;
  double t2255;
  double t2256;
  double t2292;
  double t2293;
  double t2294;
  double t2295;
  double t2296;
  double t2297;
  double t2283;
  double t2284;
  double t2289;
  double t2290;
  t2249 = Cos(var1[8]);
  t2252 = Sin(var1[7]);
  t2233 = Cos(var1[7]);
  t2253 = Sin(var1[8]);
  t2261 = -1.*t2249*t2252;
  t2262 = -1.*t2233*t2253;
  t2264 = 0. + t2261 + t2262;
  t2274 = -1.*t2249;
  t2277 = 1. + t2274;
  t2268 = t2249*t2252;
  t2269 = t2233*t2253;
  t2270 = 0. + t2268 + t2269;
  t2279 = -0.209*t2277;
  t2281 = -0.4165*t2249;
  t2282 = 0. + t2279 + t2281;
  t2285 = -0.19*t2277;
  t2286 = -0.19*t2249;
  t2287 = 0.2075*t2253;
  t2288 = 0. + t2285 + t2286 + t2287;
  t2251 = t2233*t2249;
  t2255 = -1.*t2252*t2253;
  t2256 = 0. + t2251 + t2255;
  t2292 = -1.*t2233;
  t2293 = 1. + t2292;
  t2294 = -0.19*t2293;
  t2295 = -1.*t2282*t2252;
  t2296 = t2233*t2288;
  t2297 = 0. + t2294 + t2295 + t2296;
  t2283 = t2233*t2282;
  t2284 = 0.19*t2252;
  t2289 = t2252*t2288;
  t2290 = 0. + t2283 + t2284 + t2289;

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
  p_output1(36)=t2256;
  p_output1(37)=0.;
  p_output1(38)=t2264;
  p_output1(39)=0. + 0.111*t2264 + 0.049*t2270;
  p_output1(40)=0. + t2264*(0. + t2270*t2290 + t2256*t2297) + t2256*(0. - 1.*t2256*t2290 - 1.*t2264*t2297);
  p_output1(41)=0. - 0.062*t2256;
  p_output1(42)=0.;
  p_output1(43)=-1.;
  p_output1(44)=0.;
  p_output1(45)=0. + 0.19*t2253 - 1.*t2249*t2282 + t2253*t2288;
  p_output1(46)=0.;
  p_output1(47)=0. + 0.19*t2249 + t2253*t2282 + t2249*t2288;
  p_output1(48)=0.;
  p_output1(49)=-1.;
  p_output1(50)=0.;
  p_output1(51)=0.2075;
  p_output1(52)=0.;
  p_output1(53)=0.;
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


       
Eigen::Matrix<double,6,12> Jb_Body_to_HindRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jb_Body_to_HindRightFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



