/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:05:07 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jvb_Body_to_HindRightFoot.h"

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
  double t2289;
  double t2291;
  double t2296;
  double t2298;
  double t2294;
  double t2299;
  double t2302;
  double t2310;
  double t2311;
  double t2305;
  double t2306;
  double t2307;
  double t2313;
  double t2316;
  double t2318;
  double t2324;
  double t2325;
  double t2326;
  double t2327;
  double t2337;
  double t2338;
  double t2339;
  double t2331;
  double t2332;
  double t2333;
  double t2334;
  double t2335;
  double t2336;
  double t2320;
  double t2323;
  double t2328;
  double t2329;
  t2289 = Cos(var1[8]);
  t2291 = Sin(var1[7]);
  t2296 = Cos(var1[7]);
  t2298 = Sin(var1[8]);
  t2294 = -1.*t2289*t2291;
  t2299 = -1.*t2296*t2298;
  t2302 = 0. + t2294 + t2299;
  t2310 = -1.*t2289;
  t2311 = 1. + t2310;
  t2305 = t2289*t2291;
  t2306 = t2296*t2298;
  t2307 = 0. + t2305 + t2306;
  t2313 = -0.209*t2311;
  t2316 = -0.4165*t2289;
  t2318 = 0. + t2313 + t2316;
  t2324 = -0.19*t2311;
  t2325 = -0.19*t2289;
  t2326 = 0.2075*t2298;
  t2327 = 0. + t2324 + t2325 + t2326;
  t2337 = t2296*t2289;
  t2338 = -1.*t2291*t2298;
  t2339 = 0. + t2337 + t2338;
  t2331 = -1.*t2296;
  t2332 = 1. + t2331;
  t2333 = -0.19*t2332;
  t2334 = -1.*t2318*t2291;
  t2335 = t2296*t2327;
  t2336 = 0. + t2333 + t2334 + t2335;
  t2320 = t2296*t2318;
  t2323 = 0.19*t2291;
  t2328 = t2291*t2327;
  t2329 = 0. + t2320 + t2323 + t2328;

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
  p_output1(18)=0. + 0.111*t2302 + 0.049*t2307;
  p_output1(19)=0. + t2339*(0. - 1.*t2302*t2336 - 1.*t2329*t2339) + t2302*(0. + t2307*t2329 + t2336*t2339);
  p_output1(20)=0. - 0.062*t2339;
  p_output1(21)=0. + 0.19*t2298 - 1.*t2289*t2318 + t2298*t2327;
  p_output1(22)=0.;
  p_output1(23)=0. + 0.19*t2289 + t2298*t2318 + t2289*t2327;
  p_output1(24)=0.2075;
  p_output1(25)=0.;
  p_output1(26)=0.;
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


       
Eigen::Matrix<double,3,12> Jvb_Body_to_HindRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jvb_Body_to_HindRightFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



