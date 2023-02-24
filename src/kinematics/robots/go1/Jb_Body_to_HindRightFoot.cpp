/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:23 GMT-05:00
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
  double t6187;
  double t6208;
  double t2642;
  double t6209;
  double t6216;
  double t6217;
  double t6223;
  double t6238;
  double t6240;
  double t6226;
  double t6227;
  double t6229;
  double t6241;
  double t6242;
  double t6243;
  double t6246;
  double t6247;
  double t6248;
  double t6249;
  double t6188;
  double t6210;
  double t6211;
  double t6253;
  double t6254;
  double t6255;
  double t6256;
  double t6257;
  double t6258;
  double t6244;
  double t6245;
  double t6250;
  double t6251;
  t6187 = Cos(var1[8]);
  t6208 = Sin(var1[7]);
  t2642 = Cos(var1[7]);
  t6209 = Sin(var1[8]);
  t6216 = t6187*t6208;
  t6217 = t2642*t6209;
  t6223 = t6216 + t6217;
  t6238 = -1.*t6187;
  t6240 = 1. + t6238;
  t6226 = -1.*t6187*t6208;
  t6227 = -1.*t2642*t6209;
  t6229 = t6226 + t6227;
  t6241 = -0.213*t6240;
  t6242 = -0.4205*t6187;
  t6243 = t6241 + t6242;
  t6246 = -0.1881*t6240;
  t6247 = -0.1881*t6187;
  t6248 = -0.2075*t6209;
  t6249 = t6246 + t6247 + t6248;
  t6188 = t2642*t6187;
  t6210 = -1.*t6208*t6209;
  t6211 = t6188 + t6210;
  t6253 = -1.*t2642;
  t6254 = 1. + t6253;
  t6255 = -0.1881*t6254;
  t6256 = t6243*t6208;
  t6257 = t2642*t6249;
  t6258 = t6255 + t6256 + t6257;
  t6244 = t2642*t6243;
  t6245 = -0.1881*t6208;
  t6250 = -1.*t6208*t6249;
  t6251 = t6244 + t6245 + t6250;

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
  p_output1(36)=t6211;
  p_output1(37)=0.;
  p_output1(38)=t6223;
  p_output1(39)=0.12675*t6223 + 0.04675*t6229;
  p_output1(40)=0. + t6223*(t6229*t6251 + t6211*t6258) + t6211*(-1.*t6211*t6251 - 1.*t6223*t6258);
  p_output1(41)=-0.08*t6211;
  p_output1(42)=0;
  p_output1(43)=1.;
  p_output1(44)=0;
  p_output1(45)=0.1881*t6209 + t6187*t6243 + t6209*t6249;
  p_output1(46)=0.;
  p_output1(47)=t6209*t6243 + t6247 - 1.*t6187*t6249;
  p_output1(48)=0;
  p_output1(49)=1.;
  p_output1(50)=0;
  p_output1(51)=-0.2075;
  p_output1(52)=0.;
  p_output1(53)=0;
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



