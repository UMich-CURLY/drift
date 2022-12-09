/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:59 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jvb_Body_to_HindLeftFoot.h"

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
  double t2171;
  double t2173;
  double t2178;
  double t2180;
  double t2176;
  double t2181;
  double t2184;
  double t2192;
  double t2193;
  double t2187;
  double t2188;
  double t2189;
  double t2195;
  double t2198;
  double t2200;
  double t2206;
  double t2207;
  double t2208;
  double t2209;
  double t2219;
  double t2220;
  double t2221;
  double t2213;
  double t2214;
  double t2215;
  double t2216;
  double t2217;
  double t2218;
  double t2202;
  double t2205;
  double t2210;
  double t2211;
  t2171 = Cos(var1[11]);
  t2173 = Sin(var1[10]);
  t2178 = Cos(var1[10]);
  t2180 = Sin(var1[11]);
  t2176 = -1.*t2171*t2173;
  t2181 = -1.*t2178*t2180;
  t2184 = 0. + t2176 + t2181;
  t2192 = -1.*t2171;
  t2193 = 1. + t2192;
  t2187 = t2171*t2173;
  t2188 = t2178*t2180;
  t2189 = 0. + t2187 + t2188;
  t2195 = -0.209*t2193;
  t2198 = -0.4165*t2171;
  t2200 = 0. + t2195 + t2198;
  t2206 = -0.19*t2193;
  t2207 = -0.19*t2171;
  t2208 = 0.2075*t2180;
  t2209 = 0. + t2206 + t2207 + t2208;
  t2219 = t2178*t2171;
  t2220 = -1.*t2173*t2180;
  t2221 = 0. + t2219 + t2220;
  t2213 = -1.*t2178;
  t2214 = 1. + t2213;
  t2215 = -0.19*t2214;
  t2216 = -1.*t2200*t2173;
  t2217 = t2178*t2209;
  t2218 = 0. + t2215 + t2216 + t2217;
  t2202 = t2178*t2200;
  t2205 = 0.19*t2173;
  t2210 = t2173*t2209;
  t2211 = 0. + t2202 + t2205 + t2210;

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
  p_output1(27)=0. - 0.111*t2184 - 0.049*t2189;
  p_output1(28)=0. + t2221*(0. - 1.*t2184*t2218 - 1.*t2211*t2221) + t2184*(0. + t2189*t2211 + t2218*t2221);
  p_output1(29)=0. + 0.062*t2221;
  p_output1(30)=0. + 0.19*t2180 - 1.*t2171*t2200 + t2180*t2209;
  p_output1(31)=0.;
  p_output1(32)=0. + 0.19*t2171 + t2180*t2200 + t2171*t2209;
  p_output1(33)=0.2075;
  p_output1(34)=0.;
  p_output1(35)=0.;
}


       
Eigen::Matrix<double,3,12> Jvb_Body_to_HindLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jvb_Body_to_HindLeftFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



