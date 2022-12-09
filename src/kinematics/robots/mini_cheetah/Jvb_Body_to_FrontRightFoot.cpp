/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:51 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jvb_Body_to_FrontRightFoot.h"

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
  double t2053;
  double t2055;
  double t2060;
  double t2062;
  double t2058;
  double t2063;
  double t2066;
  double t2074;
  double t2075;
  double t2069;
  double t2070;
  double t2071;
  double t2077;
  double t2080;
  double t2082;
  double t2088;
  double t2089;
  double t2090;
  double t2091;
  double t2101;
  double t2102;
  double t2103;
  double t2095;
  double t2096;
  double t2097;
  double t2098;
  double t2099;
  double t2100;
  double t2084;
  double t2087;
  double t2092;
  double t2093;
  t2053 = Cos(var1[2]);
  t2055 = Sin(var1[1]);
  t2060 = Cos(var1[1]);
  t2062 = Sin(var1[2]);
  t2058 = -1.*t2053*t2055;
  t2063 = -1.*t2060*t2062;
  t2066 = 0. + t2058 + t2063;
  t2074 = -1.*t2053;
  t2075 = 1. + t2074;
  t2069 = t2053*t2055;
  t2070 = t2060*t2062;
  t2071 = 0. + t2069 + t2070;
  t2077 = -0.209*t2075;
  t2080 = -0.4165*t2053;
  t2082 = 0. + t2077 + t2080;
  t2088 = 0.19*t2075;
  t2089 = 0.19*t2053;
  t2090 = 0.2075*t2062;
  t2091 = 0. + t2088 + t2089 + t2090;
  t2101 = t2060*t2053;
  t2102 = -1.*t2055*t2062;
  t2103 = 0. + t2101 + t2102;
  t2095 = -1.*t2060;
  t2096 = 1. + t2095;
  t2097 = 0.19*t2096;
  t2098 = -1.*t2082*t2055;
  t2099 = t2060*t2091;
  t2100 = 0. + t2097 + t2098 + t2099;
  t2084 = t2060*t2082;
  t2087 = -0.19*t2055;
  t2092 = t2055*t2091;
  t2093 = 0. + t2084 + t2087 + t2092;

  p_output1(0)=0. + 0.111*t2066 + 0.049*t2071;
  p_output1(1)=0. + t2103*(0. - 1.*t2066*t2100 - 1.*t2093*t2103) + t2066*(0. + t2071*t2093 + t2100*t2103);
  p_output1(2)=0. - 0.062*t2103;
  p_output1(3)=0. - 0.19*t2062 - 1.*t2053*t2082 + t2062*t2091;
  p_output1(4)=0.;
  p_output1(5)=0. - 0.19*t2053 + t2062*t2082 + t2053*t2091;
  p_output1(6)=0.2075;
  p_output1(7)=0.;
  p_output1(8)=0.;
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
}


       
Eigen::Matrix<double,3,12> Jvb_Body_to_FrontRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jvb_Body_to_FrontRightFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



