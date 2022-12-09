/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:48 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jb_Body_to_FrontRightFoot.h"

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
  double t2013;
  double t2016;
  double t1997;
  double t2017;
  double t2025;
  double t2026;
  double t2028;
  double t2038;
  double t2041;
  double t2032;
  double t2033;
  double t2034;
  double t2043;
  double t2045;
  double t2046;
  double t2049;
  double t2050;
  double t2051;
  double t2052;
  double t2015;
  double t2019;
  double t2020;
  double t2056;
  double t2057;
  double t2058;
  double t2059;
  double t2060;
  double t2061;
  double t2047;
  double t2048;
  double t2053;
  double t2054;
  t2013 = Cos(var1[2]);
  t2016 = Sin(var1[1]);
  t1997 = Cos(var1[1]);
  t2017 = Sin(var1[2]);
  t2025 = -1.*t2013*t2016;
  t2026 = -1.*t1997*t2017;
  t2028 = 0. + t2025 + t2026;
  t2038 = -1.*t2013;
  t2041 = 1. + t2038;
  t2032 = t2013*t2016;
  t2033 = t1997*t2017;
  t2034 = 0. + t2032 + t2033;
  t2043 = -0.209*t2041;
  t2045 = -0.4165*t2013;
  t2046 = 0. + t2043 + t2045;
  t2049 = 0.19*t2041;
  t2050 = 0.19*t2013;
  t2051 = 0.2075*t2017;
  t2052 = 0. + t2049 + t2050 + t2051;
  t2015 = t1997*t2013;
  t2019 = -1.*t2016*t2017;
  t2020 = 0. + t2015 + t2019;
  t2056 = -1.*t1997;
  t2057 = 1. + t2056;
  t2058 = 0.19*t2057;
  t2059 = -1.*t2046*t2016;
  t2060 = t1997*t2052;
  t2061 = 0. + t2058 + t2059 + t2060;
  t2047 = t1997*t2046;
  t2048 = -0.19*t2016;
  t2053 = t2016*t2052;
  t2054 = 0. + t2047 + t2048 + t2053;

  p_output1(0)=t2020;
  p_output1(1)=0.;
  p_output1(2)=t2028;
  p_output1(3)=0. + 0.111*t2028 + 0.049*t2034;
  p_output1(4)=0. + t2028*(0. + t2034*t2054 + t2020*t2061) + t2020*(0. - 1.*t2020*t2054 - 1.*t2028*t2061);
  p_output1(5)=0. - 0.062*t2020;
  p_output1(6)=0.;
  p_output1(7)=-1.;
  p_output1(8)=0.;
  p_output1(9)=0. - 0.19*t2017 - 1.*t2013*t2046 + t2017*t2052;
  p_output1(10)=0.;
  p_output1(11)=0. - 0.19*t2013 + t2017*t2046 + t2013*t2052;
  p_output1(12)=0.;
  p_output1(13)=-1.;
  p_output1(14)=0.;
  p_output1(15)=0.2075;
  p_output1(16)=0.;
  p_output1(17)=0.;
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


       
Eigen::Matrix<double,6,12> Jb_Body_to_FrontRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jb_Body_to_FrontRightFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



