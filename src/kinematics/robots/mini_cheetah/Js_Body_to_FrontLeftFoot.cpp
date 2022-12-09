/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:44 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Js_Body_to_FrontLeftFoot.h"

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
  double t1952;
  double t1962;
  double t1955;
  double t1959;
  double t1966;
  double t1970;
  double t1971;
  double t1974;
  double t1976;
  double t1994;
  double t1995;
  double t1996;
  double t1992;
  double t2007;
  double t2008;
  t1952 = Cos(var1[3]);
  t1962 = Sin(var1[3]);
  t1955 = -1.*t1952;
  t1959 = 0. + t1955;
  t1966 = -1.*t1962;
  t1970 = 0. + t1966;
  t1971 = 0.049*t1962;
  t1974 = 1. + t1955;
  t1976 = 0.049*t1974;
  t1994 = Sin(var1[4]);
  t1995 = -0.19*t1994;
  t1996 = 0. + t1995;
  t1992 = Cos(var1[4]);
  t2007 = -1.*t1992;
  t2008 = 1. + t2007;

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
  p_output1(18)=1.;
  p_output1(19)=0.;
  p_output1(20)=0.;
  p_output1(21)=0.;
  p_output1(22)=0.;
  p_output1(23)=-0.049;
  p_output1(24)=0.;
  p_output1(25)=t1959;
  p_output1(26)=t1970;
  p_output1(27)=0. - 1.*t1952*(0. + t1971) - 1.*t1962*(0. + t1976);
  p_output1(28)=0. + 0.19*t1962;
  p_output1(29)=0. - 0.19*t1952;
  p_output1(30)=0.;
  p_output1(31)=t1959;
  p_output1(32)=t1970;
  p_output1(33)=-0.209*t1992 + 0.19*t1994 - 1.*t1952*(0. + t1971 - 1.*t1952*t1996) - 1.*t1962*(0. + t1976 - 1.*t1962*t1996);
  p_output1(34)=0. + 0.19*t1962*t1992 + 0.209*t1962*t1994 - 1.*t1962*(0. - 0.19*t2008);
  p_output1(35)=-0.19*t1952*t1992 - 0.209*t1952*t1994 - 1.*t1952*(0. + 0.19*t2008);
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


       
Eigen::Matrix<double,6,12> Js_Body_to_FrontLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Js_Body_to_FrontLeftFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



