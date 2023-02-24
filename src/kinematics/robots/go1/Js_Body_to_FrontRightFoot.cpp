/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:18 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Js_Body_to_FrontRightFoot.h"

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
  double t204;
  double t2926;
  double t2405;
  double t3335;
  double t6114;
  double t6069;
  double t6071;
  double t6078;
  double t6109;
  double t6128;
  double t6129;
  t204 = Cos(var1[0]);
  t2926 = Sin(var1[0]);
  t2405 = 0. + t204;
  t3335 = -0.04675*t2926;
  t6114 = Sin(var1[1]);
  t6069 = -1.*t204;
  t6071 = 1. + t6069;
  t6078 = -0.04675*t6071;
  t6109 = Cos(var1[1]);
  t6128 = -1.*t6109;
  t6129 = 1. + t6128;

  p_output1(0)=1.;
  p_output1(1)=0.;
  p_output1(2)=0;
  p_output1(3)=0.;
  p_output1(4)=0.;
  p_output1(5)=0.04675;
  p_output1(6)=0;
  p_output1(7)=t2405;
  p_output1(8)=t2926;
  p_output1(9)=0. + t204*(0. + t3335) + t2926*(0. + t6078);
  p_output1(10)=0. - 0.1881*t2926;
  p_output1(11)=0. + 0.1881*t204;
  p_output1(12)=0;
  p_output1(13)=t2405;
  p_output1(14)=t2926;
  p_output1(15)=0.213*t6109 + 0.1881*t6114 + t204*(0. + t3335 - 0.1881*t204*t6114) + t2926*(0. + t6078 - 0.1881*t2926*t6114);
  p_output1(16)=0. - 0.1881*t2926*t6109 + 0.213*t2926*t6114 + t2926*(0. - 0.1881*t6129);
  p_output1(17)=0.1881*t204*t6109 - 0.213*t204*t6114 + t204*(0. + 0.1881*t6129);
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


       
Eigen::Matrix<double,6,12> Js_Body_to_FrontRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Js_Body_to_FrontRightFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



