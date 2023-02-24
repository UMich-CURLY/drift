/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:25 GMT-05:00
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
  double t34;
  double t1432;
  double t672;
  double t1547;
  double t6292;
  double t6246;
  double t6248;
  double t6259;
  double t6289;
  double t6305;
  double t6306;
  t34 = Cos(var1[6]);
  t1432 = Sin(var1[6]);
  t672 = 0. + t34;
  t1547 = -0.04675*t1432;
  t6292 = Sin(var1[7]);
  t6246 = -1.*t34;
  t6248 = 1. + t6246;
  t6259 = -0.04675*t6248;
  t6289 = Cos(var1[7]);
  t6305 = -1.*t6289;
  t6306 = 1. + t6305;

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
  p_output1(38)=0;
  p_output1(39)=0.;
  p_output1(40)=0.;
  p_output1(41)=0.04675;
  p_output1(42)=0;
  p_output1(43)=t672;
  p_output1(44)=t1432;
  p_output1(45)=0. + (0. + t1547)*t34 + t1432*(0. + t6259);
  p_output1(46)=0. + 0.1881*t1432;
  p_output1(47)=0. - 0.1881*t34;
  p_output1(48)=0;
  p_output1(49)=t672;
  p_output1(50)=t1432;
  p_output1(51)=0.213*t6289 - 0.1881*t6292 + t1432*(0. + t6259 + 0.1881*t1432*t6292) + t34*(0. + t1547 + 0.1881*t34*t6292);
  p_output1(52)=0. + 0.1881*t1432*t6289 + 0.213*t1432*t6292 + t1432*(0. + 0.1881*t6306);
  p_output1(53)=-0.1881*t34*t6289 - 0.213*t34*t6292 + t34*(0. - 0.1881*t6306);
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



