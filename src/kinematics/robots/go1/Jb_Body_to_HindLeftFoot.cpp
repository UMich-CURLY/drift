/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:20 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jb_Body_to_HindLeftFoot.h"

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
  double t2751;
  double t2926;
  double t2707;
  double t6098;
  double t6119;
  double t6120;
  double t6122;
  double t6144;
  double t6147;
  double t6125;
  double t6130;
  double t6132;
  double t6148;
  double t6149;
  double t6151;
  double t6154;
  double t6155;
  double t6156;
  double t6157;
  double t2865;
  double t6099;
  double t6114;
  double t6161;
  double t6162;
  double t6163;
  double t6164;
  double t6165;
  double t6166;
  double t6152;
  double t6153;
  double t6158;
  double t6159;
  t2751 = Cos(var1[11]);
  t2926 = Sin(var1[10]);
  t2707 = Cos(var1[10]);
  t6098 = Sin(var1[11]);
  t6119 = t2751*t2926;
  t6120 = t2707*t6098;
  t6122 = t6119 + t6120;
  t6144 = -1.*t2751;
  t6147 = 1. + t6144;
  t6125 = -1.*t2751*t2926;
  t6130 = -1.*t2707*t6098;
  t6132 = t6125 + t6130;
  t6148 = -0.213*t6147;
  t6149 = -0.4205*t2751;
  t6151 = t6148 + t6149;
  t6154 = -0.1881*t6147;
  t6155 = -0.1881*t2751;
  t6156 = -0.2075*t6098;
  t6157 = t6154 + t6155 + t6156;
  t2865 = t2707*t2751;
  t6099 = -1.*t2926*t6098;
  t6114 = t2865 + t6099;
  t6161 = -1.*t2707;
  t6162 = 1. + t6161;
  t6163 = -0.1881*t6162;
  t6164 = t6151*t2926;
  t6165 = t2707*t6157;
  t6166 = t6163 + t6164 + t6165;
  t6152 = t2707*t6151;
  t6153 = -0.1881*t2926;
  t6158 = -1.*t2926*t6157;
  t6159 = t6152 + t6153 + t6158;

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
  p_output1(54)=t6114;
  p_output1(55)=0.;
  p_output1(56)=t6122;
  p_output1(57)=-0.12675*t6122 - 0.04675*t6132;
  p_output1(58)=0. + t6122*(t6132*t6159 + t6114*t6166) + t6114*(-1.*t6114*t6159 - 1.*t6122*t6166);
  p_output1(59)=0.08*t6114;
  p_output1(60)=0;
  p_output1(61)=1.;
  p_output1(62)=0;
  p_output1(63)=0.1881*t6098 + t2751*t6151 + t6098*t6157;
  p_output1(64)=0.;
  p_output1(65)=t6098*t6151 + t6155 - 1.*t2751*t6157;
  p_output1(66)=0;
  p_output1(67)=1.;
  p_output1(68)=0;
  p_output1(69)=-0.2075;
  p_output1(70)=0.;
  p_output1(71)=0;
}


       
Eigen::Matrix<double,6,12> Jb_Body_to_HindLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jb_Body_to_HindLeftFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



