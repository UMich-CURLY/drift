/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:16 GMT-05:00
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
  double t5984;
  double t6024;
  double t776;
  double t6031;
  double t6037;
  double t6042;
  double t6045;
  double t6061;
  double t6062;
  double t6047;
  double t6048;
  double t6051;
  double t6064;
  double t6065;
  double t6066;
  double t6069;
  double t6070;
  double t6071;
  double t6072;
  double t6021;
  double t6035;
  double t6036;
  double t6076;
  double t6077;
  double t6078;
  double t6079;
  double t6080;
  double t6081;
  double t6067;
  double t6068;
  double t6073;
  double t6074;
  t5984 = Cos(var1[2]);
  t6024 = Sin(var1[1]);
  t776 = Cos(var1[1]);
  t6031 = Sin(var1[2]);
  t6037 = t5984*t6024;
  t6042 = t776*t6031;
  t6045 = t6037 + t6042;
  t6061 = -1.*t5984;
  t6062 = 1. + t6061;
  t6047 = -1.*t5984*t6024;
  t6048 = -1.*t776*t6031;
  t6051 = t6047 + t6048;
  t6064 = -0.213*t6062;
  t6065 = -0.4205*t5984;
  t6066 = t6064 + t6065;
  t6069 = 0.1881*t6062;
  t6070 = 0.1881*t5984;
  t6071 = -0.2075*t6031;
  t6072 = t6069 + t6070 + t6071;
  t6021 = t776*t5984;
  t6035 = -1.*t6024*t6031;
  t6036 = t6021 + t6035;
  t6076 = -1.*t776;
  t6077 = 1. + t6076;
  t6078 = 0.1881*t6077;
  t6079 = t6066*t6024;
  t6080 = t776*t6072;
  t6081 = t6078 + t6079 + t6080;
  t6067 = t776*t6066;
  t6068 = 0.1881*t6024;
  t6073 = -1.*t6024*t6072;
  t6074 = t6067 + t6068 + t6073;

  p_output1(0)=t6036;
  p_output1(1)=0.;
  p_output1(2)=t6045;
  p_output1(3)=0.12675*t6045 + 0.04675*t6051;
  p_output1(4)=0. + t6045*(t6051*t6074 + t6036*t6081) + t6036*(-1.*t6036*t6074 - 1.*t6045*t6081);
  p_output1(5)=-0.08*t6036;
  p_output1(6)=0;
  p_output1(7)=1.;
  p_output1(8)=0;
  p_output1(9)=-0.1881*t6031 + t5984*t6066 + t6031*t6072;
  p_output1(10)=0.;
  p_output1(11)=t6031*t6066 + t6070 - 1.*t5984*t6072;
  p_output1(12)=0;
  p_output1(13)=1.;
  p_output1(14)=0;
  p_output1(15)=-0.2075;
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



