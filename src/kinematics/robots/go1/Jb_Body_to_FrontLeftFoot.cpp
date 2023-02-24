/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:12 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jb_Body_to_FrontLeftFoot.h"

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
  double t3287;
  double t5922;
  double t2107;
  double t5926;
  double t5940;
  double t5941;
  double t5946;
  double t5966;
  double t5967;
  double t5947;
  double t5951;
  double t5954;
  double t5968;
  double t5970;
  double t5971;
  double t5974;
  double t5975;
  double t5976;
  double t5977;
  double t5365;
  double t5932;
  double t5937;
  double t5981;
  double t5982;
  double t5983;
  double t5984;
  double t5985;
  double t5986;
  double t5972;
  double t5973;
  double t5978;
  double t5979;
  t3287 = Cos(var1[5]);
  t5922 = Sin(var1[4]);
  t2107 = Cos(var1[4]);
  t5926 = Sin(var1[5]);
  t5940 = t3287*t5922;
  t5941 = t2107*t5926;
  t5946 = t5940 + t5941;
  t5966 = -1.*t3287;
  t5967 = 1. + t5966;
  t5947 = -1.*t3287*t5922;
  t5951 = -1.*t2107*t5926;
  t5954 = t5947 + t5951;
  t5968 = -0.213*t5967;
  t5970 = -0.4205*t3287;
  t5971 = t5968 + t5970;
  t5974 = 0.1881*t5967;
  t5975 = 0.1881*t3287;
  t5976 = -0.2075*t5926;
  t5977 = t5974 + t5975 + t5976;
  t5365 = t2107*t3287;
  t5932 = -1.*t5922*t5926;
  t5937 = t5365 + t5932;
  t5981 = -1.*t2107;
  t5982 = 1. + t5981;
  t5983 = 0.1881*t5982;
  t5984 = t5971*t5922;
  t5985 = t2107*t5977;
  t5986 = t5983 + t5984 + t5985;
  t5972 = t2107*t5971;
  t5973 = 0.1881*t5922;
  t5978 = -1.*t5922*t5977;
  t5979 = t5972 + t5973 + t5978;

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
  p_output1(18)=t5937;
  p_output1(19)=0.;
  p_output1(20)=t5946;
  p_output1(21)=-0.12675*t5946 - 0.04675*t5954;
  p_output1(22)=0. + t5946*(t5954*t5979 + t5937*t5986) + t5937*(-1.*t5937*t5979 - 1.*t5946*t5986);
  p_output1(23)=0.08*t5937;
  p_output1(24)=0;
  p_output1(25)=1.;
  p_output1(26)=0;
  p_output1(27)=-0.1881*t5926 + t3287*t5971 + t5926*t5977;
  p_output1(28)=0.;
  p_output1(29)=t5926*t5971 + t5975 - 1.*t3287*t5977;
  p_output1(30)=0;
  p_output1(31)=1.;
  p_output1(32)=0;
  p_output1(33)=-0.2075;
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


       
Eigen::Matrix<double,6,12> Jb_Body_to_FrontLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jb_Body_to_FrontLeftFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



