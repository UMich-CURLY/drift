/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:56 GMT-05:00
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
  double t2131;
  double t2134;
  double t2115;
  double t2135;
  double t2143;
  double t2144;
  double t2146;
  double t2156;
  double t2159;
  double t2150;
  double t2151;
  double t2152;
  double t2161;
  double t2163;
  double t2164;
  double t2167;
  double t2168;
  double t2169;
  double t2170;
  double t2133;
  double t2137;
  double t2138;
  double t2174;
  double t2175;
  double t2176;
  double t2177;
  double t2178;
  double t2179;
  double t2165;
  double t2166;
  double t2171;
  double t2172;
  t2131 = Cos(var1[11]);
  t2134 = Sin(var1[10]);
  t2115 = Cos(var1[10]);
  t2135 = Sin(var1[11]);
  t2143 = -1.*t2131*t2134;
  t2144 = -1.*t2115*t2135;
  t2146 = 0. + t2143 + t2144;
  t2156 = -1.*t2131;
  t2159 = 1. + t2156;
  t2150 = t2131*t2134;
  t2151 = t2115*t2135;
  t2152 = 0. + t2150 + t2151;
  t2161 = -0.209*t2159;
  t2163 = -0.4165*t2131;
  t2164 = 0. + t2161 + t2163;
  t2167 = -0.19*t2159;
  t2168 = -0.19*t2131;
  t2169 = 0.2075*t2135;
  t2170 = 0. + t2167 + t2168 + t2169;
  t2133 = t2115*t2131;
  t2137 = -1.*t2134*t2135;
  t2138 = 0. + t2133 + t2137;
  t2174 = -1.*t2115;
  t2175 = 1. + t2174;
  t2176 = -0.19*t2175;
  t2177 = -1.*t2164*t2134;
  t2178 = t2115*t2170;
  t2179 = 0. + t2176 + t2177 + t2178;
  t2165 = t2115*t2164;
  t2166 = 0.19*t2134;
  t2171 = t2134*t2170;
  t2172 = 0. + t2165 + t2166 + t2171;

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
  p_output1(54)=t2138;
  p_output1(55)=0.;
  p_output1(56)=t2146;
  p_output1(57)=0. - 0.111*t2146 - 0.049*t2152;
  p_output1(58)=0. + t2146*(0. + t2152*t2172 + t2138*t2179) + t2138*(0. - 1.*t2138*t2172 - 1.*t2146*t2179);
  p_output1(59)=0. + 0.062*t2138;
  p_output1(60)=0.;
  p_output1(61)=-1.;
  p_output1(62)=0.;
  p_output1(63)=0. + 0.19*t2135 - 1.*t2131*t2164 + t2135*t2170;
  p_output1(64)=0.;
  p_output1(65)=0. + 0.19*t2131 + t2135*t2164 + t2131*t2170;
  p_output1(66)=0.;
  p_output1(67)=-1.;
  p_output1(68)=0.;
  p_output1(69)=0.2075;
  p_output1(70)=0.;
  p_output1(71)=0.;
}


       
Eigen::Matrix<double,6,12> Jb_Body_to_HindLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jb_Body_to_HindLeftFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



